 /** 
* @file llcrashlogger.cpp
* @brief Crash logger implementation
*
* $LicenseInfo:firstyear=2003&license=viewerlgpl$
* Second Life Viewer Source Code
* Copyright (C) 2010, Linden Research, Inc.
* 
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation;
* version 2.1 of the License only.
* 
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
* 
* Linden Research, Inc., 945 Battery Street, San Francisco, CA  94111  USA
* $/LicenseInfo$
*/

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <map>

#include "llcrashlogger.h"
#include "llcrashlock.h"
#include "linden_common.h"
#include "llstring.h"
#include "indra_constants.h"	// CRASH_BEHAVIOR_...
#include "llerror.h"
#include "llerrorcontrol.h"
#include "lltimer.h"
#include "lldir.h"
#include "llfile.h"
#include "llsdserialize.h"
#include "llproxy.h"
#include "llcorehttputil.h"
#include "llhttpsdhandler.h"
#include "httpcommon.h"
#include "httpresponse.h"

#include <curl/curl.h>
#include <openssl/crypto.h>
#include <boost/regex.hpp>


// [SL:KB] - Patch: Viewer-CrashLookup | Checked: 2011-03-24 (Catznip-2.6.0a) | Added: Catznip-2.6.0a
#ifdef LL_WINDOWS
	#include <shellapi.h>
#endif // LL_WINDOWS
// [/SL:KB]


BOOL gBreak = false;
BOOL gSent = false;

// <FS:CR> Various missing prototypes
void trimSLLog(std::string& sllog);
std::string getFormDataField(const std::string& strFieldName, const std::string& strFieldValue, const std::string& strBoundary);
std::string getStartupStateFromLog(std::string& sllog);
// </FS:CR>

int LLCrashLogger::ssl_mutex_count = 0;
LLCoreInt::HttpMutex ** LLCrashLogger::ssl_mutex_list = NULL;

class LLCrashLoggerHandler : public LLHttpSDHandler
{
    LOG_CLASS(LLCrashLoggerHandler);
public:
    LLCrashLoggerHandler() {}

protected:
    virtual void onSuccess(LLCore::HttpResponse * response, const LLSD &content);
    virtual void onFailure(LLCore::HttpResponse * response, LLCore::HttpStatus status);

};

void LLCrashLoggerHandler::onSuccess(LLCore::HttpResponse * response, const LLSD &content)
{
    LL_DEBUGS("CRASHREPORT") << "Request to " << response->getRequestURL() << "succeeded" << LL_ENDL;
    gBreak = true;
    gSent = true;
}

void LLCrashLoggerHandler::onFailure(LLCore::HttpResponse * response, LLCore::HttpStatus status)
{
    LL_WARNS("CRASHREPORT") << "Request to " << response->getRequestURL()
                            << " failed: " << status.toString() << LL_ENDL;
    gBreak = true;
}

LLCrashLogger::LLCrashLogger() :
// [SL:KB] - Patch: Viewer-CrashLookup | Checked: 2011-03-24 (Catznip-2.6.0a) | Added: Catznip-2.6.0a
	mCrashLookup(NULL),
// [/SL:KB]
	mCrashBehavior(CRASH_BEHAVIOR_ASK),
	mCrashInPreviousExec(false),
	mCrashSettings("CrashSettings"),
	mSentCrashLogs(false),
	mCrashHost("")
{
}

LLCrashLogger::~LLCrashLogger()
{

}

// TRIM_SIZE must remain larger than LINE_SEARCH_SIZE.
const int TRIM_SIZE = 128000;
const int LINE_SEARCH_DIST = 500;
const std::string SKIP_TEXT = "\n ...Skipping... \n";
void trimSLLog(std::string& sllog)
{
	if(sllog.length() > TRIM_SIZE * 2)
	{
		std::string::iterator head = sllog.begin() + TRIM_SIZE;
		std::string::iterator tail = sllog.begin() + sllog.length() - TRIM_SIZE;
		std::string::iterator new_head = std::find(head, head - LINE_SEARCH_DIST, '\n');
		if(new_head != head - LINE_SEARCH_DIST)
		{
			head = new_head;
		}

		std::string::iterator new_tail = std::find(tail, tail + LINE_SEARCH_DIST, '\n');
		if(new_tail != tail + LINE_SEARCH_DIST)
		{
			tail = new_tail;
		}

		sllog.erase(head, tail);
		sllog.insert(head, SKIP_TEXT.begin(), SKIP_TEXT.end());
	}
}

std::string getStartupStateFromLog(std::string& sllog)
{
	std::string startup_state = "STATE_FIRST";
	std::string startup_token = "Startup state changing from ";

	size_t index = sllog.rfind(startup_token);
	if (index != std::string::npos || index + startup_token.length() > sllog.length()) {
		return startup_state;
	}

	// find new line
	char cur_char = sllog[index + startup_token.length()];
	std::string::size_type newline_loc = index + startup_token.length();
	while(cur_char != '\n' && newline_loc < sllog.length())
	{
		newline_loc++;
		cur_char = sllog[newline_loc];
	}
	
	// get substring and find location of " to "
	std::string state_line = sllog.substr(index, newline_loc - index);
	std::string::size_type state_index = state_line.find(" to ");
	startup_state = state_line.substr(state_index + 4, state_line.length() - state_index - 4);

	return startup_state;
}

bool LLCrashLogger::readFromXML(LLSD& dest, const std::string& filename )
{
    std::string db_file_name = gDirUtilp->getExpandedFilename(LL_PATH_DUMP,filename);
	// <FS:ND> Properly handle unicode path on Windows. Maybe could use a llifstream instead of ifdef?
    // std::ifstream log_file(db_file_name.c_str());
	
#ifdef LL_WINDOWS
	std::ifstream log_file( utf8str_to_utf16str( db_file_name ).c_str());
#else
    std::ifstream log_file(db_file_name.c_str());
#endif
    
	// </FS:ND>

	// Look for it in the given file
	if (log_file.is_open())
	{
		LLSDSerialize::fromXML(dest, log_file);
        log_file.close();
        return true;
    }
    return false;
}

void LLCrashLogger::mergeLogs( LLSD src_sd )
{
    LLSD::map_iterator iter = src_sd.beginMap();
	LLSD::map_iterator end = src_sd.endMap();
	for( ; iter != end; ++iter)
    {
        mDebugLog[iter->first] = iter->second;
    }
}

bool LLCrashLogger::readMinidump(std::string minidump_path)
{
	size_t length=0;

	std::ifstream minidump_stream(minidump_path.c_str(), std::ios_base::in | std::ios_base::binary);
	if(minidump_stream.is_open())
	{
		minidump_stream.seekg(0, std::ios::end);
		length = (size_t)minidump_stream.tellg();
		minidump_stream.seekg(0, std::ios::beg);
		
		LLSD::Binary data;
		data.resize(length);
		
		minidump_stream.read(reinterpret_cast<char *>(&(data[0])),length);
		minidump_stream.close();
		
		mCrashInfo["Minidump"] = data;
	}
	return (length>0?true:false);
}

void LLCrashLogger::gatherFiles()
{
	updateApplication("Gathering logs...");

    LLSD static_sd;
    LLSD dynamic_sd;
    //if we ever want to change the endpoint we send crashes to
    //we can construct a file download ( a la feature table filename for example)
    //containing the new endpoint
    LLSD endpoint;
    std::string grid;
    std::string fqdn;
    
    bool has_logs = readFromXML( static_sd, "static_debug_info.log" );
    has_logs |= readFromXML( dynamic_sd, "dynamic_debug_info.log" );

    
    if ( has_logs )
    {
        mDebugLog = static_sd;
        mergeLogs(dynamic_sd);
		mCrashInPreviousExec = mDebugLog["CrashNotHandled"].asBoolean();

		mFileMap["SecondLifeLog"] = mDebugLog["SLLog"].asString();
		mFileMap["SettingsXml"] = mDebugLog["SettingsFilename"].asString();
        mFileMap["CrashHostUrl"] = loadCrashURLSetting();
		if(mDebugLog.has("CAFilename"))
		{
            LLCore::HttpRequest::setStaticPolicyOption(LLCore::HttpRequest::PO_CA_FILE,
                LLCore::HttpRequest::GLOBAL_POLICY_ID, mDebugLog["CAFilename"].asString(), NULL);
		}
		else
		{
            LLCore::HttpRequest::setStaticPolicyOption(LLCore::HttpRequest::PO_CA_FILE,
                LLCore::HttpRequest::GLOBAL_POLICY_ID, gDirUtilp->getCAFile(), NULL);
		}

		LL_INFOS("CRASHREPORT") << "Using log file from debug log " << mFileMap["SecondLifeLog"] << LL_ENDL;
		LL_INFOS("CRASHREPORT") << "Using settings file from debug log " << mFileMap["SettingsXml"] << LL_ENDL;
	}
	// else
	// {
	//	// Figure out the filename of the second life log
    //    LLCore::HttpRequest::setStaticPolicyOption(LLCore::HttpRequest::PO_CA_FILE,
    //        LLCore::HttpRequest::GLOBAL_POLICY_ID, gDirUtilp->getCAFile(), NULL);
    //    
	//	mFileMap["SecondLifeLog"] = gDirUtilp->getExpandedFilename(LL_PATH_DUMP,"SecondLife.log");
    //  mFileMap["SettingsXml"] = gDirUtilp->getExpandedFilename(LL_PATH_USER_SETTINGS,"settings.xml");
	// }

    // if (!gDirUtilp->fileExists(mFileMap["SecondLifeLog"]) ) //We would prefer to get this from the per-run but here's our fallback.
    // {
    //    mFileMap["SecondLifeLog"] = gDirUtilp->getExpandedFilename(LL_PATH_LOGS,"SecondLife.old");
    // }

	gatherPlatformSpecificFiles();

	mFileMap.erase( "SecondLifeLog" ); // <FS:ND/> Don't send any Firestorm.log. It's likely huge and won't help for crashdump processing.
	mDebugLog.erase( "SLLog" ); // <FS:ND/> Remove SLLog, as it's a path that contains the OS user name.
	

    if ( has_logs && (mFileMap["CrashHostUrl"] != "") )
    {
        mCrashHost = mFileMap["CrashHostUrl"];
    }
	// <FS:ND> Might hardcode mCrashHost to crashlogs.phoenixviewer.com if unset

	// <FS:ND> Do not send out crash reports to Linden Labs. They won't have much use for them without symbols.
	//default to agni, per product
	//mAltCrashHost = "http://viewercrashreport.agni.lindenlab.com/cgi-bin/viewercrashreceiver.py";

	// </FS:ND>

	mCrashInfo["DebugLog"] = mDebugLog;
	mFileMap["StatsLog"] = gDirUtilp->getExpandedFilename(LL_PATH_DUMP,"stats.log");
	
	updateApplication("Encoding files...");

	// <FS:ND> We're not using this. We do not send a LLSD xml with all data embedded.
	//for(std::map<std::string, std::string>::iterator itr = mFileMap.begin(); itr != mFileMap.end(); ++itr)
	//{
	//	std::ifstream f((*itr).second.c_str());
	//	if(f.is_open())
        //{
        //    std::stringstream s;
        //    s << f.rdbuf();

        //    std::string crash_info = s.str();
        //    if(itr->first == "SecondLifeLog")
        //    {
        //        if(!mCrashInfo["DebugLog"].has("StartupState"))
        //        {
        //            mCrashInfo["DebugLog"]["StartupState"] = getStartupStateFromLog(crash_info);
        //        }
        //        trimSLLog(crash_info);
        //    }

        //    mCrashInfo[(*itr).first] = LLStringFn::strip_invalid_xml(rawstr_to_utf8(crash_info));
        //}
        //else
        //{
        //    LL_WARNS("CRASHREPORT") << "Can't find file " << (*itr).second << LL_ENDL;
        //}
	//}

	// </FS:ND>
	
	std::string minidump_path;
	// Add minidump as binary.
    bool has_minidump = mDebugLog.has("MinidumpPath");
    
	if (has_minidump)
	{
		minidump_path = mDebugLog["MinidumpPath"].asString();
	}

	if (has_minidump)
	{
		has_minidump = readMinidump(minidump_path);
	}

    if (!has_minidump)  //Viewer was probably so hosed it couldn't write remaining data.  Try brute force.
    {
       //Look for a filename at least 30 characters long in the dump dir which contains the characters MDMP as the first 4 characters in the file.
        typedef std::vector<std::string> vec;
        std::string pathname = gDirUtilp->getExpandedFilename(LL_PATH_DUMP,"");
        vec file_vec = gDirUtilp->getFilesInDir(pathname);
        for(vec::const_iterator iter=file_vec.begin(); iter!=file_vec.end(); ++iter)
        {
            if ( ( iter->length() > 30 ) && (iter->rfind(".dmp") == (iter->length()-4) ) )
            {
                std::string fullname = pathname + *iter;
                std::ifstream fdat( fullname.c_str(), std::ifstream::binary);
                if (fdat)
                {
                    char buf[5];
                    fdat.read(buf,4);
                    fdat.close();  
                    if (!strncmp(buf,"MDMP",4))
                    {
                        minidump_path = *iter;
                        has_minidump = readMinidump(fullname);
						mDebugLog["MinidumpPath"] = fullname;
						if (has_minidump) 
						{
							break;
						}
                    }
                }
            }
        }
    }

	// <FS:ND> Put minidump file into mFileMap. Otherwise it does not get uploaded to the crashlog server.
	if( has_minidump )
	{
		std::string fullName = mDebugLog["MinidumpPath"];
		std::string dmpName( fullName );
		if( dmpName.size() )
		{
			size_t nStart( dmpName.size()-1 );
			for( std::string::reverse_iterator itr = dmpName.rbegin(); itr != dmpName.rend(); ++itr )
			{
				if( *itr == '/' || *itr == '\\' )
					break;

				--nStart;
			}

			dmpName = dmpName.substr( nStart+1 );
		}

		mFileMap[ dmpName ] = fullName;
	}
	// </FS:ND>
}

LLSD LLCrashLogger::constructPostData()
{
	return mCrashInfo;
}

const char* const CRASH_SETTINGS_FILE = "settings_crash_behavior.xml";

std::string LLCrashLogger::loadCrashURLSetting()
{

	// First check user_settings (in the user's home dir)
	std::string filename = gDirUtilp->getExpandedFilename(LL_PATH_USER_SETTINGS, CRASH_SETTINGS_FILE);
	if (! mCrashSettings.loadFromFile(filename))
	{
		// Next check app_settings (in the SL program dir)
		std::string filename = gDirUtilp->getExpandedFilename(LL_PATH_APP_SETTINGS, CRASH_SETTINGS_FILE);
		mCrashSettings.loadFromFile(filename);
	}

    if (! mCrashSettings.controlExists("CrashHostUrl"))
    {
        return "";
    }
    else
    {
        return mCrashSettings.getString("CrashHostUrl");
    }
}

// [SL:KB] - Patch: Viewer-CrashReporting | Checked: 2011-03-24 (Catznip-2.6.0a) | Added: Catznip-2.6.0a
std::string getFormDataField(const std::string& strFieldName, const std::string& strFieldValue, const std::string& strBoundary)
{
	std::ostringstream streamFormPart;

	streamFormPart << "--" << strBoundary << "\r\n"
		<< "Content-Disposition: form-data; name=\"" << strFieldName << "\"\r\n\r\n"
		<< strFieldValue << "\r\n";

	return streamFormPart.str();
}
// [/SL:KB]

bool LLCrashLogger::runCrashLogPost(std::string host, LLSD data, std::string msg, int retries, int timeout)
{
    LLCore::HttpRequest::ptr_t httpRequest(new LLCore::HttpRequest);
    LLCore::HttpOptions::ptr_t httpOpts(new LLCore::HttpOptions);

    httpOpts->setTimeout(timeout);

	for(int i = 0; i < retries; ++i)
	{
		updateApplication(llformat("%s, try %d...", msg.c_str(), i+1));

        LL_INFOS("CRASHREPORT") << "POST crash data to " << host << LL_ENDL;
        LLCore::HttpHandle handle = LLCoreHttpUtil::requestPostWithLLSD(httpRequest.get(), LLCore::HttpRequest::DEFAULT_POLICY_ID, 0,
            host, data, httpOpts, LLCore::HttpHeaders::ptr_t(), LLCore::HttpHandler::ptr_t(new LLCrashLoggerHandler));

        if (handle == LLCORE_HTTP_HANDLE_INVALID)
        {
            LLCore::HttpStatus status = httpRequest->getStatus();
            LL_WARNS("CRASHREPORT") << "Request POST failed to " << host << " with status of [" <<
                status.getType() << "]\"" << status.toString() << "\"" << LL_ENDL;
            return false;
        }

        while(!gBreak)
        {
            ms_sleep(250);
            updateApplication(); // No new message, just pump the IO
            httpRequest->update(0L);
        }
		if(gSent)
		{
			return gSent;
		}

        LL_WARNS("CRASHREPORT") << "Failed to send crash report to \"" << host << "\"" << LL_ENDL;
	}
	return gSent;
}

bool LLCrashLogger::sendCrashLog(std::string dump_dir)
{

    gDirUtilp->setDumpDir( dump_dir );
    
    // std::string dump_path = gDirUtilp->getExpandedFilename(LL_PATH_LOGS,
    //                                                        "SecondLifeCrashReport");
    std::string dump_path = gDirUtilp->getExpandedFilename(LL_PATH_LOGS, "FirestormCrashReport");
    std::string report_file = dump_path + ".log";

    LL_DEBUGS("CRASHREPORT") << "sending " << report_file << LL_ENDL;

	gatherFiles();
    
	LLSD post_data;
	post_data = constructPostData();
    
	updateApplication("Sending reports...");

#ifdef LL_WINDOWS
	std::ofstream out_file( utf8str_to_utf16str(report_file).c_str() );
#else
	std::ofstream out_file(report_file.c_str());
#endif

	LLSDSerialize::toPrettyXML(post_data, out_file);
    out_file.flush();
	out_file.close();
    
	bool sent = false;
    
    if(mCrashHost != "")
	{
        LL_WARNS("CRASHREPORT") << "Sending crash data to server from CrashHostUrl '" << mCrashHost << "'" << LL_ENDL;
        
        std::string msg = "Using override crash server... ";
        msg = msg+mCrashHost.c_str();
        updateApplication(msg.c_str());
        
		sent = runCrashLogPost(mCrashHost, post_data, std::string("Sending to server"), 3, 5);
	}
    
	// <FS:ND> We do not send to mAltCrashHost ever.

	// if(!sent)
	// {
        //updateApplication("Using default server...");
	// 	sent = runCrashLogPost(mAltCrashHost, post_data, std::string("Sending to alternate server"), 3, 5);
	// }

	// </FS:ND>
    
	mSentCrashLogs = sent;
    
	return sent;
}

bool LLCrashLogger::sendCrashLogs()
{
    
	LLSD opts = getOptionData(PRIORITY_COMMAND_LINE);
    LLSD rec;

	if ( opts.has("dumpdir") )
    {
        rec["pid"]=opts["pid"];
        rec["dumpdir"]=opts["dumpdir"];
        rec["procname"]=opts["procname"];
    }
    else
    {
        return false;
    }       

    return sendCrashLog(rec["dumpdir"].asString());
}

void LLCrashLogger::updateApplication(const std::string& message)
{
	if (!message.empty()) LL_INFOS("CRASHREPORT") << message << LL_ENDL;
}

bool LLCrashLogger::init()
{
    LL_DEBUGS("CRASHREPORT") << LL_ENDL;
    
    LLCore::LLHttp::initialize();

	// We assume that all the logs we're looking for reside on the current drive
#ifdef ND_BUILD64BIT_ARCH
	gDirUtilp->initAppDirs("Firestorm_x64");
#else
	gDirUtilp->initAppDirs("Firestorm");
#endif

	LLError::initForApplication(gDirUtilp->getExpandedFilename(LL_PATH_APP_SETTINGS, ""));

	// Default to the product name "Second Life" (this is overridden by the -name argument)
	
	// <FS:ND> Change default to Firestorm
	//	mProductName = "Second Life";
	mProductName = "Firestorm";
	// </FS:ND>

	// Rename current log file to ".old"
	std::string old_log_file = gDirUtilp->getExpandedFilename(LL_PATH_LOGS, "crashreport.log.old");
	std::string log_file = gDirUtilp->getExpandedFilename(LL_PATH_LOGS, "crashreport.log");

#if LL_WINDOWS
	LLAPRFile::remove(old_log_file);
#endif 

	LLFile::rename(log_file.c_str(), old_log_file.c_str());
    
	// Set the log file to crashreport.log
	LLError::logToFile(log_file);  //NOTE:  Until this line, LL_INFOS LL_WARNS, etc are blown to the ether. 

    LL_INFOS("CRASHREPORT") << "Crash reporter file rotation complete." << LL_ENDL;

    mCrashSettings.declareS32("CrashSubmitBehavior", CRASH_BEHAVIOR_ALWAYS_SEND,
							  "Controls behavior when viewer crashes "
							  "(0 = ask before sending crash report, "
							  "1 = always send crash report, "
							  "2 = never send crash report)");
    
    init_curl();
    LLCore::HttpRequest::createService();
    LLCore::HttpRequest::startThread();

	return true;
}

// For cleanup code common to all platforms.
void LLCrashLogger::commonCleanup()
{
    term_curl();
	LLError::logToFile("");   //close crashreport.log
	LLProxy::cleanupClass();
}

void LLCrashLogger::init_curl()
{
    curl_global_init(CURL_GLOBAL_ALL);

    ssl_mutex_count = CRYPTO_num_locks();
    if (ssl_mutex_count > 0)
    {
        ssl_mutex_list = new LLCoreInt::HttpMutex *[ssl_mutex_count];

        for (int i(0); i < ssl_mutex_count; ++i)
        {
            ssl_mutex_list[i] = new LLCoreInt::HttpMutex;
        }

        CRYPTO_set_locking_callback(ssl_locking_callback);
        CRYPTO_set_id_callback(ssl_thread_id_callback);
    }
}


void LLCrashLogger::term_curl()
{
    CRYPTO_set_locking_callback(NULL);
    for (int i(0); i < ssl_mutex_count; ++i)
    {
        delete ssl_mutex_list[i];
    }
    delete[] ssl_mutex_list;
}


unsigned long LLCrashLogger::ssl_thread_id_callback(void)
{
#if LL_WINDOWS
    return (unsigned long)GetCurrentThread();
#else
    return (unsigned long)pthread_self();
#endif
}


void LLCrashLogger::ssl_locking_callback(int mode, int type, const char * /* file */, int /* line */)
{
    if (type >= 0 && type < ssl_mutex_count)
    {
        if (mode & CRYPTO_LOCK)
        {
            ssl_mutex_list[type]->lock();
        }
        else
        {
            ssl_mutex_list[type]->unlock();
        }
    }
}

