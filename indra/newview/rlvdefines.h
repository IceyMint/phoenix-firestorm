/** 
 *
 * Copyright (c) 2009-2016, Kitty Barnett
 * 
 * The source code in this file is provided to you under the terms of the 
 * GNU Lesser General Public License, version 2.1, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 * PARTICULAR PURPOSE. Terms of the LGPL can be found in doc/LGPL-licence.txt 
 * in this distribution, or online at http://www.gnu.org/licenses/lgpl-2.1.txt
 * 
 * By copying, modifying or distributing this software, you acknowledge that
 * you have read and understood your obligations described above, and agree to 
 * abide by those obligations.
 * 
 */

#ifndef RLV_DEFINES_H
#define RLV_DEFINES_H

// ============================================================================
// Defines
//

// Version of the specifcation we support
const S32 RLV_VERSION_MAJOR = 2;
const S32 RLV_VERSION_MINOR = 8;
const S32 RLV_VERSION_PATCH = 0;
const S32 RLV_VERSION_BUILD = 0;

// Implementation version
const S32 RLVa_VERSION_MAJOR = 2;
const S32 RLVa_VERSION_MINOR = 0;
const S32 RLVa_VERSION_PATCH = 0;
const S32 RLVa_VERSION_BUILD = 0;

// Uncomment before a final release
//#define RLV_RELEASE

// Defining these makes it easier if we ever need to change our tag
#define RLV_WARNS		LL_WARNS("RLV")
#define RLV_INFOS		LL_INFOS("RLV")
#define RLV_DEBUGS		LL_DEBUGS("RLV")
#define RLV_ENDL		LL_ENDL
#define RLV_VERIFY(f)	if (!(f)) { RlvUtil::notifyFailedAssertion(#f, __FILE__, __LINE__); }

#if LL_RELEASE_WITH_DEBUG_INFO || LL_DEBUG
	// Turn on extended debugging information
	#define RLV_DEBUG
	// Make sure we halt execution on errors
	#define RLV_ERRS				LL_ERRS("RLV")
	// Keep our asserts separate from LL's
	#define RLV_ASSERT(f)			if (!(f)) { RLV_ERRS << "ASSERT (" << #f << ")" << RLV_ENDL; }
	#define RLV_ASSERT_DBG(f)		RLV_ASSERT(f)
#else
	// Don't halt execution on errors in release
	#define RLV_ERRS				LL_WARNS("RLV")
	// We don't want to check assertions in release builds
	#ifndef RLV_RELEASE
		#define RLV_ASSERT(f)		RLV_VERIFY(f)
		#define RLV_ASSERT_DBG(f)
	#else
		#define RLV_ASSERT(f)
		#define RLV_ASSERT_DBG(f)
	#endif // RLV_RELEASE
#endif // LL_RELEASE_WITH_DEBUG_INFO || LL_DEBUG

#define RLV_ROOT_FOLDER					"#RLV"
#define RLV_CMD_PREFIX					'@'
#define RLV_MODIFIER_TPLOCAL_DEFAULT    256.f			// Any teleport that's more than a region away is non-local
#define RLV_MODIFIER_FARTOUCH_DEFAULT   1.5f			// Specifies the default @fartouch distance
#define RLV_MODIFIER_SITTP_DEFAULT      1.5f			// Specifies the default @sittp distance
#define RLV_OPTION_SEPARATOR			";"				// Default separator used in command options
#define RLV_PUTINV_PREFIX				"#RLV/~"
#define RLV_PUTINV_SEPARATOR			"/"
#define RLV_PUTINV_MAXDEPTH				4
#define RLV_SETROT_OFFSET				F_PI_BY_TWO		// @setrot is off by 90� with the rest of SL
#define RLV_STRINGS_FILE				"rlva_strings.xml"

#define RLV_FOLDER_FLAG_NOSTRIP			"nostrip"
#define RLV_FOLDER_PREFIX_HIDDEN		'.'
#define RLV_FOLDER_PREFIX_PUTINV    	'~'

// ============================================================================
// Enumeration declarations
//

// NOTE: any changes to this enumeration should be reflected in the RlvBehaviourDictionary constructor
enum ERlvBehaviour {
	RLV_BHVR_DETACH = 0,			// "detach"
	RLV_BHVR_ADDATTACH,				// "addattach"
	RLV_BHVR_REMATTACH,				// "remattach"
	RLV_BHVR_ADDOUTFIT,				// "addoutfit"
	RLV_BHVR_REMOUTFIT,				// "remoutfit"
	RLV_BHVR_SHAREDWEAR,			// "sharedwear"
	RLV_BHVR_SHAREDUNWEAR,			// "sharedunwear"
	RLV_BHVR_UNSHAREDWEAR,			// "unsharedwear"
	RLV_BHVR_UNSHAREDUNWEAR,		// "unsharedunwear"
	RLV_BHVR_EMOTE,					// "emote"
	RLV_BHVR_SENDCHAT,				// "sendchat"
	RLV_BHVR_RECVCHAT,				// "recvchat"
	RLV_BHVR_RECVCHATFROM,			// "recvchatfrom"
	RLV_BHVR_RECVEMOTE,				// "recvemote"
	RLV_BHVR_RECVEMOTEFROM,			// "recvemotefrom"
	RLV_BHVR_REDIRCHAT,				// "redirchat"
	RLV_BHVR_REDIREMOTE,			// "rediremote"
	RLV_BHVR_CHATWHISPER,			// "chatwhisper"
	RLV_BHVR_CHATNORMAL,			// "chatnormal"
	RLV_BHVR_CHATSHOUT,				// "chatshout"
	RLV_BHVR_SENDCHANNEL,
	RLV_BHVR_SENDCHANNELEXCEPT,
	RLV_BHVR_SENDIM,				// "sendim"
	RLV_BHVR_SENDIMTO,				// "sendimto"
	RLV_BHVR_RECVIM,				// "recvim"
	RLV_BHVR_RECVIMFROM,			// "recvimfrom"
	RLV_BHVR_STARTIM,				// "startim"
	RLV_BHVR_STARTIMTO,				// "startimto"
	RLV_BHVR_SENDGESTURE,
	RLV_BHVR_PERMISSIVE,			// "permissive"
	RLV_BHVR_NOTIFY,				// "notify"
	RLV_BHVR_SHOWINV,				// "showinv"
	RLV_BHVR_SHOWMINIMAP,			// "showminimap"
	RLV_BHVR_SHOWWORLDMAP,			// "showworldmap"
	RLV_BHVR_SHOWLOC,				// "showloc"
	RLV_BHVR_SHOWNAMES,				// "shownames"
	RLV_BHVR_SHOWHOVERTEXT,			// "showhovertext"
	RLV_BHVR_SHOWHOVERTEXTHUD,		// "showhovertexthud"
	RLV_BHVR_SHOWHOVERTEXTWORLD,	// "showhovertextworld"
	RLV_BHVR_SHOWHOVERTEXTALL,		// "showhovertextall"
	RLV_BHVR_TPLM,					// "tplm"
	RLV_BHVR_TPLOC,					// "tploc"
	RLV_BHVR_TPLOCAL,
	RLV_BHVR_TPLURE,				// "tplure"
	RLV_BHVR_TPREQUEST,				// "tprequest"
	RLV_BHVR_VIEWNOTE,				// "viewnote"
	RLV_BHVR_VIEWSCRIPT,			// "viewscript"
	RLV_BHVR_VIEWTEXTURE,			// "viewtexture"
	RLV_BHVR_ACCEPTPERMISSION,		// "acceptpermission"
	RLV_BHVR_ACCEPTTP,				// "accepttp"
	RLV_BHVR_ACCEPTTPREQUEST,		// "accepttprequest"
	RLV_BHVR_ALLOWIDLE,				// "allowidle"
	RLV_BHVR_EDIT,					// "edit"
	RLV_BHVR_EDITOBJ,				// "editobj"
	RLV_BHVR_REZ,					// "rez"
	RLV_BHVR_FARTOUCH,				// "fartouch"
	RLV_BHVR_INTERACT,				// "interact"
	RLV_BHVR_TOUCHTHIS,				// "touchthis"
	RLV_BHVR_TOUCHATTACH,			// "touchattach"
	RLV_BHVR_TOUCHATTACHSELF,		// "touchattachself"
	RLV_BHVR_TOUCHATTACHOTHER,		// "touchattachother"
	RLV_BHVR_TOUCHHUD,				// "touchhud"
	RLV_BHVR_TOUCHWORLD,			// "touchworld"
	RLV_BHVR_TOUCHALL,				// "touchall"
	RLV_BHVR_TOUCHME,				// "touchme"
	RLV_BHVR_FLY,					// "fly"
	RLV_BHVR_SETGROUP,				// "setgroup"
	RLV_BHVR_UNSIT,					// "unsit"
	RLV_BHVR_SIT,					// "sit"
	RLV_BHVR_SITTP,					// "sittp"
	RLV_BHVR_STANDTP,				// "standtp"
	RLV_BHVR_SETDEBUG,				// "setdebug"
	RLV_BHVR_SETENV,				// "setenv"
	RLV_BHVR_ALWAYSRUN,				// "alwaysrun"
	RLV_BHVR_TEMPRUN,				// "temprun"
	RLV_BHVR_DETACHME,				// "detachme"
	RLV_BHVR_ATTACHTHIS,			// "attachthis"
	RLV_BHVR_ATTACHTHISEXCEPT,		// "attachthis_except"
	RLV_BHVR_DETACHTHIS,			// "detachthis"
	RLV_BHVR_DETACHTHISEXCEPT,		// "detachthis_except"
	RLV_BHVR_ADJUSTHEIGHT,			// "adjustheight"
	RLV_BHVR_TPTO,					// "tpto"
	RLV_BHVR_VERSION,				// "version"
	RLV_BHVR_VERSIONNEW,			// "versionnew"
	RLV_BHVR_VERSIONNUM,			// "versionnum"
	RLV_BHVR_GETATTACH,				// "getattach"
	RLV_BHVR_GETATTACHNAMES,		// "getattachnames"
	RLV_BHVR_GETADDATTACHNAMES,		// "getaddattachnames"
	RLV_BHVR_GETREMATTACHNAMES,		// "getremattachnames"
	RLV_BHVR_GETOUTFIT,				// "getoutfit"
	RLV_BHVR_GETOUTFITNAMES,		// "getoutfitnames"
	RLV_BHVR_GETADDOUTFITNAMES,		// "getaddoutfitnames"
	RLV_BHVR_GETREMOUTFITNAMES,		// "getremoutfitnames"
	RLV_BHVR_FINDFOLDER,			// "findfolder"
	RLV_BHVR_FINDFOLDERS,			// "findfolders"
	RLV_BHVR_GETPATH,				// "getpath"
	RLV_BHVR_GETPATHNEW,			// "getpathnew"
	RLV_BHVR_GETINV,				// "getinv"
	RLV_BHVR_GETINVWORN,			// "getinvworn"
	RLV_BHVR_GETGROUP,				// "getgroup"
	RLV_BHVR_GETSITID,				// "getsitid"
	RLV_BHVR_GETCOMMAND,			// "getcommand"
	RLV_BHVR_GETSTATUS,				// "getstatus"
	RLV_BHVR_GETSTATUSALL,			// "getstatusall"
	RLV_CMD_FORCEWEAR,				// Internal representation of all force wear commands

	RLV_BHVR_COUNT,
	RLV_BHVR_UNKNOWN
};

enum ERlvBehaviourModifier
{
	RLV_MODIFIER_SITTPDIST,
	RLV_MODIFIER_TPLOCALDIST,

	RLV_MODIFIER_COUNT,
	RLV_MODIFIER_UNKNOWN
};

enum ERlvBehaviourOptionType
{
	RLV_OPTION_NONE,				// Behaviour takes no parameters
	RLV_OPTION_EXCEPTION,			// Behaviour requires an exception as a parameter
	RLV_OPTION_NONE_OR_EXCEPTION,	// Behaviour takes either no parameters or an exception
	RLV_OPTION_MODIFIER,			// Behaviour requires a modifier as a parameter
	RLV_OPTION_NONE_OR_MODIFIER		// Behaviour takes either no parameters or a modifier
};

enum ERlvParamType {
	RLV_TYPE_UNKNOWN = 0x00,
	RLV_TYPE_ADD     = 0x01,		// <param> == "n"|"add"
	RLV_TYPE_REMOVE  = 0x02,		// <param> == "y"|"rem"
	RLV_TYPE_FORCE   = 0x04,		// <param> == "force"
	RLV_TYPE_REPLY   = 0x08,		// <param> == <number>
	RLV_TYPE_CLEAR   = 0x10,
	RLV_TYPE_ADDREM  = RLV_TYPE_ADD | RLV_TYPE_REMOVE
};

enum ERlvCmdRet {
	RLV_RET_UNKNOWN		= 0x0000,	// Unknown error (should only be used internally)
	RLV_RET_RETAINED,				// Command was retained
	RLV_RET_SUCCESS		= 0x0100,	// Command executed succesfully
	RLV_RET_SUCCESS_UNSET,			// Command executed succesfully (RLV_TYPE_REMOVE for an unrestricted behaviour)
	RLV_RET_SUCCESS_DUPLICATE,		// Command executed succesfully (RLV_TYPE_ADD for an already restricted behaviour)
	RLV_RET_SUCCESS_DELAYED,		// Command parsed valid but will execute at a later time
	RLV_RET_FAILED		= 0x0200,	// Command failed (general failure)
	RLV_RET_FAILED_SYNTAX,			// Command failed (syntax error)
	RLV_RET_FAILED_OPTION,			// Command failed (invalid option)
	RLV_RET_FAILED_PARAM,			// Command failed (invalid param)
	RLV_RET_FAILED_LOCK,			// Command failed (command is locked by another object)
	RLV_RET_FAILED_DISABLED,		// Command failed (command disabled by user)
	RLV_RET_FAILED_UNKNOWN,			// Command failed (unknown command)
	RLV_RET_FAILED_NOSHAREDROOT,	// Command failed (missing #RLV)
	RLV_RET_DEPRECATED,				// Command has been deprecated
	RLV_RET_NO_PROCESSOR			// Command doesn't have a template processor define (legacy code)
};

enum ERlvExceptionCheck
{
	RLV_CHECK_PERMISSIVE,			// Exception can be set by any object
	RLV_CHECK_STRICT,				// Exception must be set by all objects holding the restriction
	RLV_CHECK_DEFAULT				// Permissive or strict will be determined by currently enforced restrictions
};

enum ERlvLockMask
{
	RLV_LOCK_ADD	= 0x01,
	RLV_LOCK_REMOVE	= 0x02,
	RLV_LOCK_ANY	= RLV_LOCK_ADD | RLV_LOCK_REMOVE
};

enum ERlvWearMask
{
	RLV_WEAR_LOCKED	 = 0x00,		// User can not wear the item at all
	RLV_WEAR_ADD	 = 0x01,		// User can wear the item in addition to what's already worn
	RLV_WEAR_REPLACE = 0x02,		// User can wear the item and replace what's currently worn
	RLV_WEAR		 = 0x03			// Convenience: combines RLV_WEAR_ADD and RLV_WEAR_REPLACE
};

enum ERlvAttachGroupType
{
	RLV_ATTACHGROUP_HEAD = 0,
	RLV_ATTACHGROUP_TORSO,
	RLV_ATTACHGROUP_ARMS,
	RLV_ATTACHGROUP_LEGS,
	RLV_ATTACHGROUP_HUD,
	RLV_ATTACHGROUP_COUNT,
	RLV_ATTACHGROUP_INVALID
};

// ============================================================================
// Settings
//

#define RLV_SETTING_MAIN				"RestrainedLove"
#define RLV_SETTING_DEBUG				"RestrainedLoveDebug"
#define RLV_SETTING_CANOOC				"RestrainedLoveCanOOC"
#define RLV_SETTING_FORBIDGIVETORLV		"RestrainedLoveForbidGiveToRLV"
#define RLV_SETTING_NOSETENV			"RestrainedLoveNoSetEnv"
#define RLV_SETTING_SHOWELLIPSIS		"RestrainedLoveShowEllipsis"
#define RLV_SETTING_WEARADDPREFIX       "RestrainedLoveStackWhenFolderBeginsWith"
#define RLV_SETTING_WEARREPLACEPREFIX   "RestrainedLoveReplaceWhenFolderBeginsWith"

#define RLV_SETTING_DEBUGHIDEUNSETDUP   "RLVaDebugHideUnsetDuplicate"
#define RLV_SETTING_ENABLECOMPOSITES	"RLVaEnableCompositeFolders"
#define RLV_SETTING_ENABLELEGACYNAMING	"RLVaEnableLegacyNaming"
#define RLV_SETTING_ENABLESHAREDWEAR	"RLVaEnableSharedWear"
#define RLV_SETTING_HIDELOCKEDLAYER		"RLVaHideLockedLayers"
#define RLV_SETTING_HIDELOCKEDATTACH	"RLVaHideLockedAttachments"
#define RLV_SETTING_HIDELOCKEDINVENTORY	"RLVaHideLockedInventory"
#define RLV_SETTING_LOGINLASTLOCATION	"RLVaLoginLastLocation"
#define RLV_SETTING_SHAREDINVAUTORENAME	"RLVaSharedInvAutoRename"
#define RLV_SETTING_SHOWASSERTIONFAIL	"RLVaShowAssertionFailures"
#define RLV_SETTING_SHOWNAMETAGS		"RLVaShowNameTags"
#define RLV_SETTING_TOPLEVELMENU		"RLVaTopLevelMenu"
#define RLV_SETTING_WEARREPLACEUNLOCKED	"RLVaWearReplaceUnlocked"

#define RLV_SETTING_FIRSTUSE_PREFIX		"FirstRLV"
#define RLV_SETTING_FIRSTUSE_GIVETORLV	RLV_SETTING_FIRSTUSE_PREFIX"GiveToRLV"

// ============================================================================
// Strings (see rlva_strings.xml)
//

#define RLV_STRING_HIDDEN					"hidden_generic"
#define RLV_STRING_HIDDEN_PARCEL			"hidden_parcel"
#define RLV_STRING_HIDDEN_REGION			"hidden_region"

#define RLV_STRING_BLOCKED_GENERIC			"blocked_generic"
#define RLV_STRING_BLOCKED_PERMATTACH		"blocked_permattach"
#define RLV_STRING_BLOCKED_PERMTELEPORT		"blocked_permteleport"
#define RLV_STRING_BLOCKED_RECVIM			"blocked_recvim"
#define RLV_STRING_BLOCKED_RECVIM_REMOTE	"blocked_recvim_remote"
#define RLV_STRING_BLOCKED_SENDIM			"blocked_sendim"
#define RLV_STRING_BLOCKED_STARTCONF		"blocked_startconf"
#define RLV_STRING_BLOCKED_STARTIM			"blocked_startim"
#define RLV_STRING_BLOCKED_TELEPORT			"blocked_teleport"
#define RLV_STRING_BLOCKED_TPLUREREQ_REMOTE	"blocked_tplurerequest_remote"
#define RLV_STRING_BLOCKED_VIEWXXX			"blocked_viewxxx"
#define RLV_STRING_BLOCKED_WIREFRAME		"blocked_wireframe"

// ============================================================================

#endif // RLV_DEFINES_H
