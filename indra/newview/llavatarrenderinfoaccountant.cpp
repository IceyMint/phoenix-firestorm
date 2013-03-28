/**
 * @file   llavatarrenderinfoaccountant.cpp
 * @author Dave Simmons
 * @date   2013-02-28
 * @brief  
 * 
 * $LicenseInfo:firstyear=2013&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2013, Linden Research, Inc.
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

// Precompiled header
#include "llviewerprecompiledheaders.h"
// associated header
#include "llavatarrenderinfoaccountant.h"
// STL headers
// std headers
// external library headers
// other Linden headers
#include "llcharacter.h"
#include "llhttpclient.h"
#include "lltimer.h"
#include "llviewerobjectlist.h"
#include "llviewerregion.h"
#include "llvoavatar.h"
#include "llworld.h"


// Use this for debugging
//#define LL_AVATAR_RENDER_INFO_LOG_SPAM

static	const std::string KEY_AGENTS = "agents";			// map
static 	const std::string KEY_WEIGHT = "weight";			// integer
static	const std::string KEY_GEOMETRY = "geometry";		// integer
static	const std::string KEY_SURFACE =	"surface";			// float

static	const std::string KEY_IDENTIFIER = "identifier";
static	const std::string KEY_MESSAGE = "message";
static	const std::string KEY_ERROR = "error";


// Send data updates about once per minute, only need per-frame resolution
LLFrameTimer LLAvatarRenderInfoAccountant::sRenderInfoReportTimer;


// HTTP responder class for GET request for avatar render weight information
class LLAvatarRenderInfoGetResponder : public LLHTTPClient::Responder
{
public:
	LLAvatarRenderInfoGetResponder(U64 region_handle) : mRegionHandle(region_handle)
	{
	}

	virtual void error(U32 statusNum, const std::string& reason)
	{
		LLViewerRegion * regionp = LLWorld::getInstance()->getRegionFromHandle(mRegionHandle);
		if (regionp)
		{
			llwarns << "HTTP error result for avatar weight GET: " << statusNum 
				<< ", " << reason
				<< " returned by region " << regionp->getName()
				<< llendl;
		}
		else
		{
			llwarns << "Avatar render weight GET error recieved but region not found for " 
				<< mRegionHandle 
				<< ", error " << statusNum 
				<< ", " << reason
				<< llendl;
		}

	}

	virtual void result(const LLSD& content)
	{
		LLViewerRegion * regionp = LLWorld::getInstance()->getRegionFromHandle(mRegionHandle);
		if (regionp)
		{
			#ifdef LL_AVATAR_RENDER_INFO_LOG_SPAM
			llinfos << "Result for avatar weights request for region " << regionp->getName() << ":" << llendl;
			#endif // LL_AVATAR_RENDER_INFO_LOG_SPAM

			if (content.isMap())
			{
				if (content.has(KEY_AGENTS))
				{
					const LLSD & agents = content[KEY_AGENTS];
					if (agents.isMap())
					{
						LLSD::map_const_iterator	report_iter = agents.beginMap();
						while (report_iter != agents.endMap())
						{
							LLUUID target_agent_id = LLUUID(report_iter->first);
							const LLSD & agent_info_map = report_iter->second;
							LLViewerObject* avatarp = gObjectList.findObject(target_agent_id);
							if (avatarp && 
								avatarp->isAvatar() &&
								agent_info_map.isMap())
							{	// Extract the data for this avatar

								#ifdef LL_AVATAR_RENDER_INFO_LOG_SPAM
								llinfos << " Agent " << target_agent_id 
									<< ": " << agent_info_map << llendl;
								#endif	// LL_AVATAR_RENDER_INFO_LOG_SPAM

								if (agent_info_map.has(KEY_WEIGHT))
								{
									((LLVOAvatar *) avatarp)->setReportedVisualComplexity(agent_info_map[KEY_WEIGHT].asInteger());
								}
								if (agent_info_map.has(KEY_GEOMETRY))
								{
									((LLVOAvatar *) avatarp)->setReportedAttachmentGeometryBytes(agent_info_map[KEY_GEOMETRY].asInteger());
								}
								if (agent_info_map.has(KEY_SURFACE))
								{
									((LLVOAvatar *) avatarp)->setReportedAttachmentSurfaceArea((F32) agent_info_map[KEY_SURFACE].asReal());
								}
							}
							report_iter++;
						}
					}
				}	// has "agents"
				else if (content.has(KEY_ERROR))
				{
					const LLSD & error = content[KEY_ERROR];
					llwarns << "Avatar render info GET error: "
						<< error[KEY_IDENTIFIER]
						<< ": " << error[KEY_MESSAGE] 
						<< " from region " << regionp->getName()
						<< llendl;
				}
			}
		}
		else
		{
			llinfos << "Avatar render weight info recieved but region not found for " 
				<< mRegionHandle << llendl;
		}
	}

private:
	U64		mRegionHandle;
};


// HTTP responder class for POST request for avatar render weight information
class LLAvatarRenderInfoPostResponder : public LLHTTPClient::Responder
{
public:
	LLAvatarRenderInfoPostResponder(U64 region_handle) : mRegionHandle(region_handle)
	{
	}

	virtual void error(U32 statusNum, const std::string& reason)
	{
		LLViewerRegion * regionp = LLWorld::getInstance()->getRegionFromHandle(mRegionHandle);
		if (regionp)
		{
			llwarns << "HTTP error result for avatar weight POST: " << statusNum 
				<< ", " << reason
				<< " returned by region " << regionp->getName()
				<< llendl;
		}
		else
		{
			llwarns << "Avatar render weight POST error recieved but region not found for " 
				<< mRegionHandle 
				<< ", error " << statusNum 
				<< ", " << reason
				<< llendl;
		}
	}

	virtual void result(const LLSD& content)
	{
		LLViewerRegion * regionp = LLWorld::getInstance()->getRegionFromHandle(mRegionHandle);
		if (regionp)
		{
			#ifdef LL_AVATAR_RENDER_INFO_LOG_SPAM
			llinfos << "Result for avatar weights POST for region " << regionp->getName()
				<< ": " << content << llendl;
			#endif	// LL_AVATAR_RENDER_INFO_LOG_SPAM

			if (content.isMap())
			{
				if (content.has(KEY_ERROR))
				{
					const LLSD & error = content[KEY_ERROR];
					llwarns << "Avatar render info POST error: "
						<< error[KEY_IDENTIFIER]
						<< ": " << error[KEY_MESSAGE] 
						<< " from region " << regionp->getName()
						<< llendl;
				}
			}
		}
		else
		{
			llinfos << "Avatar render weight POST result recieved but region not found for " 
				<< mRegionHandle << llendl;
		}
	}

private:
	U64		mRegionHandle;
};


// static 
// Send request for one region, no timer checks
void LLAvatarRenderInfoAccountant::sendRenderInfoToRegion(LLViewerRegion * regionp)
{
	std::string url = regionp->getCapability("AvatarRenderInfo");
	if (!url.empty())
	{
		#ifdef LL_AVATAR_RENDER_INFO_LOG_SPAM
		llinfos << "Sending avatar render info to region "
			<< regionp->getName() 
			<< " from " << url
			<< llendl;
		#endif	// LL_AVATAR_RENDER_INFO_LOG_SPAM

		// Build the render info to POST to the region
		LLSD report = LLSD::emptyMap();
		LLSD agents = LLSD::emptyMap();
				
		std::vector<LLCharacter*>::iterator iter = LLCharacter::sInstances.begin();
		while( iter != LLCharacter::sInstances.end() )
		{
			LLVOAvatar* avatar = dynamic_cast<LLVOAvatar*>(*iter);
			if (avatar &&
				avatar->getRezzedStatus() == 2 &&					// Fully rezzed
				!avatar->isDead() &&								// Not dead yet
				avatar->getObjectHost() == regionp->getHost())		// Ensure it's on the same region
			{
				avatar->calculateUpdateRenderCost();			// Make sure the numbers are up-to-date

				LLSD info = LLSD::emptyMap();
				if (avatar->getVisualComplexity() > 0)
				{
					info[KEY_WEIGHT] = avatar->getVisualComplexity();
				}
				if (avatar->getAttachmentGeometryBytes() > 0)
				{
					info[KEY_GEOMETRY] = (S32) avatar->getAttachmentGeometryBytes();
				}
				if (avatar->getAttachmentSurfaceArea() > 0.f)
				{
					info[KEY_SURFACE] = avatar->getAttachmentSurfaceArea();
				}
				if (info.size() > 0)
				{
					agents[avatar->getID().asString()] = info;
				}

				#ifdef LL_AVATAR_RENDER_INFO_LOG_SPAM
				llinfos << "Sending avatar render info for " << avatar->getID()
					<< ": " << info << llendl;
				#endif		// LL_AVATAR_RENDER_INFO_LOG_SPAM
			}
			iter++;
		}

		report[KEY_AGENTS] = agents;
		if (agents.size() > 0)
		{
			LLHTTPClient::post(url, report, new LLAvatarRenderInfoPostResponder(regionp->getHandle()));
		}
	}
}




// static 
// Send request for one region, no timer checks
void LLAvatarRenderInfoAccountant::getRenderInfoFromRegion(LLViewerRegion * regionp)
{
	std::string url = regionp->getCapability("AvatarRenderInfo");
	if (!url.empty())
	{
		#ifdef LL_AVATAR_RENDER_INFO_LOG_SPAM
		llinfos << "Requesting avatar render info for region "
			<< regionp->getName() 
			<< " from " << url
			<< llendl;
		#endif	// LL_AVATAR_RENDER_INFO_LOG_SPAM

		// First send a request to get the latest data
		LLHTTPClient::get(url, new LLAvatarRenderInfoGetResponder(regionp->getHandle()));
	}
}


// static
// Called every frame - send render weight requests to every region
void LLAvatarRenderInfoAccountant::idle()
{
	if (sRenderInfoReportTimer.hasExpired())
	{
		const F32 SECS_BETWEEN_REGION_SCANS   =  5.f;		// Scan the region list every 5 seconds
		const F32 SECS_BETWEEN_REGION_REQUEST = 60.0;		// Update each region every 60 seconds
	
		S32 num_avs = LLCharacter::sInstances.size();

		// Check all regions and see if it's time to fetch/send data
		for (LLWorld::region_list_t::const_iterator iter = LLWorld::getInstance()->getRegionList().begin();
				iter != LLWorld::getInstance()->getRegionList().end(); ++iter)
		{
			LLViewerRegion* regionp = *iter;
			if (regionp &&
				regionp->isAlive() &&
				regionp->capabilitiesReceived() &&						// Region has capability URLs available
				regionp->getRenderInfoRequestTimer().hasExpired())		// Time to make request
			{
				sendRenderInfoToRegion(regionp);
				getRenderInfoFromRegion(regionp);

				// Reset this regions timer, moving to longer intervals if there are lots of avatars around
				regionp->getRenderInfoRequestTimer().resetWithExpiry(SECS_BETWEEN_REGION_REQUEST + (2.f * num_avs));
			}
		}

		// We scanned all the regions, reset the request timer.
		sRenderInfoReportTimer.resetWithExpiry(SECS_BETWEEN_REGION_SCANS);
	}
}


// static
// Make sRenderInfoReportTimer expire so the next call to idle() will scan and query a new region
// called via LLViewerRegion::setCapabilitiesReceived() boost signals when the capabilities
// are returned for a new LLViewerRegion, and is the earliest time to get render info
void LLAvatarRenderInfoAccountant::expireRenderInfoReportTimer()
{
	#ifdef LL_AVATAR_RENDER_INFO_LOG_SPAM
	llinfos << "Viewer has new region capabilities" << llendl;
	#endif		// LL_AVATAR_RENDER_INFO_LOG_SPAM

	sRenderInfoReportTimer.resetWithExpiry(0.f);
}

