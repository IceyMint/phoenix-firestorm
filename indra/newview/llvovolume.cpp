/** 
 * @file llvovolume.cpp
 * @brief LLVOVolume class implementation
 *
 * $LicenseInfo:firstyear=2001&license=viewerlgpl$
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

// A "volume" is a box, cylinder, sphere, or other primitive shape.

#include "llviewerprecompiledheaders.h"

#include "llvovolume.h"

#include <sstream>

#include "llviewercontrol.h"
#include "lldir.h"
#include "llflexibleobject.h"
#include "llfloatertools.h"
#include "llmaterialid.h"
#include "llmaterialtable.h"
#include "llprimitive.h"
#include "llvolume.h"
#include "llvolumeoctree.h"
#include "llvolumemgr.h"
#include "llvolumemessage.h"
#include "material_codes.h"
#include "message.h"
#include "llpluginclassmedia.h" // for code in the mediaEvent handler
#include "object_flags.h"
#include "lldrawable.h"
#include "lldrawpoolavatar.h"
#include "lldrawpoolbump.h"
#include "llface.h"
#include "llspatialpartition.h"
#include "llhudmanager.h"
#include "llflexibleobject.h"
#include "llskinningutil.h"
#include "llsky.h"
#include "lltexturefetch.h"
#include "llvector4a.h"
#include "llviewercamera.h"
#include "llviewertexturelist.h"
#include "llviewerobjectlist.h"
#include "llviewerregion.h"
#include "llviewertextureanim.h"
#include "llworld.h"
#include "llselectmgr.h"
#include "pipeline.h"
#include "llsdutil.h"
#include "llmatrix4a.h"
#include "llmediaentry.h"
#include "llmediadataclient.h"
#include "llmeshrepository.h"
#include "llagent.h"
#include "llviewermediafocus.h"
#include "lldatapacker.h"
#include "llviewershadermgr.h"
#include "llvoavatar.h"
#include "llcontrolavatar.h"
#include "llvoavatarself.h"
#include "llvocache.h"
#include "llmaterialmgr.h"
#include "llanimationstates.h"
#include "llinventorytype.h"
#include "llviewerinventory.h"

const F32 FORCE_SIMPLE_RENDER_AREA = 512.f;
const F32 FORCE_CULL_AREA = 8.f;
U32 JOINT_COUNT_REQUIRED_FOR_FULLRIG = 1;

BOOL gAnimateTextures = TRUE;
//extern BOOL gHideSelectedObjects;

// AXON TEMP
S32 LLVOVolume::sForceLOD = -1;
F32 LLVOVolume::sLODFactor = 1.f;
F32	LLVOVolume::sLODSlopDistanceFactor = 0.5f; //Changing this to zero, effectively disables the LOD transition slop 
F32 LLVOVolume::sDistanceFactor = 1.0f;
S32 LLVOVolume::sNumLODChanges = 0;
S32 LLVOVolume::mRenderComplexity_last = 0;
S32 LLVOVolume::mRenderComplexity_current = 0;
LLPointer<LLObjectMediaDataClient> LLVOVolume::sObjectMediaClient = NULL;
LLPointer<LLObjectMediaNavigateClient> LLVOVolume::sObjectMediaNavigateClient = NULL;

static LLTrace::BlockTimerStatHandle FTM_GEN_TRIANGLES("Generate Triangles");
static LLTrace::BlockTimerStatHandle FTM_GEN_VOLUME("Generate Volumes");
static LLTrace::BlockTimerStatHandle FTM_VOLUME_TEXTURES("Volume Textures");

extern BOOL gGLDebugLoggingEnabled;

// Implementation class of LLMediaDataClientObject.  See llmediadataclient.h
class LLMediaDataClientObjectImpl : public LLMediaDataClientObject
{
public:
	LLMediaDataClientObjectImpl(LLVOVolume *obj, bool isNew) : mObject(obj), mNew(isNew) 
	{
		mObject->addMDCImpl();
	}
	~LLMediaDataClientObjectImpl()
	{
		mObject->removeMDCImpl();
	}
	
	virtual U8 getMediaDataCount() const 
		{ return mObject->getNumTEs(); }

	virtual LLSD getMediaDataLLSD(U8 index) const 
		{
			LLSD result;
			LLTextureEntry *te = mObject->getTE(index); 
			if (NULL != te)
			{
				llassert((te->getMediaData() != NULL) == te->hasMedia());
				if (te->getMediaData() != NULL)
				{
					result = te->getMediaData()->asLLSD();
					// XXX HACK: workaround bug in asLLSD() where whitelist is not set properly
					// See DEV-41949
					if (!result.has(LLMediaEntry::WHITELIST_KEY))
					{
						result[LLMediaEntry::WHITELIST_KEY] = LLSD::emptyArray();
					}
				}
			}
			return result;
		}
	virtual bool isCurrentMediaUrl(U8 index, const std::string &url) const
		{
			LLTextureEntry *te = mObject->getTE(index); 
			if (te)
			{
				if (te->getMediaData())
				{
					return (te->getMediaData()->getCurrentURL() == url);
				}
			}
			return url.empty();
		}

	virtual LLUUID getID() const
		{ return mObject->getID(); }

	virtual void mediaNavigateBounceBack(U8 index)
		{ mObject->mediaNavigateBounceBack(index); }
	
	virtual bool hasMedia() const
		{ return mObject->hasMedia(); }
	
	virtual void updateObjectMediaData(LLSD const &data, const std::string &version_string) 
		{ mObject->updateObjectMediaData(data, version_string); }
	
	virtual F64 getMediaInterest() const 
		{ 
			F64 interest = mObject->getTotalMediaInterest();
			if (interest < (F64)0.0)
			{
				// media interest not valid yet, try pixel area
				interest = mObject->getPixelArea();
				// HACK: force recalculation of pixel area if interest is the "magic default" of 1024.
				if (interest == 1024.f)
				{
					const_cast<LLVOVolume*>(static_cast<LLVOVolume*>(mObject))->setPixelAreaAndAngle(gAgent);
					interest = mObject->getPixelArea();
				}
			}
			return interest; 
		}
	
	virtual bool isInterestingEnough() const
		{
			return LLViewerMedia::isInterestingEnough(mObject, getMediaInterest());
		}

	virtual std::string getCapabilityUrl(const std::string &name) const
		{ return mObject->getRegion()->getCapability(name); }
	
	virtual bool isDead() const
		{ return mObject->isDead(); }
	
	virtual U32 getMediaVersion() const
		{ return LLTextureEntry::getVersionFromMediaVersionString(mObject->getMediaURL()); }
	
	virtual bool isNew() const
		{ return mNew; }

private:
	LLPointer<LLVOVolume> mObject;
	bool mNew;
};


LLVOVolume::LLVOVolume(const LLUUID &id, const LLPCode pcode, LLViewerRegion *regionp)
	: LLViewerObject(id, pcode, regionp),
	  mVolumeImpl(NULL)
{
	mTexAnimMode = 0;
	mRelativeXform.setIdentity();
	mRelativeXformInvTrans.setIdentity();

	mFaceMappingChanged = FALSE;
	mLOD = MIN_LOD;
	mTextureAnimp = NULL;
	mVolumeChanged = FALSE;
	mVObjRadius = LLVector3(1,1,0.5f).length();
	mNumFaces = 0;
	mLODChanged = FALSE;
	mSculptChanged = FALSE;
	mSpotLightPriority = 0.f;

	mMediaImplList.resize(getNumTEs());
	mLastFetchedMediaVersion = -1;
	mIndexInTex = 0;
	mMDCImplCount = 0;
}

LLVOVolume::~LLVOVolume()
{
	delete mTextureAnimp;
	mTextureAnimp = NULL;
	delete mVolumeImpl;
	mVolumeImpl = NULL;

	if(!mMediaImplList.empty())
	{
		for(U32 i = 0 ; i < mMediaImplList.size() ; i++)
		{
			if(mMediaImplList[i].notNull())
			{
				mMediaImplList[i]->removeObject(this) ;
			}
		}
	}
}

void LLVOVolume::markDead()
{
	if (!mDead)
	{
		if(getMDCImplCount() > 0)
		{
			LLMediaDataClientObject::ptr_t obj = new LLMediaDataClientObjectImpl(const_cast<LLVOVolume*>(this), false);
			if (sObjectMediaClient) sObjectMediaClient->removeFromQueue(obj);
			if (sObjectMediaNavigateClient) sObjectMediaNavigateClient->removeFromQueue(obj);
		}
		
		// Detach all media impls from this object
		for(U32 i = 0 ; i < mMediaImplList.size() ; i++)
		{
			removeMediaImpl(i);
		}

		if (mSculptTexture.notNull())
		{
			mSculptTexture->removeVolume(this);
		}
	}
	
	LLViewerObject::markDead();
}


// static
void LLVOVolume::initClass()
{
	// gSavedSettings better be around
	if (gSavedSettings.getBOOL("PrimMediaMasterEnabled"))
	{
		const F32 queue_timer_delay = gSavedSettings.getF32("PrimMediaRequestQueueDelay");
		const F32 retry_timer_delay = gSavedSettings.getF32("PrimMediaRetryTimerDelay");
		const U32 max_retries = gSavedSettings.getU32("PrimMediaMaxRetries");
		const U32 max_sorted_queue_size = gSavedSettings.getU32("PrimMediaMaxSortedQueueSize");
		const U32 max_round_robin_queue_size = gSavedSettings.getU32("PrimMediaMaxRoundRobinQueueSize");
		sObjectMediaClient = new LLObjectMediaDataClient(queue_timer_delay, retry_timer_delay, max_retries, 
														 max_sorted_queue_size, max_round_robin_queue_size);
		sObjectMediaNavigateClient = new LLObjectMediaNavigateClient(queue_timer_delay, retry_timer_delay, 
																	 max_retries, max_sorted_queue_size, max_round_robin_queue_size);
	}
}

// static
void LLVOVolume::cleanupClass()
{
    sObjectMediaClient = NULL;
    sObjectMediaNavigateClient = NULL;
}

U32 LLVOVolume::processUpdateMessage(LLMessageSystem *mesgsys,
										  void **user_data,
										  U32 block_num, EObjectUpdateType update_type,
										  LLDataPacker *dp)
{
	LLColor4U color;
	const S32 teDirtyBits = (TEM_CHANGE_TEXTURE|TEM_CHANGE_COLOR|TEM_CHANGE_MEDIA);

	// Do base class updates...
	U32 retval = LLViewerObject::processUpdateMessage(mesgsys, user_data, block_num, update_type, dp);

	LLUUID sculpt_id;
	U8 sculpt_type = 0;
	if (isSculpted())
	{
		LLSculptParams *sculpt_params = (LLSculptParams *)getParameterEntry(LLNetworkData::PARAMS_SCULPT);
		sculpt_id = sculpt_params->getSculptTexture();
		sculpt_type = sculpt_params->getSculptType();
	}

	if (!dp)
	{
		if (update_type == OUT_FULL)
		{
			////////////////////////////////
			//
			// Unpack texture animation data
			//
			//

			if (mesgsys->getSizeFast(_PREHASH_ObjectData, block_num, _PREHASH_TextureAnim))
			{
				if (!mTextureAnimp)
				{
					mTextureAnimp = new LLViewerTextureAnim(this);
				}
				else
				{
					if (!(mTextureAnimp->mMode & LLTextureAnim::SMOOTH))
					{
						mTextureAnimp->reset();
					}
				}
				mTexAnimMode = 0;
				
				mTextureAnimp->unpackTAMessage(mesgsys, block_num);
			}
			else
			{
				if (mTextureAnimp)
				{
					delete mTextureAnimp;
					mTextureAnimp = NULL;
					gPipeline.markTextured(mDrawable);
					mFaceMappingChanged = TRUE;
					mTexAnimMode = 0;
				}
			}

			// Unpack volume data
			LLVolumeParams volume_params;
			LLVolumeMessage::unpackVolumeParams(&volume_params, mesgsys, _PREHASH_ObjectData, block_num);
			volume_params.setSculptID(sculpt_id, sculpt_type);

			if (setVolume(volume_params, 0))
			{
				markForUpdate(TRUE);
			}
		}

		// Sigh, this needs to be done AFTER the volume is set as well, otherwise bad stuff happens...
		////////////////////////////
		//
		// Unpack texture entry data
		//

		S32 result = unpackTEMessage(mesgsys, _PREHASH_ObjectData, (S32) block_num);
		if (result & teDirtyBits)
		{
			updateTEData();
		}
		if (result & TEM_CHANGE_MEDIA)
		{
			retval |= MEDIA_FLAGS_CHANGED;
		}
	}
	else
	{
		if (update_type != OUT_TERSE_IMPROVED)
		{
			LLVolumeParams volume_params;
			BOOL res = LLVolumeMessage::unpackVolumeParams(&volume_params, *dp);
			if (!res)
			{
				LL_WARNS() << "Bogus volume parameters in object " << getID() << LL_ENDL;
				LL_WARNS() << getRegion()->getOriginGlobal() << LL_ENDL;
			}

			volume_params.setSculptID(sculpt_id, sculpt_type);

			if (setVolume(volume_params, 0))
			{
				markForUpdate(TRUE);
			}
			S32 res2 = unpackTEMessage(*dp);
			if (TEM_INVALID == res2)
			{
				// There's something bogus in the data that we're unpacking.
				dp->dumpBufferToLog();
				LL_WARNS() << "Flushing cache files" << LL_ENDL;

				if(LLVOCache::instanceExists() && getRegion())
				{
					LLVOCache::getInstance()->removeEntry(getRegion()->getHandle()) ;
				}
				
				LL_WARNS() << "Bogus TE data in " << getID() << LL_ENDL;
			}
			else 
			{
				if (res2 & teDirtyBits) 
				{
					updateTEData();
				}
				if (res2 & TEM_CHANGE_MEDIA)
				{
					retval |= MEDIA_FLAGS_CHANGED;
				}
			}

			U32 value = dp->getPassFlags();

			if (value & 0x40)
			{
				if (!mTextureAnimp)
				{
					mTextureAnimp = new LLViewerTextureAnim(this);
				}
				else
				{
					if (!(mTextureAnimp->mMode & LLTextureAnim::SMOOTH))
					{
						mTextureAnimp->reset();
					}
				}
				mTexAnimMode = 0;
				mTextureAnimp->unpackTAMessage(*dp);
			}
			else if (mTextureAnimp)
			{
				delete mTextureAnimp;
				mTextureAnimp = NULL;
				gPipeline.markTextured(mDrawable);
				mFaceMappingChanged = TRUE;
				mTexAnimMode = 0;
			}

			if (value & 0x400)
			{ //particle system (new)
				unpackParticleSource(*dp, mOwnerID, false);
			}
		}
		else
		{
			S32 texture_length = mesgsys->getSizeFast(_PREHASH_ObjectData, block_num, _PREHASH_TextureEntry);
			if (texture_length)
			{
				U8							tdpbuffer[1024];
				LLDataPackerBinaryBuffer	tdp(tdpbuffer, 1024);
				mesgsys->getBinaryDataFast(_PREHASH_ObjectData, _PREHASH_TextureEntry, tdpbuffer, 0, block_num);
				S32 result = unpackTEMessage(tdp);
				if (result & teDirtyBits)
				{
					updateTEData();
				}
				if (result & TEM_CHANGE_MEDIA)
				{
					retval |= MEDIA_FLAGS_CHANGED;
				}
			}
		}
	}
	if (retval & (MEDIA_URL_REMOVED | MEDIA_URL_ADDED | MEDIA_URL_UPDATED | MEDIA_FLAGS_CHANGED)) 
	{
		// If only the media URL changed, and it isn't a media version URL,
		// ignore it
		if ( ! ( retval & (MEDIA_URL_ADDED | MEDIA_URL_UPDATED) &&
				 mMedia && ! mMedia->mMediaURL.empty() &&
				 ! LLTextureEntry::isMediaVersionString(mMedia->mMediaURL) ) )
		{
			// If the media changed at all, request new media data
			LL_DEBUGS("MediaOnAPrim") << "Media update: " << getID() << ": retval=" << retval << " Media URL: " <<
                ((mMedia) ?  mMedia->mMediaURL : std::string("")) << LL_ENDL;
			requestMediaDataUpdate(retval & MEDIA_FLAGS_CHANGED);
		}
        else {
            LL_INFOS("MediaOnAPrim") << "Ignoring media update for: " << getID() << " Media URL: " <<
                ((mMedia) ?  mMedia->mMediaURL : std::string("")) << LL_ENDL;
        }
	}
	// ...and clean up any media impls
	cleanUpMediaImpls();

	return retval;
}


void LLVOVolume::animateTextures()
{
	if (!mDead)
	{
		F32 off_s = 0.f, off_t = 0.f, scale_s = 1.f, scale_t = 1.f, rot = 0.f;
		S32 result = mTextureAnimp->animateTextures(off_s, off_t, scale_s, scale_t, rot);
	
		if (result)
		{
			if (!mTexAnimMode)
			{
				mFaceMappingChanged = TRUE;
				gPipeline.markTextured(mDrawable);
			}
			mTexAnimMode = result | mTextureAnimp->mMode;
				
			S32 start=0, end=mDrawable->getNumFaces()-1;
			if (mTextureAnimp->mFace >= 0 && mTextureAnimp->mFace <= end)
			{
				start = end = mTextureAnimp->mFace;
			}
		
			for (S32 i = start; i <= end; i++)
			{
				LLFace* facep = mDrawable->getFace(i);
				if (!facep) continue;
				if(facep->getVirtualSize() <= MIN_TEX_ANIM_SIZE && facep->mTextureMatrix) continue;

				const LLTextureEntry* te = facep->getTextureEntry();
			
				if (!te)
				{
					continue;
				}
		
				if (!(result & LLViewerTextureAnim::ROTATE))
				{
					te->getRotation(&rot);
				}
				if (!(result & LLViewerTextureAnim::TRANSLATE))
				{
					te->getOffset(&off_s,&off_t);
				}			
				if (!(result & LLViewerTextureAnim::SCALE))
				{
					te->getScale(&scale_s, &scale_t);
				}

				if (!facep->mTextureMatrix)
				{
					facep->mTextureMatrix = new LLMatrix4();
				}

				LLMatrix4& tex_mat = *facep->mTextureMatrix;
				tex_mat.setIdentity();
				LLVector3 trans ;

					trans.set(LLVector3(off_s+0.5f, off_t+0.5f, 0.f));			
					tex_mat.translate(LLVector3(-0.5f, -0.5f, 0.f));

				LLVector3 scale(scale_s, scale_t, 1.f);			
				LLQuaternion quat;
				quat.setQuat(rot, 0, 0, -1.f);
		
				tex_mat.rotate(quat);				

				LLMatrix4 mat;
				mat.initAll(scale, LLQuaternion(), LLVector3());
				tex_mat *= mat;
		
				tex_mat.translate(trans);
			}
		}
		else
		{
			if (mTexAnimMode && mTextureAnimp->mRate == 0)
			{
				U8 start, count;

				if (mTextureAnimp->mFace == -1)
				{
					start = 0;
					count = getNumTEs();
				}
				else
				{
					start = (U8) mTextureAnimp->mFace;
					count = 1;
				}

				for (S32 i = start; i < start + count; i++)
				{
					if (mTexAnimMode & LLViewerTextureAnim::TRANSLATE)
					{
						setTEOffset(i, mTextureAnimp->mOffS, mTextureAnimp->mOffT);				
					}
					if (mTexAnimMode & LLViewerTextureAnim::SCALE)
					{
						setTEScale(i, mTextureAnimp->mScaleS, mTextureAnimp->mScaleT);	
					}
					if (mTexAnimMode & LLViewerTextureAnim::ROTATE)
					{
						setTERotation(i, mTextureAnimp->mRot);
					}
				}

				gPipeline.markTextured(mDrawable);
				mFaceMappingChanged = TRUE;
				mTexAnimMode = 0;
			}
		}
	}
}

void LLVOVolume::updateTextures()
{
	const F32 TEXTURE_AREA_REFRESH_TIME = 5.f; // seconds
	if (mTextureUpdateTimer.getElapsedTimeF32() > TEXTURE_AREA_REFRESH_TIME)
	{
		updateTextureVirtualSize();

		if (mDrawable.notNull() && !isVisible() && !mDrawable->isActive())
		{ //delete vertex buffer to free up some VRAM
			LLSpatialGroup* group  = mDrawable->getSpatialGroup();
			if (group)
			{
				group->destroyGL(true);

				//flag the group as having changed geometry so it gets a rebuild next time
				//it becomes visible
				group->setState(LLSpatialGroup::GEOM_DIRTY | LLSpatialGroup::MESH_DIRTY | LLSpatialGroup::NEW_DRAWINFO);
			}
		}


    }
}

BOOL LLVOVolume::isVisible() const 
{
	if(mDrawable.notNull() && mDrawable->isVisible())
	{
		return TRUE ;
	}

	if(isAttachment())
	{
		LLViewerObject* objp = (LLViewerObject*)getParent() ;
		while(objp && !objp->isAvatar())
		{
			objp = (LLViewerObject*)objp->getParent() ;
		}

		return objp && objp->mDrawable.notNull() && objp->mDrawable->isVisible() ;
	}

	return FALSE ;
}

void LLVOVolume::updateTextureVirtualSize(bool forced)
{
	LL_RECORD_BLOCK_TIME(FTM_VOLUME_TEXTURES);
	// Update the pixel area of all faces

	if(!forced)
	{
		if(!isVisible())
		{ //don't load textures for non-visible faces
			const S32 num_faces = mDrawable->getNumFaces();
			for (S32 i = 0; i < num_faces; i++)
			{
				LLFace* face = mDrawable->getFace(i);
				if (face)
				{
					face->setPixelArea(0.f); 
					face->setVirtualSize(0.f);
				}
			}

			return ;
		}

		if (!gPipeline.hasRenderType(LLPipeline::RENDER_TYPE_SIMPLE))
		{
			return;
		}
	}

	static LLCachedControl<bool> dont_load_textures(gSavedSettings,"TextureDisable", false);
		
	if (dont_load_textures || LLAppViewer::getTextureFetch()->mDebugPause) // || !mDrawable->isVisible())
	{
		return;
	}

	mTextureUpdateTimer.reset();
	
	F32 old_area = mPixelArea;
	mPixelArea = 0.f;

	const S32 num_faces = mDrawable->getNumFaces();
	F32 min_vsize=999999999.f, max_vsize=0.f;
	LLViewerCamera* camera = LLViewerCamera::getInstance();
	for (S32 i = 0; i < num_faces; i++)
	{
		LLFace* face = mDrawable->getFace(i);
		if (!face) continue;
		const LLTextureEntry *te = face->getTextureEntry();
		LLViewerTexture *imagep = face->getTexture();
		if (!imagep || !te ||			
			face->mExtents[0].equals3(face->mExtents[1]))
		{
			continue;
		}
		
		F32 vsize;
		F32 old_size = face->getVirtualSize();

		if (isHUDAttachment())
		{
			F32 area = (F32) camera->getScreenPixelArea();
			vsize = area;
			imagep->setBoostLevel(LLGLTexture::BOOST_HUD);
 			face->setPixelArea(area); // treat as full screen
			face->setVirtualSize(vsize);
		}
		else
		{
			vsize = face->getTextureVirtualSize();
		}

		mPixelArea = llmax(mPixelArea, face->getPixelArea());		

		if (face->mTextureMatrix != NULL)
		{
			if ((vsize < MIN_TEX_ANIM_SIZE && old_size > MIN_TEX_ANIM_SIZE) ||
				(vsize > MIN_TEX_ANIM_SIZE && old_size < MIN_TEX_ANIM_SIZE))
			{
				gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_TCOORD, FALSE);
			}
		}
				
		if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXTURE_AREA))
		{
			if (vsize < min_vsize) min_vsize = vsize;
			if (vsize > max_vsize) max_vsize = vsize;
		}
		else if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXTURE_PRIORITY))
		{
			LLViewerFetchedTexture* img = LLViewerTextureManager::staticCastToFetchedTexture(imagep) ;
			if(img)
			{
				F32 pri = img->getDecodePriority();
				pri = llmax(pri, 0.0f);
				if (pri < min_vsize) min_vsize = pri;
				if (pri > max_vsize) max_vsize = pri;
			}
		}
		else if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_FACE_AREA))
		{
			F32 pri = mPixelArea;
			if (pri < min_vsize) min_vsize = pri;
			if (pri > max_vsize) max_vsize = pri;
		}	
	}
	
	if (isSculpted())
	{
		LLSculptParams *sculpt_params = (LLSculptParams *)getParameterEntry(LLNetworkData::PARAMS_SCULPT);
		LLUUID id =  sculpt_params->getSculptTexture();
		
		updateSculptTexture();
		
		

		if (mSculptTexture.notNull())
		{
			mSculptTexture->setBoostLevel(llmax((S32)mSculptTexture->getBoostLevel(),
												(S32)LLGLTexture::BOOST_SCULPTED));
			mSculptTexture->setForSculpt() ;
			
			if(!mSculptTexture->isCachedRawImageReady())
			{
				S32 lod = llmin(mLOD, 3);
				F32 lodf = ((F32)(lod + 1.0f)/4.f);
				F32 tex_size = lodf * LLViewerTexture::sMaxSculptRez ;
				mSculptTexture->addTextureStats(2.f * tex_size * tex_size, FALSE);
			
				//if the sculpty very close to the view point, load first
				{				
					LLVector3 lookAt = getPositionAgent() - camera->getOrigin();
					F32 dist = lookAt.normVec() ;
					F32 cos_angle_to_view_dir = lookAt * camera->getXAxis() ;				
					mSculptTexture->setAdditionalDecodePriority(0.8f * LLFace::calcImportanceToCamera(cos_angle_to_view_dir, dist)) ;
				}
			}
	
			S32 texture_discard = mSculptTexture->getCachedRawImageLevel(); //try to match the texture
			S32 current_discard = getVolume() ? getVolume()->getSculptLevel() : -2 ;

			if (texture_discard >= 0 && //texture has some data available
				(texture_discard < current_discard || //texture has more data than last rebuild
				current_discard < 0)) //no previous rebuild
			{
				gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_VOLUME, FALSE);
				mSculptChanged = TRUE;
			}

			if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_SCULPTED))
			{
				setDebugText(llformat("T%d C%d V%d\n%dx%d",
										  texture_discard, current_discard, getVolume()->getSculptLevel(),
										  mSculptTexture->getHeight(), mSculptTexture->getWidth()));
			}
		}

	}

	if (getLightTextureID().notNull())
	{
		LLLightImageParams* params = (LLLightImageParams*) getParameterEntry(LLNetworkData::PARAMS_LIGHT_IMAGE);
		LLUUID id = params->getLightTexture();
		mLightTexture = LLViewerTextureManager::getFetchedTexture(id);
		if (mLightTexture.notNull())
		{
			F32 rad = getLightRadius();
			mLightTexture->addTextureStats(gPipeline.calcPixelArea(getPositionAgent(), 
																	LLVector3(rad,rad,rad),
																	*camera));
		}	
	}
	
	if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXTURE_AREA))
	{
		setDebugText(llformat("%.0f:%.0f", (F32) sqrt(min_vsize),(F32) sqrt(max_vsize)));
	}
 	else if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TEXTURE_PRIORITY))
 	{
 		setDebugText(llformat("%.0f:%.0f", (F32) sqrt(min_vsize),(F32) sqrt(max_vsize)));
 	}
	else if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_FACE_AREA))
	{
		setDebugText(llformat("%.0f:%.0f", (F32) sqrt(min_vsize),(F32) sqrt(max_vsize)));
	}

	if (mPixelArea == 0)
	{ //flexi phasing issues make this happen
		mPixelArea = old_area;
	}
}

BOOL LLVOVolume::isActive() const
{
	return !mStatic;
}

BOOL LLVOVolume::setMaterial(const U8 material)
{
	BOOL res = LLViewerObject::setMaterial(material);
	
	return res;
}

void LLVOVolume::setTexture(const S32 face)
{
	llassert(face < getNumTEs());
	gGL.getTexUnit(0)->bind(getTEImage(face));
}

void LLVOVolume::setScale(const LLVector3 &scale, BOOL damped)
{
	if (scale != getScale())
	{
		// store local radius
		LLViewerObject::setScale(scale);

		if (mVolumeImpl)
		{
			mVolumeImpl->onSetScale(scale, damped);
		}
		
		updateRadius();

		//since drawable transforms do not include scale, changing volume scale
		//requires an immediate rebuild of volume verts.
		gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_POSITION, TRUE);
	}
}

LLFace* LLVOVolume::addFace(S32 f)
{
	const LLTextureEntry* te = getTE(f);
	LLViewerTexture* imagep = getTEImage(f);
	if (te->getMaterialParams().notNull())
	{
		LLViewerTexture* normalp = getTENormalMap(f);
		LLViewerTexture* specularp = getTESpecularMap(f);
		return mDrawable->addFace(te, imagep, normalp, specularp);
	}
	return mDrawable->addFace(te, imagep);
}

LLDrawable *LLVOVolume::createDrawable(LLPipeline *pipeline)
{
	pipeline->allocDrawable(this);
		
	mDrawable->setRenderType(LLPipeline::RENDER_TYPE_VOLUME);

	S32 max_tes_to_set = getNumTEs();
	for (S32 i = 0; i < max_tes_to_set; i++)
	{
		addFace(i);
	}
	mNumFaces = max_tes_to_set;

	if (isAttachment())
	{
		mDrawable->makeActive();
	}

	if (getIsLight())
	{
		// Add it to the pipeline mLightSet
		gPipeline.setLight(mDrawable, TRUE);
	}
	
	updateRadius();
	bool force_update = true; // avoid non-alpha mDistance update being optimized away
	mDrawable->updateDistance(*LLViewerCamera::getInstance(), force_update);

	return mDrawable;
}

BOOL LLVOVolume::setVolume(const LLVolumeParams &params_in, const S32 detail, bool unique_volume)
{
	LLVolumeParams volume_params = params_in;

	S32 last_lod = mVolumep.notNull() ? LLVolumeLODGroup::getVolumeDetailFromScale(mVolumep->getDetail()) : -1;
	S32 lod = mLOD;

	BOOL is404 = FALSE;
	
	if (isSculpted())
	{
		// if it's a mesh
		if ((volume_params.getSculptType() & LL_SCULPT_TYPE_MASK) == LL_SCULPT_TYPE_MESH)
		{ //meshes might not have all LODs, get the force detail to best existing LOD
			LLUUID mesh_id = volume_params.getSculptID();

			lod = gMeshRepo.getActualMeshLOD(volume_params, lod);
			if (lod == -1)
			{
				is404 = TRUE;
				lod = 0;
			}
		}
	}

	// Check if we need to change implementations
	bool is_flexible = (volume_params.getPathParams().getCurveType() == LL_PCODE_PATH_FLEXIBLE);
	if (is_flexible)
	{
		setParameterEntryInUse(LLNetworkData::PARAMS_FLEXIBLE, TRUE, false);
		if (!mVolumeImpl)
		{
			LLFlexibleObjectData* data = (LLFlexibleObjectData*)getParameterEntry(LLNetworkData::PARAMS_FLEXIBLE);
			mVolumeImpl = new LLVolumeImplFlexible(this, data);
		}
	}
	else
	{
		// Mark the parameter not in use
		setParameterEntryInUse(LLNetworkData::PARAMS_FLEXIBLE, FALSE, false);
		if (mVolumeImpl)
		{
			delete mVolumeImpl;
			mVolumeImpl = NULL;
			if (mDrawable.notNull())
			{
				// Undo the damage we did to this matrix
				mDrawable->updateXform(FALSE);
			}
		}
	}
	
	if (is404)
	{
		setIcon(LLViewerTextureManager::getFetchedTextureFromFile("icons/Inv_Mesh.png", FTT_LOCAL_FILE, TRUE, LLGLTexture::BOOST_UI));
		//render prim proxy when mesh loading attempts give up
		volume_params.setSculptID(LLUUID::null, LL_SCULPT_TYPE_NONE);

	}

	if ((LLPrimitive::setVolume(volume_params, lod, (mVolumeImpl && mVolumeImpl->isVolumeUnique()))) || mSculptChanged)
	{
		mFaceMappingChanged = TRUE;
		
		if (mVolumeImpl)
		{
			mVolumeImpl->onSetVolume(volume_params, mLOD);
		}
	
		updateSculptTexture();

		if (isSculpted())
		{
			updateSculptTexture();
			// if it's a mesh
			if ((volume_params.getSculptType() & LL_SCULPT_TYPE_MASK) == LL_SCULPT_TYPE_MESH)
			{
				if (!getVolume()->isMeshAssetLoaded())
				{ 
					//load request not yet issued, request pipeline load this mesh
					LLUUID asset_id = volume_params.getSculptID();
					S32 available_lod = gMeshRepo.loadMesh(this, volume_params, lod, last_lod);
					if (available_lod != lod)
					{
						LLPrimitive::setVolume(volume_params, available_lod);
					}
				}
				
			}
			else // otherwise is sculptie
			{
				if (mSculptTexture.notNull())
				{
					sculpt();
				}
			}
		}

		static LLCachedControl<bool> use_transform_feedback(gSavedSettings, "RenderUseTransformFeedback", false);

		bool cache_in_vram = use_transform_feedback && gTransformPositionProgram.mProgramObject &&
			(!mVolumeImpl || !mVolumeImpl->isVolumeUnique());

		if (cache_in_vram)
		{ //this volume might be used as source data for a transform object, put it in vram
			LLVolume* volume = getVolume();
			for (S32 i = 0; i < volume->getNumFaces(); ++i)
			{
				const LLVolumeFace& face = volume->getVolumeFace(i);
				if (face.mVertexBuffer.notNull())
				{ //already cached
					break;
				}
				volume->genTangents(i);
				LLFace::cacheFaceInVRAM(face);
			}
		}
		

		return TRUE;
	}



	return FALSE;
}

void LLVOVolume::updateSculptTexture()
{
	LLPointer<LLViewerFetchedTexture> old_sculpt = mSculptTexture;

	if (isSculpted() && !isMesh())
	{
		LLSculptParams *sculpt_params = (LLSculptParams *)getParameterEntry(LLNetworkData::PARAMS_SCULPT);
		LLUUID id =  sculpt_params->getSculptTexture();
		if (id.notNull())
		{
			mSculptTexture = LLViewerTextureManager::getFetchedTexture(id, FTT_DEFAULT, TRUE, LLGLTexture::BOOST_NONE, LLViewerTexture::LOD_TEXTURE);
		}
	}
	else
	{
		mSculptTexture = NULL;
	}

	if (mSculptTexture != old_sculpt)
	{
		if (old_sculpt.notNull())
		{
			old_sculpt->removeVolume(this);
		}
		if (mSculptTexture.notNull())
		{
			mSculptTexture->addVolume(this);
		}
	}
	
}

void LLVOVolume::updateVisualComplexity()
{
    LLVOAvatar* avatar = getAvatarAncestor();
    if (avatar)
    {
        avatar->updateVisualComplexity();
    }
    LLVOAvatar* rigged_avatar = getAvatar();
    if(rigged_avatar && (rigged_avatar != avatar))
    {
        rigged_avatar->updateVisualComplexity();
    }
}

void LLVOVolume::notifyMeshLoaded()
{ 
	mSculptChanged = TRUE;
	gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_GEOMETRY, TRUE);

    updateVisualComplexity();
}

// sculpt replaces generate() for sculpted surfaces
void LLVOVolume::sculpt()
{	
	if (mSculptTexture.notNull())
	{				
		U16 sculpt_height = 0;
		U16 sculpt_width = 0;
		S8 sculpt_components = 0;
		const U8* sculpt_data = NULL;
	
		S32 discard_level = mSculptTexture->getCachedRawImageLevel() ;
		LLImageRaw* raw_image = mSculptTexture->getCachedRawImage() ;
		
		S32 max_discard = mSculptTexture->getMaxDiscardLevel();
		if (discard_level > max_discard)
		{
			discard_level = max_discard;    // clamp to the best we can do			
		}
		if(discard_level > MAX_DISCARD_LEVEL)
		{
			return; //we think data is not ready yet.
		}

		S32 current_discard = getVolume()->getSculptLevel() ;
		if(current_discard < -2)
		{
			static S32 low_sculpty_discard_warning_count = 1;
			S32 exponent = llmax(1, llfloor( log10((F64) low_sculpty_discard_warning_count) ));
			S32 interval = pow(10.0, exponent);
			if ( low_sculpty_discard_warning_count < 10 ||
				(low_sculpty_discard_warning_count % interval) == 0)
			{	// Log first 10 time, then decreasing intervals afterwards otherwise this can flood the logs
				LL_WARNS() << "WARNING!!: Current discard for sculpty " << mSculptTexture->getID() 
					<< " at " << current_discard 
					<< " is less than -2." 
					<< " Hit this " << low_sculpty_discard_warning_count << " times"
					<< LL_ENDL;
			}
			low_sculpty_discard_warning_count++;
			
			// corrupted volume... don't update the sculpty
			return;
		}
		else if (current_discard > MAX_DISCARD_LEVEL)
		{
			static S32 high_sculpty_discard_warning_count = 1;
			S32 exponent = llmax(1, llfloor( log10((F64) high_sculpty_discard_warning_count) ));
			S32 interval = pow(10.0, exponent);
			if ( high_sculpty_discard_warning_count < 10 ||
				(high_sculpty_discard_warning_count % interval) == 0)
			{	// Log first 10 time, then decreasing intervals afterwards otherwise this can flood the logs
				LL_WARNS() << "WARNING!!: Current discard for sculpty " << mSculptTexture->getID() 
					<< " at " << current_discard 
					<< " is more than than allowed max of " << MAX_DISCARD_LEVEL
					<< ".  Hit this " << high_sculpty_discard_warning_count << " times"
					<< LL_ENDL;
			}
			high_sculpty_discard_warning_count++;

			// corrupted volume... don't update the sculpty			
			return;
		}

		if (current_discard == discard_level)  // no work to do here
			return;
		
		if(!raw_image)
		{
			llassert(discard_level < 0) ;

			sculpt_width = 0;
			sculpt_height = 0;
			sculpt_data = NULL ;

			if(LLViewerTextureManager::sTesterp)
			{
				LLViewerTextureManager::sTesterp->updateGrayTextureBinding();
			}
		}
		else
		{					
			sculpt_height = raw_image->getHeight();
			sculpt_width = raw_image->getWidth();
			sculpt_components = raw_image->getComponents();		
					   
			sculpt_data = raw_image->getData();

			if(LLViewerTextureManager::sTesterp)
			{
				mSculptTexture->updateBindStatsForTester() ;
			}
		}
		getVolume()->sculpt(sculpt_width, sculpt_height, sculpt_components, sculpt_data, discard_level);

		//notify rebuild any other VOVolumes that reference this sculpty volume
		for (S32 i = 0; i < mSculptTexture->getNumVolumes(); ++i)
		{
			LLVOVolume* volume = (*(mSculptTexture->getVolumeList()))[i];
			if (volume != this && volume->getVolume() == getVolume())
			{
				gPipeline.markRebuild(volume->mDrawable, LLDrawable::REBUILD_GEOMETRY, FALSE);
			}
		}
	}
}

S32	LLVOVolume::computeLODDetail(F32 distance, F32 radius)
{
	S32	cur_detail;
    // AXON TEMP
    if (LLVOVolume::sForceLOD>=0 && LLVOVolume::sForceLOD<=3)
    {
        cur_detail = LLVOVolume::sForceLOD;
    }
    else
    {
        if (LLPipeline::sDynamicLOD)
        {
            // We've got LOD in the profile, and in the twist.  Use radius.
            F32 tan_angle = (LLVOVolume::sLODFactor*radius)/distance;
            cur_detail = LLVolumeLODGroup::getDetailFromTan(ll_round(tan_angle, 0.01f));
        }
        else
        {
            cur_detail = llclamp((S32) (sqrtf(radius)*LLVOVolume::sLODFactor*4.f), 0, 3);		
        }
    }
	return cur_detail;
}

BOOL LLVOVolume::calcLOD()
{
	if (mDrawable.isNull())
	{
		return FALSE;
	}

	S32 cur_detail = 0;
	
	F32 radius;
	F32 distance;

	if (mDrawable->isState(LLDrawable::RIGGED))
	{
		LLVOAvatar* avatar = getAvatar(); 
		
		// Not sure how this can really happen, but alas it does. Better exit here than crashing.
		if( !avatar || !avatar->mDrawable )
		{
			return FALSE;
		}

		distance = avatar->mDrawable->mDistanceWRTCamera;
		radius = avatar->getBinRadius();
	}
	else
	{
		distance = mDrawable->mDistanceWRTCamera;
		radius = getVolume() ? getVolume()->mLODScaleBias.scaledVec(getScale()).length() : getScale().length();
	}
	
	//hold onto unmodified distance for debugging
	//F32 debug_distance = distance;
	
	distance *= sDistanceFactor;

	F32 rampDist = LLVOVolume::sLODFactor * 2;
	
	if (distance < rampDist)
	{
		// Boost LOD when you're REALLY close
		distance *= 1.0f/rampDist;
		distance *= distance;
		distance *= rampDist;
	}
	
	// DON'T Compensate for field of view changing on FOV zoom.
	distance *= F_PI/3.f;

	cur_detail = computeLODDetail(ll_round(distance, 0.01f), 
                                  ll_round(radius, 0.01f));

    if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_TRIANGLE_COUNT) && mDrawable->getFace(0))
    {
        if (isRootEdit() && getChildren().size()>0)
        {
            S32 total_tris = recursiveGetTriangleCount();
            setDebugText(llformat("TRIS %d TOTAL %d", getTriangleCount(), total_tris));
        }
        else
        {
            setDebugText(llformat("TRIS %d", getTriangleCount()));
        }
	
    }
	if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_LOD_INFO) &&
		mDrawable->getFace(0))
	{
        // This is a debug display for LODs. Please don't put the texture index here.
        setDebugText(llformat("%d", cur_detail));
	}

	if (cur_detail != mLOD)
	{
		mAppAngle = ll_round((F32) atan2( mDrawable->getRadius(), mDrawable->mDistanceWRTCamera) * RAD_TO_DEG, 0.01f);
		mLOD = cur_detail;		

        return TRUE;
	}

	return FALSE;
}

BOOL LLVOVolume::updateLOD()
{
	if (mDrawable.isNull())
	{
		return FALSE;
	}
	
	BOOL lod_changed = calcLOD();

	if (lod_changed)
	{
#if 1
        // AXON debugging
        if (isAnimatedObject() && isRiggedMesh())
        {
            std::string vobj_name = llformat("Vol%u", (U32) this);
            F32 est_tris = getEstTrianglesMax();
            LL_DEBUGS("AXONLinkset") << vobj_name << " updateLOD to " << getLOD() << ", tris " << est_tris << LL_ENDL; 
        }
#endif
		gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_VOLUME, FALSE);
		mLODChanged = TRUE;
	}
	else
	{
		F32 new_radius = getBinRadius();
		F32 old_radius = mDrawable->getBinRadius();
		if (new_radius < old_radius * 0.9f || new_radius > old_radius*1.1f)
		{
			gPipeline.markPartitionMove(mDrawable);
		}
	}

	lod_changed = lod_changed || LLViewerObject::updateLOD();
	
	return lod_changed;
}

BOOL LLVOVolume::setDrawableParent(LLDrawable* parentp)
{
	if (!LLViewerObject::setDrawableParent(parentp))
	{
		// no change in drawable parent
		return FALSE;
	}

	if (!mDrawable->isRoot())
	{
		// rebuild vertices in parent relative space
		gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_VOLUME, TRUE);

		if (mDrawable->isActive() && !parentp->isActive())
		{
			parentp->makeActive();
		}
		else if (mDrawable->isStatic() && parentp->isActive())
		{
			mDrawable->makeActive();
		}
	}
	
	return TRUE;
}

void LLVOVolume::updateFaceFlags()
{
	// There's no guarantee that getVolume()->getNumFaces() == mDrawable->getNumFaces()
	for (S32 i = 0; i < getVolume()->getNumFaces() && i < mDrawable->getNumFaces(); i++)
	{
		LLFace *face = mDrawable->getFace(i);
		if (face)
		{
			BOOL fullbright = getTE(i)->getFullbright();
			face->clearState(LLFace::FULLBRIGHT | LLFace::HUD_RENDER | LLFace::LIGHT);

			if (fullbright || (mMaterial == LL_MCODE_LIGHT))
			{
				face->setState(LLFace::FULLBRIGHT);
			}
			if (mDrawable->isLight())
			{
				face->setState(LLFace::LIGHT);
			}
			if (isHUDAttachment())
			{
				face->setState(LLFace::HUD_RENDER);
			}
		}
	}
}

BOOL LLVOVolume::setParent(LLViewerObject* parent)
{
	BOOL ret = FALSE ;
    LLViewerObject *old_parent = (LLViewerObject*) getParent();
	if (parent != old_parent)
	{
		ret = LLViewerObject::setParent(parent);
		if (ret && mDrawable)
		{
			gPipeline.markMoved(mDrawable);
			gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_VOLUME, TRUE);
		}
        updateAnimatedObjectStateOnReparent(old_parent, parent);
	}

	return ret ;
}

// NOTE: regenFaces() MUST be followed by genTriangles()!
void LLVOVolume::regenFaces()
{
	// remove existing faces
	BOOL count_changed = mNumFaces != getNumTEs();
	
	if (count_changed)
	{
		deleteFaces();		
		// add new faces
		mNumFaces = getNumTEs();
	}
		
	for (S32 i = 0; i < mNumFaces; i++)
	{
		LLFace* facep = count_changed ? addFace(i) : mDrawable->getFace(i);
		if (!facep) continue;

		facep->setTEOffset(i);
		facep->setTexture(getTEImage(i));
		if (facep->getTextureEntry()->getMaterialParams().notNull())
		{
			facep->setNormalMap(getTENormalMap(i));
			facep->setSpecularMap(getTESpecularMap(i));
		}
		facep->setViewerObject(this);
		
		// If the face had media on it, this will have broken the link between the LLViewerMediaTexture and the face.
		// Re-establish the link.
		if((int)mMediaImplList.size() > i)
		{
			if(mMediaImplList[i])
			{
				LLViewerMediaTexture* media_tex = LLViewerTextureManager::findMediaTexture(mMediaImplList[i]->getMediaTextureID()) ;
				if(media_tex)
				{
					media_tex->addMediaToFace(facep) ;
				}
			}
		}
	}
	
	if (!count_changed)
	{
		updateFaceFlags();
	}
}

BOOL LLVOVolume::genBBoxes(BOOL force_global)
{
	BOOL res = TRUE;

	LLVector4a min,max;

	min.clear();
	max.clear();

	BOOL rebuild = mDrawable->isState(LLDrawable::REBUILD_VOLUME | LLDrawable::REBUILD_POSITION | LLDrawable::REBUILD_RIGGED);

	//	bool rigged = false;
	LLVolume* volume = mRiggedVolume;
	if (!volume)
	{
		volume = getVolume();
	}

	// There's no guarantee that getVolume()->getNumFaces() == mDrawable->getNumFaces()
	for (S32 i = 0;
		 i < getVolume()->getNumVolumeFaces() && i < mDrawable->getNumFaces() && i < getNumTEs();
		 i++)
	{
		LLFace *face = mDrawable->getFace(i);
		if (!face)
		{
			continue;
		}
		res &= face->genVolumeBBoxes(*volume, i,
										mRelativeXform, 
										(mVolumeImpl && mVolumeImpl->isVolumeGlobal()) || force_global);
		
		if (rebuild)
		{
			if (i == 0)
			{
				min = face->mExtents[0];
				max = face->mExtents[1];
			}
			else
			{
				min.setMin(min, face->mExtents[0]);
				max.setMax(max, face->mExtents[1]);
			}
		}
	}
	
	if (rebuild)
	{
		mDrawable->setSpatialExtents(min,max);
		min.add(max);
		min.mul(0.5f);
		mDrawable->setPositionGroup(min);	
	}

	updateRadius();
	mDrawable->movePartition();
				
	return res;
}

void LLVOVolume::preRebuild()
{
	if (mVolumeImpl != NULL)
	{
		mVolumeImpl->preRebuild();
	}
}

void LLVOVolume::updateRelativeXform(bool force_identity)
{
	if (mVolumeImpl)
	{
		mVolumeImpl->updateRelativeXform(force_identity);
		return;
	}
	
	LLDrawable* drawable = mDrawable;
	
	if (drawable->isState(LLDrawable::RIGGED) && mRiggedVolume.notNull())
	{ //rigged volume (which is in agent space) is used for generating bounding boxes etc
	  //inverse of render matrix should go to partition space
		mRelativeXform = getRenderMatrix();

		F32* dst = (F32*) mRelativeXformInvTrans.mMatrix;
		F32* src = (F32*) mRelativeXform.mMatrix;
		dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
		dst[3] = src[4]; dst[4] = src[5]; dst[5] = src[6];
		dst[6] = src[8]; dst[7] = src[9]; dst[8] = src[10];
		
		mRelativeXform.invert();
		mRelativeXformInvTrans.transpose();
	}
	else if (drawable->isActive() || force_identity)
	{				
		// setup relative transforms
		LLQuaternion delta_rot;
		LLVector3 delta_pos, delta_scale;
		
		//matrix from local space to parent relative/global space
		bool use_identity = force_identity || drawable->isSpatialRoot();
		delta_rot = use_identity ? LLQuaternion() : mDrawable->getRotation();
		delta_pos = use_identity ? LLVector3(0,0,0) : mDrawable->getPosition();
		delta_scale = mDrawable->getScale();

		// Vertex transform (4x4)
		LLVector3 x_axis = LLVector3(delta_scale.mV[VX], 0.f, 0.f) * delta_rot;
		LLVector3 y_axis = LLVector3(0.f, delta_scale.mV[VY], 0.f) * delta_rot;
		LLVector3 z_axis = LLVector3(0.f, 0.f, delta_scale.mV[VZ]) * delta_rot;

		mRelativeXform.initRows(LLVector4(x_axis, 0.f),
								LLVector4(y_axis, 0.f),
								LLVector4(z_axis, 0.f),
								LLVector4(delta_pos, 1.f));

		
		// compute inverse transpose for normals
		// mRelativeXformInvTrans.setRows(x_axis, y_axis, z_axis);
		// mRelativeXformInvTrans.invert(); 
		// mRelativeXformInvTrans.setRows(x_axis, y_axis, z_axis);
		// grumble - invert is NOT a matrix invert, so we do it by hand:

		LLMatrix3 rot_inverse = LLMatrix3(~delta_rot);

		LLMatrix3 scale_inverse;
		scale_inverse.setRows(LLVector3(1.0, 0.0, 0.0) / delta_scale.mV[VX],
							  LLVector3(0.0, 1.0, 0.0) / delta_scale.mV[VY],
							  LLVector3(0.0, 0.0, 1.0) / delta_scale.mV[VZ]);
							   
		
		mRelativeXformInvTrans = rot_inverse * scale_inverse;

		mRelativeXformInvTrans.transpose();
	}
	else
	{
		LLVector3 pos = getPosition();
		LLVector3 scale = getScale();
		LLQuaternion rot = getRotation();
	
		if (mParent)
		{
			pos *= mParent->getRotation();
			pos += mParent->getPosition();
			rot *= mParent->getRotation();
		}
		
		//LLViewerRegion* region = getRegion();
		//pos += region->getOriginAgent();
		
		LLVector3 x_axis = LLVector3(scale.mV[VX], 0.f, 0.f) * rot;
		LLVector3 y_axis = LLVector3(0.f, scale.mV[VY], 0.f) * rot;
		LLVector3 z_axis = LLVector3(0.f, 0.f, scale.mV[VZ]) * rot;

		mRelativeXform.initRows(LLVector4(x_axis, 0.f),
								LLVector4(y_axis, 0.f),
								LLVector4(z_axis, 0.f),
								LLVector4(pos, 1.f));

		// compute inverse transpose for normals
		LLMatrix3 rot_inverse = LLMatrix3(~rot);

		LLMatrix3 scale_inverse;
		scale_inverse.setRows(LLVector3(1.0, 0.0, 0.0) / scale.mV[VX],
							  LLVector3(0.0, 1.0, 0.0) / scale.mV[VY],
							  LLVector3(0.0, 0.0, 1.0) / scale.mV[VZ]);
							   
		
		mRelativeXformInvTrans = rot_inverse * scale_inverse;

		mRelativeXformInvTrans.transpose();
	}
}

static LLTrace::BlockTimerStatHandle FTM_GEN_FLEX("Generate Flexies");
static LLTrace::BlockTimerStatHandle FTM_UPDATE_PRIMITIVES("Update Primitives");
static LLTrace::BlockTimerStatHandle FTM_UPDATE_RIGGED_VOLUME("Update Rigged");

bool LLVOVolume::lodOrSculptChanged(LLDrawable *drawable, BOOL &compiled)
{
	bool regen_faces = false;

	LLVolume *old_volumep, *new_volumep;
	F32 old_lod, new_lod;
	S32 old_num_faces, new_num_faces;

	old_volumep = getVolume();
	old_lod = old_volumep->getDetail();
	old_num_faces = old_volumep->getNumFaces();
	old_volumep = NULL;

	{
		LL_RECORD_BLOCK_TIME(FTM_GEN_VOLUME);
		const LLVolumeParams &volume_params = getVolume()->getParams();
		setVolume(volume_params, 0);
	}

	new_volumep = getVolume();
	new_lod = new_volumep->getDetail();
	new_num_faces = new_volumep->getNumFaces();
	new_volumep = NULL;

	if ((new_lod != old_lod) || mSculptChanged)
	{
        if (mDrawable->isState(LLDrawable::RIGGED))
        {
            updateVisualComplexity();
        }

		compiled = TRUE;
		sNumLODChanges += new_num_faces;

		if ((S32)getNumTEs() != getVolume()->getNumFaces())
		{
			setNumTEs(getVolume()->getNumFaces()); //mesh loading may change number of faces.
		}

		drawable->setState(LLDrawable::REBUILD_VOLUME); // for face->genVolumeTriangles()

		{
			LL_RECORD_BLOCK_TIME(FTM_GEN_TRIANGLES);
			regen_faces = new_num_faces != old_num_faces || mNumFaces != (S32)getNumTEs();
			if (regen_faces)
			{
				regenFaces();
			}

			if (mSculptChanged)
			{ //changes in sculpt maps can thrash an object bounding box without 
				//triggering a spatial group bounding box update -- force spatial group
				//to update bounding boxes
				LLSpatialGroup* group = mDrawable->getSpatialGroup();
				if (group)
				{
					group->unbound();
				}
			}
		}
	}

	return regen_faces;
}

BOOL LLVOVolume::updateGeometry(LLDrawable *drawable)
{
	LL_RECORD_BLOCK_TIME(FTM_UPDATE_PRIMITIVES);
	
	if (mDrawable->isState(LLDrawable::REBUILD_RIGGED))
	{
		{
			LL_RECORD_BLOCK_TIME(FTM_UPDATE_RIGGED_VOLUME);
			updateRiggedVolume();
		}
		genBBoxes(FALSE);
		mDrawable->clearState(LLDrawable::REBUILD_RIGGED);
	}

	if (mVolumeImpl != NULL)
	{
		BOOL res;
		{
			LL_RECORD_BLOCK_TIME(FTM_GEN_FLEX);
			res = mVolumeImpl->doUpdateGeometry(drawable);
		}
		updateFaceFlags();
		return res;
	}
	
	LLSpatialGroup* group = drawable->getSpatialGroup();
	if (group)
	{
		group->dirtyMesh();
	}

	BOOL compiled = FALSE;
			
	updateRelativeXform();
	
	if (mDrawable.isNull()) // Not sure why this is happening, but it is...
	{
		return TRUE; // No update to complete
	}

	if (mVolumeChanged || mFaceMappingChanged)
	{
		dirtySpatialGroup(drawable->isState(LLDrawable::IN_REBUILD_Q1));

		bool was_regen_faces = false;

		if (mVolumeChanged)
		{
			was_regen_faces = lodOrSculptChanged(drawable, compiled);
			drawable->setState(LLDrawable::REBUILD_VOLUME);
		}
		else if (mSculptChanged || mLODChanged)
		{
			compiled = TRUE;
			was_regen_faces = lodOrSculptChanged(drawable, compiled);
		}

		if (!was_regen_faces) {
			LL_RECORD_BLOCK_TIME(FTM_GEN_TRIANGLES);
			regenFaces();
		}

		genBBoxes(FALSE);
	}
	else if (mLODChanged || mSculptChanged)
	{
		dirtySpatialGroup(drawable->isState(LLDrawable::IN_REBUILD_Q1));
		compiled = TRUE;
		lodOrSculptChanged(drawable, compiled);
		genBBoxes(FALSE);
	}
	// it has its own drawable (it's moved) or it has changed UVs or it has changed xforms from global<->local
	else
	{
		compiled = TRUE;
		// All it did was move or we changed the texture coordinate offset
		LL_RECORD_BLOCK_TIME(FTM_GEN_TRIANGLES);
		genBBoxes(FALSE);
	}

	// Update face flags
	updateFaceFlags();
	
	if(compiled)
	{
		LLPipeline::sCompiles++;
	}
		
	mVolumeChanged = FALSE;
	mLODChanged = FALSE;
	mSculptChanged = FALSE;
	mFaceMappingChanged = FALSE;
	
	return LLViewerObject::updateGeometry(drawable);
}

void LLVOVolume::updateFaceSize(S32 idx)
{
	if( mDrawable->getNumFaces() <= idx )
	{
		return;
	}

	LLFace* facep = mDrawable->getFace(idx);
	if (facep)
	{
		if (idx >= getVolume()->getNumVolumeFaces())
		{
			facep->setSize(0,0, true);
		}
		else
		{
			const LLVolumeFace& vol_face = getVolume()->getVolumeFace(idx);
			facep->setSize(vol_face.mNumVertices, vol_face.mNumIndices, 
							true); // <--- volume faces should be padded for 16-byte alignment
		
		}
	}
}

BOOL LLVOVolume::isRootEdit() const
{
	if (mParent && !((LLViewerObject*)mParent)->isAvatar())
	{
		return FALSE;
	}
	return TRUE;
}

//virtual
void LLVOVolume::setNumTEs(const U8 num_tes)
{
	const U8 old_num_tes = getNumTEs() ;
	
	if(old_num_tes && old_num_tes < num_tes) //new faces added
	{
		LLViewerObject::setNumTEs(num_tes) ;

		if(mMediaImplList.size() >= old_num_tes && mMediaImplList[old_num_tes -1].notNull())//duplicate the last media textures if exists.
		{
			mMediaImplList.resize(num_tes) ;
			const LLTextureEntry* te = getTE(old_num_tes - 1) ;
			for(U8 i = old_num_tes; i < num_tes ; i++)
			{
				setTE(i, *te) ;
				mMediaImplList[i] = mMediaImplList[old_num_tes -1] ;
			}
			mMediaImplList[old_num_tes -1]->setUpdated(TRUE) ;
		}
	}
	else if(old_num_tes > num_tes && mMediaImplList.size() > num_tes) //old faces removed
	{
		U8 end = mMediaImplList.size() ;
		for(U8 i = num_tes; i < end ; i++)
		{
			removeMediaImpl(i) ;				
		}
		mMediaImplList.resize(num_tes) ;

		LLViewerObject::setNumTEs(num_tes) ;
	}
	else
	{
		LLViewerObject::setNumTEs(num_tes) ;
	}

	return ;
}

//virtual     
void LLVOVolume::changeTEImage(S32 index, LLViewerTexture* imagep)
{
	BOOL changed = (mTEImages[index] != imagep);
	LLViewerObject::changeTEImage(index, imagep);
	if (changed)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
}

void LLVOVolume::setTEImage(const U8 te, LLViewerTexture *imagep)
{
	BOOL changed = (mTEImages[te] != imagep);
	LLViewerObject::setTEImage(te, imagep);
	if (changed)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
}

S32 LLVOVolume::setTETexture(const U8 te, const LLUUID &uuid)
{
	S32 res = LLViewerObject::setTETexture(te, uuid);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return res;
}

S32 LLVOVolume::setTEColor(const U8 te, const LLColor3& color)
{
	return setTEColor(te, LLColor4(color));
}

S32 LLVOVolume::setTEColor(const U8 te, const LLColor4& color)
{
	S32 retval = 0;
	const LLTextureEntry *tep = getTE(te);
	if (!tep)
	{
		LL_WARNS("MaterialTEs") << "No texture entry for te " << (S32)te << ", object " << mID << LL_ENDL;
	}
	else if (color != tep->getColor())
	{
		F32 old_alpha = tep->getColor().mV[3];
		if (color.mV[3] != old_alpha)
		{
			gPipeline.markTextured(mDrawable);
			//treat this alpha change as an LoD update since render batches may need to get rebuilt
			mLODChanged = TRUE;
			gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_VOLUME, FALSE);
		}
		retval = LLPrimitive::setTEColor(te, color);
		if (mDrawable.notNull() && retval)
		{
			// These should only happen on updates which are not the initial update.
			mDrawable->setState(LLDrawable::REBUILD_COLOR);
			dirtyMesh();
		}
	}

	return  retval;
}

S32 LLVOVolume::setTEBumpmap(const U8 te, const U8 bumpmap)
{
	S32 res = LLViewerObject::setTEBumpmap(te, bumpmap);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return  res;
}

S32 LLVOVolume::setTETexGen(const U8 te, const U8 texgen)
{
	S32 res = LLViewerObject::setTETexGen(te, texgen);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return  res;
}

S32 LLVOVolume::setTEMediaTexGen(const U8 te, const U8 media)
{
	S32 res = LLViewerObject::setTEMediaTexGen(te, media);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return  res;
}

S32 LLVOVolume::setTEShiny(const U8 te, const U8 shiny)
{
	S32 res = LLViewerObject::setTEShiny(te, shiny);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return  res;
}

S32 LLVOVolume::setTEFullbright(const U8 te, const U8 fullbright)
{
	S32 res = LLViewerObject::setTEFullbright(te, fullbright);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return  res;
}

S32 LLVOVolume::setTEBumpShinyFullbright(const U8 te, const U8 bump)
{
	S32 res = LLViewerObject::setTEBumpShinyFullbright(te, bump);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return res;
}

S32 LLVOVolume::setTEMediaFlags(const U8 te, const U8 media_flags)
{
	S32 res = LLViewerObject::setTEMediaFlags(te, media_flags);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return  res;
}

S32 LLVOVolume::setTEGlow(const U8 te, const F32 glow)
{
	S32 res = LLViewerObject::setTEGlow(te, glow);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return  res;
}

void LLVOVolume::setTEMaterialParamsCallbackTE(const LLUUID& objectID, const LLMaterialID &pMaterialID, const LLMaterialPtr pMaterialParams, U32 te)
{
	LLVOVolume* pVol = (LLVOVolume*)gObjectList.findObject(objectID);
	if (pVol)
	{
		LL_DEBUGS("MaterialTEs") << "materialid " << pMaterialID.asString() << " to TE " << te << LL_ENDL;
		if (te >= pVol->getNumTEs())
			return;

		LLTextureEntry* texture_entry = pVol->getTE(te);
		if (texture_entry && (texture_entry->getMaterialID() == pMaterialID))
		{
			pVol->setTEMaterialParams(te, pMaterialParams);
		}
	}
}

S32 LLVOVolume::setTEMaterialID(const U8 te, const LLMaterialID& pMaterialID)
{
	S32 res = LLViewerObject::setTEMaterialID(te, pMaterialID);
	LL_DEBUGS("MaterialTEs") << "te "<< (S32)te << " materialid " << pMaterialID.asString() << " res " << res
								<< ( LLSelectMgr::getInstance()->getSelection()->contains(const_cast<LLVOVolume*>(this), te) ? " selected" : " not selected" )
								<< LL_ENDL;
		
	LL_DEBUGS("MaterialTEs") << " " << pMaterialID.asString() << LL_ENDL;
	if (res)
	{
		LLMaterialMgr::instance().getTE(getRegion()->getRegionID(), pMaterialID, te, boost::bind(&LLVOVolume::setTEMaterialParamsCallbackTE, getID(), _1, _2, _3));

		setChanged(ALL_CHANGED);
		if (!mDrawable.isNull())
		{
			gPipeline.markTextured(mDrawable);
			gPipeline.markRebuild(mDrawable,LLDrawable::REBUILD_ALL);
		}
		mFaceMappingChanged = TRUE;
	}
	return res;
}

bool LLVOVolume::notifyAboutCreatingTexture(LLViewerTexture *texture)
{ //Ok, here we have confirmation about texture creation, check our wait-list
  //and make changes, or return false

	std::pair<mmap_UUID_MAP_t::iterator, mmap_UUID_MAP_t::iterator> range = mWaitingTextureInfo.equal_range(texture->getID());

	typedef std::map<U8, LLMaterialPtr> map_te_material;
	map_te_material new_material;

	for(mmap_UUID_MAP_t::iterator range_it = range.first; range_it != range.second; ++range_it)
	{
		LLMaterialPtr cur_material = getTEMaterialParams(range_it->second.te);

		//here we just interesting in DIFFUSE_MAP only!
		if(NULL != cur_material.get() && LLRender::DIFFUSE_MAP == range_it->second.map && GL_RGBA != texture->getPrimaryFormat())
		{ //ok let's check the diffuse mode
			switch(cur_material->getDiffuseAlphaMode())
			{
			case LLMaterial::DIFFUSE_ALPHA_MODE_BLEND:
			case LLMaterial::DIFFUSE_ALPHA_MODE_EMISSIVE:
			case LLMaterial::DIFFUSE_ALPHA_MODE_MASK:
				{ //uups... we have non 32 bit texture with LLMaterial::DIFFUSE_ALPHA_MODE_* => LLMaterial::DIFFUSE_ALPHA_MODE_NONE

					LLMaterialPtr mat = NULL;
					map_te_material::iterator it = new_material.find(range_it->second.te);
					if(new_material.end() == it) {
						mat = new LLMaterial(cur_material->asLLSD());
						new_material.insert(map_te_material::value_type(range_it->second.te, mat));
					} else {
						mat = it->second;
					}

					mat->setDiffuseAlphaMode(LLMaterial::DIFFUSE_ALPHA_MODE_NONE);

				} break;
			} //switch
		} //if
	} //for

	//setup new materials
	for(map_te_material::const_iterator it = new_material.begin(), end = new_material.end(); it != end; ++it)
	{
		LLMaterialMgr::getInstance()->put(getID(), it->first, *it->second);
		LLViewerObject::setTEMaterialParams(it->first, it->second);
	}

	//clear wait-list
	mWaitingTextureInfo.erase(range.first, range.second);

	return 0 != new_material.size();
}

bool LLVOVolume::notifyAboutMissingAsset(LLViewerTexture *texture)
{ //Ok, here if we wait information about texture and it's missing
  //then depending from the texture map (diffuse, normal, or specular)
  //make changes in material and confirm it. If not return false.
	std::pair<mmap_UUID_MAP_t::iterator, mmap_UUID_MAP_t::iterator> range = mWaitingTextureInfo.equal_range(texture->getID());
	if(range.first == range.second) return false;

	typedef std::map<U8, LLMaterialPtr> map_te_material;
	map_te_material new_material;
	
	for(mmap_UUID_MAP_t::iterator range_it = range.first; range_it != range.second; ++range_it)
	{
		LLMaterialPtr cur_material = getTEMaterialParams(range_it->second.te);
		if (cur_material.isNull())
			continue;

		switch(range_it->second.map)
		{
		case LLRender::DIFFUSE_MAP:
			{
				if(LLMaterial::DIFFUSE_ALPHA_MODE_NONE != cur_material->getDiffuseAlphaMode())
				{ //missing texture + !LLMaterial::DIFFUSE_ALPHA_MODE_NONE => LLMaterial::DIFFUSE_ALPHA_MODE_NONE
					LLMaterialPtr mat = NULL;
					map_te_material::iterator it = new_material.find(range_it->second.te);
					if(new_material.end() == it) {
						mat = new LLMaterial(cur_material->asLLSD());
						new_material.insert(map_te_material::value_type(range_it->second.te, mat));
					} else {
						mat = it->second;
					}

					mat->setDiffuseAlphaMode(LLMaterial::DIFFUSE_ALPHA_MODE_NONE);
				}
			} break;
		case LLRender::NORMAL_MAP:
			{ //missing texture => reset material texture id
				LLMaterialPtr mat = NULL;
				map_te_material::iterator it = new_material.find(range_it->second.te);
				if(new_material.end() == it) {
					mat = new LLMaterial(cur_material->asLLSD());
					new_material.insert(map_te_material::value_type(range_it->second.te, mat));
				} else {
					mat = it->second;
				}

				mat->setNormalID(LLUUID::null);
			} break;
		case LLRender::SPECULAR_MAP:
			{ //missing texture => reset material texture id
				LLMaterialPtr mat = NULL;
				map_te_material::iterator it = new_material.find(range_it->second.te);
				if(new_material.end() == it) {
					mat = new LLMaterial(cur_material->asLLSD());
					new_material.insert(map_te_material::value_type(range_it->second.te, mat));
				} else {
					mat = it->second;
				}

				mat->setSpecularID(LLUUID::null);
			} break;
		case LLRender::NUM_TEXTURE_CHANNELS:
				//nothing to do, make compiler happy
			break;
		} //switch
	} //for

	//setup new materials
	for(map_te_material::const_iterator it = new_material.begin(), end = new_material.end(); it != end; ++it)
	{
		LLMaterialMgr::getInstance()->put(getID(), it->first, *it->second);
		LLViewerObject::setTEMaterialParams(it->first, it->second);
	}

	//clear wait-list
	mWaitingTextureInfo.erase(range.first, range.second);

	return 0 != new_material.size();
}

S32 LLVOVolume::setTEMaterialParams(const U8 te, const LLMaterialPtr pMaterialParams)
{
	LLMaterialPtr pMaterial = const_cast<LLMaterialPtr&>(pMaterialParams);

	if(pMaterialParams)
	{ //check all of them according to material settings

		LLViewerTexture *img_diffuse = getTEImage(te);
		LLViewerTexture *img_normal = getTENormalMap(te);
		LLViewerTexture *img_specular = getTESpecularMap(te);

		llassert(NULL != img_diffuse);

		LLMaterialPtr new_material = NULL;

		//diffuse
		if(NULL != img_diffuse)
		{ //guard
			if(0 == img_diffuse->getPrimaryFormat() && !img_diffuse->isMissingAsset())
			{ //ok here we don't have information about texture, let's belief and leave material settings
			  //but we remember this case
				mWaitingTextureInfo.insert(mmap_UUID_MAP_t::value_type(img_diffuse->getID(), material_info(LLRender::DIFFUSE_MAP, te)));
			}
			else
			{
				bool bSetDiffuseNone = false;
				if(img_diffuse->isMissingAsset())
				{
					bSetDiffuseNone = true;
				}
				else
				{
					switch(pMaterialParams->getDiffuseAlphaMode())
					{
					case LLMaterial::DIFFUSE_ALPHA_MODE_BLEND:
					case LLMaterial::DIFFUSE_ALPHA_MODE_EMISSIVE:
					case LLMaterial::DIFFUSE_ALPHA_MODE_MASK:
						{ //all of them modes available only for 32 bit textures
							if(GL_RGBA != img_diffuse->getPrimaryFormat())
							{
								bSetDiffuseNone = true;
							}
						} break;
					}
				} //else


				if(bSetDiffuseNone)
				{ //upps... we should substitute this material with LLMaterial::DIFFUSE_ALPHA_MODE_NONE
					new_material = new LLMaterial(pMaterialParams->asLLSD());
					new_material->setDiffuseAlphaMode(LLMaterial::DIFFUSE_ALPHA_MODE_NONE);
				}
			}
		}

		//normal
		if(LLUUID::null != pMaterialParams->getNormalID())
		{
			if(img_normal && img_normal->isMissingAsset() && img_normal->getID() == pMaterialParams->getNormalID())
			{
				if(!new_material) {
					new_material = new LLMaterial(pMaterialParams->asLLSD());
				}
				new_material->setNormalID(LLUUID::null);
			}
			else if(NULL == img_normal || 0 == img_normal->getPrimaryFormat())
			{ //ok here we don't have information about texture, let's belief and leave material settings
				//but we remember this case
				mWaitingTextureInfo.insert(mmap_UUID_MAP_t::value_type(pMaterialParams->getNormalID(), material_info(LLRender::NORMAL_MAP,te)));
			}

		}


		//specular
		if(LLUUID::null != pMaterialParams->getSpecularID())
		{
			if(img_specular && img_specular->isMissingAsset() && img_specular->getID() == pMaterialParams->getSpecularID())
			{
				if(!new_material) {
					new_material = new LLMaterial(pMaterialParams->asLLSD());
				}
				new_material->setSpecularID(LLUUID::null);
			}
			else if(NULL == img_specular || 0 == img_specular->getPrimaryFormat())
			{ //ok here we don't have information about texture, let's belief and leave material settings
				//but we remember this case
				mWaitingTextureInfo.insert(mmap_UUID_MAP_t::value_type(pMaterialParams->getSpecularID(), material_info(LLRender::SPECULAR_MAP, te)));
			}
		}

		if(new_material) {
			pMaterial = new_material;
			LLMaterialMgr::getInstance()->put(getID(),te,*pMaterial);
		}
	}

	S32 res = LLViewerObject::setTEMaterialParams(te, pMaterial);

	LL_DEBUGS("MaterialTEs") << "te " << (S32)te << " material " << ((pMaterial) ? pMaterial->asLLSD() : LLSD("null")) << " res " << res
							 << ( LLSelectMgr::getInstance()->getSelection()->contains(const_cast<LLVOVolume*>(this), te) ? " selected" : " not selected" )
							 << LL_ENDL;
	setChanged(ALL_CHANGED);
	if (!mDrawable.isNull())
	{
		gPipeline.markTextured(mDrawable);
		gPipeline.markRebuild(mDrawable,LLDrawable::REBUILD_ALL);
	}
	mFaceMappingChanged = TRUE;
	return TEM_CHANGE_TEXTURE;
}

S32 LLVOVolume::setTEScale(const U8 te, const F32 s, const F32 t)
{
	S32 res = LLViewerObject::setTEScale(te, s, t);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return res;
}

S32 LLVOVolume::setTEScaleS(const U8 te, const F32 s)
{
	S32 res = LLViewerObject::setTEScaleS(te, s);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return res;
}

S32 LLVOVolume::setTEScaleT(const U8 te, const F32 t)
{
	S32 res = LLViewerObject::setTEScaleT(te, t);
	if (res)
	{
		gPipeline.markTextured(mDrawable);
		mFaceMappingChanged = TRUE;
	}
	return res;
}

void LLVOVolume::updateTEData()
{
	/*if (mDrawable.notNull())
	{
		mFaceMappingChanged = TRUE;
		gPipeline.markRebuild(mDrawable, LLDrawable::REBUILD_MATERIAL, TRUE);
	}*/
}

bool LLVOVolume::hasMedia() const
{
	bool result = false;
	const U8 numTEs = getNumTEs();
	for (U8 i = 0; i < numTEs; i++)
	{
		const LLTextureEntry* te = getTE(i);
		if(te->hasMedia())
		{
			result = true;
			break;
		}
	}
	return result;
}

LLVector3 LLVOVolume::getApproximateFaceNormal(U8 face_id)
{
	LLVolume* volume = getVolume();
	LLVector4a result;
	result.clear();

	LLVector3 ret;

	if (volume && face_id < volume->getNumVolumeFaces())
	{
		const LLVolumeFace& face = volume->getVolumeFace(face_id);
		for (S32 i = 0; i < (S32)face.mNumVertices; ++i)
		{
			result.add(face.mNormals[i]);
		}

		LLVector3 ret(result.getF32ptr());
		ret = volumeDirectionToAgent(ret);
		ret.normVec();
	}
	
	return ret;
}

void LLVOVolume::requestMediaDataUpdate(bool isNew)
{
    if (sObjectMediaClient)
		sObjectMediaClient->fetchMedia(new LLMediaDataClientObjectImpl(this, isNew));
}

bool LLVOVolume::isMediaDataBeingFetched() const
{
	// I know what I'm doing by const_casting this away: this is just 
	// a wrapper class that is only going to do a lookup.
	return (sObjectMediaClient) ? sObjectMediaClient->isInQueue(new LLMediaDataClientObjectImpl(const_cast<LLVOVolume*>(this), false)) : false;
}

void LLVOVolume::cleanUpMediaImpls()
{
	// Iterate through our TEs and remove any Impls that are no longer used
	const U8 numTEs = getNumTEs();
	for (U8 i = 0; i < numTEs; i++)
	{
		const LLTextureEntry* te = getTE(i);
		if( ! te->hasMedia())
		{
			// Delete the media IMPL!
			removeMediaImpl(i) ;
		}
	}
}

void LLVOVolume::updateObjectMediaData(const LLSD &media_data_array, const std::string &media_version)
{
	// media_data_array is an array of media entry maps
	// media_version is the version string in the response.
	U32 fetched_version = LLTextureEntry::getVersionFromMediaVersionString(media_version);

	// Only update it if it is newer!
	if ( (S32)fetched_version > mLastFetchedMediaVersion)
	{
		mLastFetchedMediaVersion = fetched_version;
		//LL_INFOS() << "updating:" << this->getID() << " " << ll_pretty_print_sd(media_data_array) << LL_ENDL;
		
		LLSD::array_const_iterator iter = media_data_array.beginArray();
		LLSD::array_const_iterator end = media_data_array.endArray();
		U8 texture_index = 0;
		for (; iter != end; ++iter, ++texture_index)
		{
			syncMediaData(texture_index, *iter, false/*merge*/, false/*ignore_agent*/);
		}
	}
}

void LLVOVolume::syncMediaData(S32 texture_index, const LLSD &media_data, bool merge, bool ignore_agent)
{
	if(mDead)
	{
		// If the object has been marked dead, don't process media updates.
		return;
	}
	
	LLTextureEntry *te = getTE(texture_index);
	if(!te)
	{
		return ;
	}

	LL_DEBUGS("MediaOnAPrim") << "BEFORE: texture_index = " << texture_index
		<< " hasMedia = " << te->hasMedia() << " : " 
		<< ((NULL == te->getMediaData()) ? "NULL MEDIA DATA" : ll_pretty_print_sd(te->getMediaData()->asLLSD())) << LL_ENDL;

	std::string previous_url;
	LLMediaEntry* mep = te->getMediaData();
	if(mep)
	{
		// Save the "current url" from before the update so we can tell if
		// it changes. 
		previous_url = mep->getCurrentURL();
	}

	if (merge)
	{
		te->mergeIntoMediaData(media_data);
	}
	else {
		// XXX Question: what if the media data is undefined LLSD, but the
		// update we got above said that we have media flags??	Here we clobber
		// that, assuming the data from the service is more up-to-date. 
		te->updateMediaData(media_data);
	}

	mep = te->getMediaData();
	if(mep)
	{
		bool update_from_self = false;
		if (!ignore_agent) 
		{
			LLUUID updating_agent = LLTextureEntry::getAgentIDFromMediaVersionString(getMediaURL());
			update_from_self = (updating_agent == gAgent.getID());
		}
		viewer_media_t media_impl = LLViewerMedia::updateMediaImpl(mep, previous_url, update_from_self);
			
		addMediaImpl(media_impl, texture_index) ;
	}
	else
	{
		removeMediaImpl(texture_index);
	}

	LL_DEBUGS("MediaOnAPrim") << "AFTER: texture_index = " << texture_index
		<< " hasMedia = " << te->hasMedia() << " : " 
		<< ((NULL == te->getMediaData()) ? "NULL MEDIA DATA" : ll_pretty_print_sd(te->getMediaData()->asLLSD())) << LL_ENDL;
}

void LLVOVolume::mediaNavigateBounceBack(U8 texture_index)
{
	// Find the media entry for this navigate
	const LLMediaEntry* mep = NULL;
	viewer_media_t impl = getMediaImpl(texture_index);
	LLTextureEntry *te = getTE(texture_index);
	if(te)
	{
		mep = te->getMediaData();
	}
	
	if (mep && impl)
	{
        std::string url = mep->getCurrentURL();
		// Look for a ":", if not there, assume "http://"
		if (!url.empty() && std::string::npos == url.find(':')) 
		{
			url = "http://" + url;
		}
		// If the url we're trying to "bounce back" to is either empty or not
		// allowed by the whitelist, try the home url.  If *that* doesn't work,
		// set the media as failed and unload it
        if (url.empty() || !mep->checkCandidateUrl(url))
        {
            url = mep->getHomeURL();
			// Look for a ":", if not there, assume "http://"
			if (!url.empty() && std::string::npos == url.find(':')) 
			{
				url = "http://" + url;
			}
        }
        if (url.empty() || !mep->checkCandidateUrl(url))
		{
			// The url to navigate back to is not good, and we have nowhere else
			// to go.
			LL_WARNS("MediaOnAPrim") << "FAILED to bounce back URL \"" << url << "\" -- unloading impl" << LL_ENDL;
			impl->setMediaFailed(true);
		}
		else {
			// Okay, navigate now
            LL_INFOS("MediaOnAPrim") << "bouncing back to URL: " << url << LL_ENDL;
            impl->navigateTo(url, "", false, true);
        }
    }
}

bool LLVOVolume::hasMediaPermission(const LLMediaEntry* media_entry, MediaPermType perm_type)
{
    // NOTE: This logic ALMOST duplicates the logic in the server (in particular, in llmediaservice.cpp).
    if (NULL == media_entry ) return false; // XXX should we assert here?
    
    // The agent has permissions if:
    // - world permissions are on, or
    // - group permissions are on, and agent_id is in the group, or
    // - agent permissions are on, and agent_id is the owner
    
	// *NOTE: We *used* to check for modify permissions here (i.e. permissions were
	// granted if permModify() was true).  However, this doesn't make sense in the
	// viewer: we don't want to show controls or allow interaction if the author
	// has deemed it so.  See DEV-42115.
	
    U8 media_perms = (perm_type == MEDIA_PERM_INTERACT) ? media_entry->getPermsInteract() : media_entry->getPermsControl();
    
    // World permissions
    if (0 != (media_perms & LLMediaEntry::PERM_ANYONE)) 
    {
        return true;
    }
    
    // Group permissions
    else if (0 != (media_perms & LLMediaEntry::PERM_GROUP))
    {
		LLPermissions* obj_perm = LLSelectMgr::getInstance()->findObjectPermissions(this);
		if (obj_perm && gAgent.isInGroup(obj_perm->getGroup()))
		{
			return true;
		}
    }
    
    // Owner permissions
    else if (0 != (media_perms & LLMediaEntry::PERM_OWNER) && permYouOwner()) 
    {
        return true;
    }
    
    return false;
    
}

void LLVOVolume::mediaNavigated(LLViewerMediaImpl *impl, LLPluginClassMedia* plugin, std::string new_location)
{
	bool block_navigation = false;
	// FIXME: if/when we allow the same media impl to be used by multiple faces, the logic here will need to be fixed
	// to deal with multiple face indices.
	int face_index = getFaceIndexWithMediaImpl(impl, -1);
	
	// Find the media entry for this navigate
	LLMediaEntry* mep = NULL;
	LLTextureEntry *te = getTE(face_index);
	if(te)
	{
		mep = te->getMediaData();
	}
	
	if(mep)
	{
		if(!mep->checkCandidateUrl(new_location))
		{
			block_navigation = true;
		}
		if (!block_navigation && !hasMediaPermission(mep, MEDIA_PERM_INTERACT))
		{
			block_navigation = true;
		}
	}
	else
	{
		LL_WARNS("MediaOnAPrim") << "Couldn't find media entry!" << LL_ENDL;
	}
						
	if(block_navigation)
	{
		LL_INFOS("MediaOnAPrim") << "blocking navigate to URI " << new_location << LL_ENDL;

		// "bounce back" to the current URL from the media entry
		mediaNavigateBounceBack(face_index);
	}
	else if (sObjectMediaNavigateClient)
	{
		
		LL_DEBUGS("MediaOnAPrim") << "broadcasting navigate with URI " << new_location << LL_ENDL;

		sObjectMediaNavigateClient->navigate(new LLMediaDataClientObjectImpl(this, false), face_index, new_location);
	}
}

void LLVOVolume::mediaEvent(LLViewerMediaImpl *impl, LLPluginClassMedia* plugin, LLViewerMediaObserver::EMediaEvent event)
{
	switch(event)
	{
		
		case LLViewerMediaObserver::MEDIA_EVENT_LOCATION_CHANGED:
		{			
			switch(impl->getNavState())
			{
				case LLViewerMediaImpl::MEDIANAVSTATE_FIRST_LOCATION_CHANGED:
				{
					// This is the first location changed event after the start of a non-server-directed nav.  It may need to be broadcast or bounced back.
					mediaNavigated(impl, plugin, plugin->getLocation());
				}
				break;
				
				case LLViewerMediaImpl::MEDIANAVSTATE_FIRST_LOCATION_CHANGED_SPURIOUS:
					// This navigate didn't change the current URL.  
					LL_DEBUGS("MediaOnAPrim") << "	NOT broadcasting navigate (spurious)" << LL_ENDL;
				break;
				
				case LLViewerMediaImpl::MEDIANAVSTATE_SERVER_FIRST_LOCATION_CHANGED:
					// This is the first location changed event after the start of a server-directed nav.  Don't broadcast it.
					LL_INFOS("MediaOnAPrim") << "	NOT broadcasting navigate (server-directed)" << LL_ENDL;
				break;
				
				default:
					// This is a subsequent location-changed due to a redirect.	 Don't broadcast.
					LL_INFOS("MediaOnAPrim") << "	NOT broadcasting navigate (redirect)" << LL_ENDL;
				break;
			}
		}
		break;
		
		case LLViewerMediaObserver::MEDIA_EVENT_NAVIGATE_COMPLETE:
		{
			switch(impl->getNavState())
			{
				case LLViewerMediaImpl::MEDIANAVSTATE_COMPLETE_BEFORE_LOCATION_CHANGED:
				{
					// This is the first location changed event after the start of a non-server-directed nav.  It may need to be broadcast or bounced back.
					mediaNavigated(impl, plugin, plugin->getNavigateURI());
				}
				break;
				
				case LLViewerMediaImpl::MEDIANAVSTATE_COMPLETE_BEFORE_LOCATION_CHANGED_SPURIOUS:
					// This navigate didn't change the current URL.  
					LL_DEBUGS("MediaOnAPrim") << "	NOT broadcasting navigate (spurious)" << LL_ENDL;
				break;

				case LLViewerMediaImpl::MEDIANAVSTATE_SERVER_COMPLETE_BEFORE_LOCATION_CHANGED:
					// This is the the navigate complete event from a server-directed nav.  Don't broadcast it.
					LL_INFOS("MediaOnAPrim") << "	NOT broadcasting navigate (server-directed)" << LL_ENDL;
				break;
				
				default:
					// For all other states, the navigate should have been handled by LOCATION_CHANGED events already.
				break;
			}
		}
		break;
		
		default:
		break;
	}

}

void LLVOVolume::sendMediaDataUpdate()
{
    if (sObjectMediaClient)
		sObjectMediaClient->updateMedia(new LLMediaDataClientObjectImpl(this, false));
}

void LLVOVolume::removeMediaImpl(S32 texture_index)
{
	if(mMediaImplList.size() <= (U32)texture_index || mMediaImplList[texture_index].isNull())
	{
		return ;
	}

	//make the face referencing to mMediaImplList[texture_index] to point back to the old texture.
	if(mDrawable && texture_index < mDrawable->getNumFaces())
	{
		LLFace* facep = mDrawable->getFace(texture_index) ;
		if(facep)
		{
			LLViewerMediaTexture* media_tex = LLViewerTextureManager::findMediaTexture(mMediaImplList[texture_index]->getMediaTextureID()) ;
			if(media_tex)
			{
				media_tex->removeMediaFromFace(facep) ;
			}
		}
	}		
	
	//check if some other face(s) of this object reference(s)to this media impl.
	S32 i ;
	S32 end = (S32)mMediaImplList.size() ;
	for(i = 0; i < end ; i++)
	{
		if( i != texture_index && mMediaImplList[i] == mMediaImplList[texture_index])
		{
			break ;
		}
	}

	if(i == end) //this object does not need this media impl.
	{
		mMediaImplList[texture_index]->removeObject(this) ;
	}

	mMediaImplList[texture_index] = NULL ;
	return ;
}

void LLVOVolume::addMediaImpl(LLViewerMediaImpl* media_impl, S32 texture_index)
{
	if((S32)mMediaImplList.size() < texture_index + 1)
	{
		mMediaImplList.resize(texture_index + 1) ;
	}
	
	if(mMediaImplList[texture_index].notNull())
	{
		if(mMediaImplList[texture_index] == media_impl)
		{
			return ;
		}

		removeMediaImpl(texture_index) ;
	}

	mMediaImplList[texture_index] = media_impl;
	media_impl->addObject(this) ;	

	//add the face to show the media if it is in playing
	if(mDrawable)
	{
		LLFace* facep(NULL);
		if( texture_index < mDrawable->getNumFaces() )
		{
			facep = mDrawable->getFace(texture_index) ;
		}

		if(facep)
		{
			LLViewerMediaTexture* media_tex = LLViewerTextureManager::findMediaTexture(mMediaImplList[texture_index]->getMediaTextureID()) ;
			if(media_tex)
			{
				media_tex->addMediaToFace(facep) ;
			}
		}
		else //the face is not available now, start media on this face later.
		{
			media_impl->setUpdated(TRUE) ;
		}
	}
	return ;
}

viewer_media_t LLVOVolume::getMediaImpl(U8 face_id) const
{
	if(mMediaImplList.size() > face_id)
	{
		return mMediaImplList[face_id];
	}
	return NULL;
}

F64 LLVOVolume::getTotalMediaInterest() const
{
	// If this object is currently focused, this object has "high" interest
	if (LLViewerMediaFocus::getInstance()->getFocusedObjectID() == getID())
		return F64_MAX;
	
	F64 interest = (F64)-1.0;  // means not interested;
    
	// If this object is selected, this object has "high" interest, but since 
	// there can be more than one, we still add in calculated impl interest
	// XXX Sadly, 'contains()' doesn't take a const :(
	if (LLSelectMgr::getInstance()->getSelection()->contains(const_cast<LLVOVolume*>(this)))
		interest = F64_MAX / 2.0;
	
	int i = 0;
	const int end = getNumTEs();
	for ( ; i < end; ++i)
	{
		const viewer_media_t &impl = getMediaImpl(i);
		if (!impl.isNull())
		{
			if (interest == (F64)-1.0) interest = (F64)0.0;
			interest += impl->getInterest();
		}
	}
	return interest;
}

S32 LLVOVolume::getFaceIndexWithMediaImpl(const LLViewerMediaImpl* media_impl, S32 start_face_id)
{
	S32 end = (S32)mMediaImplList.size() ;
	for(S32 face_id = start_face_id + 1; face_id < end; face_id++)
	{
		if(mMediaImplList[face_id] == media_impl)
		{
			return face_id ;
		}
	}
	return -1 ;
}

//----------------------------------------------------------------------------

void LLVOVolume::setLightTextureID(LLUUID id)
{
	if (id.notNull())
	{
		if (!hasLightTexture())
		{
			setParameterEntryInUse(LLNetworkData::PARAMS_LIGHT_IMAGE, TRUE, true);
		}
		LLLightImageParams* param_block = (LLLightImageParams*) getParameterEntry(LLNetworkData::PARAMS_LIGHT_IMAGE);
		if (param_block && param_block->getLightTexture() != id)
		{
			param_block->setLightTexture(id);
			parameterChanged(LLNetworkData::PARAMS_LIGHT_IMAGE, true);
		}
	}
	else
	{
		if (hasLightTexture())
		{
			setParameterEntryInUse(LLNetworkData::PARAMS_LIGHT_IMAGE, FALSE, true);
			parameterChanged(LLNetworkData::PARAMS_LIGHT_IMAGE, true);
			mLightTexture = NULL;
		}
	}		
}

void LLVOVolume::setSpotLightParams(LLVector3 params)
{
	LLLightImageParams* param_block = (LLLightImageParams*) getParameterEntry(LLNetworkData::PARAMS_LIGHT_IMAGE);
	if (param_block && param_block->getParams() != params)
	{
		param_block->setParams(params);
		parameterChanged(LLNetworkData::PARAMS_LIGHT_IMAGE, true);
	}
}
		
void LLVOVolume::setIsLight(BOOL is_light)
{
	BOOL was_light = getIsLight();
	if (is_light != was_light)
	{
		if (is_light)
		{
			setParameterEntryInUse(LLNetworkData::PARAMS_LIGHT, TRUE, true);
		}
		else
		{
			setParameterEntryInUse(LLNetworkData::PARAMS_LIGHT, FALSE, true);
		}

		if (is_light)
		{
			// Add it to the pipeline mLightSet
			gPipeline.setLight(mDrawable, TRUE);
		}
		else
		{
			// Not a light.  Remove it from the pipeline's light set.
			gPipeline.setLight(mDrawable, FALSE);
		}
	}
}

void LLVOVolume::setLightColor(const LLColor3& color)
{
	LLLightParams *param_block = (LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		if (param_block->getColor() != color)
		{
			param_block->setColor(LLColor4(color, param_block->getColor().mV[3]));
			parameterChanged(LLNetworkData::PARAMS_LIGHT, true);
			gPipeline.markTextured(mDrawable);
			mFaceMappingChanged = TRUE;
		}
	}
}

void LLVOVolume::setLightIntensity(F32 intensity)
{
	LLLightParams *param_block = (LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		if (param_block->getColor().mV[3] != intensity)
		{
			param_block->setColor(LLColor4(LLColor3(param_block->getColor()), intensity));
			parameterChanged(LLNetworkData::PARAMS_LIGHT, true);
		}
	}
}

void LLVOVolume::setLightRadius(F32 radius)
{
	LLLightParams *param_block = (LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		if (param_block->getRadius() != radius)
		{
			param_block->setRadius(radius);
			parameterChanged(LLNetworkData::PARAMS_LIGHT, true);
		}
	}
}

void LLVOVolume::setLightFalloff(F32 falloff)
{
	LLLightParams *param_block = (LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		if (param_block->getFalloff() != falloff)
		{
			param_block->setFalloff(falloff);
			parameterChanged(LLNetworkData::PARAMS_LIGHT, true);
		}
	}
}

void LLVOVolume::setLightCutoff(F32 cutoff)
{
	LLLightParams *param_block = (LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		if (param_block->getCutoff() != cutoff)
		{
			param_block->setCutoff(cutoff);
			parameterChanged(LLNetworkData::PARAMS_LIGHT, true);
		}
	}
}

//----------------------------------------------------------------------------

BOOL LLVOVolume::getIsLight() const
{
	return getParameterEntryInUse(LLNetworkData::PARAMS_LIGHT);
}

LLColor3 LLVOVolume::getLightBaseColor() const
{
	const LLLightParams *param_block = (const LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		return LLColor3(param_block->getColor());
	}
	else
	{
		return LLColor3(1,1,1);
	}
}

LLColor3 LLVOVolume::getLightColor() const
{
	const LLLightParams *param_block = (const LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		return LLColor3(param_block->getColor()) * param_block->getColor().mV[3];
	}
	else
	{
		return LLColor3(1,1,1);
	}
}

LLUUID LLVOVolume::getLightTextureID() const
{
	if (getParameterEntryInUse(LLNetworkData::PARAMS_LIGHT_IMAGE))
	{
		const LLLightImageParams *param_block = (const LLLightImageParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT_IMAGE);
		if (param_block)
		{
			return param_block->getLightTexture();
		}
	}
	
	return LLUUID::null;
}


LLVector3 LLVOVolume::getSpotLightParams() const
{
	if (getParameterEntryInUse(LLNetworkData::PARAMS_LIGHT_IMAGE))
	{
		const LLLightImageParams *param_block = (const LLLightImageParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT_IMAGE);
		if (param_block)
		{
			return param_block->getParams();
		}
	}
	
	return LLVector3();
}

F32 LLVOVolume::getSpotLightPriority() const
{
	return mSpotLightPriority;
}

void LLVOVolume::updateSpotLightPriority()
{
	LLVector3 pos = mDrawable->getPositionAgent();
	LLVector3 at(0,0,-1);
	at *= getRenderRotation();

	F32 r = getLightRadius()*0.5f;

	pos += at * r;

	at = LLViewerCamera::getInstance()->getAtAxis();

	pos -= at * r;
	
	mSpotLightPriority = gPipeline.calcPixelArea(pos, LLVector3(r,r,r), *LLViewerCamera::getInstance());

	if (mLightTexture.notNull())
	{
		mLightTexture->addTextureStats(mSpotLightPriority);
		mLightTexture->setBoostLevel(LLGLTexture::BOOST_CLOUDS);
	}
}


bool LLVOVolume::isLightSpotlight() const
{
	LLLightImageParams* params = (LLLightImageParams*) getParameterEntry(LLNetworkData::PARAMS_LIGHT_IMAGE);
	if (params && getParameterEntryInUse(LLNetworkData::PARAMS_LIGHT_IMAGE))
	{
		return params->isLightSpotlight();
	}
	return false;
}


LLViewerTexture* LLVOVolume::getLightTexture()
{
	LLUUID id = getLightTextureID();

	if (id.notNull())
	{
		if (mLightTexture.isNull() || id != mLightTexture->getID())
		{
			mLightTexture = LLViewerTextureManager::getFetchedTexture(id);
		}
	}
	else
	{
		mLightTexture = NULL;
	}

	return mLightTexture;
}

F32 LLVOVolume::getLightIntensity() const
{
	const LLLightParams *param_block = (const LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		return param_block->getColor().mV[3];
	}
	else
	{
		return 1.f;
	}
}

F32 LLVOVolume::getLightRadius() const
{
	const LLLightParams *param_block = (const LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		return param_block->getRadius();
	}
	else
	{
		return 0.f;
	}
}

F32 LLVOVolume::getLightFalloff() const
{
	const LLLightParams *param_block = (const LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		return param_block->getFalloff();
	}
	else
	{
		return 0.f;
	}
}

F32 LLVOVolume::getLightCutoff() const
{
	const LLLightParams *param_block = (const LLLightParams *)getParameterEntry(LLNetworkData::PARAMS_LIGHT);
	if (param_block)
	{
		return param_block->getCutoff();
	}
	else
	{
		return 0.f;
	}
}

U32 LLVOVolume::getVolumeInterfaceID() const
{
	if (mVolumeImpl)
	{
		return mVolumeImpl->getID();
	}

	return 0;
}

BOOL LLVOVolume::isFlexible() const
{
	if (getParameterEntryInUse(LLNetworkData::PARAMS_FLEXIBLE))
	{
		LLVolume* volume = getVolume();
		if (volume && volume->getParams().getPathParams().getCurveType() != LL_PCODE_PATH_FLEXIBLE)
		{
			LLVolumeParams volume_params = getVolume()->getParams();
			U8 profile_and_hole = volume_params.getProfileParams().getCurveType();
			volume_params.setType(profile_and_hole, LL_PCODE_PATH_FLEXIBLE);
		}
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL LLVOVolume::isSculpted() const
{
	if (getParameterEntryInUse(LLNetworkData::PARAMS_SCULPT))
	{
		return TRUE;
	}
	
	return FALSE;
}

BOOL LLVOVolume::isMesh() const
{
	if (isSculpted())
	{
		LLSculptParams *sculpt_params = (LLSculptParams *)getParameterEntry(LLNetworkData::PARAMS_SCULPT);
		U8 sculpt_type = sculpt_params->getSculptType();

		if ((sculpt_type & LL_SCULPT_TYPE_MASK) == LL_SCULPT_TYPE_MESH)
			// mesh is a mesh
		{
			return TRUE;	
		}
	}

	return FALSE;
}

BOOL LLVOVolume::hasLightTexture() const
{
	if (getParameterEntryInUse(LLNetworkData::PARAMS_LIGHT_IMAGE))
	{
		return TRUE;
	}

	return FALSE;
}

BOOL LLVOVolume::isVolumeGlobal() const
{
	if (mVolumeImpl)
	{
		return mVolumeImpl->isVolumeGlobal() ? TRUE : FALSE;
	}
	else if (mRiggedVolume.notNull())
	{
		return TRUE;
	}

	return FALSE;
}

BOOL LLVOVolume::canBeFlexible() const
{
	U8 path = getVolume()->getParams().getPathParams().getCurveType();
	return (path == LL_PCODE_PATH_FLEXIBLE || path == LL_PCODE_PATH_LINE);
}

BOOL LLVOVolume::setIsFlexible(BOOL is_flexible)
{
	BOOL res = FALSE;
	BOOL was_flexible = isFlexible();
	LLVolumeParams volume_params;
	if (is_flexible)
	{
		if (!was_flexible)
		{
			volume_params = getVolume()->getParams();
			U8 profile_and_hole = volume_params.getProfileParams().getCurveType();
			volume_params.setType(profile_and_hole, LL_PCODE_PATH_FLEXIBLE);
			res = TRUE;
			setFlags(FLAGS_USE_PHYSICS, FALSE);
			setFlags(FLAGS_PHANTOM, TRUE);
			setParameterEntryInUse(LLNetworkData::PARAMS_FLEXIBLE, TRUE, true);
			if (mDrawable)
			{
				mDrawable->makeActive();
			}
		}
	}
	else
	{
		if (was_flexible)
		{
			volume_params = getVolume()->getParams();
			U8 profile_and_hole = volume_params.getProfileParams().getCurveType();
			volume_params.setType(profile_and_hole, LL_PCODE_PATH_LINE);
			res = TRUE;
			setFlags(FLAGS_PHANTOM, FALSE);
			setParameterEntryInUse(LLNetworkData::PARAMS_FLEXIBLE, FALSE, true);
		}
	}
	if (res)
	{
		res = setVolume(volume_params, 1);
		if (res)
		{
			markForUpdate(TRUE);
		}
	}
	return res;
}

const LLMeshSkinInfo* LLVOVolume::getSkinInfo() const
{
    if (getVolume())
    {
        return gMeshRepo.getSkinInfo(getVolume()->getParams().getSculptID(), this);
    }
    else
    {
        return NULL;
    }
}

// virtual
BOOL LLVOVolume::isRiggedMesh() const
{
    return isMesh() && getSkinInfo();
}

//----------------------------------------------------------------------------
// AXON - methods related to extended mesh flags

U32 LLVOVolume::getExtendedMeshFlags() const
{
	const LLExtendedMeshParams *param_block = 
        (const LLExtendedMeshParams *)getParameterEntry(LLNetworkData::PARAMS_EXTENDED_MESH);
	if (param_block)
	{
		return param_block->getFlags();
	}
	else
	{
		return 0;
	}
}

void LLVOVolume::onSetExtendedMeshFlags(U32 flags)
{
    // AXON - the check against isAnySelected() is "empirically
    // derived": doing rebuildGeom() while in selection trashes the
    // graphics state of animated objects. Skipping this update is OK
    // because we get another one on deselect.

	if (!getRootEdit()->isAnySelected() && mDrawable.notNull())
    {
        // Need to trigger rebuildGeom(), which is where control avatars get created/removed
        getRootEdit()->recursiveMarkForUpdate(TRUE);
    }
    if (isAttachment() && getAvatarAncestor())
    {
        updateVisualComplexity();
        if (flags & LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG)
        {
            // Making a rigged mesh into an animated object
            getAvatarAncestor()->removeAttachmentOverridesForObject(this);
        }
        else
        {
            // Making an animated object into a rigged mesh
            getAvatarAncestor()->addAttachmentOverridesForObject(this);
        }
    }
}

void LLVOVolume::setExtendedMeshFlags(U32 flags)
{
    U32 curr_flags = getExtendedMeshFlags();
    if (curr_flags != flags)
    {
        bool in_use = true;
        setParameterEntryInUse(LLNetworkData::PARAMS_EXTENDED_MESH, in_use, true);
        LLExtendedMeshParams *param_block = 
            (LLExtendedMeshParams *)getParameterEntry(LLNetworkData::PARAMS_EXTENDED_MESH);
        if (param_block)
        {
            param_block->setFlags(flags);
        }
        parameterChanged(LLNetworkData::PARAMS_EXTENDED_MESH, true);
        LL_DEBUGS("AXON") << (U32) this
                   << " new flags " << flags << " curr_flags " << curr_flags
                   << ", calling onSetExtendedMeshFlags()"
                   << LL_ENDL;
        onSetExtendedMeshFlags(flags);
    }
}

bool LLVOVolume::canBeAnimatedObject() const
{
    F32 est_tris = recursiveGetEstTrianglesMax();
    if (est_tris < 0 || est_tris > getAnimatedObjectMaxTris())
    {
        LL_DEBUGS("AXON") << "est_tris " << est_tris << " is outside limit of 0-" << getAnimatedObjectMaxTris() << LL_ENDL;
        return false;
    }
    return true;
}

bool LLVOVolume::isAnimatedObject() const
{
    LLVOVolume *root_vol = (LLVOVolume*)getRootEdit();
    bool root_is_animated_flag = root_vol->getExtendedMeshFlags() & LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG;
    return root_is_animated_flag;
#if 0
    if (root_is_animated_flag)
    {
        bool root_can_be_animated = root_vol->canBeAnimatedObject();
        bool this_can_be_animated = (root_vol == this) ? root_can_be_animated : canBeAnimatedObject();
        if (this_can_be_animated && root_can_be_animated)
        {
            return true;
        }
    }
    return false;
#endif
}

// Make sure animated objects in a linkset are consistent. The rules are:
// Only the root of a linkset can have the animated object flag set
// Only the root of a linkset can have a control avatar (iff the animated object flag is set)
// Only skinned mesh volumes can have the animated object flag set, or a control avatar
bool LLVOVolume::isAnimatedObjectStateConsistent() const
{
    if (!canBeAnimatedObject())
    {
        if ((getExtendedMeshFlags() & LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG) ||
            mControlAvatar.notNull())
        {
            LL_WARNS("AXON") << "Non animatable object has mesh enabled flag or mControlAvatar. Flags " 
                             << getExtendedMeshFlags() << " cav " << mControlAvatar.get() << LL_ENDL;
            return false;
        }
    }
    if (!isRootEdit())
    {
        if ((getExtendedMeshFlags() & LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG) ||
            mControlAvatar.notNull())
        {
            LL_WARNS("AXON") << "Non root object has mesh enabled flag or mControlAvatar. Flags " 
                             << getExtendedMeshFlags() << " cav " << mControlAvatar.get() << LL_ENDL;
            return false;
        }
    }
    // If we get here, we have a potentially animatable root volume.
    bool is_animation_enabled = getExtendedMeshFlags() & LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG;
    bool has_control_avatar = (mControlAvatar.notNull());
    if (is_animation_enabled != has_control_avatar)
    {
        LL_WARNS("AXON") << "Inconsistent state: animation enabled " << is_animation_enabled
                         << " has control avatar " << has_control_avatar 
                         << " flags " << getExtendedMeshFlags() << " cav " << mControlAvatar.get() << LL_ENDL;
        return false;
    }
    return true;
}

// Called any time parenting changes for a volume. Update flags and
// control av accordingly.  This is called after parent has been
// changed to new_parent.
void LLVOVolume::updateAnimatedObjectStateOnReparent(LLViewerObject *old_parent, LLViewerObject *new_parent)
{
    LLVOVolume *old_volp = dynamic_cast<LLVOVolume*>(old_parent);
    LLVOVolume *new_volp = dynamic_cast<LLVOVolume*>(new_parent);

    // AXON - depending on whether animated objects can be attached,
    // we may want to include or remove the isAvatar() check.
    if (new_parent && !new_parent->isAvatar())
    {
#if 0 // AXON - MAINT-7819
        // Object should inherit control avatar and animated mesh flag
        // from parent, so clear them out from our own state
        if (getExtendedMeshFlags() & LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG)
        {
            setExtendedMeshFlags(getExtendedMeshFlags() & ~LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG);
        }
#endif
        if (mControlAvatar.notNull())
        {
            LLControlAvatar *av = mControlAvatar;
            mControlAvatar = NULL;
            av->markForDeath();
        }
        // If this succeeds now, it's because the new_parent is an animated object
        if (isAnimatedObject() && getControlAvatar())
        {
            getControlAvatar()->addAttachmentOverridesForObject(this);
        }
    }
    if (old_volp && old_volp->isAnimatedObject())
    {
        if (old_volp->getControlAvatar())
        {
            // We have been removed from an animated object, need to do cleanup.
            old_volp->getControlAvatar()->removeAttachmentOverridesForObject(this);
        }
    }
    
    if (old_volp)
    {
        if (!old_volp->isAnimatedObjectStateConsistent())
        {
            LL_WARNS("AXON") << "old_volp failed consistency check" << LL_ENDL;
        }
    }
    if (new_volp)
    {
        if (!new_volp->isAnimatedObjectStateConsistent())
        {
            LL_WARNS("AXON") << "new_volp failed consistency check" << LL_ENDL;
        }
    }
    if (!isAnimatedObjectStateConsistent())
    {
        LL_WARNS("AXON") << "child object failed consistency check" << LL_ENDL;
    }
}

//----------------------------------------------------------------------------

void LLVOVolume::generateSilhouette(LLSelectNode* nodep, const LLVector3& view_point)
{
	LLVolume *volume = getVolume();

	if (volume)
	{
		LLVector3 view_vector;
		view_vector = view_point; 

		//transform view vector into volume space
		view_vector -= getRenderPosition();
		//mDrawable->mDistanceWRTCamera = view_vector.length();
		LLQuaternion worldRot = getRenderRotation();
		view_vector = view_vector * ~worldRot;
		if (!isVolumeGlobal())
		{
			LLVector3 objScale = getScale();
			LLVector3 invObjScale(1.f / objScale.mV[VX], 1.f / objScale.mV[VY], 1.f / objScale.mV[VZ]);
			view_vector.scaleVec(invObjScale);
		}
		
		updateRelativeXform();
		LLMatrix4 trans_mat = mRelativeXform;
		if (mDrawable->isStatic())
		{
			trans_mat.translate(getRegion()->getOriginAgent());
		}

		volume->generateSilhouetteVertices(nodep->mSilhouetteVertices, nodep->mSilhouetteNormals, view_vector, trans_mat, mRelativeXformInvTrans, nodep->getTESelectMask());

		nodep->mSilhouetteExists = TRUE;
	}
}

void LLVOVolume::deleteFaces()
{
	S32 face_count = mNumFaces;
	if (mDrawable.notNull())
	{
		mDrawable->deleteFaces(0, face_count);
	}

	mNumFaces = 0;
}

void LLVOVolume::updateRadius()
{
	if (mDrawable.isNull())
	{
		return;
	}
	
	mVObjRadius = getScale().length();
	mDrawable->setRadius(mVObjRadius);
}


BOOL LLVOVolume::isAttachment() const
{
	return mAttachmentState != 0 ;
}

BOOL LLVOVolume::isHUDAttachment() const
{
	// *NOTE: we assume hud attachment points are in defined range
	// since this range is constant for backwards compatibility
	// reasons this is probably a reasonable assumption to make
	S32 attachment_id = ATTACHMENT_ID_FROM_STATE(mAttachmentState);
	return ( attachment_id >= 31 && attachment_id <= 38 );
}


const LLMatrix4 LLVOVolume::getRenderMatrix() const
{
	if (mDrawable->isActive() && !mDrawable->isRoot())
	{
		return mDrawable->getParent()->getWorldMatrix();
	}
	return mDrawable->getWorldMatrix();
}

// Returns a base cost and adds textures to passed in set.
// total cost is returned value + 5 * size of the resulting set.
// Cannot include cost of textures, as they may be re-used in linked
// children, and cost should only be increased for unique textures  -Nyx
U32 LLVOVolume::getRenderCost(texture_cost_t &textures) const
{
    /*****************************************************************
     * This calculation should not be modified by third party viewers,
     * since it is used to limit rendering and should be uniform for
     * everyone. If you have suggested improvements, submit them to
     * the official viewer for consideration.
     *****************************************************************/

	// Get access to params we'll need at various points.  
	// Skip if this is object doesn't have a volume (e.g. is an avatar).
	BOOL has_volume = (getVolume() != NULL);
	LLVolumeParams volume_params;
	LLPathParams path_params;
	LLProfileParams profile_params;

	U32 num_triangles = 0;

	// per-prim costs
	static const U32 ARC_PARTICLE_COST = 1; // determined experimentally
	static const U32 ARC_PARTICLE_MAX = 2048; // default values
	static const U32 ARC_TEXTURE_COST = 16; // multiplier for texture resolution - performance tested
	static const U32 ARC_LIGHT_COST = 500; // static cost for light-producing prims 
	static const U32 ARC_MEDIA_FACE_COST = 1500; // static cost per media-enabled face 


	// per-prim multipliers
	static const F32 ARC_GLOW_MULT = 1.5f; // tested based on performance
	static const F32 ARC_BUMP_MULT = 1.25f; // tested based on performance
	static const F32 ARC_FLEXI_MULT = 5; // tested based on performance
	static const F32 ARC_SHINY_MULT = 1.6f; // tested based on performance
	static const F32 ARC_INVISI_COST = 1.2f; // tested based on performance
	static const F32 ARC_WEIGHTED_MESH = 1.2f; // tested based on performance

	static const F32 ARC_PLANAR_COST = 1.0f; // tested based on performance to have negligible impact
	static const F32 ARC_ANIM_TEX_COST = 4.f; // tested based on performance
	static const F32 ARC_ALPHA_COST = 4.f; // 4x max - based on performance

	F32 shame = 0;

	U32 invisi = 0;
	U32 shiny = 0;
	U32 glow = 0;
	U32 alpha = 0;
	U32 flexi = 0;
	U32 animtex = 0;
	U32 particles = 0;
	U32 bump = 0;
	U32 planar = 0;
	U32 weighted_mesh = 0;
	U32 produces_light = 0;
	U32 media_faces = 0;

	const LLDrawable* drawablep = mDrawable;
	U32 num_faces = drawablep->getNumFaces();

	if (has_volume)
	{
		volume_params = getVolume()->getParams();
		path_params = volume_params.getPathParams();
		profile_params = volume_params.getProfileParams();

		F32 weighted_triangles = -1.0;
		getStreamingCost(NULL, NULL, &weighted_triangles);

		if (weighted_triangles > 0.0)
		{
			num_triangles = (U32)(weighted_triangles); 
		}
	}

	if (num_triangles == 0)
	{
		num_triangles = 4;
	}

	if (isSculpted())
	{
		if (isMesh())
		{
			// base cost is dependent on mesh complexity
			// note that 3 is the highest LOD as of the time of this coding.
			S32 size = gMeshRepo.getMeshSize(volume_params.getSculptID(), getLOD());
			if ( size > 0)
			{
				if (isRiggedMesh())
				{
					// weighted attachment - 1 point for every 3 bytes
					weighted_mesh = 1;
				}

			}
			else
			{
				// something went wrong - user should know their content isn't render-free
				return 0;
			}
		}
		else
		{
			const LLSculptParams *sculpt_params = (LLSculptParams *) getParameterEntry(LLNetworkData::PARAMS_SCULPT);
			LLUUID sculpt_id = sculpt_params->getSculptTexture();
			if (textures.find(sculpt_id) == textures.end())
			{
				LLViewerFetchedTexture *texture = LLViewerTextureManager::getFetchedTexture(sculpt_id);
				if (texture)
				{
					S32 texture_cost = 256 + (S32)(ARC_TEXTURE_COST * (texture->getFullHeight() / 128.f + texture->getFullWidth() / 128.f));
					textures.insert(texture_cost_t::value_type(sculpt_id, texture_cost));
				}
			}
		}
	}

	if (isFlexible())
	{
		flexi = 1;
	}
	if (isParticleSource())
	{
		particles = 1;
	}

	if (getIsLight())
	{
		produces_light = 1;
	}

	for (S32 i = 0; i < num_faces; ++i)
	{
		const LLFace* face = drawablep->getFace(i);
		if (!face) continue;
		const LLTextureEntry* te = face->getTextureEntry();
		const LLViewerTexture* img = face->getTexture();

		if (img)
		{
			if (textures.find(img->getID()) == textures.end())
			{
				S32 texture_cost = 256 + (S32)(ARC_TEXTURE_COST * (img->getFullHeight() / 128.f + img->getFullWidth() / 128.f));
				textures.insert(texture_cost_t::value_type(img->getID(), texture_cost));
			}
		}

		if (face->getPoolType() == LLDrawPool::POOL_ALPHA)
		{
			alpha = 1;
		}
		else if (img && img->getPrimaryFormat() == GL_ALPHA)
		{
			invisi = 1;
		}
		if (face->hasMedia())
		{
			media_faces++;
		}

		if (te)
		{
			if (te->getBumpmap())
			{
				// bump is a multiplier, don't add per-face
				bump = 1;
			}
			if (te->getShiny())
			{
				// shiny is a multiplier, don't add per-face
				shiny = 1;
			}
			if (te->getGlow() > 0.f)
			{
				// glow is a multiplier, don't add per-face
				glow = 1;
			}
			if (face->mTextureMatrix != NULL)
			{
				animtex = 1;
			}
			if (te->getTexGen())
			{
				planar = 1;
			}
		}
	}

	// shame currently has the "base" cost of 1 point per 15 triangles, min 2.
	shame = num_triangles  * 5.f;
	shame = shame < 2.f ? 2.f : shame;

	// multiply by per-face modifiers
	if (planar)
	{
		shame *= planar * ARC_PLANAR_COST;
	}

	if (animtex)
	{
		shame *= animtex * ARC_ANIM_TEX_COST;
	}

	if (alpha)
	{
		shame *= alpha * ARC_ALPHA_COST;
	}

	if(invisi)
	{
		shame *= invisi * ARC_INVISI_COST;
	}

	if (glow)
	{
		shame *= glow * ARC_GLOW_MULT;
	}

	if (bump)
	{
		shame *= bump * ARC_BUMP_MULT;
	}

	if (shiny)
	{
		shame *= shiny * ARC_SHINY_MULT;
	}


	// multiply shame by multipliers
	if (weighted_mesh)
	{
		shame *= weighted_mesh * ARC_WEIGHTED_MESH;
	}

	if (flexi)
	{
		shame *= flexi * ARC_FLEXI_MULT;
	}


	// add additional costs
	if (particles)
	{
		const LLPartSysData *part_sys_data = &(mPartSourcep->mPartSysData);
		const LLPartData *part_data = &(part_sys_data->mPartData);
		U32 num_particles = (U32)(part_sys_data->mBurstPartCount * llceil( part_data->mMaxAge / part_sys_data->mBurstRate));
		num_particles = num_particles > ARC_PARTICLE_MAX ? ARC_PARTICLE_MAX : num_particles;
		F32 part_size = (llmax(part_data->mStartScale[0], part_data->mEndScale[0]) + llmax(part_data->mStartScale[1], part_data->mEndScale[1])) / 2.f;
		shame += num_particles * part_size * ARC_PARTICLE_COST;
	}

	if (produces_light)
	{
		shame += ARC_LIGHT_COST;
	}

	if (media_faces)
	{
		shame += media_faces * ARC_MEDIA_FACE_COST;
	}

	if (shame > mRenderComplexity_current)
	{
		mRenderComplexity_current = (S32)shame;
	}

	return (U32)shame;
}

F32 LLVOVolume::getEstTrianglesMax() const
{
	if (isMesh() && getVolume())
	{
		return gMeshRepo.getEstTrianglesMax(getVolume()->getParams().getSculptID());
	}
    return 0.f;
}

F32 LLVOVolume::getStreamingCost(S32* bytes, S32* visible_bytes, F32* unscaled_value) const
{
	F32 radius = getScale().length()*0.5f;

	if (isMesh())
	{
		return gMeshRepo.getStreamingCost(getVolume()->getParams().getSculptID(), radius, bytes, visible_bytes, mLOD, unscaled_value);
	}
	else
	{
		LLVolume* volume = getVolume();
		S32 counts[4];
		LLVolume::getLoDTriangleCounts(volume->getParams(), counts);

		LLSD header;
		header["lowest_lod"]["size"] = counts[0] * 10;
		header["low_lod"]["size"] = counts[1] * 10;
		header["medium_lod"]["size"] = counts[2] * 10;
		header["high_lod"]["size"] = counts[3] * 10;

		return LLMeshRepository::getStreamingCost(header, radius, NULL, NULL, -1, unscaled_value);
	}	
}

//static 
void LLVOVolume::updateRenderComplexity()
{
	mRenderComplexity_last = mRenderComplexity_current;
	mRenderComplexity_current = 0;
}

U32 LLVOVolume::getTriangleCount(S32* vcount) const
{
	U32 count = 0;
	LLVolume* volume = getVolume();
	if (volume)
	{
		count = volume->getNumTriangles(vcount);
	}

	return count;
}

U32 LLVOVolume::getHighLODTriangleCount()
{
	U32 ret = 0;

	LLVolume* volume = getVolume();

	if (!isSculpted())
	{
		LLVolume* ref = LLPrimitive::getVolumeManager()->refVolume(volume->getParams(), 3);
		ret = ref->getNumTriangles();
		LLPrimitive::getVolumeManager()->unrefVolume(ref);
	}
	else if (isMesh())
	{
		LLVolume* ref = LLPrimitive::getVolumeManager()->refVolume(volume->getParams(), 3);
		if (!ref->isMeshAssetLoaded() || ref->getNumVolumeFaces() == 0)
		{
			gMeshRepo.loadMesh(this, volume->getParams(), LLModel::LOD_HIGH);
		}
		ret = ref->getNumTriangles();
		LLPrimitive::getVolumeManager()->unrefVolume(ref);
	}
	else
	{ //default sculpts have a constant number of triangles
		ret = 31*2*31;  //31 rows of 31 columns of quads for a 32x32 vertex patch
	}

	return ret;
}

//static
void LLVOVolume::preUpdateGeom()
{
	sNumLODChanges = 0;
}

void LLVOVolume::parameterChanged(U16 param_type, bool local_origin)
{
	LLViewerObject::parameterChanged(param_type, local_origin);
}

void LLVOVolume::parameterChanged(U16 param_type, LLNetworkData* data, BOOL in_use, bool local_origin)
{
	LLViewerObject::parameterChanged(param_type, data, in_use, local_origin);
	if (mVolumeImpl)
	{
		mVolumeImpl->onParameterChanged(param_type, data, in_use, local_origin);
	}
    if (!local_origin && param_type == LLNetworkData::PARAMS_EXTENDED_MESH)
    {
        // AXON better if we could compare the before and after flags directly.
        U32 extended_mesh_flags = getExtendedMeshFlags();
        bool enabled =  (extended_mesh_flags & LLExtendedMeshParams::ANIMATED_MESH_ENABLED_FLAG);
        bool was_enabled = (getControlAvatar() != NULL);
        if (enabled != was_enabled)
        {
            LL_DEBUGS("AXON") << (U32) this
                       << " calling onSetExtendedMeshFlags, enabled " << (U32) enabled
                       << " was_enabled " << (U32) was_enabled
                       << " local_origin " << (U32) local_origin
                       << LL_ENDL;
            onSetExtendedMeshFlags(extended_mesh_flags);
        }
    }
	if (mDrawable.notNull())
	{
		BOOL is_light = getIsLight();
		if (is_light != mDrawable->isState(LLDrawable::LIGHT))
		{
			gPipeline.setLight(mDrawable, is_light);
		}
	}
}

void LLVOVolume::setSelected(BOOL sel)
{
	LLViewerObject::setSelected(sel);
    if (isAnimatedObject())
    {
        getRootEdit()->recursiveMarkForUpdate(TRUE);
    }
    else
    {
        if (mDrawable.notNull())
        {
            markForUpdate(TRUE);
        }
    }
}

void LLVOVolume::updateSpatialExtents(LLVector4a& newMin, LLVector4a& newMax)
{		
}

F32 LLVOVolume::getBinRadius()
{
	F32 radius;
	
	F32 scale = 1.f;

	S32 size_factor = llmax(gSavedSettings.getS32("OctreeStaticObjectSizeFactor"), 1);
	S32 attachment_size_factor = llmax(gSavedSettings.getS32("OctreeAttachmentSizeFactor"), 1);
	LLVector3 distance_factor = gSavedSettings.getVector3("OctreeDistanceFactor");
	LLVector3 alpha_distance_factor = gSavedSettings.getVector3("OctreeAlphaDistanceFactor");
	const LLVector4a* ext = mDrawable->getSpatialExtents();
	
	BOOL shrink_wrap = mDrawable->isAnimating();
	BOOL alpha_wrap = FALSE;

	if (!isHUDAttachment())
	{
		for (S32 i = 0; i < mDrawable->getNumFaces(); i++)
		{
			LLFace* face = mDrawable->getFace(i);
			if (!face) continue;
			if (face->getPoolType() == LLDrawPool::POOL_ALPHA &&
			    !face->canRenderAsMask())
			{
				alpha_wrap = TRUE;
				break;
			}
		}
	}
	else
	{
		shrink_wrap = FALSE;
	}

	if (alpha_wrap)
	{
		LLVector3 bounds = getScale();
		radius = llmin(bounds.mV[1], bounds.mV[2]);
		radius = llmin(radius, bounds.mV[0]);
		radius *= 0.5f;
		radius *= 1.f+mDrawable->mDistanceWRTCamera*alpha_distance_factor[1];
		radius += mDrawable->mDistanceWRTCamera*alpha_distance_factor[0];
	}
	else if (shrink_wrap)
	{
		LLVector4a rad;
		rad.setSub(ext[1], ext[0]);
		
		radius = rad.getLength3().getF32()*0.5f;
	}
	else if (mDrawable->isStatic())
	{
		F32 szf = size_factor;

		radius = llmax(mDrawable->getRadius(), szf);
		
		radius = powf(radius, 1.f+szf/radius);

		radius *= 1.f + mDrawable->mDistanceWRTCamera * distance_factor[1];
		radius += mDrawable->mDistanceWRTCamera * distance_factor[0];
	}
	else if (mDrawable->getVObj()->isAttachment())
	{
		radius = llmax((S32) mDrawable->getRadius(),1)*attachment_size_factor;
	}
	else
	{
		radius = mDrawable->getRadius();
		radius *= 1.f + mDrawable->mDistanceWRTCamera * distance_factor[1];
		radius += mDrawable->mDistanceWRTCamera * distance_factor[0];
	}

	return llclamp(radius*scale, 0.5f, 256.f);
}

const LLVector3 LLVOVolume::getPivotPositionAgent() const
{
	if (mVolumeImpl)
	{
		return mVolumeImpl->getPivotPosition();
	}
	return LLViewerObject::getPivotPositionAgent();
}

void LLVOVolume::onShift(const LLVector4a &shift_vector)
{
	if (mVolumeImpl)
	{
		mVolumeImpl->onShift(shift_vector);
	}

	updateRelativeXform();
}

const LLMatrix4& LLVOVolume::getWorldMatrix(LLXformMatrix* xform) const
{
	if (mVolumeImpl)
	{
		return mVolumeImpl->getWorldMatrix(xform);
	}
	return xform->getWorldMatrix();
}

void LLVOVolume::markForUpdate(BOOL priority)
{ 
#if 1
	// AXON debugging
    if (isAnimatedObject() && isRiggedMesh())
    {
        std::string vobj_name = llformat("Vol%u", (U32) this);
        F32 est_tris = getEstTrianglesMax();
        LL_DEBUGS("AXONLinkset") << vobj_name << " markForUpdate, tris " << est_tris << LL_ENDL; 
    }
#endif
    LLViewerObject::markForUpdate(priority); 
    mVolumeChanged = TRUE; 
}

LLVector3 LLVOVolume::agentPositionToVolume(const LLVector3& pos) const
{
	LLVector3 ret = pos - getRenderPosition();
	ret = ret * ~getRenderRotation();
	if (!isVolumeGlobal())
	{
		LLVector3 objScale = getScale();
		LLVector3 invObjScale(1.f / objScale.mV[VX], 1.f / objScale.mV[VY], 1.f / objScale.mV[VZ]);
		ret.scaleVec(invObjScale);
	}
	
	return ret;
}

LLVector3 LLVOVolume::agentDirectionToVolume(const LLVector3& dir) const
{
	LLVector3 ret = dir * ~getRenderRotation();
	
	LLVector3 objScale = isVolumeGlobal() ? LLVector3(1,1,1) : getScale();
	ret.scaleVec(objScale);

	return ret;
}

LLVector3 LLVOVolume::volumePositionToAgent(const LLVector3& dir) const
{
	LLVector3 ret = dir;
	if (!isVolumeGlobal())
	{
		LLVector3 objScale = getScale();
		ret.scaleVec(objScale);
	}

	ret = ret * getRenderRotation();
	ret += getRenderPosition();
	
	return ret;
}

LLVector3 LLVOVolume::volumeDirectionToAgent(const LLVector3& dir) const
{
	LLVector3 ret = dir;
	LLVector3 objScale = isVolumeGlobal() ? LLVector3(1,1,1) : getScale();
	LLVector3 invObjScale(1.f / objScale.mV[VX], 1.f / objScale.mV[VY], 1.f / objScale.mV[VZ]);
	ret.scaleVec(invObjScale);
	ret = ret * getRenderRotation();

	return ret;
}


BOOL LLVOVolume::lineSegmentIntersect(const LLVector4a& start, const LLVector4a& end, S32 face, BOOL pick_transparent, BOOL pick_rigged, S32 *face_hitp,
									  LLVector4a* intersection,LLVector2* tex_coord, LLVector4a* normal, LLVector4a* tangent)
	
{
	if (!mbCanSelect 
		|| mDrawable->isDead() 
		|| !gPipeline.hasRenderType(mDrawable->getRenderType()))
	{
		return FALSE;
	}

	BOOL ret = FALSE;

	LLVolume* volume = getVolume();

	bool transform = true;

	if (mDrawable->isState(LLDrawable::RIGGED))
	{
		if ((pick_rigged) || (getAvatar() && (getAvatar()->isSelf()) && (LLFloater::isVisible(gFloaterTools))))
		{
			updateRiggedVolume(true);
			volume = mRiggedVolume;
			transform = false;
		}
		else
		{ //cannot pick rigged attachments on other avatars or when not in build mode
			return FALSE;
		}
	}
	
	if (volume)
	{	
		LLVector4a local_start = start;
		LLVector4a local_end = end;
	
		if (transform)
		{
			LLVector3 v_start(start.getF32ptr());
			LLVector3 v_end(end.getF32ptr());
		
			v_start = agentPositionToVolume(v_start);
			v_end = agentPositionToVolume(v_end);

			local_start.load3(v_start.mV);
			local_end.load3(v_end.mV);
		}
		
		LLVector4a p;
		LLVector4a n;
		LLVector2 tc;
		LLVector4a tn;

		if (intersection != NULL)
		{
			p = *intersection;
		}

		if (tex_coord != NULL)
		{
			tc = *tex_coord;
		}

		if (normal != NULL)
		{
			n = *normal;
		}

		if (tangent != NULL)
		{
			tn = *tangent;
		}

		S32 face_hit = -1;

		S32 start_face, end_face;
		if (face == -1)
		{
			start_face = 0;
			end_face = volume->getNumVolumeFaces();
		}
		else
		{
			start_face = face;
			end_face = face+1;
		}
		pick_transparent |= isHiglightedOrBeacon();
		bool special_cursor = specialHoverCursor();
		for (S32 i = start_face; i < end_face; ++i)
		{
			if (!special_cursor && !pick_transparent && getTE(i) && getTE(i)->getColor().mV[3] == 0.f)
			{ //don't attempt to pick completely transparent faces unless
				//pick_transparent is true
				continue;
			}

			face_hit = volume->lineSegmentIntersect(local_start, local_end, i,
													&p, &tc, &n, &tn);
			
			if (face_hit >= 0 && mDrawable->getNumFaces() > face_hit)
			{
				LLFace* face = mDrawable->getFace(face_hit);				

				bool ignore_alpha = false;

				const LLTextureEntry* te = face->getTextureEntry();
				if (te)
				{
					LLMaterial* mat = te->getMaterialParams();
					if (mat)
					{
						U8 mode = mat->getDiffuseAlphaMode();

						if (mode == LLMaterial::DIFFUSE_ALPHA_MODE_EMISSIVE ||
							mode == LLMaterial::DIFFUSE_ALPHA_MODE_NONE)
						{
							ignore_alpha = true;
						}
					}
				}

				if (face &&
					(ignore_alpha ||
					pick_transparent || 
					!face->getTexture() || 
					!face->getTexture()->hasGLTexture() || 
					face->getTexture()->getMask(face->surfaceToTexture(tc, p, n))))
				{
					local_end = p;
					if (face_hitp != NULL)
					{
						*face_hitp = face_hit;
					}
					
					if (intersection != NULL)
					{
						if (transform)
						{
							LLVector3 v_p(p.getF32ptr());

							intersection->load3(volumePositionToAgent(v_p).mV);  // must map back to agent space
						}
						else
						{
							*intersection = p;
						}
					}

					if (normal != NULL)
					{
						if (transform)
						{
							LLVector3 v_n(n.getF32ptr());
							normal->load3(volumeDirectionToAgent(v_n).mV);
						}
						else
						{
							*normal = n;
						}
						(*normal).normalize3fast();
					}

					if (tangent != NULL)
					{
						if (transform)
						{
							LLVector3 v_tn(tn.getF32ptr());

							LLVector4a trans_tangent;
							trans_tangent.load3(volumeDirectionToAgent(v_tn).mV);

							LLVector4Logical mask;
							mask.clear();
							mask.setElement<3>();

							tangent->setSelectWithMask(mask, tn, trans_tangent);
						}
						else
						{
							*tangent = tn;
						}
						(*tangent).normalize3fast();
					}

					if (tex_coord != NULL)
					{
						*tex_coord = tc;
					}
					
					ret = TRUE;
				}
			}
		}
	}
		
	return ret;
}

bool LLVOVolume::treatAsRigged()
{
	return isSelected() &&
        (isAttachment() || isAnimatedObject()) &&
        mDrawable.notNull() &&
        mDrawable->isState(LLDrawable::RIGGED);
}

LLRiggedVolume* LLVOVolume::getRiggedVolume()
{
	return mRiggedVolume;
}

void LLVOVolume::clearRiggedVolume()
{
	if (mRiggedVolume.notNull())
	{
		mRiggedVolume = NULL;
		updateRelativeXform();
	}
}

void LLVOVolume::updateRiggedVolume(bool force_update)
{
	//Update mRiggedVolume to match current animation frame of avatar. 
	//Also update position/size in octree.  

	if ((!force_update) && (!treatAsRigged()))
	{
		clearRiggedVolume();
		
		return;
	}

	LLVolume* volume = getVolume();
	const LLMeshSkinInfo* skin = getSkinInfo();
	if (!skin)
	{
		clearRiggedVolume();
		return;
	}

	LLVOAvatar* avatar = getAvatar();
	if (!avatar)
	{
		clearRiggedVolume();
		return;
	}

	if (!mRiggedVolume)
	{
		LLVolumeParams p;
		mRiggedVolume = new LLRiggedVolume(p);
		updateRelativeXform();
	}

	mRiggedVolume->update(skin, avatar, volume);
}

static LLTrace::BlockTimerStatHandle FTM_SKIN_RIGGED("Skin");
static LLTrace::BlockTimerStatHandle FTM_RIGGED_OCTREE("Octree");

void LLRiggedVolume::update(const LLMeshSkinInfo* skin, LLVOAvatar* avatar, const LLVolume* volume)
{
	bool copy = false;
	if (volume->getNumVolumeFaces() != getNumVolumeFaces())
	{ 
		copy = true;
	}

	for (S32 i = 0; i < volume->getNumVolumeFaces() && !copy; ++i)
	{
		const LLVolumeFace& src_face = volume->getVolumeFace(i);
		const LLVolumeFace& dst_face = getVolumeFace(i);

		if (src_face.mNumIndices != dst_face.mNumIndices ||
			src_face.mNumVertices != dst_face.mNumVertices)
		{
			copy = true;
		}
	}

	if (copy)
	{
		copyVolumeFaces(volume);	
	}

	//build matrix palette
	static const size_t kMaxJoints = LL_MAX_JOINTS_PER_MESH_OBJECT;

	LLMatrix4a mat[kMaxJoints];
	U32 maxJoints = LLSkinningUtil::getMeshJointCount(skin);
    LLSkinningUtil::initSkinningMatrixPalette((LLMatrix4*)mat, maxJoints, skin, avatar);

    S32 rigged_vert_count = 0;
    S32 rigged_face_count = 0;
    LLVector4a box_min, box_max;
	for (S32 i = 0; i < volume->getNumVolumeFaces(); ++i)
	{
		const LLVolumeFace& vol_face = volume->getVolumeFace(i);
		
		LLVolumeFace& dst_face = mVolumeFaces[i];
		
		LLVector4a* weight = vol_face.mWeights;

		if ( weight )
		{
            LLSkinningUtil::checkSkinWeights(weight, dst_face.mNumVertices, skin);
			LLMatrix4a bind_shape_matrix;
			bind_shape_matrix.loadu(skin->mBindShapeMatrix);

			LLVector4a* pos = dst_face.mPositions;

			if( pos && weight && dst_face.mExtents )
			{
				LL_RECORD_BLOCK_TIME(FTM_SKIN_RIGGED);

                U32 max_joints = LLSkinningUtil::getMaxJointCount();
                rigged_vert_count += dst_face.mNumVertices;
                rigged_face_count++;
				for (U32 j = 0; j < dst_face.mNumVertices; ++j)
				{
					LLMatrix4a final_mat;
                    LLSkinningUtil::getPerVertexSkinMatrix(weight[j].getF32ptr(), mat, false, final_mat, max_joints);
				
					LLVector4a& v = vol_face.mPositions[j];
					LLVector4a t;
					LLVector4a dst;
					bind_shape_matrix.affineTransform(v, t);
					final_mat.affineTransform(t, dst);
					pos[j] = dst;
				}

				//update bounding box
				LLVector4a& min = dst_face.mExtents[0];
				LLVector4a& max = dst_face.mExtents[1];

				min = pos[0];
				max = pos[1];
                if (i==0)
                {
                    box_min = min;
                    box_max = max;
                }

				for (U32 j = 1; j < dst_face.mNumVertices; ++j)
				{
					min.setMin(min, pos[j]);
					max.setMax(max, pos[j]);
				}
                box_min.setMin(min,box_min);
                box_max.setMax(max,box_max);

				dst_face.mCenter->setAdd(dst_face.mExtents[0], dst_face.mExtents[1]);
				dst_face.mCenter->mul(0.5f);

			}

			{
				LL_RECORD_BLOCK_TIME(FTM_RIGGED_OCTREE);
				delete dst_face.mOctree;
				dst_face.mOctree = NULL;

				LLVector4a size;
				size.setSub(dst_face.mExtents[1], dst_face.mExtents[0]);
				size.splat(size.getLength3().getF32()*0.5f);
			
				dst_face.createOctree(1.f);
			}
		}
	}
    mExtraDebugText = llformat("rigged %d/%d - box (%f %f %f) (%f %f %f)",
                               rigged_face_count, rigged_vert_count,
                               box_min[0], box_min[1], box_min[2],
                               box_max[0], box_max[1], box_max[2]);
}

U32 LLVOVolume::getPartitionType() const
{
	if (isHUDAttachment())
	{
		return LLViewerRegion::PARTITION_HUD;
	}

	return LLViewerRegion::PARTITION_VOLUME;
}

LLVolumePartition::LLVolumePartition(LLViewerRegion* regionp)
: LLSpatialPartition(LLVOVolume::VERTEX_DATA_MASK, TRUE, GL_DYNAMIC_DRAW_ARB, regionp),
LLVolumeGeometryManager()
{
	mLODPeriod = 32;
	mDepthMask = FALSE;
	mDrawableType = LLPipeline::RENDER_TYPE_VOLUME;
	mPartitionType = LLViewerRegion::PARTITION_VOLUME;
	mSlopRatio = 0.25f;
	mBufferUsage = GL_DYNAMIC_DRAW_ARB;
}

LLVolumeBridge::LLVolumeBridge(LLDrawable* drawablep, LLViewerRegion* regionp)
: LLSpatialBridge(drawablep, TRUE, LLVOVolume::VERTEX_DATA_MASK, regionp),
LLVolumeGeometryManager()
{
	mDepthMask = FALSE;
	mLODPeriod = 32;
	mDrawableType = LLPipeline::RENDER_TYPE_VOLUME;
	mPartitionType = LLViewerRegion::PARTITION_BRIDGE;
	
	mBufferUsage = GL_DYNAMIC_DRAW_ARB;

	mSlopRatio = 0.25f;
}

bool can_batch_texture(LLFace* facep)
{
	if (facep->getTextureEntry()->getBumpmap())
	{ //bump maps aren't worked into texture batching yet
		return false;
	}

	if (facep->getTextureEntry()->getMaterialParams().notNull())
	{ //materials don't work with texture batching yet
		return false;
	}

	if (facep->getTexture() && facep->getTexture()->getPrimaryFormat() == GL_ALPHA)
	{ //can't batch invisiprims
		return false;
	}

	if (facep->isState(LLFace::TEXTURE_ANIM) && facep->getVirtualSize() > MIN_TEX_ANIM_SIZE)
	{ //texture animation breaks batches
		return false;
	}
	
	return true;
}

const static U32 MAX_FACE_COUNT = 4096U;
int32_t LLVolumeGeometryManager::sInstanceCount = 0;
LLFace** LLVolumeGeometryManager::sFullbrightFaces = NULL;
LLFace** LLVolumeGeometryManager::sBumpFaces = NULL;
LLFace** LLVolumeGeometryManager::sSimpleFaces = NULL;
LLFace** LLVolumeGeometryManager::sNormFaces = NULL;
LLFace** LLVolumeGeometryManager::sSpecFaces = NULL;
LLFace** LLVolumeGeometryManager::sNormSpecFaces = NULL;
LLFace** LLVolumeGeometryManager::sAlphaFaces = NULL;

LLVolumeGeometryManager::LLVolumeGeometryManager()
	: LLGeometryManager()
{
	llassert(sInstanceCount >= 0);
	if (sInstanceCount == 0)
	{
		allocateFaces(MAX_FACE_COUNT);
	}

	++sInstanceCount;
}

LLVolumeGeometryManager::~LLVolumeGeometryManager()
{
	llassert(sInstanceCount > 0);
	--sInstanceCount;

	if (sInstanceCount <= 0)
	{
		freeFaces();
		sInstanceCount = 0;
	}
}

void LLVolumeGeometryManager::allocateFaces(U32 pMaxFaceCount)
{
	sFullbrightFaces = static_cast<LLFace**>(ll_aligned_malloc<64>(pMaxFaceCount*sizeof(LLFace*)));
	sBumpFaces = static_cast<LLFace**>(ll_aligned_malloc<64>(pMaxFaceCount*sizeof(LLFace*)));
	sSimpleFaces = static_cast<LLFace**>(ll_aligned_malloc<64>(pMaxFaceCount*sizeof(LLFace*)));
	sNormFaces = static_cast<LLFace**>(ll_aligned_malloc<64>(pMaxFaceCount*sizeof(LLFace*)));
	sSpecFaces = static_cast<LLFace**>(ll_aligned_malloc<64>(pMaxFaceCount*sizeof(LLFace*)));
	sNormSpecFaces = static_cast<LLFace**>(ll_aligned_malloc<64>(pMaxFaceCount*sizeof(LLFace*)));
	sAlphaFaces = static_cast<LLFace**>(ll_aligned_malloc<64>(pMaxFaceCount*sizeof(LLFace*)));
}

void LLVolumeGeometryManager::freeFaces()
{
	ll_aligned_free<64>(sFullbrightFaces);
	ll_aligned_free<64>(sBumpFaces);
	ll_aligned_free<64>(sSimpleFaces);
	ll_aligned_free<64>(sNormFaces);
	ll_aligned_free<64>(sSpecFaces);
	ll_aligned_free<64>(sNormSpecFaces);
	ll_aligned_free<64>(sAlphaFaces);

	sFullbrightFaces = NULL;
	sBumpFaces = NULL;
	sSimpleFaces = NULL;
	sNormFaces = NULL;
	sSpecFaces = NULL;
	sNormSpecFaces = NULL;
	sAlphaFaces = NULL;
}

static LLTrace::BlockTimerStatHandle FTM_REGISTER_FACE("Register Face");

void LLVolumeGeometryManager::registerFace(LLSpatialGroup* group, LLFace* facep, U32 type)
{
	LL_RECORD_BLOCK_TIME(FTM_REGISTER_FACE);
	if (type == LLRenderPass::PASS_ALPHA && facep->getTextureEntry()->getMaterialParams().notNull() && !facep->getVertexBuffer()->hasDataType(LLVertexBuffer::TYPE_TANGENT))
	{
		LL_WARNS_ONCE("RenderMaterials") << "Oh no! No binormals for this alpha blended face!" << LL_ENDL;
	}

	if (facep->getViewerObject()->isSelected() && LLSelectMgr::getInstance()->mHideSelectedObjects)
	{
		return;
	}

	//add face to drawmap
	LLSpatialGroup::drawmap_elem_t& draw_vec = group->mDrawMap[type];	

	S32 idx = draw_vec.size()-1;

	BOOL fullbright = (type == LLRenderPass::PASS_FULLBRIGHT) ||
		(type == LLRenderPass::PASS_INVISIBLE) ||
		(type == LLRenderPass::PASS_FULLBRIGHT_ALPHA_MASK) ||
		(type == LLRenderPass::PASS_ALPHA && facep->isState(LLFace::FULLBRIGHT)) ||
		(facep->getTextureEntry()->getFullbright());
	
	if (!fullbright && type != LLRenderPass::PASS_GLOW && !facep->getVertexBuffer()->hasDataType(LLVertexBuffer::TYPE_NORMAL))
	{
		LL_WARNS() << "Non fullbright face has no normals!" << LL_ENDL;
		return;
	}

	const LLMatrix4* tex_mat = NULL;
	if (facep->isState(LLFace::TEXTURE_ANIM) && facep->getVirtualSize() > MIN_TEX_ANIM_SIZE)
	{
		tex_mat = facep->mTextureMatrix;	
	}

	const LLMatrix4* model_mat = NULL;

	LLDrawable* drawable = facep->getDrawable();
	
	if (drawable->isState(LLDrawable::ANIMATED_CHILD))
	{
		model_mat = &drawable->getWorldMatrix();
	}
	else if (drawable->isActive())
	{
		model_mat = &drawable->getRenderMatrix();
	}
	else
	{
		model_mat = &(drawable->getRegion()->mRenderMatrix);
	}

	//drawable->getVObj()->setDebugText(llformat("%d", drawable->isState(LLDrawable::ANIMATED_CHILD)));

	U8 bump = (type == LLRenderPass::PASS_BUMP || type == LLRenderPass::PASS_POST_BUMP) ? facep->getTextureEntry()->getBumpmap() : 0;
	U8 shiny = facep->getTextureEntry()->getShiny();
	
	LLViewerTexture* tex = facep->getTexture();

	U8 index = facep->getTextureIndex();

	LLMaterial* mat = facep->getTextureEntry()->getMaterialParams().get(); 
	LLMaterialID mat_id = facep->getTextureEntry()->getMaterialID();

	bool batchable = false;

	U32 shader_mask = 0xFFFFFFFF; //no shader

	if (mat)
	{
		if (type == LLRenderPass::PASS_ALPHA)
		{
			shader_mask = mat->getShaderMask(LLMaterial::DIFFUSE_ALPHA_MODE_BLEND);
		}
		else
		{
			shader_mask = mat->getShaderMask();
		}
	}


	if (index < 255 && idx >= 0)
	{
		if (mat || draw_vec[idx]->mMaterial)
		{ //can't batch textures when materials are present (yet)
			batchable = false;
		}
		else if (index < draw_vec[idx]->mTextureList.size())
		{
			if (draw_vec[idx]->mTextureList[index].isNull())
			{
				batchable = true;
				draw_vec[idx]->mTextureList[index] = tex;
			}
			else if (draw_vec[idx]->mTextureList[index] == tex)
			{ //this face's texture index can be used with this batch
				batchable = true;
			}
		}
		else
		{ //texture list can be expanded to fit this texture index
			batchable = true;
		}
	}
	
	if (idx >= 0 && 
		draw_vec[idx]->mVertexBuffer == facep->getVertexBuffer() &&
		draw_vec[idx]->mEnd == facep->getGeomIndex()-1 &&
		(LLPipeline::sTextureBindTest || draw_vec[idx]->mTexture == tex || batchable) &&
#if LL_DARWIN
		draw_vec[idx]->mEnd - draw_vec[idx]->mStart + facep->getGeomCount() <= (U32) gGLManager.mGLMaxVertexRange &&
		draw_vec[idx]->mCount + facep->getIndicesCount() <= (U32) gGLManager.mGLMaxIndexRange &&
#endif
		draw_vec[idx]->mMaterial == mat &&
		draw_vec[idx]->mMaterialID == mat_id &&
		draw_vec[idx]->mFullbright == fullbright &&
		draw_vec[idx]->mBump == bump &&
		(!mat || (draw_vec[idx]->mShiny == shiny)) && // need to break batches when a material is shared, but legacy settings are different
		draw_vec[idx]->mTextureMatrix == tex_mat &&
		draw_vec[idx]->mModelMatrix == model_mat &&
		draw_vec[idx]->mShaderMask == shader_mask)
	{
		draw_vec[idx]->mCount += facep->getIndicesCount();
		draw_vec[idx]->mEnd += facep->getGeomCount();
		draw_vec[idx]->mVSize = llmax(draw_vec[idx]->mVSize, facep->getVirtualSize());

		if (index < 255 && index >= draw_vec[idx]->mTextureList.size())
		{
			draw_vec[idx]->mTextureList.resize(index+1);
			draw_vec[idx]->mTextureList[index] = tex;
		}
		draw_vec[idx]->validate();
		update_min_max(draw_vec[idx]->mExtents[0], draw_vec[idx]->mExtents[1], facep->mExtents[0]);
		update_min_max(draw_vec[idx]->mExtents[0], draw_vec[idx]->mExtents[1], facep->mExtents[1]);
	}
	else
	{
		U32 start = facep->getGeomIndex();
		U32 end = start + facep->getGeomCount()-1;
		U32 offset = facep->getIndicesStart();
		U32 count = facep->getIndicesCount();
		LLPointer<LLDrawInfo> draw_info = new LLDrawInfo(start,end,count,offset, tex, 
			facep->getVertexBuffer(), fullbright, bump); 
		draw_info->mGroup = group;
		draw_info->mVSize = facep->getVirtualSize();
		draw_vec.push_back(draw_info);
		draw_info->mTextureMatrix = tex_mat;
		draw_info->mModelMatrix = model_mat;
		
		draw_info->mBump  = bump;
		draw_info->mShiny = shiny;

		static const float alpha[4] =
		{
			0.00f,
			0.25f,
			0.5f,
			0.75f
		};
		float spec = alpha[shiny & TEM_SHINY_MASK];
		LLVector4 specColor(spec, spec, spec, spec);
		draw_info->mSpecColor = specColor;
		draw_info->mEnvIntensity = spec;
		draw_info->mSpecularMap = NULL;
		draw_info->mMaterial = mat;
		draw_info->mShaderMask = shader_mask;

		if (mat)
		{
				draw_info->mMaterialID = mat_id;

				// We have a material.  Update our draw info accordingly.
				
				if (!mat->getSpecularID().isNull())
				{
					LLVector4 specColor;
					specColor.mV[0] = mat->getSpecularLightColor().mV[0] * (1.f / 255.f);
					specColor.mV[1] = mat->getSpecularLightColor().mV[1] * (1.f / 255.f);
					specColor.mV[2] = mat->getSpecularLightColor().mV[2] * (1.f / 255.f);
					specColor.mV[3] = mat->getSpecularLightExponent() * (1.f / 255.f);
					draw_info->mSpecColor = specColor;
					draw_info->mEnvIntensity = mat->getEnvironmentIntensity() * (1.f / 255.f);
					draw_info->mSpecularMap = facep->getViewerObject()->getTESpecularMap(facep->getTEOffset());
				}

				draw_info->mAlphaMaskCutoff = mat->getAlphaMaskCutoff() * (1.f / 255.f);
				draw_info->mDiffuseAlphaMode = mat->getDiffuseAlphaMode();
				draw_info->mNormalMap = facep->getViewerObject()->getTENormalMap(facep->getTEOffset());
				
		}
		else 
		{
			if (type == LLRenderPass::PASS_GRASS)
			{
				draw_info->mAlphaMaskCutoff = 0.5f;
			}
			else
			{
				draw_info->mAlphaMaskCutoff = 0.33f;
			}
		}
		
		if (type == LLRenderPass::PASS_ALPHA)
		{ //for alpha sorting
			facep->setDrawInfo(draw_info);
		}
		draw_info->mExtents[0] = facep->mExtents[0];
		draw_info->mExtents[1] = facep->mExtents[1];

		if (LLPipeline::sUseTriStrips)
		{
			draw_info->mDrawMode = LLRender::TRIANGLE_STRIP;
		}

		if (index < 255)
		{ //initialize texture list for texture batching
			draw_info->mTextureList.resize(index+1);
			draw_info->mTextureList[index] = tex;
		}
		draw_info->validate();
	}
}

void LLVolumeGeometryManager::getGeometry(LLSpatialGroup* group)
{

}

static LLTrace::BlockTimerStatHandle FTM_REBUILD_VOLUME_VB("Volume VB");
static LLTrace::BlockTimerStatHandle FTM_REBUILD_VOLUME_FACE_LIST("Build Face List");
static LLTrace::BlockTimerStatHandle FTM_REBUILD_VOLUME_GEN_DRAW_INFO("Gen Draw Info");

static LLDrawPoolAvatar* get_avatar_drawpool(LLViewerObject* vobj)
{
	LLVOAvatar* avatar = vobj->getAvatar();
					
	if (avatar)
	{
		LLDrawable* drawable = avatar->mDrawable;
		if (drawable && drawable->getNumFaces() > 0)
		{
			LLFace* face = drawable->getFace(0);
			if (face)
			{
				LLDrawPool* drawpool = face->getPool();
				if (drawpool)
				{
					if (drawpool->getType() == LLDrawPool::POOL_AVATAR)
					{
						return (LLDrawPoolAvatar*) drawpool;
					}
				}
			}
		}
	}

	return NULL;
}

void LLVolumeGeometryManager::rebuildGeom(LLSpatialGroup* group)
{
	if (group->changeLOD())
	{
		group->mLastUpdateDistance = group->mDistance;
	}

	group->mLastUpdateViewAngle = group->mViewAngle;

	if (!group->hasState(LLSpatialGroup::GEOM_DIRTY | LLSpatialGroup::ALPHA_DIRTY))
	{
		if (group->hasState(LLSpatialGroup::MESH_DIRTY) && !LLPipeline::sDelayVBUpdate)
		{
			rebuildMesh(group);
		}
		return;
	}

	LL_RECORD_BLOCK_TIME(FTM_REBUILD_VOLUME_VB);

	group->mBuilt = 1.f;
	
	LLVOAvatar *rigged_av = NULL;
    LLVOAvatar *attached_av = NULL;

	LLSpatialBridge* bridge = group->getSpatialPartition()->asBridge();
    LLViewerObject *vobj = NULL;
    LLVOVolume *vol_obj = NULL;

	if (bridge)
	{
        vobj = bridge->mDrawable->getVObj();
        vol_obj = dynamic_cast<LLVOVolume*>(vobj);
		if (bridge->mAvatar.isNull())
		{
			if (vobj)
			{
				bridge->mAvatar = vobj->getAvatar();
			}
		}
        rigged_av = bridge->mAvatar;
	}
    if (vobj)
    {
        attached_av = vobj->getAvatarAncestor();
    }

    if (attached_av)
    {
        attached_av->subtractAttachmentArea( group->mSurfaceArea );
    }
    if (rigged_av && (rigged_av != attached_av))
    {
        rigged_av->subtractAttachmentArea( group->mSurfaceArea );
    }
    if (vol_obj)
    {
        vol_obj->updateVisualComplexity();
    }

	group->mGeometryBytes = 0;
	group->mSurfaceArea = 0;
	
	//cache object box size since it might be used for determining visibility
	const LLVector4a* bounds = group->getObjectBounds();
	group->mObjectBoxSize = bounds[1].getLength3().getF32();

	group->clearDrawMap();

	mFaceList.clear();

	U32 fullbright_count = 0;
	U32 bump_count = 0;
	U32 simple_count = 0;
	U32 alpha_count = 0;
	U32 norm_count = 0;
	U32 spec_count = 0;
	U32 normspec_count = 0;


	U32 useage = group->getSpatialPartition()->mBufferUsage;

	U32 max_vertices = (gSavedSettings.getS32("RenderMaxVBOSize")*1024)/LLVertexBuffer::calcVertexSize(group->getSpatialPartition()->mVertexDataMask);
	U32 max_total = (gSavedSettings.getS32("RenderMaxNodeSize")*1024)/LLVertexBuffer::calcVertexSize(group->getSpatialPartition()->mVertexDataMask);
	max_vertices = llmin(max_vertices, (U32) 65535);

	U32 cur_total = 0;

	bool emissive = false;

	//Determine if we've received skininfo that contains an
	//alternate bind matrix - if it does then apply the translational component
	//to the joints of the avatar.
#if 0
	bool pelvisGotSet = false;
#endif

	{
		LL_RECORD_BLOCK_TIME(FTM_REBUILD_VOLUME_FACE_LIST);

		//get all the faces into a list
		for (LLSpatialGroup::element_iter drawable_iter = group->getDataBegin(); 
             drawable_iter != group->getDataEnd(); ++drawable_iter)
		{
			LLDrawable* drawablep = (LLDrawable*)(*drawable_iter)->getDrawable();
		
			if (!drawablep || drawablep->isDead() || drawablep->isState(LLDrawable::FORCE_INVISIBLE) )
			{
				continue;
			}
	
			if (drawablep->isAnimating())
			{ //fall back to stream draw for animating verts
				useage = GL_STREAM_DRAW_ARB;
			}

			LLVOVolume* vobj = drawablep->getVOVolume();
            
			if (!vobj)
			{
				continue;
			}

            std::string vobj_name = llformat("Vol%u", (U32) vobj);

			if (vobj->isMesh() &&
				((vobj->getVolume() && !vobj->getVolume()->isMeshAssetLoaded()) || !gMeshRepo.meshRezEnabled()))
			{
				continue;
			}

			LLVolume* volume = vobj->getVolume();
			if (volume)
			{
				const LLVector3& scale = vobj->getScale();
				group->mSurfaceArea += volume->getSurfaceArea() * llmax(llmax(scale.mV[0], scale.mV[1]), scale.mV[2]);
			}

            bool is_mesh = vobj->isMesh();
            F32 est_tris = vobj->getEstTrianglesMax();

#if 1
            LL_DEBUGS("AXONLinkset") << vobj_name << " rebuilding, isAttachment: " << (U32) vobj->isAttachment()
                              << " is_mesh " << is_mesh
                              << " est_tris " << est_tris
                              << " is_animated " << vobj->isAnimatedObject()
                              << " can_animate " << vobj->canBeAnimatedObject() 
                              << " cav " << vobj->getControlAvatar() 
                              << " playing " << (U32) (vobj->getControlAvatar() ? vobj->getControlAvatar()->mPlaying : false)
                              << " frame " << LLFrameTimer::getFrameCount()
                              << LL_ENDL;
#endif

			llassert_always(vobj);
			vobj->updateTextureVirtualSize(true);
			vobj->preRebuild();

			drawablep->clearState(LLDrawable::HAS_ALPHA);

            // Standard rigged mesh attachments: 
			bool rigged = !vobj->isAnimatedObject() && vobj->isRiggedMesh() && vobj->isAttachment();
            // Animated objects. Have to check for isRiggedMesh() to
            // exclude static objects in animated object linksets.
			rigged = rigged || (vobj->isAnimatedObject() && vobj->isRiggedMesh() &&
                vobj->getControlAvatar() && vobj->getControlAvatar()->mPlaying);

            if (vobj->isAnimatedObject())
            {
                if (!vobj->getControlAvatar())
                {
                    F32 tri_count = vobj->getRootEdit()->recursiveGetEstTrianglesMax();
                    if (tri_count <= 0.f)
                    {
                        LL_DEBUGS("AXON") << vobj_name << " not calling linkControlAvatar(), because no tris" << LL_ENDL;
                    }
                    else
                    {
                        LL_DEBUGS("AXON") << vobj_name << " calling linkControlAvatar()" << LL_ENDL;
                        vobj->linkControlAvatar();
                    }
                }
                if (vobj->getControlAvatar())
                {
                    rigged_av = vobj->getControlAvatar();
                    rigged_av->rebuildAttachmentOverrides();
                }
            }
            else
            {
                // Not animated but has a control avatar - probably
                // the checkbox has changed since the last rebuild.
                if (vobj->getControlAvatar())
                {
                    LL_DEBUGS("AXON") << vobj_name << " calling unlinkControlAvatar()" << LL_ENDL;
                    vobj->unlinkControlAvatar();
                }
            }

			bool bake_sunlight = LLPipeline::sBakeSunlight && drawablep->isStatic();

            // AXON why this variable? Only different from rigged if
            // there are no LLFaces associated with the drawable.
			bool is_rigged = false;

            if (rigged && rigged_av)
            {
                // AXON don't we want to do this for standalone animesh as well?
                rigged_av->addAttachmentOverridesForObject(vobj);
				if (!LLApp::isExiting() && rigged_av->isSelf() && debugLoggingEnabled("AvatarAttachments"))
                {
                    bool verbose = true;
					rigged_av->showAttachmentOverrides(verbose);
				}
            }

			//for each face
			for (S32 i = 0; i < drawablep->getNumFaces(); i++)
			{
				LLFace* facep = drawablep->getFace(i);
				if (!facep)
				{
					continue;
				}

				//ALWAYS null out vertex buffer on rebuild -- if the face lands in a render
				// batch, it will recover its vertex buffer reference from the spatial group
				facep->setVertexBuffer(NULL);
			
				//sum up face verts and indices
				drawablep->updateFaceSize(i);
			
				if (rigged)
				{
					if (!facep->isState(LLFace::RIGGED))
					{ //completely reset vertex buffer
						facep->clearVertexBuffer();
					}
		
					facep->setState(LLFace::RIGGED);
					is_rigged = true;
				
					//get drawpool of avatar with rigged face
					LLDrawPoolAvatar* pool = get_avatar_drawpool(vobj);				
					
					if (pool)
					{
						const LLTextureEntry* te = facep->getTextureEntry();

						//remove face from old pool if it exists
						LLDrawPool* old_pool = facep->getPool();
						if (old_pool && old_pool->getType() == LLDrawPool::POOL_AVATAR)
						{
							((LLDrawPoolAvatar*) old_pool)->removeRiggedFace(facep);
						}

						//add face to new pool
						LLViewerTexture* tex = facep->getTexture();
						U32 type = gPipeline.getPoolTypeFromTE(te, tex);


						if (te->getGlow())
						{
							pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_GLOW);
						}

						LLMaterial* mat = te->getMaterialParams().get();

						if (mat && LLPipeline::sRenderDeferred)
						{
							U8 alpha_mode = mat->getDiffuseAlphaMode();

							bool is_alpha = type == LLDrawPool::POOL_ALPHA &&
								(alpha_mode == LLMaterial::DIFFUSE_ALPHA_MODE_BLEND ||
								te->getColor().mV[3] < 0.999f);

							if (is_alpha)
							{ //this face needs alpha blending, override alpha mode
								alpha_mode = LLMaterial::DIFFUSE_ALPHA_MODE_BLEND;
							}

							if (!is_alpha || te->getColor().mV[3] > 0.f)  // //only add the face if it will actually be visible
							{ 
								U32 mask = mat->getShaderMask(alpha_mode);
								pool->addRiggedFace(facep, mask);
							}
						}
						else if (mat)
						{
							bool fullbright = te->getFullbright();
							bool is_alpha = type == LLDrawPool::POOL_ALPHA;
							U8 mode = mat->getDiffuseAlphaMode();
							bool can_be_shiny = mode == LLMaterial::DIFFUSE_ALPHA_MODE_NONE ||
												mode == LLMaterial::DIFFUSE_ALPHA_MODE_EMISSIVE;
							
							if (mode == LLMaterial::DIFFUSE_ALPHA_MODE_MASK && te->getColor().mV[3] >= 0.999f)
							{
								pool->addRiggedFace(facep, fullbright ? LLDrawPoolAvatar::RIGGED_FULLBRIGHT : LLDrawPoolAvatar::RIGGED_SIMPLE);
							}
							else if (is_alpha || (te->getColor().mV[3] < 0.999f))
							{
								if (te->getColor().mV[3] > 0.f)
								{
									pool->addRiggedFace(facep, fullbright ? LLDrawPoolAvatar::RIGGED_FULLBRIGHT_ALPHA : LLDrawPoolAvatar::RIGGED_ALPHA);
								}
							}
							else if (gPipeline.canUseVertexShaders()
								&& LLPipeline::sRenderBump 
								&& te->getShiny() 
								&& can_be_shiny)
							{
								pool->addRiggedFace(facep, fullbright ? LLDrawPoolAvatar::RIGGED_FULLBRIGHT_SHINY : LLDrawPoolAvatar::RIGGED_SHINY);
							}
							else
							{
								pool->addRiggedFace(facep, fullbright ? LLDrawPoolAvatar::RIGGED_FULLBRIGHT : LLDrawPoolAvatar::RIGGED_SIMPLE);
							}
						}
						else
						{
						if (type == LLDrawPool::POOL_ALPHA)
						{
							if (te->getColor().mV[3] > 0.f)
							{
								if (te->getFullbright())
								{
									pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_FULLBRIGHT_ALPHA);
								}
								else
								{
									pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_ALPHA);
								}
							}
						}
						else if (te->getShiny())
						{
							if (te->getFullbright())
							{
								pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_FULLBRIGHT_SHINY);
							}
							else
							{
								if (LLPipeline::sRenderDeferred)
								{
									pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_SIMPLE);
								}
								else
								{
									pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_SHINY);
								}
							}
						}
						else
						{
							if (te->getFullbright())
							{
								pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_FULLBRIGHT);
							}
							else
							{
								pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_SIMPLE);
							}
						}


						if (LLPipeline::sRenderDeferred)
						{
							if (type != LLDrawPool::POOL_ALPHA && !te->getFullbright())
							{
								if (te->getBumpmap())
								{
									pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_DEFERRED_BUMP);
								}
								else
								{
									pool->addRiggedFace(facep, LLDrawPoolAvatar::RIGGED_DEFERRED_SIMPLE);
								}
							}
						}
					}
					}

					continue;
				}
				else
				{
					if (facep->isState(LLFace::RIGGED))
					{ //face is not rigged but used to be, remove from rigged face pool
						LLDrawPoolAvatar* pool = (LLDrawPoolAvatar*) facep->getPool();
						if (pool)
						{
							pool->removeRiggedFace(facep);
						}
						facep->clearState(LLFace::RIGGED);
					}
				}


				if (cur_total > max_total || facep->getIndicesCount() <= 0 || facep->getGeomCount() <= 0)
				{
					facep->clearVertexBuffer();
					continue;
				}

				cur_total += facep->getGeomCount();

				if (facep->hasGeometry() && facep->getPixelArea() > FORCE_CULL_AREA)
				{
					const LLTextureEntry* te = facep->getTextureEntry();
					LLViewerTexture* tex = facep->getTexture();

					if (te->getGlow() >= 1.f/255.f)
					{
						emissive = true;
					}

					if (facep->isState(LLFace::TEXTURE_ANIM))
					{
						if (!vobj->mTexAnimMode)
						{
							facep->clearState(LLFace::TEXTURE_ANIM);
						}
					}

					BOOL force_simple = (facep->getPixelArea() < FORCE_SIMPLE_RENDER_AREA);
					U32 type = gPipeline.getPoolTypeFromTE(te, tex);
					if (type != LLDrawPool::POOL_ALPHA && force_simple)
					{
						type = LLDrawPool::POOL_SIMPLE;
					}
					facep->setPoolType(type);

					if (vobj->isHUDAttachment())
					{
						facep->setState(LLFace::FULLBRIGHT);
					}

					if (vobj->mTextureAnimp && vobj->mTexAnimMode)
					{
						if (vobj->mTextureAnimp->mFace <= -1)
						{
							S32 face;
							for (face = 0; face < vobj->getNumTEs(); face++)
							{
								LLFace * facep = drawablep->getFace(face);
								if (facep)
								{
									facep->setState(LLFace::TEXTURE_ANIM);
								}
							}
						}
						else if (vobj->mTextureAnimp->mFace < vobj->getNumTEs())
						{
							LLFace * facep = drawablep->getFace(vobj->mTextureAnimp->mFace);
							if (facep)
							{
								facep->setState(LLFace::TEXTURE_ANIM);
							}
						}
					}

					if (type == LLDrawPool::POOL_ALPHA)
					{
						if (facep->canRenderAsMask())
						{ //can be treated as alpha mask
							if (simple_count < MAX_FACE_COUNT)
							{
								sSimpleFaces[simple_count++] = facep;
							}
						}
						else
						{
							if (te->getColor().mV[3] > 0.f)
							{ //only treat as alpha in the pipeline if < 100% transparent
								drawablep->setState(LLDrawable::HAS_ALPHA);
							}
							if (alpha_count < MAX_FACE_COUNT)
							{
								sAlphaFaces[alpha_count++] = facep;
							}
						}
					}
					else
					{
						if (drawablep->isState(LLDrawable::REBUILD_VOLUME))
						{
							facep->mLastUpdateTime = gFrameTimeSeconds;
						}

						if (gPipeline.canUseWindLightShadersOnObjects()
							&& LLPipeline::sRenderBump)
						{
							if (LLPipeline::sRenderDeferred && te->getMaterialParams().notNull()  && !te->getMaterialID().isNull())
							{
								LLMaterial* mat = te->getMaterialParams().get();
								if (mat->getNormalID().notNull())
								{
									if (mat->getSpecularID().notNull())
									{ //has normal and specular maps (needs texcoord1, texcoord2, and tangent)
										if (normspec_count < MAX_FACE_COUNT)
										{
											sNormSpecFaces[normspec_count++] = facep;
										}
									}
									else
									{ //has normal map (needs texcoord1 and tangent)
										if (norm_count < MAX_FACE_COUNT)
										{
											sNormFaces[norm_count++] = facep;
										}
									}
								}
								else if (mat->getSpecularID().notNull())
								{ //has specular map but no normal map, needs texcoord2
									if (spec_count < MAX_FACE_COUNT)
									{
										sSpecFaces[spec_count++] = facep;
									}
								}
								else
								{ //has neither specular map nor normal map, only needs texcoord0
									if (simple_count < MAX_FACE_COUNT)
									{
										sSimpleFaces[simple_count++] = facep;
									}
								}									
							}
							else if (te->getBumpmap())
							{ //needs normal + tangent
								if (bump_count < MAX_FACE_COUNT)
								{
									sBumpFaces[bump_count++] = facep;
								}
							}
							else if (te->getShiny() || !te->getFullbright())
							{ //needs normal
								if (simple_count < MAX_FACE_COUNT)
								{
									sSimpleFaces[simple_count++] = facep;
								}
							}
							else 
							{ //doesn't need normal
								facep->setState(LLFace::FULLBRIGHT);
								if (fullbright_count < MAX_FACE_COUNT)
								{
									sFullbrightFaces[fullbright_count++] = facep;
								}
							}
						}
						else
						{
							if (te->getBumpmap() && LLPipeline::sRenderBump)
							{ //needs normal + tangent
								if (bump_count < MAX_FACE_COUNT)
								{
									sBumpFaces[bump_count++] = facep;
								}
							}
							else if ((te->getShiny() && LLPipeline::sRenderBump) ||
								!(te->getFullbright() || bake_sunlight))
							{ //needs normal
								if (simple_count < MAX_FACE_COUNT)
								{
									sSimpleFaces[simple_count++] = facep;
								}
							}
							else 
							{ //doesn't need normal
								facep->setState(LLFace::FULLBRIGHT);
								if (fullbright_count < MAX_FACE_COUNT)
								{
									sFullbrightFaces[fullbright_count++] = facep;
								}
							}
						}
					}
				}
				else
				{	//face has no renderable geometry
					facep->clearVertexBuffer();
				}		
			}
			
			if (is_rigged)
			{
				if (!drawablep->isState(LLDrawable::RIGGED))
				{
					drawablep->setState(LLDrawable::RIGGED);

					//first time this is drawable is being marked as rigged,
					// do another LoD update to use avatar bounding box
					vobj->updateLOD();
				}
			}
			else
			{
				drawablep->clearState(LLDrawable::RIGGED);
                vobj->updateRiggedVolume();
			}
		}
	}

	group->mBufferUsage = useage;

	//PROCESS NON-ALPHA FACES
	U32 simple_mask = LLVertexBuffer::MAP_TEXCOORD0 | LLVertexBuffer::MAP_NORMAL | LLVertexBuffer::MAP_VERTEX | LLVertexBuffer::MAP_COLOR;
	U32 alpha_mask = simple_mask | 0x80000000; //hack to give alpha verts their own VBO
	U32 bump_mask = LLVertexBuffer::MAP_TEXCOORD0 | LLVertexBuffer::MAP_TEXCOORD1 | LLVertexBuffer::MAP_NORMAL | LLVertexBuffer::MAP_VERTEX | LLVertexBuffer::MAP_COLOR;
	U32 fullbright_mask = LLVertexBuffer::MAP_TEXCOORD0 | LLVertexBuffer::MAP_VERTEX | LLVertexBuffer::MAP_COLOR;

	U32 norm_mask = simple_mask | LLVertexBuffer::MAP_TEXCOORD1 | LLVertexBuffer::MAP_TANGENT;
	U32 normspec_mask = norm_mask | LLVertexBuffer::MAP_TEXCOORD2;
	U32 spec_mask = simple_mask | LLVertexBuffer::MAP_TEXCOORD2;

	if (emissive)
	{ //emissive faces are present, include emissive byte to preserve batching
		simple_mask = simple_mask | LLVertexBuffer::MAP_EMISSIVE;
		alpha_mask = alpha_mask | LLVertexBuffer::MAP_EMISSIVE;
		bump_mask = bump_mask | LLVertexBuffer::MAP_EMISSIVE;
		fullbright_mask = fullbright_mask | LLVertexBuffer::MAP_EMISSIVE;
		norm_mask = norm_mask | LLVertexBuffer::MAP_EMISSIVE;
		normspec_mask = normspec_mask | LLVertexBuffer::MAP_EMISSIVE;
		spec_mask = spec_mask | LLVertexBuffer::MAP_EMISSIVE;
	}

	BOOL batch_textures = LLViewerShaderMgr::instance()->getVertexShaderLevel(LLViewerShaderMgr::SHADER_OBJECT) > 1;

	if (batch_textures)
	{
		bump_mask = bump_mask | LLVertexBuffer::MAP_TANGENT;
		simple_mask = simple_mask | LLVertexBuffer::MAP_TEXTURE_INDEX;
		alpha_mask = alpha_mask | LLVertexBuffer::MAP_TEXTURE_INDEX | LLVertexBuffer::MAP_TANGENT | LLVertexBuffer::MAP_TEXCOORD1 | LLVertexBuffer::MAP_TEXCOORD2;
		fullbright_mask = fullbright_mask | LLVertexBuffer::MAP_TEXTURE_INDEX;
	}

	genDrawInfo(group, simple_mask | LLVertexBuffer::MAP_TEXTURE_INDEX, sSimpleFaces, simple_count, FALSE, batch_textures, FALSE);
	genDrawInfo(group, fullbright_mask | LLVertexBuffer::MAP_TEXTURE_INDEX, sFullbrightFaces, fullbright_count, FALSE, batch_textures);
	genDrawInfo(group, alpha_mask | LLVertexBuffer::MAP_TEXTURE_INDEX, sAlphaFaces, alpha_count, TRUE, batch_textures);
	genDrawInfo(group, bump_mask | LLVertexBuffer::MAP_TEXTURE_INDEX, sBumpFaces, bump_count, FALSE, FALSE);
	genDrawInfo(group, norm_mask | LLVertexBuffer::MAP_TEXTURE_INDEX, sNormFaces, norm_count, FALSE, FALSE);
	genDrawInfo(group, spec_mask | LLVertexBuffer::MAP_TEXTURE_INDEX, sSpecFaces, spec_count, FALSE, FALSE);
	genDrawInfo(group, normspec_mask | LLVertexBuffer::MAP_TEXTURE_INDEX, sNormSpecFaces, normspec_count, FALSE, FALSE);

	if (!LLPipeline::sDelayVBUpdate)
	{
		//drawables have been rebuilt, clear rebuild status
		for (LLSpatialGroup::element_iter drawable_iter = group->getDataBegin(); drawable_iter != group->getDataEnd(); ++drawable_iter)
		{
			LLDrawable* drawablep = (LLDrawable*)(*drawable_iter)->getDrawable();
			if(drawablep)
			{
                drawablep->clearState(LLDrawable::REBUILD_ALL);
            }
        }
	}

	group->mLastUpdateTime = gFrameTimeSeconds;
	group->mBuilt = 1.f;
	group->clearState(LLSpatialGroup::GEOM_DIRTY | LLSpatialGroup::ALPHA_DIRTY);

	if (LLPipeline::sDelayVBUpdate)
	{
		group->setState(LLSpatialGroup::MESH_DIRTY | LLSpatialGroup::NEW_DRAWINFO);
	}

	mFaceList.clear();

    if (attached_av)
    {
        attached_av->addAttachmentArea( group->mSurfaceArea );
    }
    if (rigged_av && (rigged_av != attached_av))
    {
        rigged_av->addAttachmentArea( group->mSurfaceArea );
    }
}

static LLTrace::BlockTimerStatHandle FTM_REBUILD_MESH_FLUSH("Flush Mesh");

void LLVolumeGeometryManager::rebuildMesh(LLSpatialGroup* group)
{
	llassert(group);
	if (group && group->hasState(LLSpatialGroup::MESH_DIRTY) && !group->hasState(LLSpatialGroup::GEOM_DIRTY))
	{
		LL_RECORD_BLOCK_TIME(FTM_REBUILD_VOLUME_VB);
		LL_RECORD_BLOCK_TIME(FTM_REBUILD_VOLUME_GEN_DRAW_INFO); //make sure getgeometryvolume shows up in the right place in timers

		group->mBuilt = 1.f;
		
		S32 num_mapped_vertex_buffer = LLVertexBuffer::sMappedCount ;

		const U32 MAX_BUFFER_COUNT = 4096;
		LLVertexBuffer* locked_buffer[MAX_BUFFER_COUNT];
		
		U32 buffer_count = 0;

		for (LLSpatialGroup::element_iter drawable_iter = group->getDataBegin(); drawable_iter != group->getDataEnd(); ++drawable_iter)
		{
			LLDrawable* drawablep = (LLDrawable*)(*drawable_iter)->getDrawable();

			if (drawablep && !drawablep->isDead() && drawablep->isState(LLDrawable::REBUILD_ALL) && !drawablep->isState(LLDrawable::RIGGED) )
			{
				LLVOVolume* vobj = drawablep->getVOVolume();
#if 1 
                // AXON debugging
                if (vobj->isAnimatedObject() && vobj->isRiggedMesh())
                {
                    std::string vobj_name = llformat("Vol%u", (U32) vobj);
                    F32 est_tris = vobj->getEstTrianglesMax();
                    LL_DEBUGS("AXONLinkset") << vobj_name << " rebuildMesh, tris " << est_tris << LL_ENDL; 
        }
#endif
				vobj->preRebuild();

				if (drawablep->isState(LLDrawable::ANIMATED_CHILD))
				{
					vobj->updateRelativeXform(true);
				}

				LLVolume* volume = vobj->getVolume();
				for (S32 i = 0; i < drawablep->getNumFaces(); ++i)
				{
					LLFace* face = drawablep->getFace(i);
					if (face)
					{
						LLVertexBuffer* buff = face->getVertexBuffer();
						if (buff)
						{
							llassert(!face->isState(LLFace::RIGGED));

							if (!face->getGeometryVolume(*volume, face->getTEOffset(), 
								vobj->getRelativeXform(), vobj->getRelativeXformInvTrans(), face->getGeomIndex()))
							{ //something's gone wrong with the vertex buffer accounting, rebuild this group 
								group->dirtyGeom();
								gPipeline.markRebuild(group, TRUE);
							}


							if (buff->isLocked() && buffer_count < MAX_BUFFER_COUNT)
							{
								locked_buffer[buffer_count++] = buff;
							}
						}
					}
				}

				if (drawablep->isState(LLDrawable::ANIMATED_CHILD))
				{
					vobj->updateRelativeXform();
				}

				
				drawablep->clearState(LLDrawable::REBUILD_ALL);
			}
		}
		
		{
			LL_RECORD_BLOCK_TIME(FTM_REBUILD_MESH_FLUSH);
			for (LLVertexBuffer** iter = locked_buffer, ** end_iter = locked_buffer+buffer_count; iter != end_iter; ++iter)
		{
			(*iter)->flush();
		}

		// don't forget alpha
		if(group != NULL && 
		   !group->mVertexBuffer.isNull() && 
		   group->mVertexBuffer->isLocked())
		{
			group->mVertexBuffer->flush();
		}
		}

		//if not all buffers are unmapped
		if(num_mapped_vertex_buffer != LLVertexBuffer::sMappedCount) 
		{
			LL_WARNS() << "Not all mapped vertex buffers are unmapped!" << LL_ENDL ; 
			for (LLSpatialGroup::element_iter drawable_iter = group->getDataBegin(); drawable_iter != group->getDataEnd(); ++drawable_iter)
			{
				LLDrawable* drawablep = (LLDrawable*)(*drawable_iter)->getDrawable();
				if(!drawablep)
				{
					continue;
				}
				for (S32 i = 0; i < drawablep->getNumFaces(); ++i)
				{
					LLFace* face = drawablep->getFace(i);
					if (face)
					{
						LLVertexBuffer* buff = face->getVertexBuffer();
						if (buff && buff->isLocked())
						{
							buff->flush();
						}
					}
				}
			} 
		}

		group->clearState(LLSpatialGroup::MESH_DIRTY | LLSpatialGroup::NEW_DRAWINFO);
	}

//	llassert(!group || !group->isState(LLSpatialGroup::NEW_DRAWINFO));
}

struct CompareBatchBreakerModified
{
	bool operator()(const LLFace* const& lhs, const LLFace* const& rhs)
	{
		const LLTextureEntry* lte = lhs->getTextureEntry();
		const LLTextureEntry* rte = rhs->getTextureEntry();

		if (lte->getBumpmap() != rte->getBumpmap())
		{
			return lte->getBumpmap() < rte->getBumpmap();
		}
		else if (lte->getFullbright() != rte->getFullbright())
		{
			return lte->getFullbright() < rte->getFullbright();
		}
		else if (LLPipeline::sRenderDeferred && lte->getMaterialParams() != rte->getMaterialParams())
		{
			return lte->getMaterialParams() < rte->getMaterialParams();
		}
		else if (LLPipeline::sRenderDeferred && (lte->getMaterialParams() == rte->getMaterialParams()) && (lte->getShiny() != rte->getShiny()))
		{
			return lte->getShiny() < rte->getShiny();
		}
		else
		{
			return lhs->getTexture() < rhs->getTexture();
		}
	}
};

static LLTrace::BlockTimerStatHandle FTM_GEN_DRAW_INFO_SORT("Draw Info Face Sort");
static LLTrace::BlockTimerStatHandle FTM_GEN_DRAW_INFO_FACE_SIZE("Face Sizing");
static LLTrace::BlockTimerStatHandle FTM_GEN_DRAW_INFO_ALLOCATE("Allocate VB");
static LLTrace::BlockTimerStatHandle FTM_GEN_DRAW_INFO_FIND_VB("Find VB");
static LLTrace::BlockTimerStatHandle FTM_GEN_DRAW_INFO_RESIZE_VB("Resize VB");





void LLVolumeGeometryManager::genDrawInfo(LLSpatialGroup* group, U32 mask, LLFace** faces, U32 face_count, BOOL distance_sort, BOOL batch_textures, BOOL no_materials)
{
	LL_RECORD_BLOCK_TIME(FTM_REBUILD_VOLUME_GEN_DRAW_INFO);

	U32 buffer_usage = group->mBufferUsage;
	
	static LLCachedControl<bool> use_transform_feedback(gSavedSettings, "RenderUseTransformFeedback", false);

	if (use_transform_feedback &&
		gTransformPositionProgram.mProgramObject && //transform shaders are loaded
		buffer_usage == GL_DYNAMIC_DRAW_ARB && //target buffer is in VRAM
		!(mask & LLVertexBuffer::MAP_WEIGHT4)) //TODO: add support for weights
	{
		buffer_usage = GL_DYNAMIC_COPY_ARB;
	}

#if LL_DARWIN
	// HACK from Leslie:
	// Disable VBO usage for alpha on Mac OS X because it kills the framerate
	// due to implicit calls to glTexSubImage that are beyond our control.
	// (this works because the only calls here that sort by distance are alpha)
	if (distance_sort)
	{
		buffer_usage = 0x0;
	}
#endif
	
	//calculate maximum number of vertices to store in a single buffer
	U32 max_vertices = (gSavedSettings.getS32("RenderMaxVBOSize")*1024)/LLVertexBuffer::calcVertexSize(group->getSpatialPartition()->mVertexDataMask);
	max_vertices = llmin(max_vertices, (U32) 65535);

	{
		LL_RECORD_BLOCK_TIME(FTM_GEN_DRAW_INFO_SORT);
		if (!distance_sort)
		{
			//sort faces by things that break batches
			std::sort(faces, faces+face_count, CompareBatchBreakerModified());
		}
		else
		{
			//sort faces by distance
			std::sort(faces, faces+face_count, LLFace::CompareDistanceGreater());
		}
	}
				
	bool hud_group = group->isHUDGroup() ;
	LLFace** face_iter = faces;
	LLFace** end_faces = faces+face_count;
	
	LLSpatialGroup::buffer_map_t buffer_map;

	LLViewerTexture* last_tex = NULL;
	S32 buffer_index = 0;

	if (distance_sort)
	{
		buffer_index = -1;
	}

	S32 texture_index_channels = 1;
	
	if (gGLManager.mGLSLVersionMajor > 1 || gGLManager.mGLSLVersionMinor >= 30)
	{
		texture_index_channels = LLGLSLShader::sIndexedTextureChannels-1; //always reserve one for shiny for now just for simplicity;
	}

	if (LLPipeline::sRenderDeferred && distance_sort)
	{
		texture_index_channels = gDeferredAlphaProgram.mFeatures.mIndexedTextureChannels;
	}

	texture_index_channels = llmin(texture_index_channels, (S32) gSavedSettings.getU32("RenderMaxTextureIndex"));
	
	//NEVER use more than 16 texture index channels (workaround for prevalent driver bug)
	texture_index_channels = llmin(texture_index_channels, 16);

	bool flexi = false;

	while (face_iter != end_faces)
	{
		//pull off next face
		LLFace* facep = *face_iter;
		LLViewerTexture* tex = facep->getTexture();
		LLMaterialPtr mat = facep->getTextureEntry()->getMaterialParams();

		if (distance_sort)
		{
			tex = NULL;
		}

		if (last_tex == tex)
		{
			buffer_index++;
		}
		else
		{
			last_tex = tex;
			buffer_index = 0;
		}

		bool bake_sunlight = LLPipeline::sBakeSunlight && facep->getDrawable()->isStatic(); 

		U32 index_count = facep->getIndicesCount();
		U32 geom_count = facep->getGeomCount();

		flexi = flexi || facep->getViewerObject()->getVolume()->isUnique();

		//sum up vertices needed for this render batch
		LLFace** i = face_iter;
		++i;
		
		const U32 MAX_TEXTURE_COUNT = 32;
		LLViewerTexture* texture_list[MAX_TEXTURE_COUNT];
		
		U32 texture_count = 0;

		{
			LL_RECORD_BLOCK_TIME(FTM_GEN_DRAW_INFO_FACE_SIZE);
			if (batch_textures)
			{
				U8 cur_tex = 0;
				facep->setTextureIndex(cur_tex);
				if (texture_count < MAX_TEXTURE_COUNT)
				{
					texture_list[texture_count++] = tex;
				}

				if (can_batch_texture(facep))
				{ //populate texture_list with any textures that can be batched
				  //move i to the next unbatchable face
					while (i != end_faces)
					{
						facep = *i;
						
						if (!can_batch_texture(facep))
						{ //face is bump mapped or has an animated texture matrix -- can't 
							//batch more than 1 texture at a time
							facep->setTextureIndex(0);
							break;
						}

						if (facep->getTexture() != tex)
						{
							if (distance_sort)
							{ //textures might be out of order, see if texture exists in current batch
								bool found = false;
								for (U32 tex_idx = 0; tex_idx < texture_count; ++tex_idx)
								{
									if (facep->getTexture() == texture_list[tex_idx])
									{
										cur_tex = tex_idx;
										found = true;
										break;
									}
								}

								if (!found)
								{
									cur_tex = texture_count;
								}
							}
							else
							{
								cur_tex++;
							}

							if (cur_tex >= texture_index_channels)
							{ //cut batches when index channels are depleted
								break;
							}

							tex = facep->getTexture();

							if (texture_count < MAX_TEXTURE_COUNT)
							{
								texture_list[texture_count++] = tex;
							}
						}

						if (geom_count + facep->getGeomCount() > max_vertices)
						{ //cut batches on geom count too big
							break;
						}

						++i;

						flexi = flexi || facep->getViewerObject()->getVolume()->isUnique();

						index_count += facep->getIndicesCount();
						geom_count += facep->getGeomCount();

						facep->setTextureIndex(cur_tex);
					}
				}
				else
				{
					facep->setTextureIndex(0);
				}

				tex = texture_list[0];
			}
			else
			{
				while (i != end_faces && 
					(LLPipeline::sTextureBindTest || 
						(distance_sort || 
							((*i)->getTexture() == tex &&
							((*i)->getTextureEntry()->getMaterialParams() == mat)))))
				{
					facep = *i;
			

					//face has no texture index
					facep->mDrawInfo = NULL;
					facep->setTextureIndex(255);

					if (geom_count + facep->getGeomCount() > max_vertices)
					{ //cut batches on geom count too big
						break;
					}

					++i;
					index_count += facep->getIndicesCount();
					geom_count += facep->getGeomCount();

					flexi = flexi || facep->getViewerObject()->getVolume()->isUnique();
				}
			}
		}


		if (flexi && buffer_usage && buffer_usage != GL_STREAM_DRAW_ARB)
		{
			buffer_usage = GL_STREAM_DRAW_ARB;
		}

		//create vertex buffer
		LLVertexBuffer* buffer = NULL;

		{
			LL_RECORD_BLOCK_TIME(FTM_GEN_DRAW_INFO_ALLOCATE);
			buffer = createVertexBuffer(mask, buffer_usage);
			buffer->allocateBuffer(geom_count, index_count, TRUE);
		}

		group->mGeometryBytes += buffer->getSize() + buffer->getIndicesSize();


		buffer_map[mask][*face_iter].push_back(buffer);

		//add face geometry

		U32 indices_index = 0;
		U16 index_offset = 0;

		while (face_iter < i)
		{ //update face indices for new buffer
			facep = *face_iter;
			facep->setIndicesIndex(indices_index);
			facep->setGeomIndex(index_offset);
			facep->setVertexBuffer(buffer);	
			
			if (batch_textures && facep->getTextureIndex() == 255)
			{
				LL_ERRS() << "Invalid texture index." << LL_ENDL;
			}
			
			{
				//for debugging, set last time face was updated vs moved
				facep->updateRebuildFlags();

				if (!LLPipeline::sDelayVBUpdate)
				{ //copy face geometry into vertex buffer
					LLDrawable* drawablep = facep->getDrawable();
					LLVOVolume* vobj = drawablep->getVOVolume();
					LLVolume* volume = vobj->getVolume();

					if (drawablep->isState(LLDrawable::ANIMATED_CHILD))
					{
						vobj->updateRelativeXform(true);
					}

					U32 te_idx = facep->getTEOffset();

					llassert(!facep->isState(LLFace::RIGGED));

					if (!facep->getGeometryVolume(*volume, te_idx, 
						vobj->getRelativeXform(), vobj->getRelativeXformInvTrans(), index_offset,true))
					{
						LL_WARNS() << "Failed to get geometry for face!" << LL_ENDL;
					}

					if (drawablep->isState(LLDrawable::ANIMATED_CHILD))
					{
						vobj->updateRelativeXform(false);
					}
				}
			}

			index_offset += facep->getGeomCount();
			indices_index += facep->getIndicesCount();

			//append face to appropriate render batch

			BOOL force_simple = facep->getPixelArea() < FORCE_SIMPLE_RENDER_AREA;
			BOOL fullbright = facep->isState(LLFace::FULLBRIGHT);
			if ((mask & LLVertexBuffer::MAP_NORMAL) == 0)
			{ //paranoia check to make sure GL doesn't try to read non-existant normals
				fullbright = TRUE;
			}

			if (hud_group)
			{ //all hud attachments are fullbright
				fullbright = TRUE;
			}

			const LLTextureEntry* te = facep->getTextureEntry();
			tex = facep->getTexture();

			BOOL is_alpha = (facep->getPoolType() == LLDrawPool::POOL_ALPHA) ? TRUE : FALSE;
		
			LLMaterial* mat = te->getMaterialParams().get();

			bool can_be_shiny = true;
			if (mat)
			{
				U8 mode = mat->getDiffuseAlphaMode();
				can_be_shiny = mode == LLMaterial::DIFFUSE_ALPHA_MODE_NONE ||
								mode == LLMaterial::DIFFUSE_ALPHA_MODE_EMISSIVE;
			}

			bool use_legacy_bump = te->getBumpmap() && (te->getBumpmap() < 18) && (!mat || mat->getNormalID().isNull());
			bool opaque = te->getColor().mV[3] >= 0.999f;

			if (mat && LLPipeline::sRenderDeferred && !hud_group)
			{
				bool material_pass = false;

				// do NOT use 'fullbright' for this logic or you risk sending
				// things without normals down the materials pipeline and will
				// render poorly if not crash NORSPEC-240,314
				//
				if (te->getFullbright())
				{
					if (mat->getDiffuseAlphaMode() == LLMaterial::DIFFUSE_ALPHA_MODE_MASK)
					{
						if (opaque)
						{
							registerFace(group, facep, LLRenderPass::PASS_FULLBRIGHT_ALPHA_MASK);
						}
						else
						{
							registerFace(group, facep, LLRenderPass::PASS_ALPHA);
						}
					}
					else if (is_alpha)
					{
						registerFace(group, facep, LLRenderPass::PASS_ALPHA);
					}
					else
					{
						if (mat->getEnvironmentIntensity() > 0 ||
							te->getShiny() > 0)
						{
							material_pass = true;
						}
						else
						{
							registerFace(group, facep, LLRenderPass::PASS_FULLBRIGHT);
						}
					}
				}
				else if (no_materials)
				{
					registerFace(group, facep, LLRenderPass::PASS_SIMPLE);
				}
				else if (te->getColor().mV[3] < 0.999f)
				{
					registerFace(group, facep, LLRenderPass::PASS_ALPHA);
				}
				else if (use_legacy_bump)
				{
					// we have a material AND legacy bump settings, but no normal map
					registerFace(group, facep, LLRenderPass::PASS_BUMP);
				}
				else
				{
					material_pass = true;
				}

				if (material_pass)
				{
					static const U32 pass[] = 
					{
						LLRenderPass::PASS_MATERIAL,
						LLRenderPass::PASS_ALPHA, //LLRenderPass::PASS_MATERIAL_ALPHA,
						LLRenderPass::PASS_MATERIAL_ALPHA_MASK,
						LLRenderPass::PASS_MATERIAL_ALPHA_EMISSIVE,
						LLRenderPass::PASS_SPECMAP,
						LLRenderPass::PASS_ALPHA, //LLRenderPass::PASS_SPECMAP_BLEND,
						LLRenderPass::PASS_SPECMAP_MASK,
						LLRenderPass::PASS_SPECMAP_EMISSIVE,
						LLRenderPass::PASS_NORMMAP,
						LLRenderPass::PASS_ALPHA, //LLRenderPass::PASS_NORMMAP_BLEND,
						LLRenderPass::PASS_NORMMAP_MASK,
						LLRenderPass::PASS_NORMMAP_EMISSIVE,
						LLRenderPass::PASS_NORMSPEC,
						LLRenderPass::PASS_ALPHA, //LLRenderPass::PASS_NORMSPEC_BLEND,
						LLRenderPass::PASS_NORMSPEC_MASK,
						LLRenderPass::PASS_NORMSPEC_EMISSIVE,
					};

					U32 mask = mat->getShaderMask();

					llassert(mask < sizeof(pass)/sizeof(U32));

					mask = llmin(mask, (U32)(sizeof(pass)/sizeof(U32)-1));

					registerFace(group, facep, pass[mask]);
				}
			}
			else if (mat)
			{
				U8 mode = mat->getDiffuseAlphaMode();
				if (te->getColor().mV[3] < 0.999f)
				{
					mode = LLMaterial::DIFFUSE_ALPHA_MODE_BLEND;
				}

				if (mode == LLMaterial::DIFFUSE_ALPHA_MODE_MASK)
				{
					registerFace(group, facep, fullbright ? LLRenderPass::PASS_FULLBRIGHT_ALPHA_MASK : LLRenderPass::PASS_ALPHA_MASK);
				}
				else if (is_alpha || (te->getColor().mV[3] < 0.999f))
				{
					registerFace(group, facep, LLRenderPass::PASS_ALPHA);
				}
				else if (gPipeline.canUseVertexShaders()
					&& LLPipeline::sRenderBump 
					&& te->getShiny() 
					&& can_be_shiny)
				{
					registerFace(group, facep, fullbright ? LLRenderPass::PASS_FULLBRIGHT_SHINY : LLRenderPass::PASS_SHINY);
				}
				else
				{
					registerFace(group, facep, fullbright ? LLRenderPass::PASS_FULLBRIGHT : LLRenderPass::PASS_SIMPLE);
				}
			}
			else if (is_alpha)
			{
				// can we safely treat this as an alpha mask?
				if (facep->getFaceColor().mV[3] <= 0.f)
				{ //100% transparent, don't render unless we're highlighting transparent
					registerFace(group, facep, LLRenderPass::PASS_ALPHA_INVISIBLE);
				}
				else if (facep->canRenderAsMask())
				{
					if (te->getFullbright() || LLPipeline::sNoAlpha)
					{
						registerFace(group, facep, LLRenderPass::PASS_FULLBRIGHT_ALPHA_MASK);
					}
					else
					{
						registerFace(group, facep, LLRenderPass::PASS_ALPHA_MASK);
					}
				}
				else
				{
					registerFace(group, facep, LLRenderPass::PASS_ALPHA);
				}
			}
			else if (gPipeline.canUseVertexShaders()
				&& LLPipeline::sRenderBump 
				&& te->getShiny() 
				&& can_be_shiny)
			{ //shiny
				if (tex->getPrimaryFormat() == GL_ALPHA)
				{ //invisiprim+shiny
					registerFace(group, facep, LLRenderPass::PASS_INVISI_SHINY);
					registerFace(group, facep, LLRenderPass::PASS_INVISIBLE);
				}
				else if (LLPipeline::sRenderDeferred && !hud_group)
				{ //deferred rendering
					if (te->getFullbright())
					{ //register in post deferred fullbright shiny pass
						registerFace(group, facep, LLRenderPass::PASS_FULLBRIGHT_SHINY);
						if (te->getBumpmap())
						{ //register in post deferred bump pass
							registerFace(group, facep, LLRenderPass::PASS_POST_BUMP);
						}
					}
					else if (use_legacy_bump)
					{ //register in deferred bump pass
						registerFace(group, facep, LLRenderPass::PASS_BUMP);
					}
					else
					{ //register in deferred simple pass (deferred simple includes shiny)
						llassert(mask & LLVertexBuffer::MAP_NORMAL);
						registerFace(group, facep, LLRenderPass::PASS_SIMPLE);
					}
				}
				else if (fullbright)
				{	//not deferred, register in standard fullbright shiny pass					
					registerFace(group, facep, LLRenderPass::PASS_FULLBRIGHT_SHINY);
				}
				else
				{ //not deferred or fullbright, register in standard shiny pass
					registerFace(group, facep, LLRenderPass::PASS_SHINY);
				}
			}
			else
			{ //not alpha and not shiny
				if (!is_alpha && tex->getPrimaryFormat() == GL_ALPHA)
				{ //invisiprim
					registerFace(group, facep, LLRenderPass::PASS_INVISIBLE);
				}
				else if (fullbright || bake_sunlight)
				{ //fullbright
					if (mat && mat->getDiffuseAlphaMode() == LLMaterial::DIFFUSE_ALPHA_MODE_MASK)
					{
						registerFace(group, facep, LLRenderPass::PASS_FULLBRIGHT_ALPHA_MASK);
					}
					else
					{
					registerFace(group, facep, LLRenderPass::PASS_FULLBRIGHT);
					}
					if (LLPipeline::sRenderDeferred && !hud_group && LLPipeline::sRenderBump && use_legacy_bump)
					{ //if this is the deferred render and a bump map is present, register in post deferred bump
						registerFace(group, facep, LLRenderPass::PASS_POST_BUMP);
					}
				}
				else
				{
					if (LLPipeline::sRenderDeferred && LLPipeline::sRenderBump && use_legacy_bump)
					{ //non-shiny or fullbright deferred bump
						registerFace(group, facep, LLRenderPass::PASS_BUMP);
					}
					else
					{ //all around simple
						llassert(mask & LLVertexBuffer::MAP_NORMAL);
						if (mat && mat->getDiffuseAlphaMode() == LLMaterial::DIFFUSE_ALPHA_MODE_MASK)
						{ //material alpha mask can be respected in non-deferred
							registerFace(group, facep, LLRenderPass::PASS_ALPHA_MASK);
						}
						else
						{
						registerFace(group, facep, LLRenderPass::PASS_SIMPLE);
					}
				}
				}
				
				
				if (!gPipeline.canUseVertexShaders() && 
					!is_alpha && 
					te->getShiny() && 
					LLPipeline::sRenderBump)
				{ //shiny as an extra pass when shaders are disabled
					registerFace(group, facep, LLRenderPass::PASS_SHINY);
				}
			}
			
			//not sure why this is here, and looks like it might cause bump mapped objects to get rendered redundantly -- davep 5/11/2010
			if (!is_alpha && (hud_group || !LLPipeline::sRenderDeferred))
			{
				llassert((mask & LLVertexBuffer::MAP_NORMAL) || fullbright);
				facep->setPoolType((fullbright) ? LLDrawPool::POOL_FULLBRIGHT : LLDrawPool::POOL_SIMPLE);
				
				if (!force_simple && LLPipeline::sRenderBump && use_legacy_bump)
				{
					registerFace(group, facep, LLRenderPass::PASS_BUMP);
				}
			}

			if (!is_alpha && LLPipeline::sRenderGlow && te->getGlow() > 0.f)
			{
				registerFace(group, facep, LLRenderPass::PASS_GLOW);
			}
						
			++face_iter;
		}

		buffer->flush();
	}

	group->mBufferMap[mask].clear();
	for (LLSpatialGroup::buffer_texture_map_t::iterator i = buffer_map[mask].begin(); i != buffer_map[mask].end(); ++i)
	{
		group->mBufferMap[mask][i->first] = i->second;
	}
}

void LLGeometryManager::addGeometryCount(LLSpatialGroup* group, U32 &vertex_count, U32 &index_count)
{	
	//initialize to default usage for this partition
	U32 usage = group->getSpatialPartition()->mBufferUsage;
	
	//clear off any old faces
	mFaceList.clear();

	//for each drawable

	for (LLSpatialGroup::element_iter drawable_iter = group->getDataBegin(); drawable_iter != group->getDataEnd(); ++drawable_iter)
	{
		LLDrawable* drawablep = (LLDrawable*)(*drawable_iter)->getDrawable();
		
		if (!drawablep || drawablep->isDead())
		{
			continue;
		}
	
		if (drawablep->isAnimating())
		{ //fall back to stream draw for animating verts
			usage = GL_STREAM_DRAW_ARB;
		}

		//for each face
		for (S32 i = 0; i < drawablep->getNumFaces(); i++)
		{
			//sum up face verts and indices
			drawablep->updateFaceSize(i);
			LLFace* facep = drawablep->getFace(i);
			if (facep)
			{
				if (facep->hasGeometry() && facep->getPixelArea() > FORCE_CULL_AREA && 
					facep->getGeomCount() + vertex_count <= 65536)
				{
					vertex_count += facep->getGeomCount();
					index_count += facep->getIndicesCount();
				
					//remember face (for sorting)
					mFaceList.push_back(facep);
				}
				else
				{
					facep->clearVertexBuffer();
				}
			}
		}
	}
	
	group->mBufferUsage = usage;
}

LLHUDPartition::LLHUDPartition(LLViewerRegion* regionp) : LLBridgePartition(regionp)
{
	mPartitionType = LLViewerRegion::PARTITION_HUD;
	mDrawableType = LLPipeline::RENDER_TYPE_HUD;
	mSlopRatio = 0.f;
	mLODPeriod = 1;
}

void LLHUDPartition::shift(const LLVector4a &offset)
{
	//HUD objects don't shift with region crossing.  That would be silly.
}

