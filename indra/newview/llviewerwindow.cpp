/** 
 * @file llviewerwindow.cpp
 * @brief Implementation of the LLViewerWindow class.
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

#include "llviewerprecompiledheaders.h"
#include "llviewerwindow.h"


// system library includes
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/lambda/core.hpp>
#include <boost/regex.hpp>

#include "llagent.h"
#include "llagentcamera.h"
#include "llcommunicationchannel.h"
#include "llfloaterreg.h"
#include "llhudicon.h"
#include "llmeshrepository.h"
#include "llnotificationhandler.h"
#include "llpanellogin.h"
#include "llsetkeybinddialog.h"
#include "llviewerinput.h"
#include "llviewermenu.h"
//<FS:Beq> physics display changes
#include "llspatialpartition.h"
#include "llphysicsshapebuilderutil.h"
#include "llvolumemgr.h"
//</FS:Beq>

#include "llviewquery.h"
#include "llxmltree.h"
#include "llslurl.h"
#include "llrender.h"

#include "llvoiceclient.h"	// for push-to-talk button handling
#include "stringize.h"

//
// TODO: Many of these includes are unnecessary.  Remove them.
//

// linden library includes
#include "llaudioengine.h"		// mute on minimize
#include "llchatentry.h"
#include "indra_constants.h"
#include "llassetstorage.h"
#include "llerrorcontrol.h"
#include "llfontgl.h"
#include "llmousehandler.h"
#include "llrect.h"
#include "llsky.h"
#include "llstring.h"
#include "llui.h"
#include "lluuid.h"
#include "llview.h"
#include "llxfermanager.h"
#include "message.h"
#include "object_flags.h"
#include "lltimer.h"
#include "llviewermenu.h"
#include "lltooltip.h"
#include "llmediaentry.h"
#include "llurldispatcher.h"
#include "raytrace.h"

// newview includes
#include "fscommon.h"
#include "llagent.h"
#include "llbox.h"
#include "llchicletbar.h"
#include "llconsole.h"
#include "llviewercontrol.h"
#include "llcylinder.h"
#include "lldebugview.h"
#include "lldir.h"
#include "lldrawable.h"
#include "lldrawpoolalpha.h"
#include "lldrawpoolbump.h"
#include "lldrawpoolwater.h"
#include "llmaniptranslate.h"
#include "llface.h"
#include "llfeaturemanager.h"
#include "llfilepicker.h"
#include "llfirstuse.h"
#include "llfloater.h"
#include "llfloaterbuildoptions.h"
#include "llfloaterbuyland.h"
#include "llfloatercamera.h"
#include "llfloaterland.h"
#include "llfloaterinspect.h"
#include "llfloatermap.h"
#include "llfloaternamedesc.h"
#include "llfloaterpreference.h"
#include "llfloatersnapshot.h"
#include "llfloatertools.h"
#include "llfloaterworldmap.h"
#include "llfocusmgr.h"
#include "llfontfreetype.h"
#include "llgesturemgr.h"
#include "llglheaders.h"
#include "lltooltip.h"
#include "llhudmanager.h"
#include "llhudobject.h"
#include "llhudview.h"
#include "llimage.h"
#include "llimagej2c.h"
#include "llimageworker.h"
#include "llkeyboard.h"
#include "lllineeditor.h"
#include "lllogininstance.h"
#include "llmenugl.h"
#include "llmenuoptionpathfindingrebakenavmesh.h"
#include "llmodaldialog.h"
#include "llmorphview.h"
#include "llmoveview.h"
#include "llnavigationbar.h"
#include "llnotificationhandler.h"
#include "llpaneltopinfobar.h"
#include "llpopupview.h"
#include "llpreviewtexture.h"
#include "llprogressview.h"
#include "llresmgr.h"
#include "llselectmgr.h"
#include "llrootview.h"
#include "llrendersphere.h"
#include "llstartup.h"
#include "llstatusbar.h"
#include "llstatview.h"
#include "llsurface.h"
#include "llsurfacepatch.h"
#include "lltexlayer.h"
#include "lltextbox.h"
#include "lltexturecache.h"
#include "lltexturefetch.h"
#include "lltextureview.h"
#include "lltoast.h"
#include "lltool.h"
#include "lltoolbarview.h"
#include "lltoolcomp.h"
#include "lltooldraganddrop.h"
#include "lltoolface.h"
#include "lltoolfocus.h"
#include "lltoolgrab.h"
#include "lltoolmgr.h"
#include "lltoolmorph.h"
#include "lltoolpie.h"
#include "lltoolselectland.h"
#include "lltrans.h"
#include "lluictrlfactory.h"
#include "llurldispatcher.h"		// SLURL from other app instance
#include "llversioninfo.h"
#include "llvieweraudio.h"
#include "llviewercamera.h"
#include "llviewergesture.h"
#include "llviewertexturelist.h"
#include "llviewerinventory.h"
#include "llviewerinput.h"
#include "llviewermedia.h"
#include "llviewermediafocus.h"
#include "llviewermenu.h"
#include "llviewermessage.h"
#include "llviewerobjectlist.h"
#include "llviewerparcelmgr.h"
#include "llviewerregion.h"
#include "llviewershadermgr.h"
#include "llviewerstats.h"
#include "llvoavatarself.h"
#include "llvopartgroup.h"
#include "llvovolume.h"
#include "llworld.h"
#include "llworldmapview.h"
#include "pipeline.h"
#include "llappviewer.h"
#include "llviewerdisplay.h"
#include "llspatialpartition.h"
#include "llviewerjoystick.h"
#include "llviewermenufile.h" // LLFilePickerReplyThread
#include "llviewernetwork.h"
#include "llpostprocess.h"
// <FS:Ansariel> [FS communication UI]
//#include "llfloaterimnearbychat.h"
// </FS:Ansariel> [FS communication UI]
#include "llagentui.h"
#include "llwearablelist.h"

#include "llviewereventrecorder.h"

#include "llnotifications.h"
#include "llnotificationsutil.h"
#include "llnotificationmanager.h"

#include "llfloaternotificationsconsole.h"

// <FS:Ansariel> [FS communication UI]
#include "fsfloaternearbychat.h"
#include "fsnearbychathub.h"
// </FS:Ansariel> [FS communication UI]
#include "llwindowlistener.h"
#include "llviewerwindowlistener.h"
#include "llpaneltopinfobar.h"
#include "llcleanup.h"
#include "llimview.h"
#include "llviewermenufile.h"

// [RLVa:KB] - Checked: 2010-03-31 (RLVa-1.2.0c)
#include "rlvhandler.h"
// [/RLVa:KB]

#if LL_WINDOWS
#include <tchar.h> // For Unicode conversion methods
#endif

#include "utilitybar.h"		// <FS:Zi> Support for the classic V1 style buttons in some skins
#include "exopostprocess.h"	// <FS:Ansariel> Exodus Vignette
#include "llnetmap.h"
#include "lggcontactsets.h"

#include "lltracerecording.h"

//
// Globals
//
void render_ui(F32 zoom_factor = 1.f, int subfield = 0);
void swap();

extern BOOL gDebugClicks;
extern BOOL gDisplaySwapBuffers;
extern BOOL gDepthDirty;
extern BOOL gResizeScreenTexture;

LLViewerWindow	*gViewerWindow = NULL;

LLFrameTimer	gAwayTimer;
LLFrameTimer	gAwayTriggerTimer;

BOOL			gShowOverlayTitle = FALSE;

LLViewerObject*  gDebugRaycastObject = NULL;
LLVOPartGroup* gDebugRaycastParticle = NULL;
LLVector4a       gDebugRaycastIntersection;
LLVector4a		gDebugRaycastParticleIntersection;
LLVector2        gDebugRaycastTexCoord;
LLVector4a       gDebugRaycastNormal;
LLVector4a       gDebugRaycastTangent;
S32				gDebugRaycastFaceHit;
LLVector4a		 gDebugRaycastStart;
LLVector4a		 gDebugRaycastEnd;

// HUD display lines in lower right
BOOL				gDisplayWindInfo = FALSE;
BOOL				gDisplayCameraPos = FALSE;
BOOL				gDisplayFOV = FALSE;

static const U8 NO_FACE = 255;
BOOL gQuietSnapshot = FALSE;

const F32 MIN_AFK_TIME = 6.f; // minimum time after setting away state before coming back

// Minimum value for UIScaleFactor, also defined in preferences, ui_scale_slider
static const F32 MIN_UI_SCALE = 0.75f;
// 4.0 in preferences, but win10 supports larger scaling and value is used more as
// sanity check, so leaving space for larger values from DPI updates.
static const F32 MAX_UI_SCALE = 7.0f;
static const F32 MIN_DISPLAY_SCALE = 0.75f;

static LLCachedControl<std::string>	sSnapshotBaseName(LLCachedControl<std::string>(gSavedPerAccountSettings, "SnapshotBaseName", "Snapshot"));
static LLCachedControl<std::string>	sSnapshotDir(LLCachedControl<std::string>(gSavedPerAccountSettings, "SnapshotBaseDir", ""));

LLTrace::SampleStatHandle<> LLViewerWindow::sMouseVelocityStat("Mouse Velocity");

class RecordToChatConsoleRecorder : public LLError::Recorder
{
public:
	virtual void recordMessage(LLError::ELevel level,
								const std::string& message)
	{
		//FIXME: this is NOT thread safe, and will do bad things when a warning is issued from a non-UI thread

		// only log warnings to chat console
		//if (level == LLError::LEVEL_WARN)
		//{
			//LLFloaterChat* chat_floater = LLFloaterReg::findTypedInstance<LLFloaterChat>("chat");
			//if (chat_floater && gSavedSettings.getBOOL("WarningsAsChat"))
			//{
			//	LLChat chat;
			//	chat.mText = message;
			//	chat.mSourceType = CHAT_SOURCE_SYSTEM;

			//	chat_floater->addChat(chat, FALSE, FALSE);
			//}
		//}
	}
};

class RecordToChatConsole : public LLSingleton<RecordToChatConsole>
{
	LLSINGLETON(RecordToChatConsole);
public:
	void startRecorder() { LLError::addRecorder(mRecorder); }
	void stopRecorder() { LLError::removeRecorder(mRecorder); }

private:
	LLError::RecorderPtr mRecorder;
};

RecordToChatConsole::RecordToChatConsole():
	mRecorder(new RecordToChatConsoleRecorder())
{
    mRecorder->showTags(false);
    mRecorder->showLocation(false);
    mRecorder->showMultiline(true);
}

////////////////////////////////////////////////////////////////////////////
//
// LLDebugText
//

static LLTrace::BlockTimerStatHandle FTM_DISPLAY_DEBUG_TEXT("Display Debug Text");

class LLDebugText
{
private:
	struct Line
	{
		Line(const std::string& in_text, S32 in_x, S32 in_y) : text(in_text), x(in_x), y(in_y) {}
		std::string text;
		S32 x,y;
	};

	LLViewerWindow *mWindow;
	
	typedef std::vector<Line> line_list_t;
	line_list_t mLineList;
	LLColor4 mTextColor;
	
	void addText(S32 x, S32 y, const std::string &text) 
	{
		mLineList.push_back(Line(text, x, y));
	}
	
public:
	LLDebugText(LLViewerWindow* window) : mWindow(window) {}

	void clearText() { mLineList.clear(); }

	void update()
	{
		static LLCachedControl<bool> log_texture_traffic(gSavedSettings,"LogTextureNetworkTraffic", false) ;

		std::string wind_vel_text;
		std::string wind_vector_text;
		std::string rwind_vel_text;
		std::string rwind_vector_text;
		std::string audio_text;

		static const std::string beacon_particle = LLTrans::getString("BeaconParticle");
		static const std::string beacon_physical = LLTrans::getString("BeaconPhysical");
		static const std::string beacon_scripted = LLTrans::getString("BeaconScripted");
		static const std::string beacon_scripted_touch = LLTrans::getString("BeaconScriptedTouch");
		static const std::string beacon_sound = LLTrans::getString("BeaconSound");
		static const std::string beacon_media = LLTrans::getString("BeaconMedia");
		static const std::string particle_hiding = LLTrans::getString("ParticleHiding");

		// Draw the statistics in a light gray
		// and in a thin font
		mTextColor = LLColor4( 0.86f, 0.86f, 0.86f, 1.f );

		// Draw stuff growing up from right lower corner of screen
		S32 xpos = mWindow->getWorldViewWidthScaled() - 400;
		xpos = llmax(xpos, 0);
		S32 ypos = 64;
		const S32 y_inc = 20;

		clearText();
		
		//if (gSavedSettings.getBOOL("DebugShowTime"))
		static LLCachedControl<bool> debugShowTime(gSavedSettings, "DebugShowTime");
		if (debugShowTime)
		{
			{
			const U32 y_inc2 = 15;
				// <FS:Ansariel> FIRE-9746: Show FPS with DebugShowTime
				addText(xpos, ypos, llformat("FPS: %3.1f", LLTrace::get_frame_recording().getPeriodMeanPerSec(LLStatViewer::FPS))); ypos += y_inc2;
				
				// </FS:Ansariel>
				LLFrameTimer& timer = gTextureTimer;
				F32 time = timer.getElapsedTimeF32();
				S32 hours = (S32)(time / (60*60));
				S32 mins = (S32)((time - hours*(60*60)) / 60);
				S32 secs = (S32)((time - hours*(60*60) - mins*60));
				addText(xpos, ypos, llformat("Texture: %d:%02d:%02d", hours,mins,secs)); ypos += y_inc2;
			}
			
			{
			F32 time = gFrameTimeSeconds;
			S32 hours = (S32)(time / (60*60));
			S32 mins = (S32)((time - hours*(60*60)) / 60);
			S32 secs = (S32)((time - hours*(60*60) - mins*60));
			addText(xpos, ypos, llformat("Time: %d:%02d:%02d", hours,mins,secs)); ypos += y_inc;
		}
		}
		
		//if (gSavedSettings.getBOOL("DebugShowMemory"))
		static LLCachedControl<bool> debugShowMemory(gSavedSettings, "DebugShowMemory");
		if (debugShowMemory)
		{
			addText(xpos, ypos,
					STRINGIZE("Memory: " << (LLMemory::getCurrentRSS() / 1024) << " (KB)"));
			ypos += y_inc;
		}

		if (gDisplayCameraPos)
		{
			std::string camera_view_text;
			std::string camera_center_text;
			std::string agent_view_text;
			std::string agent_left_text;
			std::string agent_center_text;
			std::string agent_root_center_text;

			LLVector3d tvector; // Temporary vector to hold data for printing.

			// Update camera center, camera view, wind info every other frame
			tvector = gAgent.getPositionGlobal();
			agent_center_text = llformat("AgentCenter  %f %f %f",
										 (F32)(tvector.mdV[VX]), (F32)(tvector.mdV[VY]), (F32)(tvector.mdV[VZ]));

			if (isAgentAvatarValid())
			{
				tvector = gAgent.getPosGlobalFromAgent(gAgentAvatarp->mRoot->getWorldPosition());
				agent_root_center_text = llformat("AgentRootCenter %f %f %f",
												  (F32)(tvector.mdV[VX]), (F32)(tvector.mdV[VY]), (F32)(tvector.mdV[VZ]));
			}
			else
			{
				agent_root_center_text = "---";
			}


			tvector = LLVector4(gAgent.getFrameAgent().getAtAxis());
			agent_view_text = llformat("AgentAtAxis  %f %f %f",
									   (F32)(tvector.mdV[VX]), (F32)(tvector.mdV[VY]), (F32)(tvector.mdV[VZ]));

			tvector = LLVector4(gAgent.getFrameAgent().getLeftAxis());
			agent_left_text = llformat("AgentLeftAxis  %f %f %f",
									   (F32)(tvector.mdV[VX]), (F32)(tvector.mdV[VY]), (F32)(tvector.mdV[VZ]));

			tvector = gAgentCamera.getCameraPositionGlobal();
			camera_center_text = llformat("CameraCenter %f %f %f",
										  (F32)(tvector.mdV[VX]), (F32)(tvector.mdV[VY]), (F32)(tvector.mdV[VZ]));

			tvector = LLVector4(LLViewerCamera::getInstance()->getAtAxis());
			camera_view_text = llformat("CameraAtAxis    %f %f %f",
										(F32)(tvector.mdV[VX]), (F32)(tvector.mdV[VY]), (F32)(tvector.mdV[VZ]));
		
			addText(xpos, ypos, agent_center_text);  ypos += y_inc;
			addText(xpos, ypos, agent_root_center_text);  ypos += y_inc;
			addText(xpos, ypos, agent_view_text);  ypos += y_inc;
			addText(xpos, ypos, agent_left_text);  ypos += y_inc;
			addText(xpos, ypos, camera_center_text);  ypos += y_inc;
			addText(xpos, ypos, camera_view_text);  ypos += y_inc;
		}

		if (gDisplayWindInfo)
		{
			wind_vel_text = llformat("Wind velocity %.2f m/s", gWindVec.magVec());
			wind_vector_text = llformat("Wind vector   %.2f %.2f %.2f", gWindVec.mV[0], gWindVec.mV[1], gWindVec.mV[2]);
			rwind_vel_text = llformat("RWind vel %.2f m/s", gRelativeWindVec.magVec());
			rwind_vector_text = llformat("RWind vec   %.2f %.2f %.2f", gRelativeWindVec.mV[0], gRelativeWindVec.mV[1], gRelativeWindVec.mV[2]);

			addText(xpos, ypos, wind_vel_text);  ypos += y_inc;
			addText(xpos, ypos, wind_vector_text);  ypos += y_inc;
			addText(xpos, ypos, rwind_vel_text);  ypos += y_inc;
			addText(xpos, ypos, rwind_vector_text);  ypos += y_inc;
		}
		if (gDisplayWindInfo)
		{
			audio_text = llformat("Audio for wind: %d", gAudiop ? gAudiop->isWindEnabled() : -1);
			addText(xpos, ypos, audio_text);  ypos += y_inc;
		}
		if (gDisplayFOV)
		{
			addText(xpos, ypos, llformat("FOV: %2.1f deg", RAD_TO_DEG * LLViewerCamera::getInstance()->getView()));
			ypos += y_inc;
		}
		
		/*if (LLViewerJoystick::getInstance()->getOverrideCamera())
		{
			addText(xpos + 200, ypos, llformat("Flycam"));
			ypos += y_inc;
		}*/
		
		//if (gSavedSettings.getBOOL("DebugShowRenderInfo"))
		static LLCachedControl<bool> debugShowRenderInfo(gSavedSettings, "DebugShowRenderInfo");
		if (debugShowRenderInfo)
		{
			LLTrace::Recording& last_frame_recording = LLTrace::get_frame_recording().getLastRecording();

			if (gPipeline.getUseVertexShaders() == 0)
			{
				addText(xpos, ypos, "Shaders Disabled");
				ypos += y_inc;
			}

			if (gGLManager.mHasATIMemInfo)
			{
				S32 meminfo[4];
				glGetIntegerv(GL_TEXTURE_FREE_MEMORY_ATI, meminfo);

				addText(xpos, ypos, llformat("%.2f MB Texture Memory Free", meminfo[0]/1024.f));
				ypos += y_inc;

				if (gGLManager.mHasVertexBufferObject)
				{
					glGetIntegerv(GL_VBO_FREE_MEMORY_ATI, meminfo);
					addText(xpos, ypos, llformat("%.2f MB VBO Memory Free", meminfo[0]/1024.f));
					ypos += y_inc;
				}
			}
			else if (gGLManager.mHasNVXMemInfo)
			{
				S32 free_memory;
				glGetIntegerv(GL_GPU_MEMORY_INFO_CURRENT_AVAILABLE_VIDMEM_NVX, &free_memory);
				addText(xpos, ypos, llformat("%.2f MB Video Memory Free", free_memory/1024.f));
				ypos += y_inc;
			}

			//show streaming cost/triangle count of known prims in current region OR selection
			{
				F32 cost = 0.f;
				S32 count = 0;
				S32 vcount = 0;
				S32 object_count = 0;
				S32 total_bytes = 0;
				S32 visible_bytes = 0;

				const char* label = "Region";
				if (LLSelectMgr::getInstance()->getSelection()->getObjectCount() == 0)
				{ //region
					LLViewerRegion* region = gAgent.getRegion();
					if (region)
					{
						for (U32 i = 0; i < gObjectList.getNumObjects(); ++i)
						{
							LLViewerObject* object = gObjectList.getObject(i);
							if (object && 
								object->getRegion() == region &&
								object->getVolume())
							{
								object_count++;
								S32 bytes = 0;	
								S32 visible = 0;
								cost += object->getStreamingCost();
                                LLMeshCostData costs;
                                if (object->getCostData(costs))
                                {
                                    bytes = costs.getSizeTotal();
                                    visible = costs.getSizeByLOD(object->getLOD());
                                }

								S32 vt = 0;
								count += object->getTriangleCount(&vt);
								vcount += vt;
								total_bytes += bytes;
								visible_bytes += visible;
							}
						}
					}
				}
				else
				{
					label = "Selection";
					cost = LLSelectMgr::getInstance()->getSelection()->getSelectedObjectStreamingCost(&total_bytes, &visible_bytes);
					count = LLSelectMgr::getInstance()->getSelection()->getSelectedObjectTriangleCount(&vcount);
					object_count = LLSelectMgr::getInstance()->getSelection()->getObjectCount();
				}
					
				addText(xpos,ypos, llformat("%s streaming cost: %.1f", label, cost));
				ypos += y_inc;

				addText(xpos, ypos, llformat("    %.3f KTris, %.3f KVerts, %.1f/%.1f KB, %d objects",
										count/1000.f, vcount/1000.f, visible_bytes/1024.f, total_bytes/1024.f, object_count));
				ypos += y_inc;
			
			}

			addText(xpos, ypos, llformat("%d MB Index Data (%d MB Pooled, %d KIndices)", LLVertexBuffer::sAllocatedIndexBytes/(1024*1024), LLVBOPool::sIndexBytesPooled/(1024*1024), LLVertexBuffer::sIndexCount/1024));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d MB Vertex Data (%d MB Pooled, %d KVerts)", LLVertexBuffer::sAllocatedBytes/(1024*1024), LLVBOPool::sBytesPooled/(1024*1024), LLVertexBuffer::sVertexCount/1024));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Vertex Buffers", LLVertexBuffer::sGLCount));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Mapped Buffers", LLVertexBuffer::sMappedCount));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Vertex Buffer Binds", LLVertexBuffer::sBindCount));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Vertex Buffer Sets", LLVertexBuffer::sSetCount));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Texture Binds", LLImageGL::sBindCount));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Unique Textures", LLImageGL::sUniqueCount));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Render Calls", last_frame_recording.getSampleCount(LLPipeline::sStatBatchSize)));
            ypos += y_inc;

			addText(xpos, ypos, llformat("%d/%d Objects Active", gObjectList.getNumActiveObjects(), gObjectList.getNumObjects()));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Matrix Ops", gPipeline.mMatrixOpCount));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%d Texture Matrix Ops", gPipeline.mTextureMatrixOps));
			ypos += y_inc;

			gPipeline.mTextureMatrixOps = 0;
			gPipeline.mMatrixOpCount = 0;

 			if (last_frame_recording.getSampleCount(LLPipeline::sStatBatchSize) > 0)
			{
                addText(xpos, ypos, llformat("Batch min/max/mean: %d/%d/%d", (U32)last_frame_recording.getMin(LLPipeline::sStatBatchSize), (U32)last_frame_recording.getMax(LLPipeline::sStatBatchSize), (U32)last_frame_recording.getMean(LLPipeline::sStatBatchSize)));
			}
            ypos += y_inc;

			addText(xpos, ypos, llformat("UI Verts/Calls: %d/%d", LLRender::sUIVerts, LLRender::sUICalls));
			LLRender::sUICalls = LLRender::sUIVerts = 0;
			ypos += y_inc;

			addText(xpos,ypos, llformat("%d/%d Nodes visible", gPipeline.mNumVisibleNodes, LLSpatialGroup::sNodeCount));
			
			ypos += y_inc;

			if (!LLOcclusionCullingGroup::sPendingQueries.empty())
			{
				addText(xpos,ypos, llformat("%d Queries pending", LLOcclusionCullingGroup::sPendingQueries.size()));
				ypos += y_inc;
			}


			addText(xpos,ypos, llformat("%d Avatars visible", LLVOAvatar::sNumVisibleAvatars));
			
			ypos += y_inc;

			addText(xpos,ypos, llformat("%d Lights visible", LLPipeline::sVisibleLightCount));
			
			ypos += y_inc;

			if (gMeshRepo.meshRezEnabled())
			{
				addText(xpos, ypos, llformat("%.3f MB Mesh Data Received", LLMeshRepository::sBytesReceived/(1024.f*1024.f)));
				
				ypos += y_inc;
				
				addText(xpos, ypos, llformat("%d/%d Mesh HTTP Requests/Retries", LLMeshRepository::sHTTPRequestCount,
					LLMeshRepository::sHTTPRetryCount));
				ypos += y_inc;

				addText(xpos, ypos, llformat("%d/%d Mesh LOD Pending/Processing", LLMeshRepository::sLODPending, LLMeshRepository::sLODProcessing));
				ypos += y_inc;

				// <FS:Ansariel> Mesh debugging
				addText(xpos, ypos, llformat("%d Mesh Active LOD Requests", LLMeshRepoThread::sActiveLODRequests));
				ypos += y_inc;
				// </FS:Ansariel>

				addText(xpos, ypos, llformat("%.3f/%.3f MB Mesh Cache Read/Write ", LLMeshRepository::sCacheBytesRead/(1024.f*1024.f), LLMeshRepository::sCacheBytesWritten/(1024.f*1024.f)));

				ypos += y_inc;
			}

			LLVertexBuffer::sBindCount = LLImageGL::sBindCount = 
				LLVertexBuffer::sSetCount = LLImageGL::sUniqueCount = 
				gPipeline.mNumVisibleNodes = LLPipeline::sVisibleLightCount = 0;
		}
		static LLCachedControl<bool> sDebugShowAvatarRenderInfo(gSavedSettings, "DebugShowAvatarRenderInfo");
		if (sDebugShowAvatarRenderInfo)
		{
			std::map<std::string, LLVOAvatar*> sorted_avs;
			
			std::vector<LLCharacter*>::iterator sort_iter = LLCharacter::sInstances.begin();
			while (sort_iter != LLCharacter::sInstances.end())
			{
				LLVOAvatar* avatar = dynamic_cast<LLVOAvatar*>(*sort_iter);
				if (avatar &&
					!avatar->isDead())						// Not dead yet
				{
					// Stuff into a sorted map so the display is ordered
					sorted_avs[avatar->getFullname()] = avatar;
				}
				sort_iter++;
			}

			std::string trunc_name;
			std::map<std::string, LLVOAvatar*>::reverse_iterator av_iter = sorted_avs.rbegin();		// Put "A" at the top
			while (av_iter != sorted_avs.rend())
			{
				LLVOAvatar* avatar = av_iter->second;

				avatar->calculateUpdateRenderComplexity(); // Make sure the numbers are up-to-date

				trunc_name = utf8str_truncate(avatar->getFullname(), 16);
				addText(xpos, ypos, llformat("%s : %s, complexity %d, area %.2f",
					trunc_name.c_str(),
                    LLVOAvatar::rezStatusToString(avatar->getRezzedStatus()).c_str(),
					avatar->getVisualComplexity(),
					avatar->getAttachmentSurfaceArea()));
				ypos += y_inc;
				av_iter++;
			}
		}

		//if (gSavedSettings.getBOOL("DebugShowRenderMatrices"))
		static LLCachedControl<bool> debugShowRenderMatrices(gSavedSettings, "DebugShowRenderMatrices");
		if (debugShowRenderMatrices)
		{
			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLProjection[12], gGLProjection[13], gGLProjection[14], gGLProjection[15]));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLProjection[8], gGLProjection[9], gGLProjection[10], gGLProjection[11]));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLProjection[4], gGLProjection[5], gGLProjection[6], gGLProjection[7]));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLProjection[0], gGLProjection[1], gGLProjection[2], gGLProjection[3]));
			ypos += y_inc;

			addText(xpos, ypos, "Projection Matrix");
			ypos += y_inc;


			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLModelView[12], gGLModelView[13], gGLModelView[14], gGLModelView[15]));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLModelView[8], gGLModelView[9], gGLModelView[10], gGLModelView[11]));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLModelView[4], gGLModelView[5], gGLModelView[6], gGLModelView[7]));
			ypos += y_inc;

			addText(xpos, ypos, llformat("%.4f    .%4f    %.4f    %.4f", gGLModelView[0], gGLModelView[1], gGLModelView[2], gGLModelView[3]));
			ypos += y_inc;

			addText(xpos, ypos, "View Matrix");
			ypos += y_inc;
		}
		//<FS:AO improve use of controls with radiogroups>
		//if (gSavedSettings.getBOOL("DebugShowColor") && !LLRender::sNsightDebugSupport)
		//static LLCachedControl<bool> debugShowColor(gSavedSettings, "DebugShowColor");
		static LLCachedControl<S32> debugShowColor(gSavedSettings, "DebugShowColor");
		//</FS:AO>
		if (debugShowColor && !LLRender::sNsightDebugSupport)
		{
			U8 color[4];
			LLCoordGL coord = gViewerWindow->getCurrentMouse();
			glReadPixels(coord.mX, coord.mY, 1,1,GL_RGBA, GL_UNSIGNED_BYTE, color);
			addText(xpos, ypos, llformat("%d %d %d %d", color[0], color[1], color[2], color[3]));
			ypos += y_inc;
		}

		// <FS:LO> pull the text saying if particles are hidden out from beacons
		if (LLPipeline::toggleRenderTypeControlNegated(LLPipeline::RENDER_TYPE_PARTICLES))
		{
			addText(xpos, ypos, particle_hiding);
			ypos += y_inc;
		}
		// </FS:LO>
		// only display these messages if we are actually rendering beacons at this moment
		// <FS:LO> Always show the beacon text regardless if the floater is visible
		// <FS:Ansa> ...and if we want to see it
		//if (LLPipeline::getRenderBeacons() && LLFloaterReg::instanceVisible("beacons"))
		static LLCachedControl<bool> fsRenderBeaconText(gSavedSettings, "FSRenderBeaconText");
		if (LLPipeline::getRenderBeacons() && fsRenderBeaconText)
		// </FS:Ansa>
		{
			if (LLPipeline::getRenderMOAPBeacons())
			{
				// <FS:Ansariel> Localization fix for render beacon info (FIRE-7216)
				//addText(xpos, ypos, "Viewing media beacons (white)");
				addText(xpos, ypos, beacon_media);
				ypos += y_inc;
			}

			// <FS:LO> pull the text saying if particles are hidden out from beacons
			/*if (LLPipeline::toggleRenderTypeControlNegated(LLPipeline::RENDER_TYPE_PARTICLES))
			{
				addText(xpos, ypos, particle_hiding);
				ypos += y_inc;
			}*/
			// </FS:LO>

			if (LLPipeline::getRenderParticleBeacons())
			{
				// <FS:Ansariel> Localization fix for render beacon info (FIRE-7216)
				//addText(xpos, ypos, "Viewing particle beacons (blue)");
				addText(xpos, ypos, beacon_particle);
				ypos += y_inc;
			}

			if (LLPipeline::getRenderSoundBeacons())
			{
				// <FS:Ansariel> Localization fix for render beacon info (FIRE-7216)
				//addText(xpos, ypos, "Viewing sound beacons (yellow)");
				addText(xpos, ypos, beacon_sound);
				ypos += y_inc;
			}

			if (LLPipeline::getRenderScriptedBeacons())
			{
				addText(xpos, ypos, beacon_scripted);
				ypos += y_inc;
			}
			else
				if (LLPipeline::getRenderScriptedTouchBeacons())
				{
					addText(xpos, ypos, beacon_scripted_touch);
					ypos += y_inc;
				}

			if (LLPipeline::getRenderPhysicalBeacons())
			{
				// <FS:Ansariel> Localization fix for render beacon info (FIRE-7216)
				//addText(xpos, ypos, "Viewing physical object beacons (green)");
				addText(xpos, ypos, beacon_physical);
				ypos += y_inc;
			}
		}

		if(log_texture_traffic)
		{	
			U32 old_y = ypos ;
			for(S32 i = LLViewerTexture::BOOST_NONE; i < LLViewerTexture::MAX_GL_IMAGE_CATEGORY; i++)
			{
				if(gTotalTextureBytesPerBoostLevel[i] > (S32Bytes)0)
				{
					addText(xpos, ypos, llformat("Boost_Level %d:  %.3f MB", i, F32Megabytes(gTotalTextureBytesPerBoostLevel[i]).value()));
					ypos += y_inc;
				}
			}
			if(ypos != old_y)
			{
				addText(xpos, ypos, "Network traffic for textures:");
				ypos += y_inc;
			}
		}				

		//if (gSavedSettings.getBOOL("DebugShowTextureInfo"))
		static LLCachedControl<bool> debugShowTextureInfo(gSavedSettings, "DebugShowTextureInfo");
		if (debugShowTextureInfo)
		{
			LLViewerObject* objectp = NULL ;
			
			LLSelectNode* nodep = LLSelectMgr::instance().getHoverNode();
			if (nodep)
			{
				objectp = nodep->getObject();
			}

			if (objectp && !objectp->isDead())
			{
				S32 num_faces = objectp->mDrawable->getNumFaces() ;
				std::set<LLViewerFetchedTexture*> tex_list;

				for(S32 i = 0 ; i < num_faces; i++)
				{
					LLFace* facep = objectp->mDrawable->getFace(i) ;
					if(facep)
					{						
						LLViewerFetchedTexture* tex = dynamic_cast<LLViewerFetchedTexture*>(facep->getTexture()) ;
						if(tex)
						{
							if(tex_list.find(tex) != tex_list.end())
							{
								continue ; //already displayed.
							}
							tex_list.insert(tex);

							std::string uuid_str;
							tex->getID().toString(uuid_str);
							uuid_str = uuid_str.substr(0,7);

							addText(xpos, ypos, llformat("ID: %s v_size: %.3f", uuid_str.c_str(), tex->getMaxVirtualSize()));
							ypos += y_inc;

							addText(xpos, ypos, llformat("discard level: %d desired level: %d Missing: %s", tex->getDiscardLevel(), 
								tex->getDesiredDiscardLevel(), tex->isMissingAsset() ? "Y" : "N"));
							ypos += y_inc;
						}
					}
				}
			}
		}
		
		// <FS:ND> Report amount of failed texture buffer allocations if any.
		if( LLImageBase::getAllocationErrors() )
			addText( xpos, ypos, llformat( "# textures discarded due to insufficient memory %ld", LLImageBase::getAllocationErrors() ) );
		// </FS:ND>
	}

	void draw()
	{
		LL_RECORD_BLOCK_TIME(FTM_DISPLAY_DEBUG_TEXT);
		for (line_list_t::iterator iter = mLineList.begin();
			 iter != mLineList.end(); ++iter)
		{
			const Line& line = *iter;
			LLFontGL::getFontMonospace()->renderUTF8(line.text, 0, (F32)line.x, (F32)line.y, mTextColor,
											 LLFontGL::LEFT, LLFontGL::TOP,
											 LLFontGL::NORMAL, LLFontGL::NO_SHADOW, S32_MAX, S32_MAX, NULL, FALSE);
		}
		mLineList.clear();
	}

};

void LLViewerWindow::updateDebugText()
{
	mDebugText->update();
}

////////////////////////////////////////////////////////////////////////////
//
// LLViewerWindow
//

LLViewerWindow::Params::Params()
:	title("title"),
	name("name"),
	x("x"),
	y("y"),
	width("width"),
	height("height"),
	min_width("min_width"),
	min_height("min_height"),
	fullscreen("fullscreen", false),
	ignore_pixel_depth("ignore_pixel_depth", false)
{}


void LLViewerWindow::handlePieMenu(S32 x, S32 y, MASK mask)
{
    if (CAMERA_MODE_CUSTOMIZE_AVATAR != gAgentCamera.getCameraMode() && LLToolMgr::getInstance()->getCurrentTool() != LLToolPie::getInstance() && gAgent.isInitialized())
    {
        // If the current tool didn't process the click, we should show
        // the pie menu.  This can be done by passing the event to the pie
        // menu tool.
        LLToolPie::getInstance()->handleRightMouseDown(x, y, mask);
    }
}

BOOL LLViewerWindow::handleAnyMouseClick(LLWindow *window, LLCoordGL pos, MASK mask, EMouseClickType clicktype, BOOL down)
{
	const char* buttonname = "";
	const char* buttonstatestr = "";
	S32 x = pos.mX;
	S32 y = pos.mY;
	x = ll_round((F32)x / mDisplayScale.mV[VX]);
	y = ll_round((F32)y / mDisplayScale.mV[VY]);

	// only send mouse clicks to UI if UI is visible
	if(gPipeline.hasRenderDebugFeatureMask(LLPipeline::RENDER_DEBUG_FEATURE_UI))
	{	

		if (down)
		{
			buttonstatestr = "down" ;
		}
		else
		{
			buttonstatestr = "up" ;
		}
		
		switch (clicktype)
		{
		case CLICK_LEFT:
			mLeftMouseDown = down;
			buttonname = "Left";
			break;
		case CLICK_RIGHT:
			mRightMouseDown = down;
			buttonname = "Right";
			break;
		case CLICK_MIDDLE:
			mMiddleMouseDown = down;
			buttonname = "Middle";
			break;
		case CLICK_DOUBLELEFT:
			mLeftMouseDown = down;
			buttonname = "Left Double Click";
			break;
		case CLICK_BUTTON4:
			buttonname = "Button 4";
			break;
		case CLICK_BUTTON5:
			buttonname = "Button 5";
			break;
		default:
			break; // COUNT and NONE
		}
		
		LLView::sMouseHandlerMessage.clear();

		if (gMenuBarView)
		{
			// stop ALT-key access to menu
			gMenuBarView->resetMenuTrigger();
		}

		if (gDebugClicks)
		{	
			LL_INFOS() << "ViewerWindow " << buttonname << " mouse " << buttonstatestr << " at " << x << "," << y << LL_ENDL;
		}

		// Make sure we get a corresponding mouseup event, even if the mouse leaves the window
		if (down)
			mWindow->captureMouse();
		else
			mWindow->releaseMouse();

		// Indicate mouse was active
		LLUI::resetMouseIdleTimer();

		// Don't let the user move the mouse out of the window until mouse up.
		if( LLToolMgr::getInstance()->getCurrentTool()->clipMouseWhenDown() )
		{
			mWindow->setMouseClipping(down);
		}

		LLMouseHandler* mouse_captor = gFocusMgr.getMouseCapture();
		if( mouse_captor )
		{
			S32 local_x;
			S32 local_y;
			mouse_captor->screenPointToLocal( x, y, &local_x, &local_y );
			if (LLView::sDebugMouseHandling)
			{
				LL_INFOS() << buttonname << " Mouse " << buttonstatestr << " handled by captor " << mouse_captor->getName() << LL_ENDL;
			}

			BOOL r = mouse_captor->handleAnyMouseClick(local_x, local_y, mask, clicktype, down); 
			if (r) {

				LL_DEBUGS() << "LLViewerWindow::handleAnyMouseClick viewer with mousecaptor calling updatemouseeventinfo - local_x|global x  "<< local_x << " " << x  << "local/global y " << local_y << " " << y << LL_ENDL;

				LLViewerEventRecorder::instance().setMouseGlobalCoords(x,y);
				LLViewerEventRecorder::instance().logMouseEvent(std::string(buttonstatestr),std::string(buttonname)); 

			}
			else if (down && clicktype == CLICK_RIGHT)
			{
				handlePieMenu(x, y, mask);
				r = TRUE;
			}
			return r;
		}

		// Mark the click as handled and return if we aren't within the root view to avoid spurious bugs
		if( !mRootView->pointInView(x, y) )
		{
			return TRUE;
		}
		// Give the UI views a chance to process the click

		BOOL r= mRootView->handleAnyMouseClick(x, y, mask, clicktype, down) ;
		if (r) 
		{

			LL_DEBUGS() << "LLViewerWindow::handleAnyMouseClick calling updatemouseeventinfo - global x  "<< " " << x	<< "global y " << y	 << "buttonstate: " << buttonstatestr << " buttonname " << buttonname << LL_ENDL;

			LLViewerEventRecorder::instance().setMouseGlobalCoords(x,y);

			// Clear local coords - this was a click on root window so these are not needed
			// By not including them, this allows the test skeleton generation tool to be smarter when generating code
			// the code generator can be smarter because when local coords are present it can try the xui path with local coords
			// and fallback to global coordinates only if needed. 
			// The drawback to this approach is sometimes a valid xui path will appear to work fine, but NOT interact with the UI element
			// (VITA support not implemented yet or not visible to VITA due to widget further up xui path not being visible to VITA)
			// For this reason it's best to provide hints where possible here by leaving out local coordinates
			LLViewerEventRecorder::instance().setMouseLocalCoords(-1,-1);
			LLViewerEventRecorder::instance().logMouseEvent(buttonstatestr,buttonname); 

			if (LLView::sDebugMouseHandling)
			{
				LL_INFOS() << buttonname << " Mouse " << buttonstatestr << " " << LLViewerEventRecorder::instance().get_xui()	<< LL_ENDL;
			} 
			return TRUE;
		} else if (LLView::sDebugMouseHandling)
			{
				LL_INFOS() << buttonname << " Mouse " << buttonstatestr << " not handled by view" << LL_ENDL;
			}
	}

	// Do not allow tool manager to handle mouseclicks if we have disconnected	
	if(!gDisconnected && LLToolMgr::getInstance()->getCurrentTool()->handleAnyMouseClick( x, y, mask, clicktype, down ) )
	{
		LLViewerEventRecorder::instance().clear_xui(); 
		return TRUE;
	}

	if (down && clicktype == CLICK_RIGHT)
	{
		handlePieMenu(x, y, mask);
		return TRUE;
	}

	// If we got this far on a down-click, it wasn't handled.
	// Up-clicks, though, are always handled as far as the OS is concerned.
	BOOL default_rtn = !down;
	return default_rtn;
}

BOOL LLViewerWindow::handleMouseDown(LLWindow *window,  LLCoordGL pos, MASK mask)
{
    mAllowMouseDragging = FALSE;
    if (!mMouseDownTimer.getStarted())
    {
        mMouseDownTimer.start();
    }
    else
    {
        mMouseDownTimer.reset();
    }    
    BOOL down = TRUE;
    //handleMouse() loops back to LLViewerWindow::handleAnyMouseClick
    return gViewerInput.handleMouse(window, pos, mask, CLICK_LEFT, down);
}

BOOL LLViewerWindow::handleDoubleClick(LLWindow *window,  LLCoordGL pos, MASK mask)
{
	// try handling as a double-click first, then a single-click if that
	// wasn't handled.
	BOOL down = TRUE;
	if (gViewerInput.handleMouse(window, pos, mask, CLICK_DOUBLELEFT, down))
	{
		return TRUE;
	}
	return handleMouseDown(window, pos, mask);
}

BOOL LLViewerWindow::handleMouseUp(LLWindow *window,  LLCoordGL pos, MASK mask)
{
    if (mMouseDownTimer.getStarted())
    {
        mMouseDownTimer.stop();
    }
    BOOL down = FALSE;
    return gViewerInput.handleMouse(window, pos, mask, CLICK_LEFT, down);
}
BOOL LLViewerWindow::handleRightMouseDown(LLWindow *window,  LLCoordGL pos, MASK mask)
{
	BOOL down = TRUE;
	return gViewerInput.handleMouse(window, pos, mask, CLICK_RIGHT, down);
}

BOOL LLViewerWindow::handleRightMouseUp(LLWindow *window,  LLCoordGL pos, MASK mask)
{
	BOOL down = FALSE;
 	return gViewerInput.handleMouse(window, pos, mask, CLICK_RIGHT, down);
}

BOOL LLViewerWindow::handleMiddleMouseDown(LLWindow *window,  LLCoordGL pos, MASK mask)
{
	BOOL down = TRUE;
 	gViewerInput.handleMouse(window, pos, mask, CLICK_MIDDLE, down);
  
  	// Always handled as far as the OS is concerned.
	return TRUE;
}

LLWindowCallbacks::DragNDropResult LLViewerWindow::handleDragNDrop( LLWindow *window, LLCoordGL pos, MASK mask, LLWindowCallbacks::DragNDropAction action, std::string data)
{
	LLWindowCallbacks::DragNDropResult result = LLWindowCallbacks::DND_NONE;

	const bool prim_media_dnd_enabled = gSavedSettings.getBOOL("PrimMediaDragNDrop");
	const bool slurl_dnd_enabled = gSavedSettings.getBOOL("SLURLDragNDrop");
	
	if ( prim_media_dnd_enabled || slurl_dnd_enabled )
	{
		switch(action)
		{
			// Much of the handling for these two cases is the same.
			case LLWindowCallbacks::DNDA_TRACK:
			case LLWindowCallbacks::DNDA_DROPPED:
			case LLWindowCallbacks::DNDA_START_TRACKING:
			{
				bool drop = (LLWindowCallbacks::DNDA_DROPPED == action);
					
				if (slurl_dnd_enabled)
				{
					LLSLURL dropped_slurl(data);
					if(dropped_slurl.isSpatial())
					{
						if (drop)
						{
							LLURLDispatcher::dispatch( dropped_slurl.getSLURLString(), "clicked", NULL, true );
							return LLWindowCallbacks::DND_MOVE;
						}
						return LLWindowCallbacks::DND_COPY;
					}
				}

				if (prim_media_dnd_enabled)
				{
					LLPickInfo pick_info = pickImmediate( pos.mX, pos.mY,
                                                          TRUE /* pick_transparent */, 
                                                          FALSE /* pick_rigged */);

					LLUUID object_id = pick_info.getObjectID();
					S32 object_face = pick_info.mObjectFace;
					std::string url = data;

					LL_DEBUGS() << "Object: picked at " << pos.mX << ", " << pos.mY << " - face = " << object_face << " - URL = " << url << LL_ENDL;

					LLVOVolume *obj = dynamic_cast<LLVOVolume*>(static_cast<LLViewerObject*>(pick_info.getObject()));
				
					if (obj && !obj->getRegion()->getCapability("ObjectMedia").empty())
					{
						LLTextureEntry *te = obj->getTE(object_face);

						// can modify URL if we can modify the object or we have navigate permissions
						bool allow_modify_url = obj->permModify() || (te && obj->hasMediaPermission( te->getMediaData(), LLVOVolume::MEDIA_PERM_INTERACT ));

						if (te && allow_modify_url )
						{
							if (drop)
							{
								// object does NOT have media already
								if ( ! te->hasMedia() )
								{
									// we are allowed to modify the object
									if ( obj->permModify() )
									{
										// Create new media entry
										LLSD media_data;
										// XXX Should we really do Home URL too?
										media_data[LLMediaEntry::HOME_URL_KEY] = url;
										media_data[LLMediaEntry::CURRENT_URL_KEY] = url;
										media_data[LLMediaEntry::AUTO_PLAY_KEY] = true;
										obj->syncMediaData(object_face, media_data, true, true);
										// XXX This shouldn't be necessary, should it ?!?
										if (obj->getMediaImpl(object_face))
											obj->getMediaImpl(object_face)->navigateReload();
										obj->sendMediaDataUpdate();

										result = LLWindowCallbacks::DND_COPY;
									}
								}
								else 
								// object HAS media already
								{
									// URL passes the whitelist
									if (te->getMediaData()->checkCandidateUrl( url ) )
									{
										// just navigate to the URL
										if (obj->getMediaImpl(object_face))
										{
											obj->getMediaImpl(object_face)->navigateTo(url);
										}
										else 
										{
											// This is very strange.  Navigation should
											// happen via the Impl, but we don't have one.
											// This sends it to the server, which /should/
											// trigger us getting it.  Hopefully.
											LLSD media_data;
											media_data[LLMediaEntry::CURRENT_URL_KEY] = url;
											obj->syncMediaData(object_face, media_data, true, true);
											obj->sendMediaDataUpdate();
										}
										result = LLWindowCallbacks::DND_LINK;
										
									}
								}
								LLSelectMgr::getInstance()->unhighlightObjectOnly(mDragHoveredObject);
								mDragHoveredObject = NULL;
							
							}
							else 
							{
								// Check the whitelist, if there's media (otherwise just show it)
								if (te->getMediaData() == NULL || te->getMediaData()->checkCandidateUrl(url))
								{
									if ( obj != mDragHoveredObject)
									{
										// Highlight the dragged object
										LLSelectMgr::getInstance()->unhighlightObjectOnly(mDragHoveredObject);
										mDragHoveredObject = obj;
										LLSelectMgr::getInstance()->highlightObjectOnly(mDragHoveredObject);
									}
									result = (! te->hasMedia()) ? LLWindowCallbacks::DND_COPY : LLWindowCallbacks::DND_LINK;

								}
							}
						}
					}
				}
			}
			break;
			
			case LLWindowCallbacks::DNDA_STOP_TRACKING:
				// The cleanup case below will make sure things are unhilighted if necessary.
			break;
		}

		if (prim_media_dnd_enabled &&
			result == LLWindowCallbacks::DND_NONE && !mDragHoveredObject.isNull())
		{
			LLSelectMgr::getInstance()->unhighlightObjectOnly(mDragHoveredObject);
			mDragHoveredObject = NULL;
		}
	}
	
	return result;
}

BOOL LLViewerWindow::handleMiddleMouseUp(LLWindow *window,  LLCoordGL pos, MASK mask)
{
	BOOL down = FALSE;
 	gViewerInput.handleMouse(window, pos, mask, CLICK_MIDDLE, down);
  
  	// Always handled as far as the OS is concerned.
	return TRUE;
}

BOOL LLViewerWindow::handleOtherMouse(LLWindow *window, LLCoordGL pos, MASK mask, S32 button, bool down)
{
    switch (button)
    {
    case 4:
        gViewerInput.handleMouse(window, pos, mask, CLICK_BUTTON4, down);
        break;
    case 5:
        gViewerInput.handleMouse(window, pos, mask, CLICK_BUTTON5, down);
        break;
    default:
        break;
    }

    // Always handled as far as the OS is concerned.
    return TRUE;
}

BOOL LLViewerWindow::handleOtherMouseDown(LLWindow *window, LLCoordGL pos, MASK mask, S32 button)
{
    return handleOtherMouse(window, pos, mask, button, TRUE);
}

BOOL LLViewerWindow::handleOtherMouseUp(LLWindow *window, LLCoordGL pos, MASK mask, S32 button)
{
    return handleOtherMouse(window, pos, mask, button, FALSE);
}

// WARNING: this is potentially called multiple times per frame
void LLViewerWindow::handleMouseMove(LLWindow *window,  LLCoordGL pos, MASK mask)
{
	S32 x = pos.mX;
	S32 y = pos.mY;

	x = ll_round((F32)x / mDisplayScale.mV[VX]);
	y = ll_round((F32)y / mDisplayScale.mV[VY]);

	mMouseInWindow = TRUE;

	// Save mouse point for access during idle() and display()

	LLCoordGL mouse_point(x, y);

	if (mouse_point != mCurrentMousePoint)
	{
		LLUI::resetMouseIdleTimer();
	}

	saveLastMouse(mouse_point);

	mWindow->showCursorFromMouseMove();

	if (gAwayTimer.getElapsedTimeF32() > LLAgent::MIN_AFK_TIME
		&& !gDisconnected)
	{
		gAgent.clearAFK();
	}
}

void LLViewerWindow::handleMouseDragged(LLWindow *window,  LLCoordGL pos, MASK mask)
{
    if (mMouseDownTimer.getStarted())
    {
        if (mMouseDownTimer.getElapsedTimeF32() > 0.1)
        {
            mAllowMouseDragging = TRUE;
            mMouseDownTimer.stop();
        }
    }
    if(mAllowMouseDragging || !LLToolCamera::getInstance()->hasMouseCapture())
    {
        handleMouseMove(window, pos, mask);
    }
}

void LLViewerWindow::handleMouseLeave(LLWindow *window)
{
	// Note: we won't get this if we have captured the mouse.
	llassert( gFocusMgr.getMouseCapture() == NULL );
	mMouseInWindow = FALSE;
	LLToolTipMgr::instance().blockToolTips();
}

BOOL LLViewerWindow::handleCloseRequest(LLWindow *window)
{
	// User has indicated they want to close, but we may need to ask
	// about modified documents.
	LLAppViewer::instance()->userQuit();
	// Don't quit immediately
	return FALSE;
}

void LLViewerWindow::handleQuit(LLWindow *window)
{
	LLAppViewer::instance()->forceQuit();
}

void LLViewerWindow::handleResize(LLWindow *window,  S32 width,  S32 height)
{
	reshape(width, height);
	mResDirty = true;
}

// The top-level window has gained focus (e.g. via ALT-TAB)
void LLViewerWindow::handleFocus(LLWindow *window)
{
	gFocusMgr.setAppHasFocus(TRUE);
	LLModalDialog::onAppFocusGained();

	gAgent.onAppFocusGained();
	LLToolMgr::getInstance()->onAppFocusGained();

	// See if we're coming in with modifier keys held down
	if (gKeyboard)
	{
		gKeyboard->resetMaskKeys();
	}

	// resume foreground running timer
	// since we artifically limit framerate when not frontmost
	gForegroundTime.unpause();
}

// The top-level window has lost focus (e.g. via ALT-TAB)
void LLViewerWindow::handleFocusLost(LLWindow *window)
{
	gFocusMgr.setAppHasFocus(FALSE);
	//LLModalDialog::onAppFocusLost();
	LLToolMgr::getInstance()->onAppFocusLost();
	gFocusMgr.setMouseCapture( NULL );

	if (gMenuBarView)
	{
		// stop ALT-key access to menu
		gMenuBarView->resetMenuTrigger();
	}

	// restore mouse cursor
	showCursor();
	getWindow()->setMouseClipping(FALSE);

	// If losing focus while keys are down, reset them.
	if (gKeyboard)
	{
		gKeyboard->resetKeys();
	}

	// pause timer that tracks total foreground running time
	gForegroundTime.pause();
}


BOOL LLViewerWindow::handleTranslatedKeyDown(KEY key,  MASK mask, BOOL repeated)
{
	if (gAwayTimer.getElapsedTimeF32() > LLAgent::MIN_AFK_TIME)
	{
		gAgent.clearAFK();
	}

	// *NOTE: We want to interpret KEY_RETURN later when it arrives as
	// a Unicode char, not as a keydown.  Otherwise when client frame
	// rate is really low, hitting return sends your chat text before
	// it's all entered/processed.
	if (key == KEY_RETURN && mask == MASK_NONE)
	{
        // RIDER: although, at times some of the controlls (in particular the CEF viewer
        // would like to know about the KEYDOWN for an enter key... so ask and pass it along.
        LLFocusableElement* keyboard_focus = gFocusMgr.getKeyboardFocus();
        if (keyboard_focus && !keyboard_focus->wantsReturnKey())
    		return FALSE;
	}

    // remaps, handles ignored cases and returns back to viewer window.
    return gViewerInput.handleKey(key, mask, repeated);
}

BOOL LLViewerWindow::handleTranslatedKeyUp(KEY key,  MASK mask)
{
	// Let the inspect tool code check for ALT key to set LLToolSelectRect active instead LLToolCamera
	LLToolCompInspect * tool_inspectp = LLToolCompInspect::getInstance();
	if (LLToolMgr::getInstance()->getCurrentTool() == tool_inspectp)
	{
		tool_inspectp->keyUp(key, mask);
	}

	return gViewerInput.handleKeyUp(key, mask);
}

void LLViewerWindow::handleScanKey(KEY key, BOOL key_down, BOOL key_up, BOOL key_level)
{
	LLViewerJoystick::getInstance()->setCameraNeedsUpdate(true);
	gViewerInput.scanKey(key, key_down, key_up, key_level);
	return; // Be clear this function returns nothing
}




BOOL LLViewerWindow::handleActivate(LLWindow *window, BOOL activated)
{
	if (activated)
	{
		mActive = true;
		send_agent_resume();
		gAgent.clearAFK();
		
		// Unmute audio
		audio_update_volume();
	}
	else
	{
		mActive = false;
				
		// if the user has chosen to go Away automatically after some time, then go Away when minimizing
		if (gSavedSettings.getS32("AFKTimeout"))
		{
			gAgent.setAFK();
		}
		
		// SL-53351: Make sure we're not in mouselook when minimised, to prevent control issues
		if (gAgentCamera.getCameraMode() == CAMERA_MODE_MOUSELOOK)
		{
			gAgentCamera.changeCameraToDefault();
		}
		
		send_agent_pause();
	
		// Mute audio
		audio_update_volume();
	}
	return TRUE;
}

BOOL LLViewerWindow::handleActivateApp(LLWindow *window, BOOL activating)
{
	//if (!activating) gAgentCamera.changeCameraToDefault();

	LLViewerJoystick::getInstance()->setNeedsReset(true);
	return FALSE;
}


void LLViewerWindow::handleMenuSelect(LLWindow *window,  S32 menu_item)
{
}


BOOL LLViewerWindow::handlePaint(LLWindow *window,  S32 x,  S32 y, S32 width,  S32 height)
{
	// *TODO: Enable similar information output for other platforms?  DK 2011-02-18
#if LL_WINDOWS
	if (gHeadlessClient)
	{
		HWND window_handle = (HWND)window->getPlatformWindow();
		PAINTSTRUCT ps; 
		HDC hdc; 
 
		RECT wnd_rect;
		wnd_rect.left = 0;
		wnd_rect.top = 0;
		wnd_rect.bottom = 200;
		wnd_rect.right = 500;

		hdc = BeginPaint(window_handle, &ps); 
		//SetBKColor(hdc, RGB(255, 255, 255));
		FillRect(hdc, &wnd_rect, CreateSolidBrush(RGB(255, 255, 255)));

		std::string temp_str;
		LLTrace::Recording& recording = LLViewerStats::instance().getRecording();
		temp_str = llformat( "FPS %3.1f Phy FPS %2.1f Time Dil %1.3f",		/* Flawfinder: ignore */
				recording.getPerSec(LLStatViewer::FPS), //mFPSStat.getMeanPerSec(),
				recording.getLastValue(LLStatViewer::SIM_PHYSICS_FPS), 
				recording.getLastValue(LLStatViewer::SIM_TIME_DILATION));
		S32 len = temp_str.length();
		TextOutA(hdc, 0, 0, temp_str.c_str(), len); 


		LLVector3d pos_global = gAgent.getPositionGlobal();
		temp_str = llformat( "Avatar pos %6.1lf %6.1lf %6.1lf", pos_global.mdV[0], pos_global.mdV[1], pos_global.mdV[2]);
		len = temp_str.length();
		TextOutA(hdc, 0, 25, temp_str.c_str(), len); 

		TextOutA(hdc, 0, 50, "Set \"HeadlessClient FALSE\" in settings.ini file to reenable", 61);
		EndPaint(window_handle, &ps); 
		return TRUE;
	}
#endif
	return FALSE;
}


void LLViewerWindow::handleScrollWheel(LLWindow *window,  S32 clicks)
{
	handleScrollWheel( clicks );
}

void LLViewerWindow::handleScrollHWheel(LLWindow *window,  S32 clicks)
{
	handleScrollHWheel(clicks);
}

void LLViewerWindow::handleWindowBlock(LLWindow *window)
{
	send_agent_pause();
}

void LLViewerWindow::handleWindowUnblock(LLWindow *window)
{
	send_agent_resume();
}

void LLViewerWindow::handleDataCopy(LLWindow *window, S32 data_type, void *data)
{
	const S32 SLURL_MESSAGE_TYPE = 0;
	switch (data_type)
	{
	case SLURL_MESSAGE_TYPE:
		// received URL
		std::string url = (const char*)data;
		LLMediaCtrl* web = NULL;
		const bool trusted_browser = false;
		// don't treat slapps coming from external browsers as "clicks" as this would bypass throttling
		if (LLURLDispatcher::dispatch(url, "", web, trusted_browser))
		{
			// bring window to foreground, as it has just been "launched" from a URL
			mWindow->bringToFront();
		}
		break;
	}
}

BOOL LLViewerWindow::handleTimerEvent(LLWindow *window)
{
	if (LLViewerJoystick::getInstance()->getOverrideCamera())
	{
		LLViewerJoystick::getInstance()->updateStatus();
		return TRUE;
	}
	return FALSE;
}

BOOL LLViewerWindow::handleDeviceChange(LLWindow *window)
{
	// give a chance to use a joystick after startup (hot-plugging)
	if (!LLViewerJoystick::getInstance()->isJoystickInitialized() )
	{
		LLViewerJoystick::getInstance()->init(true);
		return TRUE;
	}
	return FALSE;
}

BOOL LLViewerWindow::handleDPIChanged(LLWindow *window, F32 ui_scale_factor, S32 window_width, S32 window_height)
{
    if (ui_scale_factor >= MIN_UI_SCALE && ui_scale_factor <= MAX_UI_SCALE)
    {
        LLViewerWindow::reshape(window_width, window_height);
        mResDirty = true;
        return TRUE;
    }
    else
    {
        LL_WARNS() << "DPI change caused UI scale to go out of bounds: " << ui_scale_factor << LL_ENDL;
        return FALSE;
    }
}

BOOL LLViewerWindow::handleWindowDidChangeScreen(LLWindow *window)
{
	LLCoordScreen window_rect;
	mWindow->getSize(&window_rect);
	reshape(window_rect.mX, window_rect.mY);
	return TRUE;
}

void LLViewerWindow::handlePingWatchdog(LLWindow *window, const char * msg)
{
	LLAppViewer::instance()->pingMainloopTimeout(msg);
}


void LLViewerWindow::handleResumeWatchdog(LLWindow *window)
{
	LLAppViewer::instance()->resumeMainloopTimeout();
}

void LLViewerWindow::handlePauseWatchdog(LLWindow *window)
{
	LLAppViewer::instance()->pauseMainloopTimeout();
}

//virtual
std::string LLViewerWindow::translateString(const char* tag)
{
	return LLTrans::getString( std::string(tag) );
}

//virtual
std::string LLViewerWindow::translateString(const char* tag,
		const std::map<std::string, std::string>& args)
{
	// LLTrans uses a special subclass of std::string for format maps,
	// but we must use std::map<> in these callbacks, otherwise we create
	// a dependency between LLWindow and LLFormatMapString.  So copy the data.
	LLStringUtil::format_map_t args_copy;
	std::map<std::string,std::string>::const_iterator it = args.begin();
	for ( ; it != args.end(); ++it)
	{
		args_copy[it->first] = it->second;
	}
	return LLTrans::getString( std::string(tag), args_copy);
}

//
// Classes
//
LLViewerWindow::LLViewerWindow(const Params& p)
:	mWindow(NULL),
	mActive(true),
	mUIVisible(true),
	mWindowRectRaw(0, p.height, p.width, 0),
	mWindowRectScaled(0, p.height, p.width, 0),
	mWorldViewRectRaw(0, p.height, p.width, 0),
	mLeftMouseDown(FALSE),
	mMiddleMouseDown(FALSE),
	mRightMouseDown(FALSE),
	mMouseInWindow( FALSE ),
    mAllowMouseDragging(TRUE),
    mMouseDownTimer(),
	mLastMask( MASK_NONE ),
	mToolStored( NULL ),
	mHideCursorPermanent( FALSE ),
	mCursorHidden(FALSE),
	mIgnoreActivate( FALSE ),
	mResDirty(false),
	mStatesDirty(false),
	mCurrResolutionIndex(0),
	mProgressView(NULL),
	mProgressViewMini(NULL)
{
	// gKeyboard is still NULL, so it doesn't do LLWindowListener any good to
	// pass its value right now. Instead, pass it a nullary function that
	// will, when we later need it, return the value of gKeyboard.
	// boost::lambda::var() constructs such a functor on the fly.
	mWindowListener.reset(new LLWindowListener(this, boost::lambda::var(gKeyboard)));
	mViewerWindowListener.reset(new LLViewerWindowListener(this));

	mSystemChannel.reset(new LLNotificationChannel("System", "Visible", LLNotificationFilters::includeEverything));
	mCommunicationChannel.reset(new LLCommunicationChannel("Communication", "Visible"));
	mAlertsChannel.reset(new LLNotificationsUI::LLViewerAlertHandler("VW_alerts", "alert"));
	mModalAlertsChannel.reset(new LLNotificationsUI::LLViewerAlertHandler("VW_alertmodal", "alertmodal"));

	bool ignore = gSavedSettings.getBOOL("IgnoreAllNotifications");
	LLNotifications::instance().setIgnoreAllNotifications(ignore);
	if (ignore)
	{
	LL_INFOS() << "NOTE: ALL NOTIFICATIONS THAT OCCUR WILL GET ADDED TO IGNORE LIST FOR LATER RUNS." << LL_ENDL;
	}


	BOOL useLegacyCursors = gSavedSettings.getBOOL("FSUseLegacyCursors");//<FS:LO> Legacy cursor setting from main program

	/*
	LLWindowCallbacks* callbacks,
	const std::string& title, const std::string& name, S32 x, S32 y, S32 width, S32 height, U32 flags,
	BOOL fullscreen, 
	BOOL clearBg,
	BOOL disable_vsync,
	BOOL ignore_pixel_depth,
	U32 fsaa_samples)
	*/
	// create window
	mWindow = LLWindowManager::createWindow(this,
		p.title, p.name, p.x, p.y, p.width, p.height, 0,
		p.fullscreen, 
		gHeadlessClient,
		gSavedSettings.getBOOL("DisableVerticalSync"),
		!gHeadlessClient,
		p.ignore_pixel_depth,
		//gSavedSettings.getBOOL("RenderDeferred") ? 0 : gSavedSettings.getU32("RenderFSAASamples")); //don't use window level anti-aliasing if FBOs are enabled
		gSavedSettings.getBOOL("RenderDeferred") ? 0 : gSavedSettings.getU32("RenderFSAASamples"), //don't use window level anti-aliasing if FBOs are enabled
		useLegacyCursors); // <FS:LO> Legacy cursor setting from main program

	if (!LLViewerShaderMgr::sInitialized)
	{ //immediately initialize shaders
		LLViewerShaderMgr::sInitialized = TRUE;
		LLViewerShaderMgr::instance()->setShaders();
	}

	if (NULL == mWindow)
	{
		LLSplashScreen::update(LLTrans::getString("StartupRequireDriverUpdate"));
	
		LL_WARNS("Window") << "Failed to create window, to be shutting Down, be sure your graphics driver is updated." << LL_ENDL ;

		ms_sleep(5000) ; //wait for 5 seconds.

		LLSplashScreen::update(LLTrans::getString("ShuttingDown"));
#if LL_LINUX || LL_SOLARIS
		LL_WARNS() << "Unable to create window, be sure screen is set at 32-bit color and your graphics driver is configured correctly.  See README-linux.txt or README-solaris.txt for further information."
				<< LL_ENDL;
#else
		LL_WARNS("Window") << "Unable to create window, be sure screen is set at 32-bit color in Control Panels->Display->Settings"
				<< LL_ENDL;
#endif
        LLAppViewer::instance()->fastQuit(1);
	}
	
	if (!LLAppViewer::instance()->restoreErrorTrap())
	{
		LL_WARNS("Window") << " Someone took over my signal/exception handler (post createWindow)!" << LL_ENDL;
	}

	const bool do_not_enforce = false;
	mWindow->setMinSize(p.min_width, p.min_height, do_not_enforce);  // root view not set 
	LLCoordScreen scr;
    mWindow->getSize(&scr);

    // Reset UI scale factor on first run if OS's display scaling is not 100%
    if (gSavedSettings.getBOOL("ResetUIScaleOnFirstRun"))
    {
        if (mWindow->getSystemUISize() != 1.f)
        {
            gSavedSettings.setF32("UIScaleFactor", 1.f);
        }
        gSavedSettings.setBOOL("ResetUIScaleOnFirstRun", FALSE);
    }

	// Get the real window rect the window was created with (since there are various OS-dependent reasons why
	// the size of a window or fullscreen context may have been adjusted slightly...)
	F32 ui_scale_factor = llclamp(gSavedSettings.getF32("UIScaleFactor") * mWindow->getSystemUISize(), MIN_UI_SCALE, MAX_UI_SCALE);
	
	mDisplayScale.setVec(llmax(1.f / mWindow->getPixelAspectRatio(), 1.f), llmax(mWindow->getPixelAspectRatio(), 1.f));
	mDisplayScale *= ui_scale_factor;
	LLUI::setScaleFactor(mDisplayScale);

	{
		LLCoordWindow size;
		mWindow->getSize(&size);
		mWindowRectRaw.set(0, size.mY, size.mX, 0);
		mWindowRectScaled.set(0, ll_round((F32)size.mY / mDisplayScale.mV[VY]), ll_round((F32)size.mX / mDisplayScale.mV[VX]), 0);
	}
	
	LLFontManager::initClass();

	//
	// We want to set this stuff up BEFORE we initialize the pipeline, so we can turn off
	// stuff like AGP if we think that it'll crash the viewer.
	//
	LL_DEBUGS("Window") << "Loading feature tables." << LL_ENDL;

	LLFeatureManager::getInstance()->init();

	// Initialize OpenGL Renderer
	if (!LLFeatureManager::getInstance()->isFeatureAvailable("RenderVBOEnable") ||
		!gGLManager.mHasVertexBufferObject)
	{
		gSavedSettings.setBOOL("RenderVBOEnable", FALSE);
	}
	LLVertexBuffer::initClass(gSavedSettings.getBOOL("RenderVBOEnable"), gSavedSettings.getBOOL("RenderVBOMappingDisable"));
	LL_INFOS("RenderInit") << "LLVertexBuffer initialization done." << LL_ENDL ;
	gGL.init() ;
	// <FS:Ansariel> Exodus vignette
	exoPostProcess::getInstance(); // Make sure we've created one of these

	if (LLFeatureManager::getInstance()->isSafe()
		|| (gSavedSettings.getS32("LastFeatureVersion") != LLFeatureManager::getInstance()->getVersion())
		|| (gSavedSettings.getString("LastGPUString") != LLFeatureManager::getInstance()->getGPUString())
		|| (gSavedSettings.getBOOL("ProbeHardwareOnStartup")))
	{
		LLFeatureManager::getInstance()->applyRecommendedSettings();
		gSavedSettings.setBOOL("ProbeHardwareOnStartup", FALSE);
	}

	if (!gGLManager.mHasDepthClamp)
	{
		LL_INFOS("RenderInit") << "Missing feature GL_ARB_depth_clamp. Void water might disappear in rare cases." << LL_ENDL;
	}
	
	// If we crashed while initializng GL stuff last time, disable certain features
	if (gSavedSettings.getBOOL("RenderInitError"))
	{
		mInitAlert = "DisplaySettingsNoShaders";
		LLFeatureManager::getInstance()->setGraphicsLevel(0, false);
		gSavedSettings.setU32("RenderQualityPerformance", 0);		
	}

	// <FS:Ansariel> Texture memory management
	// On 64bit builds, allow up to 1GB texture memory on cards with 2GB video
	// memory and up to 2GB texture memory on cards with 4GB video memory. Check
	// is performed against a lower limit as not exactly 2 or 4GB might not be
	// returned.
#if ADDRESS_SIZE == 64
	LL_INFOS() << "GLManager detected " << gGLManager.mVRAM << " MB VRAM" << LL_ENDL;

	if (gGLManager.mVRAM > 3584)
	{
		gMaxVideoRam = S32Megabytes(2048);
		LL_INFOS() << "At least 4 GB video memory detected - increasing max video ram for textures to 2048 MB" << LL_ENDL;
	}
	else if (gGLManager.mVRAM > 1536)
	{
		gMaxVideoRam = S32Megabytes(1024);
		LL_INFOS() << "At least 2 GB video memory detected - increasing max video ram for textures to 1024 MB" << LL_ENDL;
	}
	else if (gGLManager.mVRAM > 768)
	{
		gMaxVideoRam = S32Megabytes(768);
		LL_INFOS() << "At least 1 GB video memory detected - increasing max video ram for textures to 768 MB" << LL_ENDL;
	}
#endif
	// </FS:Ansariel>

	// <FS:Ansariel> Max texture resolution
#if ADDRESS_SIZE == 64
	if (gSavedSettings.getBOOL("FSRestrictMaxTextureSize"))
	{
		DESIRED_NORMAL_TEXTURE_SIZE = (U32)LLViewerFetchedTexture::MAX_IMAGE_SIZE_DEFAULT / 2;
	}
#else
	gSavedSettings.setBOOL("FSRestrictMaxTextureSize", TRUE);
#endif
	LL_INFOS() << "Maximum fetched texture size: " << DESIRED_NORMAL_TEXTURE_SIZE << "px" << LL_ENDL;
	// </FS:Ansariel>
		
	// Init the image list.  Must happen after GL is initialized and before the images that
	// LLViewerWindow needs are requested.
	LLImageGL::initClass(LLViewerTexture::MAX_GL_IMAGE_CATEGORY) ;
	gTextureList.init();
	LLViewerTextureManager::init() ;
	gBumpImageList.init();
	
	// Init font system, but don't actually load the fonts yet
	// because our window isn't onscreen and they take several
	// seconds to parse.
	LLFontGL::initClass( gSavedSettings.getF32("FontScreenDPI"),
								mDisplayScale.mV[VX],
								mDisplayScale.mV[VY],
								gDirUtilp->getAppRODataDir(),
								gSavedSettings.getString("FSFontSettingsFile"),
								gSavedSettings.getF32("FSFontSizeAdjustment"));
	
	// Create container for all sub-views
	LLView::Params rvp;
	rvp.name("root");
	rvp.rect(mWindowRectScaled);
	rvp.mouse_opaque(false);
	rvp.follows.flags(FOLLOWS_NONE);
	mRootView = LLUICtrlFactory::create<LLRootView>(rvp);
	LLUI::setRootView(mRootView);

	// Make avatar head look forward at start
	mCurrentMousePoint.mX = getWindowWidthScaled() / 2;
	mCurrentMousePoint.mY = getWindowHeightScaled() / 2;

	gShowOverlayTitle = gSavedSettings.getBOOL("ShowOverlayTitle");
	mOverlayTitle = gSavedSettings.getString("OverlayTitle");
	// Can't have spaces in settings.ini strings, so use underscores instead and convert them.
	LLStringUtil::replaceChar(mOverlayTitle, '_', ' ');

	mDebugText = new LLDebugText(this);

	mWorldViewRectScaled = calcScaledRect(mWorldViewRectRaw, mDisplayScale);
}

std::string LLViewerWindow::getLastSnapshotDir()
{
    return sSnapshotDir;
}

void LLViewerWindow::initGLDefaults()
{
	gGL.setSceneBlendType(LLRender::BT_ALPHA);

	if (!LLGLSLShader::sNoFixedFunction)
	{ //initialize fixed function state
		glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );

		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,LLColor4::black.mV);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,LLColor4::white.mV);

		// lights for objects
		glShadeModel( GL_SMOOTH );

		gGL.getTexUnit(0)->enable(LLTexUnit::TT_TEXTURE);
		gGL.getTexUnit(0)->setTextureBlendType(LLTexUnit::TB_MULT);
	}

	glPixelStorei(GL_PACK_ALIGNMENT,1);
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);

	gGL.setAmbientLightColor(LLColor4::black);
		
	glCullFace(GL_BACK);

	// RN: Need this for translation and stretch manip.
	gBox.prerender();
}

struct MainPanel : public LLPanel
{
};

void LLViewerWindow::initBase()
{
	S32 height = getWindowHeightScaled();
	S32 width = getWindowWidthScaled();

	LLRect full_window(0, height, width, 0);

	////////////////////
	//
	// Set the gamma
	//

	F32 gamma = gSavedSettings.getF32("RenderGamma");
	if (gamma != 0.0f)
	{
		getWindow()->setGamma(gamma);
	}

	// Create global views

	// Login screen and main_view.xml need edit menus for preferences and browser
	LL_DEBUGS("AppInit") << "initializing edit menu" << LL_ENDL;
	initialize_edit_menu();
	initialize_spellcheck_menu(); // <FS:Zi> Set up edit menu here to get the spellcheck callbacks assigned before anyone uses them

	// <FS:Ansariel> Move console further down in the view hierarchy to not float in front of floaters!
	// Console
	llassert( !gConsole );
	LLConsole::Params cp;
	cp.name("console");
	cp.max_lines(gSavedSettings.getS32("ConsoleBufferSize"));
	cp.rect(getChatConsoleRect());
	cp.parse_urls(true); // <FS:Ansariel> Enable URL parsing for the chat console
	cp.background_image("Rounded_Square"); // <FS:Ansariel> Configurable background for different console types
	cp.session_support(true); // <FS:Ansariel> Session support
	cp.persist_time(gSavedSettings.getF32("ChatPersistTime"));
	cp.font_size_index(gSavedSettings.getS32("ChatConsoleFontSize"));
	cp.follows.flags(FOLLOWS_LEFT | FOLLOWS_RIGHT | FOLLOWS_BOTTOM);
	gConsole = LLUICtrlFactory::create<LLConsole>(cp);
	getRootView()->addChild(gConsole);
	// </FS:Ansariel>

	//<FS:KC> Centralize a some of these volume panel callbacks
	initialize_volume_controls_callbacks();
	//</FS:KC>

	// Create the floater view at the start so that other views can add children to it. 
	// (But wait to add it as a child of the root view so that it will be in front of the 
	// other views.)
	MainPanel* main_view = new MainPanel();
	if (!main_view->buildFromFile("main_view.xml"))
	{
		LL_ERRS() << "Failed to initialize viewer: Viewer couldn't process file main_view.xml, "
				<< "if this problem happens again, please validate your installation." << LL_ENDL;
	}
	main_view->setShape(full_window);
	getRootView()->addChild(main_view);

	// <FS:Zi> Moved this from the end of this function up here, so all context menus
	//         created right after this get the correct parent assigned.
	gMenuHolder = getRootView()->getChild<LLViewerMenuHolderGL>("Menu Holder");
	LLMenuGL::sMenuContainer = gMenuHolder;
	// </FS:Zi>

	// placeholder widget that controls where "world" is rendered
	mWorldViewPlaceholder = main_view->getChildView("world_view_rect")->getHandle();
	mPopupView = main_view->getChild<LLPopupView>("popup_holder");
	mHintHolder = main_view->getChild<LLView>("hint_holder")->getHandle();
	mLoginPanelHolder = main_view->getChild<LLView>("login_panel_holder")->getHandle();

	// Create the toolbar view
	// Get a pointer to the toolbar view holder
	LLPanel* panel_holder = main_view->getChild<LLPanel>("toolbar_view_holder");
	// Load the toolbar view from file 
	gToolBarView = LLUICtrlFactory::getInstance()->createFromFile<LLToolBarView>("panel_toolbar_view.xml", panel_holder, LLDefaultChildRegistry::instance());
	if (!gToolBarView)
	{
		LL_ERRS() << "Failed to initialize viewer: Viewer couldn't process file panel_toolbar_view.xml, "
				<< "if this problem happens again, please validate your installation." << LL_ENDL;
	}
	gToolBarView->setShape(panel_holder->getLocalRect());
	// Hide the toolbars for the moment: we'll make them visible after logging in world (see LLViewerWindow::initWorldUI())
	gToolBarView->setVisible(FALSE);

	// <FS:Zi> initialize the utility bar (classic V1 style buttons next to the chat bar)
	UtilityBar::instance().init();

	// Constrain floaters to inside the menu and status bar regions.
	gFloaterView = main_view->getChild<LLFloaterView>("Floater View");
	for (S32 i = 0; i < LLToolBarEnums::TOOLBAR_COUNT; ++i)
	{
		LLToolBar * toolbarp = gToolBarView->getToolbar((LLToolBarEnums::EToolBarLocation)i);
		if (toolbarp)
		{
			toolbarp->getCenterLayoutPanel()->setReshapeCallback(boost::bind(&LLFloaterView::setToolbarRect, gFloaterView, _1, _2));
		}
	}
	gFloaterView->setFloaterSnapView(main_view->getChild<LLView>("floater_snap_region")->getHandle());
	gSnapshotFloaterView = main_view->getChild<LLSnapshotFloaterView>("Snapshot Floater View");

	// <FS:Ansariel> Prevent floaters being dragged under main chat bar
	LLLayoutPanel* chatbar_panel = dynamic_cast<LLLayoutPanel*>(gToolBarView->getChildView("default_chat_bar")->getParent());
	if (chatbar_panel)
	{
		chatbar_panel->setReshapePanelCallback(boost::bind(&LLFloaterView::setMainChatbarRect, gFloaterView, _1, _2));
		gFloaterView->setMainChatbarRect(chatbar_panel, chatbar_panel->getRect());
	}
	// </FS:Ansariel>

	// optionally forward warnings to chat console/chat floater
	// for qa runs and dev builds
#if  !LL_RELEASE_FOR_DOWNLOAD
	RecordToChatConsole::getInstance()->startRecorder();
#else
	if(gSavedSettings.getBOOL("QAMode"))
	{
		RecordToChatConsole::getInstance()->startRecorder();
	}
#endif

	gDebugView = getRootView()->getChild<LLDebugView>("DebugView");
	gDebugView->init();
	gToolTipView = getRootView()->getChild<LLToolTipView>("tooltip view");

	// Initialize do not disturb response message when logged in
	LLAppViewer::instance()->setOnLoginCompletedCallback(boost::bind(&LLFloaterPreference::initDoNotDisturbResponse));

	// Add the progress bar view (startup view), which overrides everything
	mProgressView = getRootView()->findChild<LLProgressView>("progress_view");
	mProgressViewMini = getRootView()->findChild<LLProgressViewMini>("progress_view_mini");

	setShowProgress(FALSE,FALSE);
	setProgressCancelButtonVisible(FALSE);

	if(mProgressViewMini)
		mProgressViewMini->setVisible(FALSE);
	// <FS:Zi> Moved this to the top right after creation of main_view.xml, so all context menus
	//         created right after that get the correct parent assigned.
	// gMenuHolder = getRootView()->getChild<LLViewerMenuHolderGL>("Menu Holder");
	// LLMenuGL::sMenuContainer = gMenuHolder;
	// </FS:Zi>
}

void LLViewerWindow::initWorldUI()
{
	S32 height = mRootView->getRect().getHeight();
	S32 width = mRootView->getRect().getWidth();
	LLRect full_window(0, height, width, 0);


	gIMMgr = LLIMMgr::getInstance();

	//getRootView()->sendChildToFront(gFloaterView);
	//getRootView()->sendChildToFront(gSnapshotFloaterView);

	// <FS:Ansariel> Group notices, IMs and chiclets position
	//LLPanel* chiclet_container = getRootView()->getChild<LLPanel>("chiclet_container");
	LLPanel* chiclet_container;
	if (gSavedSettings.getBOOL("InternalShowGroupNoticesTopRight"))
	{
		chiclet_container = getRootView()->getChild<LLPanel>("chiclet_container");
		getRootView()->getChildView("chiclet_container_bottom")->setVisible(FALSE);
	}
	else
	{
		getRootView()->getChildView("chiclet_container")->setVisible(FALSE);
		chiclet_container = getRootView()->getChild<LLPanel>("chiclet_container_bottom");
	}
	// </FS:Ansariel> Group notices, IMs and chiclets position
	LLChicletBar* chiclet_bar = LLChicletBar::getInstance();
	chiclet_bar->setShape(chiclet_container->getLocalRect());
	chiclet_bar->setFollowsAll();
	chiclet_container->addChild(chiclet_bar);
	chiclet_container->setVisible(TRUE);

	LLRect morph_view_rect = full_window;
	morph_view_rect.stretch( -STATUS_BAR_HEIGHT );
	morph_view_rect.mTop = full_window.mTop - 32;
	LLMorphView::Params mvp;
	mvp.name("MorphView");
	mvp.rect(morph_view_rect);
	mvp.visible(false);
	gMorphView = LLUICtrlFactory::create<LLMorphView>(mvp);
	getRootView()->addChild(gMorphView);

	LLWorldMapView::initClass();
	
	// Force gFloaterWorldMap to initialize
	LLFloaterReg::getInstance("world_map");

	// Force gFloaterTools to initialize
	LLFloaterReg::getInstance("build");


	// Status bar
	LLPanel* status_bar_container = getRootView()->getChild<LLPanel>("status_bar_container");
	gStatusBar = new LLStatusBar(status_bar_container->getLocalRect());
	gStatusBar->setFollowsAll();
	gStatusBar->setShape(status_bar_container->getLocalRect());
	// sync bg color with menu bar
	gStatusBar->setBackgroundColor( gMenuBarView->getBackgroundColor().get() );
	status_bar_container->addChildInBack(gStatusBar);
	status_bar_container->setVisible(TRUE);

	// <FS:Zi> Make navigation bar part of the UI
	// // Navigation bar
	// LLPanel* nav_bar_container = getRootView()->getChild<LLPanel>("topinfo_bar_container");

	// LLNavigationBar* navbar = LLNavigationBar::getInstance();
	// navbar->setShape(nav_bar_container->getLocalRect());
	// navbar->setBackgroundColor(gMenuBarView->getBackgroundColor().get());
	// nav_bar_container->addChild(navbar);
	// nav_bar_container->setVisible(TRUE);

	// if (!gSavedSettings.getBOOL("ShowNavbarNavigationPanel"))
	// {
	//		navbar->setVisible(FALSE);
	// 	}

	// Force navigation bar to initialize
	LLNavigationBar::getInstance();
	// set navbar container visible which is initially hidden on the login screen,
	// the real visibility of navbar and favorites bar is done via visibility control -Zi
	LLNavigationBar::instance().getView()->setVisible(TRUE);
	// </FS:Zi>

	if (!gSavedSettings.getBOOL("ShowMenuBarLocation"))
	{
		gStatusBar->childSetVisible("parcel_info_panel",FALSE);
	}
	

	// <FS:Zi> We don't have the mini location bar, so no topinfo_bar required
	// // Top Info bar
	// LLPanel* topinfo_bar_container = getRootView()->getChild<LLPanel>("topinfo_bar_container");
	// LLPanelTopInfoBar* topinfo_bar = LLPanelTopInfoBar::getInstance();

	// topinfo_bar->setShape(topinfo_bar_container->getLocalRect());

	// topinfo_bar_container->addChild(topinfo_bar);
	// topinfo_bar_container->setVisible(TRUE);

	// if (!gSavedSettings.getBOOL("ShowMiniLocationPanel"))
	// {
	// 	topinfo_bar->setVisible(FALSE);
	// }
	// </FS:Zi>

	if ( gHUDView == NULL )
	{
		LLRect hud_rect = full_window;
		hud_rect.mBottom += 50;
		if (gMenuBarView && gMenuBarView->isInVisibleChain())
		{
			hud_rect.mTop -= gMenuBarView->getRect().getHeight();
		}
		gHUDView = new LLHUDView(hud_rect);
		getRootView()->addChild(gHUDView);
		getRootView()->sendChildToBack(gHUDView);
	}

	LLPanel* panel_ssf_container = getRootView()->getChild<LLPanel>("state_management_buttons_container");

	LLPanelStandStopFlying* panel_stand_stop_flying	= LLPanelStandStopFlying::getInstance();
	panel_ssf_container->addChild(panel_stand_stop_flying);

	panel_ssf_container->setVisible(TRUE);

	LLMenuOptionPathfindingRebakeNavmesh::getInstance()->initialize();

	// Load and make the toolbars visible
	// Note: we need to load the toolbars only *after* the user is logged in and IW
	if (gToolBarView)
	{
		if (gSavedSettings.getBOOL("ResetToolbarSettings"))
		{
			gToolBarView->loadDefaultToolbars();
			gSavedSettings.setBOOL("ResetToolbarSettings",FALSE);
		}
		else
		{
			gToolBarView->loadToolbars();
		}
		gToolBarView->setVisible(TRUE);
	}
// <FS:AW  opensim destinations and avatar picker>
// 	LLMediaCtrl* destinations = LLFloaterReg::getInstance("destinations")->getChild<LLMediaCtrl>("destination_guide_contents");
// 	if (destinations)
// 	{
// 		destinations->setErrorPageURL(gSavedSettings.getString("GenericErrorPageURL"));
// 		std::string url = gSavedSettings.getString("DestinationGuideURL");
// 		url = LLWeb::expandURLSubstitutions(url, LLSD());
// 		destinations->navigateTo(url, "text/html");
// 	}
// 	LLMediaCtrl* avatar_picker = LLFloaterReg::getInstance("avatar")->findChild<LLMediaCtrl>("avatar_picker_contents");
// 	if (avatar_picker)
// 	{
// 		avatar_picker->setErrorPageURL(gSavedSettings.getString("GenericErrorPageURL"));
// 		std::string url = gSavedSettings.getString("AvatarPickerURL");
// 		url = LLWeb::expandURLSubstitutions(url, LLSD());
// 		avatar_picker->navigateTo(url, "text/html");
// 	}
	std::string destination_guide_url;
#ifdef OPENSIM // <FS:AW optional opensim support>
	if (LLGridManager::getInstance()->isInOpenSim())
	{
		if (LLLoginInstance::getInstance()->hasResponse("destination_guide_url"))
		{
			destination_guide_url = LLLoginInstance::getInstance()->getResponse("destination_guide_url").asString();
		}
	}
	else
#endif // OPENSIM  // <FS:AW optional opensim support>
	{
		destination_guide_url = gSavedSettings.getString("DestinationGuideURL");
	}

	if(!destination_guide_url.empty())
	{	
		LLMediaCtrl* destinations = LLFloaterReg::getInstance("destinations")->getChild<LLMediaCtrl>("destination_guide_contents");
		if (destinations)
		{
			destinations->setErrorPageURL(gSavedSettings.getString("GenericErrorPageURL"));
			destination_guide_url = LLWeb::expandURLSubstitutions(destination_guide_url, LLSD());
			LL_DEBUGS("WebApi") << "3 DestinationGuideURL \"" << destination_guide_url << "\"" << LL_ENDL;
			destinations->navigateTo(destination_guide_url, HTTP_CONTENT_TEXT_HTML);
		}
	}

	std::string avatar_picker_url;
#ifdef OPENSIM // <FS:AW optional opensim support>
	if (LLGridManager::getInstance()->isInOpenSim())
	{
		if (LLLoginInstance::getInstance()->hasResponse("avatar_picker_url"))
		{
			avatar_picker_url = LLLoginInstance::getInstance()->getResponse("avatar_picker_url").asString();
		}
	}
	else
#endif // OPENSIM  // <FS:AW optional opensim support>
	{
		avatar_picker_url = gSavedSettings.getString("AvatarPickerURL");
	}

	if(!avatar_picker_url.empty())
	{	
		LLMediaCtrl* avatar_picker = LLFloaterReg::getInstance("avatar")->findChild<LLMediaCtrl>("avatar_picker_contents");
		if (avatar_picker)
		{
			avatar_picker->setErrorPageURL(gSavedSettings.getString("GenericErrorPageURL"));
			avatar_picker_url = LLWeb::expandURLSubstitutions(avatar_picker_url, LLSD());
			LL_DEBUGS("WebApi") << "AvatarPickerURL \"" << avatar_picker_url << "\"" << LL_ENDL;
			avatar_picker->navigateTo(avatar_picker_url, HTTP_CONTENT_TEXT_HTML);
		}
 	}
// </FS:AW  opensim destinations and avatar picker>

	// <FS:Zi> Autohide main chat bar if applicable
	BOOL visible=!gSavedSettings.getBOOL("AutohideChatBar");

	FSNearbyChat::instance().showDefaultChatBar(visible);
	gSavedSettings.setBOOL("MainChatbarVisible",visible);
	// </FS:Zi>
}

// Destroy the UI
void LLViewerWindow::shutdownViews()
{
	// clean up warning logger
	RecordToChatConsole::getInstance()->stopRecorder();
	LL_INFOS() << "Warning logger is cleaned." << LL_ENDL ;

	gFocusMgr.unlockFocus();
	gFocusMgr.setMouseCapture(NULL);
	gFocusMgr.setKeyboardFocus(NULL);
	gFocusMgr.setTopCtrl(NULL);
	if (mWindow)
	{
		mWindow->allowLanguageTextInput(NULL, FALSE);
	}

	delete mDebugText;
	mDebugText = NULL;
	
	LL_INFOS() << "DebugText deleted." << LL_ENDL ;

	// Cleanup global views
	if (gMorphView)
	{
		gMorphView->setVisible(FALSE);
	}
	LL_INFOS() << "Global views cleaned." << LL_ENDL ;

	LLNotificationsUI::LLToast::cleanupToasts();
	LL_INFOS() << "Leftover toast cleaned up." << LL_ENDL;

	// DEV-40930: Clear sModalStack. Otherwise, any LLModalDialog left open
	// will crump with LL_ERRS.
	LLModalDialog::shutdownModals();
	LL_INFOS() << "LLModalDialog shut down." << LL_ENDL; 

	// destroy the nav bar, not currently part of gViewerWindow
	// *TODO: Make LLNavigationBar part of gViewerWindow
	LLNavigationBar::deleteSingleton();
	LL_INFOS() << "LLNavigationBar destroyed." << LL_ENDL ;
	
	// destroy menus after instantiating navbar above, as it needs
	// access to gMenuHolder
	cleanup_menus();
	LL_INFOS() << "menus destroyed." << LL_ENDL ;

	view_listener_t::cleanup();
	LL_INFOS() << "view listeners destroyed." << LL_ENDL ;

	// Clean up pointers that are going to be invalid. (todo: check sMenuContainer)
	mProgressView = NULL;
	mPopupView = NULL;

	// Delete all child views.
	delete mRootView;
	mRootView = NULL;
	LL_INFOS() << "RootView deleted." << LL_ENDL ;
	
	LLMenuOptionPathfindingRebakeNavmesh::getInstance()->quit();

	// Automatically deleted as children of mRootView.  Fix the globals.
	gStatusBar = NULL;
	gIMMgr = NULL;
	gToolTipView = NULL;

	gToolBarView = NULL;
	gFloaterView = NULL;
	gMorphView = NULL;

	gHUDView = NULL;
}

void LLViewerWindow::shutdownGL()
{
	//--------------------------------------------------------
	// Shutdown GL cleanly.  Order is very important here.
	//--------------------------------------------------------
	LLFontGL::destroyDefaultFonts();
	SUBSYSTEM_CLEANUP(LLFontManager);
	stop_glerror();

	gSky.cleanup();
	stop_glerror();

	LL_INFOS() << "Cleaning up pipeline" << LL_ENDL;
	gPipeline.cleanup();
	stop_glerror();

	//MUST clean up pipeline before cleaning up wearables
	LL_INFOS() << "Cleaning up wearables" << LL_ENDL;
	LLWearableList::instance().cleanup() ;

	gTextureList.shutdown();
	stop_glerror();

	gBumpImageList.shutdown();
	stop_glerror();

	LLWorldMapView::cleanupTextures();

	LLViewerTextureManager::cleanup() ;
	SUBSYSTEM_CLEANUP(LLImageGL) ;

	LL_INFOS() << "All textures and llimagegl images are destroyed!" << LL_ENDL ;

	LL_INFOS() << "Cleaning up select manager" << LL_ENDL;
	LLSelectMgr::getInstance()->cleanup();	

	LL_INFOS() << "Stopping GL during shutdown" << LL_ENDL;
	stopGL(FALSE);
	stop_glerror();

	gGL.shutdown();
	
	// <FS:Ansariel> Exodus vignette
	// This must die before LLVertexBuffer does
	exoPostProcess::deleteSingleton();
	// </FS:Ansariel> Exodus vignette

	SUBSYSTEM_CLEANUP(LLVertexBuffer);

	LL_INFOS() << "LLVertexBuffer cleaned." << LL_ENDL ;
}

// shutdownViews() and shutdownGL() need to be called first
LLViewerWindow::~LLViewerWindow()
{
	LL_INFOS() << "Destroying Window" << LL_ENDL;
	gDebugWindowProc = TRUE; // event catching, at this point it shouldn't output at all
	destroyWindow();

	delete mDebugText;
	mDebugText = NULL;

	if (LLViewerShaderMgr::sInitialized)
	{
		LLViewerShaderMgr::releaseInstance();
		LLViewerShaderMgr::sInitialized = FALSE;
	}
}


void LLViewerWindow::setCursor( ECursorType c )
{
	mWindow->setCursor( c );
}

void LLViewerWindow::showCursor()
{
	mWindow->showCursor();
	
	mCursorHidden = FALSE;
}

void LLViewerWindow::hideCursor()
{
	// And hide the cursor
	mWindow->hideCursor();

	mCursorHidden = TRUE;
}

void LLViewerWindow::sendShapeToSim()
{
	LLMessageSystem* msg = gMessageSystem;
	if(!msg) return;
	msg->newMessageFast(_PREHASH_AgentHeightWidth);
	msg->nextBlockFast(_PREHASH_AgentData);
	msg->addUUIDFast(_PREHASH_AgentID, gAgent.getID());
	msg->addUUIDFast(_PREHASH_SessionID, gAgent.getSessionID());
	msg->addU32Fast(_PREHASH_CircuitCode, gMessageSystem->mOurCircuitCode);
	msg->nextBlockFast(_PREHASH_HeightWidthBlock);
	msg->addU32Fast(_PREHASH_GenCounter, 0);
	U16 height16 = (U16) mWorldViewRectRaw.getHeight();
	U16 width16 = (U16) mWorldViewRectRaw.getWidth();
	msg->addU16Fast(_PREHASH_Height, height16);
	msg->addU16Fast(_PREHASH_Width, width16);
	gAgent.sendReliableMessage();
}

// Must be called after window is created to set up agent
// camera variables and UI variables.
void LLViewerWindow::reshape(S32 width, S32 height)
{
	// Destroying the window at quit time generates spurious
	// reshape messages.  We don't care about these, and we
	// don't want to send messages because the message system
	// may have been destructed.
	if (!LLApp::isExiting())
	{
		gWindowResized = TRUE;

		// update our window rectangle
		mWindowRectRaw.mRight = mWindowRectRaw.mLeft + width;
		mWindowRectRaw.mTop = mWindowRectRaw.mBottom + height;

		//glViewport(0, 0, width, height );

		if (height > 0)
		{ 
			LLViewerCamera::getInstance()->setViewHeightInPixels( mWorldViewRectRaw.getHeight() );
			LLViewerCamera::getInstance()->setAspect( getWorldViewAspectRatio() );
		}

		calcDisplayScale();
	
		BOOL display_scale_changed = mDisplayScale != LLUI::getScaleFactor();
		LLUI::setScaleFactor(mDisplayScale);

		// update our window rectangle
		mWindowRectScaled.mRight = mWindowRectScaled.mLeft + ll_round((F32)width / mDisplayScale.mV[VX]);
		mWindowRectScaled.mTop = mWindowRectScaled.mBottom + ll_round((F32)height / mDisplayScale.mV[VY]);

		setup2DViewport();

		// Inform lower views of the change
		// round up when converting coordinates to make sure there are no gaps at edge of window
		LLView::sForceReshape = display_scale_changed;
		mRootView->reshape(llceil((F32)width / mDisplayScale.mV[VX]), llceil((F32)height / mDisplayScale.mV[VY]));
		LLView::sForceReshape = FALSE;

		// clear font width caches
		if (display_scale_changed)
		{
			LLHUDObject::reshapeAll();
		}

		sendShapeToSim();

		// store new settings for the mode we are in, regardless
		BOOL maximized = mWindow->getMaximized();
		gSavedSettings.setBOOL("WindowMaximized", maximized);

//<FS:KC - fix for EXP-1777/EXP-1832>
        LLCoordScreen window_size;
		if (!maximized
			&& mWindow->getSize(&window_size))
//		if (!maximized)
//</FS:KC - fix for EXP-1777/EXP-1832>
		{
			U32 min_window_width=gSavedSettings.getU32("MinWindowWidth");
			U32 min_window_height=gSavedSettings.getU32("MinWindowHeight");
			// tell the OS specific window code about min window size
			mWindow->setMinSize(min_window_width, min_window_height);

			LLCoordScreen window_rect;
			if (mWindow->getSize(&window_rect))
			{
			// Only save size if not maximized
				gSavedSettings.setU32("WindowWidth", window_rect.mX);
				gSavedSettings.setU32("WindowHeight", window_rect.mY);
			}
		}

		sample(LLStatViewer::WINDOW_WIDTH, width);
		sample(LLStatViewer::WINDOW_HEIGHT, height);

		LLLayoutStack::updateClass();
	}
}


// Hide normal UI when a logon fails
void LLViewerWindow::setNormalControlsVisible( BOOL visible )
{
	if(LLChicletBar::instanceExists())
	{
		LLChicletBar::getInstance()->setVisible(visible);
		LLChicletBar::getInstance()->setEnabled(visible);
	}

	if ( gMenuBarView )
	{
		gMenuBarView->setVisible( visible );
		gMenuBarView->setEnabled( visible );

		// ...and set the menu color appropriately.
		setMenuBackgroundColor(gAgent.getGodLevel() > GOD_NOT, 
			!LLGridManager::getInstance()->isInSLBeta());
	}
        
	if ( gStatusBar )
	{
		gStatusBar->setVisible( visible );	
		gStatusBar->setEnabled( visible );	
	}
	
	// <FS:Zi> Is done inside XUI now, using visibility_control
	//LLNavigationBar* navbarp = LLUI::getRootView()->findChild<LLNavigationBar>("navigation_bar");
	//if (navbarp)
	//{
	//	// when it's time to show navigation bar we need to ensure that the user wants to see it
	//	// i.e. ShowNavbarNavigationPanel option is true
	//	navbarp->setVisible( visible && gSavedSettings.getBOOL("ShowNavbarNavigationPanel") );
	//}
	// </FS:Zi>
}

void LLViewerWindow::setMenuBackgroundColor(bool god_mode, bool dev_grid)
{
    LLSD args;
    LLColor4 new_bg_color;

	// god more important than project, proj more important than grid
    if ( god_mode ) 
    {
		//if ( LLGridManager::getInstance()->isInProductionGrid() ) <FS:TM> use our grid code and not LL's
		if ( !LLGridManager::getInstance()->isInSLBeta() )
		{
			new_bg_color = LLUIColorTable::instance().getColor( "MenuBarGodBgColor" );
		}
		else
		{
			new_bg_color = LLUIColorTable::instance().getColor( "MenuNonProductionGodBgColor" );
		}
    }
    else
	{
		// <FS:Ansariel> Don't care about viewer maturity
        //switch (LLVersionInfo::getViewerMaturity())
        //{
        //case LLVersionInfo::TEST_VIEWER:
        //    new_bg_color = LLUIColorTable::instance().getColor( "MenuBarTestBgColor" );
        //    break;

        //case LLVersionInfo::PROJECT_VIEWER:
        //    new_bg_color = LLUIColorTable::instance().getColor( "MenuBarProjectBgColor" );
        //    break;
        //    
        //case LLVersionInfo::BETA_VIEWER:
        //    new_bg_color = LLUIColorTable::instance().getColor( "MenuBarBetaBgColor" );
        //    break;
        //    
        //case LLVersionInfo::RELEASE_VIEWER:
        //    if(!LLGridManager::getInstance()->isInProductionGrid())
        //    {
        //        new_bg_color = LLUIColorTable::instance().getColor( "MenuNonProductionBgColor" );
        //    }
        //    else 
        //    {
        //        new_bg_color = LLUIColorTable::instance().getColor( "MenuBarBgColor" );
        //    }
        //    break;
        //}
		if (LLGridManager::getInstance()->isInSLBeta())
		{
			new_bg_color = LLUIColorTable::instance().getColor( "MenuNonProductionBgColor" );
		}
		else 
		{
			new_bg_color = LLUIColorTable::instance().getColor( "MenuBarBgColor" );
		}
		// </FS:Ansariel>
    }
    
    if(gMenuBarView)
    {
        gMenuBarView->setBackgroundColor( new_bg_color );
    }

    if(gStatusBar)
    {
        gStatusBar->setBackgroundColor( new_bg_color );
    }
}

void LLViewerWindow::drawDebugText()
{
	gGL.color4f(1,1,1,1);
	gGL.pushMatrix();
	gGL.pushUIMatrix();
	if (LLGLSLShader::sNoFixedFunction)
	{
		gUIProgram.bind();
	}
	{
		// scale view by UI global scale factor and aspect ratio correction factor
		gGL.scaleUI(mDisplayScale.mV[VX], mDisplayScale.mV[VY], 1.f);
		mDebugText->draw();
	}
	gGL.popUIMatrix();
	gGL.popMatrix();

	gGL.flush();
	if (LLGLSLShader::sNoFixedFunction)
	{
		gUIProgram.unbind();
	}
}

void LLViewerWindow::draw()
{
	
//#if LL_DEBUG
	LLView::sIsDrawing = TRUE;
//#endif
	stop_glerror();
	
	LLUI::setLineWidth(1.f);

	LLUI::setLineWidth(1.f);
	// Reset any left-over transforms
	gGL.matrixMode(LLRender::MM_MODELVIEW);
	
	gGL.loadIdentity();

	//S32 screen_x, screen_y;

	//if (!gSavedSettings.getBOOL("RenderUIBuffer"))
	static LLCachedControl<bool> renderUIBuffer(gSavedSettings, "RenderUIBuffer");
	if (!renderUIBuffer)
	{
		LLUI::sDirtyRect = getWindowRectScaled();
	}

	// HACK for timecode debugging
	//if (gSavedSettings.getBOOL("DisplayTimecode"))
	static LLCachedControl<bool> displayTimecode(gSavedSettings, "DisplayTimecode");
	if (displayTimecode)
	{
		// draw timecode block
		std::string text;

		gGL.loadIdentity();

		microsecondsToTimecodeString(gFrameTime,text);
		const LLFontGL* font = LLFontGL::getFontSansSerif();
		font->renderUTF8(text, 0,
						ll_round((getWindowWidthScaled()/2)-100.f),
						ll_round((getWindowHeightScaled()-60.f)),
			LLColor4( 1.f, 1.f, 1.f, 1.f ),
			LLFontGL::LEFT, LLFontGL::TOP);
	}

	// Draw all nested UI views.
	// No translation needed, this view is glued to 0,0

	if (LLGLSLShader::sNoFixedFunction)
	{
		gUIProgram.bind();
	}

	gGL.pushMatrix();
	LLUI::pushMatrix();
	{
		
		// scale view by UI global scale factor and aspect ratio correction factor
		gGL.scaleUI(mDisplayScale.mV[VX], mDisplayScale.mV[VY], 1.f);

		LLVector2 old_scale_factor = LLUI::getScaleFactor();
		// apply camera zoom transform (for high res screenshots)
		F32 zoom_factor = LLViewerCamera::getInstance()->getZoomFactor();
		S16 sub_region = LLViewerCamera::getInstance()->getZoomSubRegion();
		if (zoom_factor > 1.f)
		{
			//decompose subregion number to x and y values
			int pos_y = sub_region / llceil(zoom_factor);
			int pos_x = sub_region - (pos_y*llceil(zoom_factor));
			// offset for this tile
			gGL.translatef((F32)getWindowWidthScaled() * -(F32)pos_x, 
						(F32)getWindowHeightScaled() * -(F32)pos_y, 
						0.f);
			gGL.scalef(zoom_factor, zoom_factor, 1.f);
			LLUI::getScaleFactor() *= zoom_factor;
		}

		// Draw tool specific overlay on world
		LLToolMgr::getInstance()->getCurrentTool()->draw();

		// <exodus> Draw HUD stuff.
		bool inMouselook = gAgentCamera.cameraMouselook();
		static LLCachedControl<bool> fsMouselookCombatFeatures(gSavedSettings, "FSMouselookCombatFeatures", true);
		if (inMouselook && fsMouselookCombatFeatures)
		{
			S32 windowWidth = gViewerWindow->getWorldViewRectScaled().getWidth();
			S32 windowHeight = gViewerWindow->getWorldViewRectScaled().getHeight();

			static const std::string unknown_agent = LLTrans::getString("Mouselook_Unknown_Avatar");
			static LLUIColor map_avatar_color = LLUIColorTable::instance().getColor("MapAvatarColor", LLColor4::white);
			static LLCachedControl<F32> renderIFFRange(gSavedSettings, "ExodusMouselookIFFRange", 380.f);
			static LLCachedControl<bool> renderIFF(gSavedSettings, "ExodusMouselookIFF", true);
			static LLUICachedControl<F32> userPresetX("ExodusMouselookTextOffsetX", 0.f);
			static LLUICachedControl<F32> userPresetY("ExodusMouselookTextOffsetY", -150.f);
			static LLUICachedControl<U32> userPresetHAlign("ExodusMouselookTextHAlign", 2);

			LLVector3d myPosition = gAgentCamera.getCameraPositionGlobal();
			LLQuaternion myRotation = LLViewerCamera::getInstance()->getQuaternion();

			myRotation.set(-myRotation.mQ[VX], -myRotation.mQ[VY], -myRotation.mQ[VZ], myRotation.mQ[VW]);

			uuid_vec_t avatars;
			std::vector<LLVector3d> positions;
			LLWorld::getInstance()->getAvatars(&avatars, &positions, gAgent.getPositionGlobal(), renderIFFRange);
	
			bool crosshairRendered = false;

			S32 length = avatars.size();
			if (length)
			{
				for (S32 i = 0; i < length; i++)
				{
					LLUUID& targetKey = avatars[i];
					if (targetKey == gAgentID)
					{
						continue;
					}

					LLVector3d targetPosition = positions[i];
					if (targetPosition.isNull())
					{
						continue;
					}

					LLColor4 targetColor = map_avatar_color.get();
					targetColor = LGGContactSets::getInstance()->colorize(targetKey, targetColor, LGG_CS_MINIMAP);

					//color based on contact sets prefs
					LGGContactSets::getInstance()->hasFriendColorThatShouldShow(targetKey, LGG_CS_MINIMAP, targetColor);

					LLColor4 mark_color;
					if (LLNetMap::getAvatarMarkColor(targetKey, mark_color))
					{
						targetColor = mark_color;
					}

					if (renderIFF)
					{
						LLTracker::instance()->drawMarker(targetPosition, targetColor, true);
					}

					if (inMouselook && !crosshairRendered && !gRlvHandler.hasBehaviour(RLV_BHVR_SHOWNAMES))
					{
						LLVector3d magicVector = (targetPosition - myPosition) * myRotation;
						magicVector.setVec(-magicVector.mdV[VY], magicVector.mdV[VZ], magicVector.mdV[VX]);

						if (magicVector.mdV[VX] > -0.75 && magicVector.mdV[VX] < 0.75 && magicVector.mdV[VZ] > 0.0 && magicVector.mdV[VY] > -1.5 && magicVector.mdV[VY] < 1.5) // Do not fuck with these, cheater. :(
						{
							LLAvatarName avatarName;
							std::string targetName = unknown_agent;
							if (LLAvatarNameCache::get(targetKey, &avatarName))
							{
								targetName = avatarName.getCompleteName();
							}

							LLFontGL::getFontSansSerifBold()->renderUTF8(
								llformat("%s, %.2fm", targetName.c_str(), (targetPosition - myPosition).magVec()),
								0, (windowWidth / 2.f) + userPresetX, (windowHeight / 2.f) + userPresetY, targetColor,
								(LLFontGL::HAlign)((S32)userPresetHAlign), LLFontGL::TOP, LLFontGL::BOLD, LLFontGL::DROP_SHADOW_SOFT
							);

							crosshairRendered = true;
						}
					}

					if (!renderIFF && inMouselook && crosshairRendered)
					{
						break;
					}
				}
			}
		}
		// </exodus>

        // Only show Mouselookinstructions if FSShowMouselookInstruction is TRUE
		static LLCachedControl<bool> fsShowMouselookInstructions(gSavedSettings, "FSShowMouselookInstructions");
		if( fsShowMouselookInstructions && (gAgentCamera.cameraMouselook() || LLFloaterCamera::inFreeCameraMode()) )
		{
			drawMouselookInstructions();
			stop_glerror();
		}

		// Draw all nested UI views.
		// No translation needed, this view is glued to 0,0
		mRootView->draw();

		if (LLView::sDebugRects)
		{
			gToolTipView->drawStickyRect();
		}

		// Draw optional on-top-of-everyone view
		LLUICtrl* top_ctrl = gFocusMgr.getTopCtrl();
		if (top_ctrl && top_ctrl->getVisible())
		{
			S32 screen_x, screen_y;
			top_ctrl->localPointToScreen(0, 0, &screen_x, &screen_y);

			gGL.matrixMode(LLRender::MM_MODELVIEW);
			LLUI::pushMatrix();
			LLUI::translate( (F32) screen_x, (F32) screen_y);
			top_ctrl->draw();	
			LLUI::popMatrix();
		}


		if( gShowOverlayTitle && !mOverlayTitle.empty() )
		{
			// Used for special titles such as "Second Life - Special E3 2003 Beta"
			const S32 DIST_FROM_TOP = 20;
			LLFontGL::getFontSansSerifBig()->renderUTF8(
				mOverlayTitle, 0,
				ll_round( getWindowWidthScaled() * 0.5f),
				getWindowHeightScaled() - DIST_FROM_TOP,
				LLColor4(1, 1, 1, 0.4f),
				LLFontGL::HCENTER, LLFontGL::TOP);
		}

		LLUI::setScaleFactor(old_scale_factor);
	}
	LLUI::popMatrix();
	gGL.popMatrix();

	if (LLGLSLShader::sNoFixedFunction)
	{
		gUIProgram.unbind();
	}

//#if LL_DEBUG
	LLView::sIsDrawing = FALSE;
//#endif
}


//-TT Window Title Access
void LLViewerWindow::setTitle(const std::string& win_title)
{
	mWindow->setTitle(win_title);
}
//-TT

// Takes a single keyup event, usually when UI is visible
BOOL LLViewerWindow::handleKeyUp(KEY key, MASK mask)
{
    LLFocusableElement* keyboard_focus = gFocusMgr.getKeyboardFocus();

    if (keyboard_focus
		&& !(mask & (MASK_CONTROL | MASK_ALT))
		&& !gFocusMgr.getKeystrokesOnly())
	{
		// We have keyboard focus, and it's not an accelerator
        if (keyboard_focus && keyboard_focus->wantsKeyUpKeyDown())
        {
            return keyboard_focus->handleKeyUp(key, mask, FALSE);
        }
        else if (key < 0x80)
		{
			// Not a special key, so likely (we hope) to generate a character.  Let it fall through to character handler first.
			return (gFocusMgr.getKeyboardFocus() != NULL);
		}
	}

	if (keyboard_focus)
	{
		if (keyboard_focus->handleKeyUp(key, mask, FALSE))
		{
			LL_DEBUGS() << "LLviewerWindow::handleKeyUp - in 'traverse up' - no loops seen... just called keyboard_focus->handleKeyUp an it returned true" << LL_ENDL;
			LLViewerEventRecorder::instance().logKeyEvent(key, mask);
			return TRUE;
		}
		else {
			LL_DEBUGS() << "LLviewerWindow::handleKeyUp - in 'traverse up' - no loops seen... just called keyboard_focus->handleKeyUp an it returned FALSE" << LL_ENDL;
		}
	}

	// don't pass keys on to world when something in ui has focus
	return gFocusMgr.childHasKeyboardFocus(mRootView)
		|| LLMenuGL::getKeyboardMode()
		|| (gMenuBarView && gMenuBarView->getHighlightedItem() && gMenuBarView->getHighlightedItem()->isActive());
}

// Takes a single keydown event, usually when UI is visible
BOOL LLViewerWindow::handleKey(KEY key, MASK mask)
{
	// hide tooltips on keypress
	LLToolTipMgr::instance().blockToolTips();

    // let menus handle navigation keys for navigation
    if (LLSetKeyBindDialog::recordKey(key, mask))
    {
        LL_DEBUGS() << "Key handled by LLSetKeyBindDialog" << LL_ENDL;
        LLViewerEventRecorder::instance().logKeyEvent(key,mask);
        return TRUE;
    }

    LLFocusableElement* keyboard_focus = gFocusMgr.getKeyboardFocus();

    if (keyboard_focus
		&& !(mask & (MASK_CONTROL | MASK_ALT))
		&& !gFocusMgr.getKeystrokesOnly())
	{
		// We have keyboard focus, and it's not an accelerator
        if (keyboard_focus && keyboard_focus->wantsKeyUpKeyDown())
        {
            return keyboard_focus->handleKey(key, mask, FALSE );
        }
		else if (key < 0x80)
		{
			// Not a special key, so likely (we hope) to generate a character.  Let it fall through to character handler first.
            return (keyboard_focus != NULL);
		}
	}

	// let menus handle navigation keys for navigation
	if ((gMenuBarView && gMenuBarView->handleKey(key, mask, TRUE))
		||(gLoginMenuBarView && gLoginMenuBarView->handleKey(key, mask, TRUE))
		||(gMenuHolder && gMenuHolder->handleKey(key, mask, TRUE)))
	{
		LL_DEBUGS() << "LLviewerWindow::handleKey handle nav keys for nav" << LL_ENDL;
		LLViewerEventRecorder::instance().logKeyEvent(key,mask);
		return TRUE;
	}


	// give menus a chance to handle modified (Ctrl, Alt) shortcut keys before current focus 
	// as long as focus isn't locked
	if (mask & (MASK_CONTROL | MASK_ALT) && !gFocusMgr.focusLocked())
	{
		// Check the current floater's menu first, if it has one.
		if (gFocusMgr.keyboardFocusHasAccelerators()
			&& keyboard_focus 
			&& keyboard_focus->handleKey(key,mask,FALSE))
		{
			LLViewerEventRecorder::instance().logKeyEvent(key,mask);
			return TRUE;
		}

		if (gAgent.isInitialized()
			&& (gAgent.getTeleportState() == LLAgent::TELEPORT_NONE || gAgent.getTeleportState() == LLAgent::TELEPORT_LOCAL)
			&& gMenuBarView
			&& gMenuBarView->handleAcceleratorKey(key, mask))
		{
			LLViewerEventRecorder::instance().logKeyEvent(key, mask);
			return TRUE;
		}

		if (gLoginMenuBarView && gLoginMenuBarView->handleAcceleratorKey(key, mask))
		{
			LLViewerEventRecorder::instance().logKeyEvent(key,mask);
			return TRUE;
		}
	}

	// give floaters first chance to handle TAB key
	// so frontmost floater gets focus
	// if nothing has focus, go to first or last UI element as appropriate
    if (key == KEY_TAB && (mask & MASK_CONTROL || keyboard_focus == NULL))
	{
		LL_WARNS() << "LLviewerWindow::handleKey give floaters first chance at tab key " << LL_ENDL;
		if (gMenuHolder) gMenuHolder->hideMenus();

		// if CTRL-tabbing (and not just TAB with no focus), go into window cycle mode
		gFloaterView->setCycleMode((mask & MASK_CONTROL) != 0);

		// do CTRL-TAB and CTRL-SHIFT-TAB logic
		if (mask & MASK_SHIFT)
		{
			mRootView->focusPrevRoot();
		}
		else
		{
			mRootView->focusNextRoot();
		}
		LLViewerEventRecorder::instance().logKeyEvent(key,mask);
		return TRUE;
	}
	// hidden edit menu for cut/copy/paste
	if (gEditMenu && gEditMenu->handleAcceleratorKey(key, mask))
	{
		LLViewerEventRecorder::instance().logKeyEvent(key,mask);
		return TRUE;
	}

	LLFloater* focused_floaterp = gFloaterView->getFocusedFloater();
	std::string focusedFloaterName = (focused_floaterp ? focused_floaterp->getInstanceName() : "");

	if( keyboard_focus )
	{
		// <FS:Ansariel> [FS Communication UI]
		//if ((focusedFloaterName == "nearby_chat") || (focusedFloaterName == "im_container") || (focusedFloaterName == "impanel"))
		//{
		//	if (gSavedSettings.getBOOL("ArrowKeysAlwaysMove"))
		//	{
		//		// let Control-Up and Control-Down through for chat line history,
		//		if (!(key == KEY_UP && mask == MASK_CONTROL)
		//			&& !(key == KEY_DOWN && mask == MASK_CONTROL)
		//			&& !(key == KEY_UP && mask == MASK_ALT)
		//			&& !(key == KEY_DOWN && mask == MASK_ALT))
		//		{
		//			switch(key)
		//			{
		//			case KEY_LEFT:
		//			case KEY_RIGHT:
		//			case KEY_UP:
		//			case KEY_DOWN:
		//			case KEY_PAGE_UP:
		//			case KEY_PAGE_DOWN:
		//			case KEY_HOME:
		//				// when chatbar is empty or ArrowKeysAlwaysMove set,
		//				// pass arrow keys on to avatar...
		//				return FALSE;
		//			default:
		//				break;
		//			}
		//		}
		//	}
		if(FSNearbyChat::instance().defaultChatBarHasFocus() &&
		   (FSNearbyChat::instance().defaultChatBarIsIdle() ||
		    gSavedSettings.getBOOL("ArrowKeysAlwaysMove")))
		{
			// let Control-Up and Control-Down through for chat line history,
			//<FS:TS> Control-Right and Control-Left too for chat line editing
			if (!(key == KEY_UP && mask == MASK_CONTROL)
				&& !(key == KEY_DOWN && mask == MASK_CONTROL)
				&& !(key == KEY_LEFT && mask == MASK_CONTROL)
				&& !(key == KEY_RIGHT && mask == MASK_CONTROL))
			{
				switch (key)
				{
					case KEY_LEFT:
					case KEY_RIGHT:
					case KEY_UP:
					case KEY_DOWN:
					case KEY_PAGE_UP:
					case KEY_PAGE_DOWN:
					case KEY_HOME:
						// when chatbar is empty or ArrowKeysAlwaysMove set,
						// pass arrow keys on to avatar...
						return FALSE;
					default:
						break;
				}
			}
		}
		// </FS:Ansariel> [FS Communication UI]

		if (keyboard_focus->handleKey(key, mask, FALSE))
		{

			LL_DEBUGS() << "LLviewerWindow::handleKey - in 'traverse up' - no loops seen... just called keyboard_focus->handleKey an it returned true" << LL_ENDL;
			LLViewerEventRecorder::instance().logKeyEvent(key,mask); 
			return TRUE;
		} else {
			LL_DEBUGS() << "LLviewerWindow::handleKey - in 'traverse up' - no loops seen... just called keyboard_focus->handleKey an it returned FALSE" << LL_ENDL;
		}
	}

	if( LLToolMgr::getInstance()->getCurrentTool()->handleKey(key, mask) )
	{
		LL_DEBUGS() << "LLviewerWindow::handleKey toolbar handling?" << LL_ENDL;
		LLViewerEventRecorder::instance().logKeyEvent(key,mask);
		return TRUE;
	}

	// Try for a new-format gesture
	if (LLGestureMgr::instance().triggerGesture(key, mask))
	{
		LL_DEBUGS() << "LLviewerWindow::handleKey new gesture feature" << LL_ENDL;
		LLViewerEventRecorder::instance().logKeyEvent(key,mask);
		return TRUE;
	}

	// See if this is a gesture trigger.  If so, eat the key and
	// don't pass it down to the menus.
	if (gGestureList.trigger(key, mask))
	{
		LL_DEBUGS() << "LLviewerWindow::handleKey check gesture trigger" << LL_ENDL;
		LLViewerEventRecorder::instance().logKeyEvent(key,mask);
		return TRUE;
	}

	// If "Pressing letter keys starts local chat" option is selected, we are not in mouselook, 
	// no view has keyboard focus, this is a printable character key (and no modifier key is 
	// pressed except shift), then give focus to nearby chat (STORM-560)

	// <FS:Ansariel> [FS Communication UI]
	// -- Also removed !gAgentCamera.cameraMouselook() because of FIRE-10906; Pressing letter keys SHOULD move focus to chat when this option is enabled, regardless of being in mouselook or not
	// -- The need to press Enter key while being in mouselook mode every time to say a sentence is not too coherent with user's expectation, if he/she checked "starts local chat"
	// -- Also check for KEY_DIVIDE as we remapped VK_OEM_2 to KEY_DIVIDE in LLKeyboardWin32 to fix starting gestures
	//if ( LLStartUp::getStartupState() >= STATE_STARTED && 
	//	gSavedSettings.getS32("LetterKeysFocusChatBar") && !gAgentCamera.cameraMouselook() && 
	//	!keyboard_focus && key < 0x80 && (mask == MASK_NONE || mask == MASK_SHIFT) )
	//{
	//	// Initialize nearby chat if it's missing
	//	LLFloaterIMNearbyChat* nearby_chat = LLFloaterReg::findTypedInstance<LLFloaterIMNearbyChat>("nearby_chat");
	//	if (!nearby_chat)
	//	{	
	//		LLSD name("im_container");
	//		LLFloaterReg::toggleInstanceOrBringToFront(name);
	//	}

	//	LLChatEntry* chat_editor = LLFloaterReg::findTypedInstance<LLFloaterIMNearbyChat>("nearby_chat")->getChatBox();
	//	if (chat_editor)
	//	{
	//		// passing NULL here, character will be added later when it is handled by character handler.
	//		nearby_chat->startChat(NULL);
	//		return TRUE;
	//	}
	//}

	static LLCachedControl<bool> LetterKeysAffectsMovementNotFocusChatBar(gSavedSettings, "LetterKeysAffectsMovementNotFocusChatBar");
	static LLCachedControl<bool> fsLetterKeysFocusNearbyChatBar(gSavedSettings, "FSLetterKeysFocusNearbyChatBar");
	static LLCachedControl<bool> fsNearbyChatbar(gSavedSettings, "FSNearbyChatbar");
	if ( !LetterKeysAffectsMovementNotFocusChatBar && 
#if LL_WINDOWS
		!keyboard_focus && ((key < 0x80 && (mask == MASK_NONE || mask == MASK_SHIFT)) || (key == KEY_DIVIDE && mask == MASK_SHIFT)) )
#else
		!keyboard_focus && key < 0x80 && (mask == MASK_NONE || mask == MASK_SHIFT) )
#endif
	{
		FSFloaterNearbyChat* nearby_chat = FSFloaterNearbyChat::findInstance();
		if (fsLetterKeysFocusNearbyChatBar && fsNearbyChatbar && nearby_chat && nearby_chat->getVisible())
		{
			nearby_chat->setFocus(TRUE);
		}
		else
		{
			FSNearbyChat::instance().showDefaultChatBar(TRUE);
		}
		return TRUE;
	}
	// </FS:Ansariel> [FS Communication UI]

	// give menus a chance to handle unmodified accelerator keys
	if (gAgent.isInitialized()
		&& (gAgent.getTeleportState() == LLAgent::TELEPORT_NONE || gAgent.getTeleportState() == LLAgent::TELEPORT_LOCAL)
		&& gMenuBarView
		&& gMenuBarView->handleAcceleratorKey(key, mask))
	{
		LLViewerEventRecorder::instance().logKeyEvent(key, mask);
		return TRUE;
	}

	if (gLoginMenuBarView && gLoginMenuBarView->handleAcceleratorKey(key, mask))
	{
		return TRUE;
	}

	// don't pass keys on to world when something in ui has focus
	return gFocusMgr.childHasKeyboardFocus(mRootView) 
		|| LLMenuGL::getKeyboardMode() 
		|| (gMenuBarView && gMenuBarView->getHighlightedItem() && gMenuBarView->getHighlightedItem()->isActive());
}


BOOL LLViewerWindow::handleUnicodeChar(llwchar uni_char, MASK mask)
{
	// HACK:  We delay processing of return keys until they arrive as a Unicode char,
	// so that if you're typing chat text at low frame rate, we don't send the chat
	// until all keystrokes have been entered. JC
	// HACK: Numeric keypad <enter> on Mac is Unicode 3
	// HACK: Control-M on Windows is Unicode 13
	if ((uni_char == 13 && mask != MASK_CONTROL)
	    || (uni_char == 3 && mask == MASK_NONE) )
	{
		if (mask != MASK_ALT)
		{
			// remaps, handles ignored cases and returns back to viewer window.
			return gViewerInput.handleKey(KEY_RETURN, mask, gKeyboard->getKeyRepeated(KEY_RETURN));
		}
	}

	// let menus handle navigation (jump) keys
	if (gMenuBarView && gMenuBarView->handleUnicodeChar(uni_char, TRUE))
	{
		return TRUE;
	}

	// Traverses up the hierarchy
	LLFocusableElement* keyboard_focus = gFocusMgr.getKeyboardFocus();
	if( keyboard_focus )
	{
		if (keyboard_focus->handleUnicodeChar(uni_char, FALSE))
		{
			return TRUE;
		}

        return TRUE;
	}

	return FALSE;
}


void LLViewerWindow::handleScrollWheel(S32 clicks)
{
	LLUI::resetMouseIdleTimer();
	
	LLMouseHandler* mouse_captor = gFocusMgr.getMouseCapture();
	if( mouse_captor )
	{
		S32 local_x;
		S32 local_y;
		mouse_captor->screenPointToLocal( mCurrentMousePoint.mX, mCurrentMousePoint.mY, &local_x, &local_y );
		mouse_captor->handleScrollWheel(local_x, local_y, clicks);
		if (LLView::sDebugMouseHandling)
		{
			LL_INFOS() << "Scroll Wheel handled by captor " << mouse_captor->getName() << LL_ENDL;
		}
		return;
	}

	LLUICtrl* top_ctrl = gFocusMgr.getTopCtrl();
	if (top_ctrl)
	{
		S32 local_x;
		S32 local_y;
		top_ctrl->screenPointToLocal( mCurrentMousePoint.mX, mCurrentMousePoint.mY, &local_x, &local_y );
		if (top_ctrl->handleScrollWheel(local_x, local_y, clicks)) return;
	}

	if (mRootView->handleScrollWheel(mCurrentMousePoint.mX, mCurrentMousePoint.mY, clicks) )
	{
		if (LLView::sDebugMouseHandling)
		{
			LL_INFOS() << "Scroll Wheel" << LLView::sMouseHandlerMessage << LL_ENDL;
		}
		return;
	}
	else if (LLView::sDebugMouseHandling)
	{
		LL_INFOS() << "Scroll Wheel not handled by view" << LL_ENDL;
	}

	// Zoom the camera in and out behavior

	if(top_ctrl == 0 
		&& getWorldViewRectScaled().pointInRect(mCurrentMousePoint.mX, mCurrentMousePoint.mY) 
		&& gAgentCamera.isInitialized())
		gAgentCamera.handleScrollWheel(clicks);

	return;
}

void LLViewerWindow::handleScrollHWheel(S32 clicks)
{
    LLUI::resetMouseIdleTimer();

    LLMouseHandler* mouse_captor = gFocusMgr.getMouseCapture();
    if (mouse_captor)
    {
        S32 local_x;
        S32 local_y;
        mouse_captor->screenPointToLocal(mCurrentMousePoint.mX, mCurrentMousePoint.mY, &local_x, &local_y);
        mouse_captor->handleScrollHWheel(local_x, local_y, clicks);
        if (LLView::sDebugMouseHandling)
        {
            LL_INFOS() << "Scroll Horizontal Wheel handled by captor " << mouse_captor->getName() << LL_ENDL;
        }
        return;
    }

    LLUICtrl* top_ctrl = gFocusMgr.getTopCtrl();
    if (top_ctrl)
    {
        S32 local_x;
        S32 local_y;
        top_ctrl->screenPointToLocal(mCurrentMousePoint.mX, mCurrentMousePoint.mY, &local_x, &local_y);
        if (top_ctrl->handleScrollHWheel(local_x, local_y, clicks)) return;
    }

    if (mRootView->handleScrollHWheel(mCurrentMousePoint.mX, mCurrentMousePoint.mY, clicks))
    {
        if (LLView::sDebugMouseHandling)
        {
            LL_INFOS() << "Scroll Horizontal Wheel" << LLView::sMouseHandlerMessage << LL_ENDL;
        }
        return;
    }
    else if (LLView::sDebugMouseHandling)
    {
        LL_INFOS() << "Scroll Horizontal Wheel not handled by view" << LL_ENDL;
    }

    return;
}

void LLViewerWindow::addPopup(LLView* popup)
{
	if (mPopupView)
	{
		mPopupView->addPopup(popup);
	}
}

void LLViewerWindow::removePopup(LLView* popup)
{
	if (mPopupView)
	{
		mPopupView->removePopup(popup);
	}
}

void LLViewerWindow::clearPopups()
{
	if (mPopupView)
	{
		mPopupView->clearPopups();
	}
}

void LLViewerWindow::moveCursorToCenter()
{
	if (! gSavedSettings.getBOOL("DisableMouseWarp"))
	{
		S32 x = getWorldViewWidthScaled() / 2;
		S32 y = getWorldViewHeightScaled() / 2;
	
		LLUI::setMousePositionScreen(x, y);
		
		//on a forced move, all deltas get zeroed out to prevent jumping
		mCurrentMousePoint.set(x,y);
		mLastMousePoint.set(x,y);
		mCurrentMouseDelta.set(0,0);	
	}
}


//////////////////////////////////////////////////////////////////////
//
// Hover handlers
//

void append_xui_tooltip(LLView* viewp, LLToolTip::Params& params)
{
	if (viewp) 
	{
		if (!params.styled_message.empty())
		{
			params.styled_message.add().text("\n---------\n"); 
		}
		LLView::root_to_view_iterator_t end_tooltip_it = viewp->endRootToView();
		// NOTE: we skip "root" since it is assumed
		for (LLView::root_to_view_iterator_t tooltip_it = ++viewp->beginRootToView();
			tooltip_it != end_tooltip_it;
			++tooltip_it)
		{
			LLView* viewp = *tooltip_it;
		
			params.styled_message.add().text(viewp->getName());

			LLPanel* panelp = dynamic_cast<LLPanel*>(viewp);
			if (panelp && !panelp->getXMLFilename().empty())
			{
				params.styled_message.add()
					.text("(" + panelp->getXMLFilename() + ")")
					//<FS:KC> Define in colors.xml instead
//					 .style.color(LLColor4(0.7f, 0.7f, 1.f, 1.f));
					.style.color(LLUIColorTable::instance().getColor("XUITooltipFileName"));
			}
			params.styled_message.add().text("/");
		}
	}
}

static LLTrace::BlockTimerStatHandle ftm("Update UI");

// Update UI based on stored mouse position from mouse-move
// event processing.
void LLViewerWindow::updateUI()
{
	LL_RECORD_BLOCK_TIME(ftm);

	static std::string last_handle_msg;

	// <FS:Ansariel> We don't show the hints anyway, so needless to check here
	//if (gLoggedInTime.getStarted())
	//{
	//	if (gLoggedInTime.getElapsedTimeF32() > gSavedSettings.getF32("DestinationGuideHintTimeout"))
	//	{
	//		LLFirstUse::notUsingDestinationGuide();
	//	}
	//	if (gLoggedInTime.getElapsedTimeF32() > gSavedSettings.getF32("SidePanelHintTimeout"))
	//	{
	//		LLFirstUse::notUsingSidePanel();
	//	}
	//}
	// </FS:Ansariel>

	LLConsole::updateClass();

	// animate layout stacks so we have up to date rect for world view
	LLLayoutStack::updateClass();

	// use full window for world view when not rendering UI
	bool world_view_uses_full_window = gAgentCamera.cameraMouselook() || !gPipeline.hasRenderDebugFeatureMask(LLPipeline::RENDER_DEBUG_FEATURE_UI);
	updateWorldViewRect(world_view_uses_full_window);

	LLView::sMouseHandlerMessage.clear();

	S32 x = mCurrentMousePoint.mX;
	S32 y = mCurrentMousePoint.mY;

	MASK	mask = gKeyboard->currentMask(TRUE);

	if (gPipeline.hasRenderDebugMask(LLPipeline::RENDER_DEBUG_RAYCAST))
	{
		gDebugRaycastFaceHit = -1;
		gDebugRaycastObject = cursorIntersect(-1, -1, 512.f, NULL, -1, FALSE, FALSE,
											  &gDebugRaycastFaceHit,
											  &gDebugRaycastIntersection,
											  &gDebugRaycastTexCoord,
											  &gDebugRaycastNormal,
											  &gDebugRaycastTangent,
											  &gDebugRaycastStart,
											  &gDebugRaycastEnd);
		gDebugRaycastParticle = gPipeline.lineSegmentIntersectParticle(gDebugRaycastStart, gDebugRaycastEnd, &gDebugRaycastParticleIntersection, NULL);
	}

	updateMouseDelta();
	updateKeyboardFocus();

	BOOL handled = FALSE;

	LLUICtrl* top_ctrl = gFocusMgr.getTopCtrl();
	LLMouseHandler* mouse_captor = gFocusMgr.getMouseCapture();
	LLView* captor_view = dynamic_cast<LLView*>(mouse_captor);

	//FIXME: only include captor and captor's ancestors if mouse is truly over them --RN

	//build set of views containing mouse cursor by traversing UI hierarchy and testing 
	//screen rect against mouse cursor
	view_handle_set_t mouse_hover_set;

	// constraint mouse enter events to children of mouse captor
	LLView* root_view = captor_view;

	// if mouse captor doesn't exist or isn't a LLView
	// then allow mouse enter events on entire UI hierarchy
	if (!root_view)
	{
		root_view = mRootView;
	}

	// only update mouse hover set when UI is visible (since we shouldn't send hover events to invisible UI
	if (gPipeline.hasRenderDebugFeatureMask(LLPipeline::RENDER_DEBUG_FEATURE_UI))
	{
		// include all ancestors of captor_view as automatically having mouse
		if (captor_view)
		{
			LLView* captor_parent_view = captor_view->getParent();
			while(captor_parent_view)
			{
				mouse_hover_set.insert(captor_parent_view->getHandle());
				captor_parent_view = captor_parent_view->getParent();
			}
		}

		// aggregate visible views that contain mouse cursor in display order
		LLPopupView::popup_list_t popups = mPopupView->getCurrentPopups();

		for(LLPopupView::popup_list_t::iterator popup_it = popups.begin(); popup_it != popups.end(); ++popup_it)
		{
			LLView* popup = popup_it->get();
			if (popup && popup->calcScreenBoundingRect().pointInRect(x, y))
			{
				// iterator over contents of top_ctrl, and throw into mouse_hover_set
				for (LLView::tree_iterator_t it = popup->beginTreeDFS();
					it != popup->endTreeDFS();
					++it)
				{
					LLView* viewp = *it;
					if (viewp->getVisible()
						&& viewp->calcScreenBoundingRect().pointInRect(x, y))
					{
						// we have a view that contains the mouse, add it to the set
						mouse_hover_set.insert(viewp->getHandle());
					}
					else
					{
						// skip this view and all of its children
						it.skipDescendants();
					}
				}
			}
		}

		// while the top_ctrl contains the mouse cursor, only it and its descendants will receive onMouseEnter events
		if (top_ctrl && top_ctrl->calcScreenBoundingRect().pointInRect(x, y))
		{
			// iterator over contents of top_ctrl, and throw into mouse_hover_set
			for (LLView::tree_iterator_t it = top_ctrl->beginTreeDFS();
				it != top_ctrl->endTreeDFS();
				++it)
			{
				LLView* viewp = *it;
				if (viewp->getVisible()
					&& viewp->calcScreenBoundingRect().pointInRect(x, y))
				{
					// we have a view that contains the mouse, add it to the set
					mouse_hover_set.insert(viewp->getHandle());
				}
				else
				{
					// skip this view and all of its children
					it.skipDescendants();
				}
			}
		}
		else
		{
			// walk UI tree in depth-first order
			for (LLView::tree_iterator_t it = root_view->beginTreeDFS();
				it != root_view->endTreeDFS();
				++it)
			{
				LLView* viewp = *it;
				// calculating the screen rect involves traversing the parent, so this is less than optimal
				if (viewp->getVisible()
					&& viewp->calcScreenBoundingRect().pointInRect(x, y))
				{

					// if this view is mouse opaque, nothing behind it should be in mouse_hover_set
					if (viewp->getMouseOpaque())
					{
						// constrain further iteration to children of this widget
						it = viewp->beginTreeDFS();
					}
		
					// we have a view that contains the mouse, add it to the set
					mouse_hover_set.insert(viewp->getHandle());
				}
				else
				{
					// skip this view and all of its children
					it.skipDescendants();
				}
			}
		}
	}

	typedef std::vector<LLHandle<LLView> > view_handle_list_t;

	// call onMouseEnter() on all views which contain the mouse cursor but did not before
	view_handle_list_t mouse_enter_views;
	std::set_difference(mouse_hover_set.begin(), mouse_hover_set.end(),
						mMouseHoverViews.begin(), mMouseHoverViews.end(),
						std::back_inserter(mouse_enter_views));
	for (view_handle_list_t::iterator it = mouse_enter_views.begin();
		it != mouse_enter_views.end();
		++it)
	{
		LLView* viewp = it->get();
		if (viewp)
		{
			LLRect view_screen_rect = viewp->calcScreenRect();
			viewp->onMouseEnter(x - view_screen_rect.mLeft, y - view_screen_rect.mBottom, mask);
		}
	}

	// call onMouseLeave() on all views which no longer contain the mouse cursor
	view_handle_list_t mouse_leave_views;
	std::set_difference(mMouseHoverViews.begin(), mMouseHoverViews.end(),
						mouse_hover_set.begin(), mouse_hover_set.end(),
						std::back_inserter(mouse_leave_views));
	for (view_handle_list_t::iterator it = mouse_leave_views.begin();
		it != mouse_leave_views.end();
		++it)
	{
		LLView* viewp = it->get();
		if (viewp)
		{
			LLRect view_screen_rect = viewp->calcScreenRect();
			viewp->onMouseLeave(x - view_screen_rect.mLeft, y - view_screen_rect.mBottom, mask);
		}
	}

	// store resulting hover set for next frame
	swap(mMouseHoverViews, mouse_hover_set);

	// only handle hover events when UI is enabled
	if (gPipeline.hasRenderDebugFeatureMask(LLPipeline::RENDER_DEBUG_FEATURE_UI))
	{	

		if( mouse_captor )
		{
			// Pass hover events to object capturing mouse events.
			S32 local_x;
			S32 local_y; 
			mouse_captor->screenPointToLocal( x, y, &local_x, &local_y );
			handled = mouse_captor->handleHover(local_x, local_y, mask);
			if (LLView::sDebugMouseHandling)
			{
				LL_INFOS() << "Hover handled by captor " << mouse_captor->getName() << LL_ENDL;
			}

			if( !handled )
			{
				LL_DEBUGS("UserInput") << "hover not handled by mouse captor" << LL_ENDL;
			}
		}
		else
		{
			if (top_ctrl)
			{
				S32 local_x, local_y;
				top_ctrl->screenPointToLocal( x, y, &local_x, &local_y );
				handled = top_ctrl->pointInView(local_x, local_y) && top_ctrl->handleHover(local_x, local_y, mask);
			}

			if ( !handled )
			{
				// x and y are from last time mouse was in window
				// mMouseInWindow tracks *actual* mouse location
				if (mMouseInWindow && mRootView->handleHover(x, y, mask) )
				{
					if (LLView::sDebugMouseHandling && LLView::sMouseHandlerMessage != last_handle_msg)
					{
						last_handle_msg = LLView::sMouseHandlerMessage;
						LL_INFOS() << "Hover" << LLView::sMouseHandlerMessage << LL_ENDL;
					}
					handled = TRUE;
				}
				else if (LLView::sDebugMouseHandling)
				{
					if (last_handle_msg != LLStringUtil::null)
					{
						last_handle_msg.clear();
						LL_INFOS() << "Hover not handled by view" << LL_ENDL;
					}
				}
			}
		
			if (!handled)
			{
				LLTool *tool = LLToolMgr::getInstance()->getCurrentTool();

				if(mMouseInWindow && tool)
				{
					handled = tool->handleHover(x, y, mask);
				}
			}
		}

		// Show a new tool tip (or update one that is already shown)
		BOOL tool_tip_handled = FALSE;
		std::string tool_tip_msg;
		if( handled 
			&& !mWindow->isCursorHidden())
		{
			LLRect screen_sticky_rect = mRootView->getLocalRect();
			S32 local_x, local_y;

			static LLCachedControl<bool> debug_show_xui_names(gSavedSettings, "DebugShowXUINames", 0);
			if (debug_show_xui_names)
			{
				LLToolTip::Params params;

				LLView* tooltip_view = mRootView;
				LLView::tree_iterator_t end_it = mRootView->endTreeDFS();
				for (LLView::tree_iterator_t it = mRootView->beginTreeDFS(); it != end_it; ++it)
				{
					LLView* viewp = *it;
					LLRect screen_rect;
					viewp->localRectToScreen(viewp->getLocalRect(), &screen_rect);
					if (!(viewp->getVisible()
						 && screen_rect.pointInRect(x, y)))
					{
						it.skipDescendants();
					}
					// only report xui names for LLUICtrls, 
					// and blacklist the various containers we don't care about
					else if (dynamic_cast<LLUICtrl*>(viewp) 
							&& viewp != gMenuHolder
							&& viewp != gFloaterView
							&& viewp != gConsole) 
					{
						if (dynamic_cast<LLFloater*>(viewp))
						{
							// constrain search to descendants of this (frontmost) floater
							// by resetting iterator
							it = viewp->beginTreeDFS();
						}

						// if we are in a new part of the tree (not a descendent of current tooltip_view)
						// then push the results for tooltip_view and start with a new potential view
						// NOTE: this emulates visiting only the leaf nodes that meet our criteria
						if (!viewp->hasAncestor(tooltip_view))
						{
							append_xui_tooltip(tooltip_view, params);
							screen_sticky_rect.intersectWith(tooltip_view->calcScreenRect());
						}
						tooltip_view = viewp;
					}
				}

				append_xui_tooltip(tooltip_view, params);
				params.styled_message.add().text("\n");

				screen_sticky_rect.intersectWith(tooltip_view->calcScreenRect());
				
				params.sticky_rect = screen_sticky_rect;
				params.max_width = 400;

				LLToolTipMgr::instance().show(params);
			}
			// if there is a mouse captor, nothing else gets a tooltip
			else if (mouse_captor)
			{
				mouse_captor->screenPointToLocal(x, y, &local_x, &local_y);
				tool_tip_handled = mouse_captor->handleToolTip(local_x, local_y, mask);
			}
			else 
			{
				// next is top_ctrl
				if (!tool_tip_handled && top_ctrl)
				{
					top_ctrl->screenPointToLocal(x, y, &local_x, &local_y);
					tool_tip_handled = top_ctrl->handleToolTip(local_x, local_y, mask );
				}
				
				if (!tool_tip_handled)
				{
					local_x = x; local_y = y;
					tool_tip_handled = mRootView->handleToolTip(local_x, local_y, mask );
				}

				LLTool* current_tool = LLToolMgr::getInstance()->getCurrentTool();
				if (!tool_tip_handled && current_tool)
				{
					current_tool->screenPointToLocal(x, y, &local_x, &local_y);
					tool_tip_handled = current_tool->handleToolTip(local_x, local_y, mask );
				}
			}
		}		
	}
	else
	{	// just have tools handle hover when UI is turned off
		LLTool *tool = LLToolMgr::getInstance()->getCurrentTool();

		if(mMouseInWindow && tool)
		{
			handled = tool->handleHover(x, y, mask);
		}
	}

	updateLayout();

	mLastMousePoint = mCurrentMousePoint;

	// cleanup unused selections when no modal dialogs are open
	if (LLModalDialog::activeCount() == 0)
	{
		LLViewerParcelMgr::getInstance()->deselectUnused();
	}

	if (LLModalDialog::activeCount() == 0)
	{
		LLSelectMgr::getInstance()->deselectUnused();
	}
}


void LLViewerWindow::updateLayout()
{
	LLTool* tool = LLToolMgr::getInstance()->getCurrentTool();
	if (gFloaterTools != NULL
		&& tool != NULL
		&& tool != gToolNull  
		&& tool != LLToolCompInspect::getInstance() 
		&& tool != LLToolDragAndDrop::getInstance() 
		&& !gSavedSettings.getBOOL("FreezeTime"))
	{ 
		// Suppress the toolbox view if our source tool was the pie tool,
		// and we've overridden to something else.
		bool suppress_toolbox = 
			(LLToolMgr::getInstance()->getBaseTool() == LLToolPie::getInstance()) &&
			(LLToolMgr::getInstance()->getCurrentTool() != LLToolPie::getInstance());

		LLMouseHandler *captor = gFocusMgr.getMouseCapture();
		// With the null, inspect, or drag and drop tool, don't muck
		// with visibility.

		if (gFloaterTools->isMinimized()
			||	(tool != LLToolPie::getInstance()						// not default tool
				&& tool != LLToolCompGun::getInstance()					// not coming out of mouselook
				&& !suppress_toolbox									// not override in third person
				&& LLToolMgr::getInstance()->getCurrentToolset()->isShowFloaterTools()
				&& (!captor || dynamic_cast<LLView*>(captor) != NULL)))						// not dragging
		{
			// Force floater tools to be visible (unless minimized)
			if (!gFloaterTools->getVisible())
			{
				gFloaterTools->openFloater();
			}
			// Update the location of the blue box tool popup
			LLCoordGL select_center_screen;
			MASK	mask = gKeyboard->currentMask(TRUE);
			gFloaterTools->updatePopup( select_center_screen, mask );
		}
		else
		{
			gFloaterTools->setVisible(FALSE);
		}
		//gMenuBarView->setItemVisible("BuildTools", gFloaterTools->getVisible());
	}

	// Always update console
	if(gConsole)
	{
		LLRect console_rect = getChatConsoleRect();
		gConsole->reshape(console_rect.getWidth(), console_rect.getHeight());
		gConsole->setRect(console_rect);
	}
}

void LLViewerWindow::updateMouseDelta()
{
	S32 dx = lltrunc((F32) (mCurrentMousePoint.mX - mLastMousePoint.mX) * LLUI::getScaleFactor().mV[VX]);
	S32 dy = lltrunc((F32) (mCurrentMousePoint.mY - mLastMousePoint.mY) * LLUI::getScaleFactor().mV[VY]);

	//RN: fix for asynchronous notification of mouse leaving window not working
	LLCoordWindow mouse_pos;
	mWindow->getCursorPosition(&mouse_pos);
	if (mouse_pos.mX < 0 || 
		mouse_pos.mY < 0 ||
		mouse_pos.mX > mWindowRectRaw.getWidth() ||
		mouse_pos.mY > mWindowRectRaw.getHeight())
	{
		mMouseInWindow = FALSE;
	}
	else
	{
		mMouseInWindow = TRUE;
	}

	LLVector2 mouse_vel; 

	//if (gSavedSettings.getBOOL("MouseSmooth"))
	static LLCachedControl<bool> mouseSmooth(gSavedSettings, "MouseSmooth");
	if (mouseSmooth)
	{
		static F32 fdx = 0.f;
		static F32 fdy = 0.f;

		F32 amount = 16.f;
		fdx = fdx + ((F32) dx - fdx) * llmin(gFrameIntervalSeconds.value()*amount,1.f);
		fdy = fdy + ((F32) dy - fdy) * llmin(gFrameIntervalSeconds.value()*amount,1.f);

		mCurrentMouseDelta.set(ll_round(fdx), ll_round(fdy));
		mouse_vel.setVec(fdx,fdy);
	}
	else
	{
		mCurrentMouseDelta.set(dx, dy);
		mouse_vel.setVec((F32) dx, (F32) dy);
	}
    
	sample(sMouseVelocityStat, mouse_vel.magVec());
}

void LLViewerWindow::updateKeyboardFocus()
{
	if (!gPipeline.hasRenderDebugFeatureMask(LLPipeline::RENDER_DEBUG_FEATURE_UI))
	{
		gFocusMgr.setKeyboardFocus(NULL);
	}

	// clean up current focus
	LLUICtrl* cur_focus = dynamic_cast<LLUICtrl*>(gFocusMgr.getKeyboardFocus());
	if (cur_focus)
	{
		if (!cur_focus->isInVisibleChain() || !cur_focus->isInEnabledChain())
		{
            // don't release focus, just reassign so that if being given
            // to a sibling won't call onFocusLost on all the ancestors
			// gFocusMgr.releaseFocusIfNeeded(cur_focus);

			LLUICtrl* parent = cur_focus->getParentUICtrl();
			const LLUICtrl* focus_root = cur_focus->findRootMostFocusRoot();
			bool new_focus_found = false;
			while(parent)
			{
				if (parent->isCtrl() 
					&& (parent->hasTabStop() || parent == focus_root) 
					&& !parent->getIsChrome() 
					&& parent->isInVisibleChain() 
					&& parent->isInEnabledChain())
				{
					if (!parent->focusFirstItem())
					{
						parent->setFocus(TRUE);
					}
					new_focus_found = true;
					break;
				}
				parent = parent->getParentUICtrl();
			}

			// if we didn't find a better place to put focus, just release it
			// hasFocus() will return true if and only if we didn't touch focus since we
			// are only moving focus higher in the hierarchy
			if (!new_focus_found)
			{
				cur_focus->setFocus(FALSE);
			}
		}
		else if (cur_focus->isFocusRoot())
		{
			// focus roots keep trying to delegate focus to their first valid descendant
			// this assumes that focus roots are not valid focus holders on their own
			cur_focus->focusFirstItem();
		}
	}

	// last ditch force of edit menu to selection manager
	if (LLEditMenuHandler::gEditMenuHandler == NULL && LLSelectMgr::getInstance()->getSelection()->getObjectCount())
	{
		LLEditMenuHandler::gEditMenuHandler = LLSelectMgr::getInstance();
	}

	if (gFloaterView->getCycleMode())
	{
		// sync all floaters with their focus state
		gFloaterView->highlightFocusedFloater();
		gSnapshotFloaterView->highlightFocusedFloater();
		MASK	mask = gKeyboard->currentMask(TRUE);
		if ((mask & MASK_CONTROL) == 0)
		{
			// control key no longer held down, finish cycle mode
			gFloaterView->setCycleMode(FALSE);

			gFloaterView->syncFloaterTabOrder();
		}
		else
		{
			// user holding down CTRL, don't update tab order of floaters
		}
	}
	else
	{
		// update focused floater
		gFloaterView->highlightFocusedFloater();
		gSnapshotFloaterView->highlightFocusedFloater();
		// make sure floater visible order is in sync with tab order
		gFloaterView->syncFloaterTabOrder();
	}
}

static LLTrace::BlockTimerStatHandle FTM_UPDATE_WORLD_VIEW("Update World View");
void LLViewerWindow::updateWorldViewRect(bool use_full_window)
{
	LL_RECORD_BLOCK_TIME(FTM_UPDATE_WORLD_VIEW);

	// start off using whole window to render world
	LLRect new_world_rect = mWindowRectRaw;

	if (use_full_window == false && mWorldViewPlaceholder.get())
	{
		new_world_rect = mWorldViewPlaceholder.get()->calcScreenRect();
		// clamp to at least a 1x1 rect so we don't try to allocate zero width gl buffers
		new_world_rect.mTop = llmax(new_world_rect.mTop, new_world_rect.mBottom + 1);
		new_world_rect.mRight = llmax(new_world_rect.mRight, new_world_rect.mLeft + 1);

		new_world_rect.mLeft = ll_round((F32)new_world_rect.mLeft * mDisplayScale.mV[VX]);
		new_world_rect.mRight = ll_round((F32)new_world_rect.mRight * mDisplayScale.mV[VX]);
		new_world_rect.mBottom = ll_round((F32)new_world_rect.mBottom * mDisplayScale.mV[VY]);
		new_world_rect.mTop = ll_round((F32)new_world_rect.mTop * mDisplayScale.mV[VY]);
	}

	if (mWorldViewRectRaw != new_world_rect)
	{
		mWorldViewRectRaw = new_world_rect;
		gResizeScreenTexture = TRUE;
		LLViewerCamera::getInstance()->setViewHeightInPixels( mWorldViewRectRaw.getHeight() );
		LLViewerCamera::getInstance()->setAspect( getWorldViewAspectRatio() );

		LLRect old_world_rect_scaled = mWorldViewRectScaled;
		mWorldViewRectScaled = calcScaledRect(mWorldViewRectRaw, mDisplayScale);

		// sending a signal with a new WorldView rect
		mOnWorldViewRectUpdated(old_world_rect_scaled, mWorldViewRectScaled);
	}
}

void LLViewerWindow::saveLastMouse(const LLCoordGL &point)
{
	// Store last mouse location.
	// If mouse leaves window, pretend last point was on edge of window

	if (point.mX < 0)
	{
		mCurrentMousePoint.mX = 0;
	}
	else if (point.mX > getWindowWidthScaled())
	{
		mCurrentMousePoint.mX = getWindowWidthScaled();
	}
	else
	{
		mCurrentMousePoint.mX = point.mX;
	}

	if (point.mY < 0)
	{
		mCurrentMousePoint.mY = 0;
	}
	else if (point.mY > getWindowHeightScaled() )
	{
		mCurrentMousePoint.mY = getWindowHeightScaled();
	}
	else
	{
		mCurrentMousePoint.mY = point.mY;
	}
}

// <FS:Beq> Changes to add physics view support into edit mode
//pragma region FSShowPhysicsInEditMode

const float offset_units = 3.0;
const float offset_factor = -3.0;

// decorator for renderMeshBaseHull from llspatialpartition. but with our own offsets to avoid glitching.
void renderMeshBaseHullPhysics(LLVOVolume* volume, U32 data_mask, LLColor4& color, LLColor4& line_color)
{
			LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glPolygonOffset(offset_factor, offset_units);
			gGL.diffuseColor4fv(color.mV);
			renderMeshBaseHull(volume, data_mask, color, line_color);
}

// decorator for render_hull from llspatialpartition. but with our own offsets to avoid glitching.
void renderHullPhysics(LLModel::PhysicsMesh& mesh, const LLColor4& color, const LLColor4& line_color)
{
	LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); 
	glPolygonOffset(offset_factor, offset_units);
	render_hull(mesh, color, line_color);
}

// Draw a physics shape with the edges highlighted in 'line_color'
void renderMeshPhysicsTriangles(const LLColor4& color, const LLColor4& line_color, LLVolume* vol, LLModel::Decomposition * decomp)
{
// Not required here, we already disable this in the outer scope
//	LLGLDisable multisample(LLPipeline::RenderFSAASamples > 0 ? GL_MULTISAMPLE_ARB : 0);

	LLGLSLShader* shader = LLGLSLShader::sCurBoundShaderPtr;

	if (shader)
	{
		gDebugProgram.bind();
	}

	gGL.matrixMode(LLRender::MM_MODELVIEW);
	gGL.pushMatrix();
	// scope for the RAII for the depth test on hidden geometry
	{
		// This draw section covers the hidden geometry

		gGL.blendFunc(LLRender::BF_SOURCE_COLOR, LLRender::BF_ONE);
		LLGLDepthTest gls_depth(GL_TRUE, GL_FALSE, GL_GEQUAL);
		if (shader)
		{
			{
				LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
				glPolygonOffset(offset_factor, offset_units);
				gGL.diffuseColor4fv(color.mV);
				//decomp has physics mesh, render that mesh
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
			}
			{
				LLGLEnable offset(GL_POLYGON_OFFSET_LINE);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				glPolygonOffset(offset_factor, offset_units);
				gGL.diffuseColor4fv(line_color.mV);
				LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
			}

		}
		else
		{
			// <FS:Ansariel> Don't use fixed functions when using shader renderer; found by Drake Arconis
			if (!LLGLSLShader::sNoFixedFunction)
			{
				// </FS:Ansariel>
				LLGLEnable fog(GL_FOG);
				glFogi(GL_FOG_MODE, GL_LINEAR);
				float d = (LLViewerCamera::getInstance()->getPointOfInterest() - LLViewerCamera::getInstance()->getOrigin()).magVec();
				LLColor4 fogCol = color * (F32)llclamp((LLSelectMgr::getInstance()->getSelectionCenterGlobal() - gAgentCamera.getCameraPositionGlobal()).magVec() / (LLSelectMgr::getInstance()->getBBoxOfSelection().getExtentLocal().magVec() * 4), 0.0, 1.0);
				glFogf(GL_FOG_START, d);
				glFogf(GL_FOG_END, d*(1 + (LLViewerCamera::getInstance()->getView() / LLViewerCamera::getInstance()->getDefaultFOV())));
				glFogfv(GL_FOG_COLOR, fogCol.mV);
				// <FS:Ansariel> Don't use fixed functions when using shader renderer; found by Drake Arconis
			}
			// </FS:Ansariel>
			gGL.setAlphaRejectSettings(LLRender::CF_DEFAULT);
			{
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				gGL.diffuseColor4fv(color.mV);
				//decomp has physics mesh, render that mesh
				LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				gGL.diffuseColor4fv(line_color.mV);
				LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
			}
		}
	}//End depth test for hidden geometry
//	gGL.flush();
	gGL.setSceneBlendType(LLRender::BT_ALPHA);

	if (shader)
	{
		{
			gGL.diffuseColor4fv(color.mV);
			LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glPolygonOffset(offset_factor, offset_units);
			glLineWidth(1.f);
			LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
		}
		{
			gGL.diffuseColor4fv(line_color.mV);
			LLGLEnable offset(GL_POLYGON_OFFSET_LINE);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			glPolygonOffset(offset_factor, offset_units);
			glLineWidth(3.f);
			LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
		}
	}
	else
	{
		// <FS:Ansariel> Don't use fixed functions when using shader renderer; found by Drake Arconis
		if (!LLGLSLShader::sNoFixedFunction)
		{
			// </FS:Ansariel>
			LLGLEnable fog(GL_FOG);
			glFogi(GL_FOG_MODE, GL_LINEAR);
			float d = (LLViewerCamera::getInstance()->getPointOfInterest() - LLViewerCamera::getInstance()->getOrigin()).magVec();
			LLColor4 fogCol = color * (F32)llclamp((LLSelectMgr::getInstance()->getSelectionCenterGlobal() - gAgentCamera.getCameraPositionGlobal()).magVec() / (LLSelectMgr::getInstance()->getBBoxOfSelection().getExtentLocal().magVec() * 4), 0.0, 1.0);
			glFogf(GL_FOG_START, d);
			glFogf(GL_FOG_END, d*(1 + (LLViewerCamera::getInstance()->getView() / LLViewerCamera::getInstance()->getDefaultFOV())));
			glFogfv(GL_FOG_COLOR, fogCol.mV);
			// <FS:Ansariel> Don't use fixed functions when using shader renderer; found by Drake Arconis
		}
		// </FS:Ansariel>
		gGL.setAlphaRejectSettings(LLRender::CF_DEFAULT);
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			gGL.diffuseColor4fv(color.mV);
			//decomp has physics mesh, render that mesh
			LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			gGL.diffuseColor4fv(line_color.mV);
			glLineWidth(3.f);
			LLVertexBuffer::drawArrays(LLRender::TRIANGLES, decomp->mPhysicsShapeMesh.mPositions, decomp->mPhysicsShapeMesh.mNormals);
		}
	}

	glLineWidth(1.f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	gGL.popMatrix();

	//restore the previous shader
	if (shader)
	{
		shader->bind();
	}
}

void renderNonMeshHullPhysics(LLVOVolume* vovolume, LLVolume* volume, LLColor4 color, LLColor4 line_color, LLVector3 center, LLVector3 size)
{
	LLVolumeParams volume_params = volume->getParams();
	S32 detail = get_physics_detail(volume_params, vovolume->getScale());

	LLVolume* phys_volume = LLPrimitive::sVolumeManager->refVolume(volume_params, detail);

	if (!phys_volume->mHullPoints)
	{ //build convex hull
		std::vector<LLVector3> pos;
		std::vector<U16> index;

		S32 index_offset = 0;
		// Build a vector of vertices in the visible LOD model determined by 'detail' 
		for (S32 i = 0; i < phys_volume->getNumVolumeFaces(); ++i)
		{
			const LLVolumeFace& face = phys_volume->getVolumeFace(i);
			if (index_offset + face.mNumVertices > 65535)
			{
				continue;
			}

			for (S32 j = 0; j < face.mNumVertices; ++j)
			{
				pos.push_back(LLVector3(face.mPositions[j].getF32ptr()));
			}

			for (S32 j = 0; j < face.mNumIndices; ++j)
			{
				index.push_back(face.mIndices[j] + index_offset);
			}

			index_offset += face.mNumVertices;
		}
		// use the array of vertices to construct a single hull based 
		if (!pos.empty() && !index.empty() && LLConvexDecomposition::getInstance()) // ND: FIRE-3427
		{
			LLCDMeshData mesh;
			mesh.mIndexBase = &index[0];
			mesh.mVertexBase = pos[0].mV;
			mesh.mNumVertices = pos.size();
			mesh.mVertexStrideBytes = 12;
			mesh.mIndexStrideBytes = 6;
			mesh.mIndexType = LLCDMeshData::INT_16;

			mesh.mNumTriangles = index.size() / 3;

			LLCDMeshData res;
			LLCDResult retval;
			if ((retval = LLConvexDecomposition::getInstance()->generateSingleHullMeshFromMesh(&mesh, &res)))
			{
				LL_WARNS() << "ConvexDecomp Failed (generateSingleHullMeshFromMesh): " << retval << LL_ENDL;
			}

			//copy res into phys_volume
			phys_volume->mHullPoints = (LLVector4a*)ll_aligned_malloc_16(sizeof(LLVector4a)*res.mNumVertices);
			phys_volume->mNumHullPoints = res.mNumVertices;

			S32 idx_size = (res.mNumTriangles * 3 * 2 + 0xF) & ~0xF;
			phys_volume->mHullIndices = (U16*)ll_aligned_malloc_16(idx_size);
			phys_volume->mNumHullIndices = res.mNumTriangles * 3;

			const F32* v = res.mVertexBase;

			for (S32 i = 0; i < res.mNumVertices; ++i)
			{
				F32* p = (F32*)((U8*)v + i*res.mVertexStrideBytes);
				phys_volume->mHullPoints[i].load3(p);
			}

			if (res.mIndexType == LLCDMeshData::INT_16)
			{
				for (S32 i = 0; i < res.mNumTriangles; ++i)
				{
					U16* idx = (U16*)(((U8*)res.mIndexBase) + i*res.mIndexStrideBytes);

					phys_volume->mHullIndices[i * 3 + 0] = idx[0];
					phys_volume->mHullIndices[i * 3 + 1] = idx[1];
					phys_volume->mHullIndices[i * 3 + 2] = idx[2];
				}
			}
			else
			{
				for (S32 i = 0; i < res.mNumTriangles; ++i)
				{
					U32* idx = (U32*)(((U8*)res.mIndexBase) + i*res.mIndexStrideBytes);

					phys_volume->mHullIndices[i * 3 + 0] = (U16)idx[0];
					phys_volume->mHullIndices[i * 3 + 1] = (U16)idx[1];
					phys_volume->mHullIndices[i * 3 + 2] = (U16)idx[2];
				}
			}
		}
	}
	//pragma endregion Build the mesh data for a convex hull for a PRIM that is explcitly in CONVEX_HULL mode
	// Now that we've got the hulldecomp let's draw it

	// <FS:Ansariel> Crash fix due to invalid calls to drawElements by Drake Arconis
	//if (phys_volume->mHullPoints)
	if (phys_volume->mHullPoints && phys_volume->mHullIndices && phys_volume->mNumHullPoints > 0 && phys_volume->mNumHullIndices > 0)
		//pragma region ConvexPrimDrawHull
		// </FS:Ansariel>
	{
		//render hull
		// TODO: (BEQ) Find out why is this not a call to render_hull? it probably could be if the data is in the right form

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		gGL.diffuseColor4fv(line_color.mV);
		LLVertexBuffer::unbind();

		llassert(!LLGLSLShader::sNoFixedFunction || LLGLSLShader::sCurBoundShader != 0);

		// <FS:Ansariel> Use a vbo for the static LLVertexBuffer::drawArray/Element functions; by Drake Arconis/Shyotl Kuhr
		//LLVertexBuffer::drawElements(LLRender::TRIANGLES, phys_volume->mHullPoints, NULL, phys_volume->mNumHullIndices, phys_volume->mHullIndices);
		LLVertexBuffer::drawElements(LLRender::TRIANGLES, phys_volume->mNumHullPoints, phys_volume->mHullPoints, NULL, phys_volume->mNumHullIndices, phys_volume->mHullIndices);

		gGL.diffuseColor4fv(color.mV);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		// <FS:Ansariel> Use a vbo for the static LLVertexBuffer::drawArray/Element functions; by Drake Arconis/Shyotl Kuhr
		//LLVertexBuffer::drawElements(LLRender::TRIANGLES, phys_volume->mHullPoints, NULL, phys_volume->mNumHullIndices, phys_volume->mHullIndices);
		LLVertexBuffer::drawElements(LLRender::TRIANGLES, phys_volume->mNumHullPoints, phys_volume->mHullPoints, NULL, phys_volume->mNumHullIndices, phys_volume->mHullIndices);

	}
	else
	{
		// if we don't have a physics convex model then draw a magenta box
		gGL.diffuseColor4f(1, 0, 1, 1);
		drawBoxOutline(center, size);
	}
	//pragma endregion
	LLPrimitive::sVolumeManager->unrefVolume(phys_volume);
}

void renderOnePhysicsShape(LLViewerObject* objectp)
{
	// sanity check this we have a drawable.
	LLDrawable* drawable = objectp->mDrawable;

	if (!drawable)
	{
		return;
	}

	// this is an attached HUD so let's just return.
	if (objectp->isHUDAttachment())
	{
		return;
	}

	LLVOVolume* vovolume = drawable->getVOVolume();

	// phsyics_type is the user selected prim property (None, Prim, Convex)
	U8 physics_type = vovolume->getPhysicsShapeType();

	// If no physics is set to NONE or we're flexi just return
	if (physics_type == LLViewerObject::PHYSICS_SHAPE_NONE || vovolume->isFlexible())
	{
		return;
	}
	// Get the shape details for this object
	LLVolume *volume = vovolume->getVolume();
	LLVolumeParams volume_params = volume->getParams();

	// setup a volume instance to hold the physics shape
	LLPhysicsVolumeParams physics_params(volume_params,
		physics_type == LLViewerObject::PHYSICS_SHAPE_CONVEX_HULL);

	// Set physics_spec to cache the info about the physics shape of our volume.
	LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification physics_spec;
	LLUUID mesh_id;
	LLModel::Decomposition* decomp = nullptr;
	bool hasConvexDecomp = FALSE;

	// If we are a mesh and the mesh has a hul decomp (is analysed) then set hasDecomp to true
	if (vovolume->isMesh()){
		mesh_id = volume_params.getSculptID();
		decomp = gMeshRepo.getDecomposition(mesh_id);
		if (decomp && !decomp->mHull.empty()){ hasConvexDecomp = TRUE; }
	}

	LLPhysicsShapeBuilderUtil::determinePhysicsShape(physics_params, vovolume->getScale(), hasConvexDecomp, physics_spec);

	U32 physicsShapeType = physics_spec.getType();
	/*
	Primitive types
	BOX,
	SPHERE,
	CYLINDER,

	USER_CONVEX,	User specified they wanted the convex hull of the volume
	PRIM_CONVEX,	Either a volume that is inherently convex but not a primitive type, or a shape
	with dimensions such that will convexify it anyway.

	SCULPT,			Special case for traditional sculpts--they are the convex hull of a single particular set of volume params

	USER_MESH,		A user mesh. May or may not contain a convex decomposition.
	PRIM_MESH,		A non-convex volume which we have to represent accurately

	INVALID
	*/

	// This is a link set. The models need an additional transform to modelview
	if (drawable->isActive())
	{
		gGL.loadMatrix(gGLModelView);
		gGL.multMatrix((F32*)objectp->getRenderMatrix().mMatrix);
	}

	gGL.multMatrix((F32*)vovolume->getRelativeXform().mMatrix);

//pragma region PhysicsRenderSettings
	LLColor4 color;

	static LLCachedControl<F32> threshold(gSavedSettings,"ObjectCostHighThreshold");
	static LLCachedControl<LLColor4> low(gSavedSettings,"ObjectCostLowColor");
	static LLCachedControl<LLColor4> mid(gSavedSettings,"ObjectCostMidColor");
	static LLCachedControl<LLColor4> high(gSavedSettings,"ObjectCostHighColor");
	static LLCachedControl<bool> usePhysicsCostOnly(gSavedSettings, "UsePhysicsCostOnly");

	F32 cost = usePhysicsCostOnly ? vovolume->getPhysicsCost() : vovolume->getObjectCost();

	F32 normalizedCost = 1.f - exp(-(cost / threshold));
	if (normalizedCost <= 0.5f)
	{
		color = lerp(low, mid, 2.f * normalizedCost);
	}
	else
	{
		color = lerp(mid, high, 2.f * (normalizedCost - 0.5f));
	}

	LLColor4 line_color = color*0.5f;
//pragma endregion Setup various values from Settings 


	U32 data_mask = LLVertexBuffer::MAP_VERTEX;

	// These two are used to draw the "error" boxes
	LLVector3 center(0, 0, 0);
	LLVector3 size(0.25f, 0.25f, 0.25f);

	if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::USER_MESH)
	{
		// USER_MESH,		A user mesh. May or may not contain a convex decomposition.
		// do we have a Mesh physics loaded yet?
		if (decomp)
		{
			if (hasConvexDecomp) // analysed mesh - we will build a mesh  representation and cache it in the mMesh vector
			{
				// This Model contains a hull based physics. This equates to "analysed" mesh physics in the uploader.
				if (decomp->mMesh.empty())
				{
					// build a mesh representation of the hulls (does nothing if Havok not present)
					gMeshRepo.buildPhysicsMesh(*decomp);
				}

				for (U32 i = 0; i < decomp->mMesh.size(); ++i)
				{
					renderHullPhysics(decomp->mMesh[i], color, line_color);
				}
			}
			else if (!decomp->mPhysicsShapeMesh.empty())
			{
				// This model has triangular mesh (non-analysed)
				renderMeshPhysicsTriangles(color, line_color, volume, decomp);
			}
			else
			{
				//no mesh or decomposition, render base hull
				renderMeshBaseHullPhysics(vovolume, data_mask, color, line_color);

				if (decomp->mPhysicsShapeMesh.empty())
				{
					//attempt to fetch physics shape mesh if available
					gMeshRepo.fetchPhysicsShape(mesh_id);
				}
			}
		}
		else
		{
			// No physics when expected, probably Havok broken/missing or asset still downloading
			// all else fails then ORANGE wireframe box.
			// This typically means you are running without Havok
			gGL.diffuseColor3f(1, 1, 0);
			drawBoxOutline(center, size);
		}
	}
	else if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::USER_CONVEX ||
		physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::PRIM_CONVEX)
	{
		// the owner of the object has selected convex, or we have forced it convex for other reasons
		if (vovolume->isMesh())
		{
			renderMeshBaseHullPhysics(vovolume, data_mask, color, line_color);
		}
		else
		{
			renderNonMeshHullPhysics(vovolume, volume, color, line_color, center, size);
		}
	}
	// vvvv Physics shape is a Havok primitive, either box, sphere or cylinder (object must be a prim)
	else if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::BOX)
	{
		LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPolygonOffset(offset_factor, offset_units);
		LLVector3 center = physics_spec.getCenter();
		LLVector3 scale = physics_spec.getScale();
		LLVector3 vscale = vovolume->getScale()*2.f;
		scale.set(scale[0] / vscale[0], scale[1] / vscale[1], scale[2] / vscale[2]);

		gGL.diffuseColor4fv(color.mV);
		drawBox(center, scale);
	}
	else if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::SPHERE)
	{
		LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPolygonOffset(offset_factor, offset_units);

		LLVolumeParams volume_params;
		volume_params.setType(LL_PCODE_PROFILE_CIRCLE_HALF, LL_PCODE_PATH_CIRCLE);
		volume_params.setBeginAndEndS(0.f, 1.f);
		volume_params.setBeginAndEndT(0.f, 1.f);
		volume_params.setRatio(1, 1);
		volume_params.setShear(0, 0);
		LLVolume* sphere = LLPrimitive::sVolumeManager->refVolume(volume_params, 3);

		gGL.diffuseColor4fv(color.mV);
		pushVerts(sphere);
		LLPrimitive::sVolumeManager->unrefVolume(sphere);
	}
	else if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::CYLINDER)
	{
		LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPolygonOffset(offset_factor, offset_units);

		LLVolumeParams volume_params;
		volume_params.setType(LL_PCODE_PROFILE_CIRCLE, LL_PCODE_PATH_LINE);
		volume_params.setBeginAndEndS(0.f, 1.f);
		volume_params.setBeginAndEndT(0.f, 1.f);
		volume_params.setRatio(1, 1);
		volume_params.setShear(0, 0);
		LLVolume* cylinder = LLPrimitive::sVolumeManager->refVolume(volume_params, 3);

		gGL.diffuseColor4fv(color.mV);
		pushVerts(cylinder);
		LLPrimitive::sVolumeManager->unrefVolume(cylinder);
	}
	else if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::PRIM_MESH)
	//PhysicsShapePrimTriangles - Physics shape for this prim is triangular mesh (typically when prim is cut/hollow etc)
	{
		LLVolumeParams volume_params = volume->getParams();
		// TODO: (Beq) refactor? detail is reused, we ought to be able to pull this out in a wider scope.
		S32 detail = get_physics_detail(volume_params, vovolume->getScale());
		LLVolume* phys_volume = LLPrimitive::sVolumeManager->refVolume(volume_params, detail);

		// TODO: (BEQ) We ought to be able to use a common draw call here too?
		glPolygonOffset(offset_factor, offset_units);

		{
			LLGLEnable offset(GL_POLYGON_OFFSET_LINE);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

			gGL.diffuseColor4fv(line_color.mV);
			pushVerts(phys_volume); // draw the outlines
		}
		{
			LLGLEnable offset(GL_POLYGON_OFFSET_FILL);
			gGL.diffuseColor4fv(color.mV);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			pushVerts(phys_volume); // draw the filled boxes
		}
		LLPrimitive::sVolumeManager->unrefVolume(phys_volume);
	}
	else if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::PRIM_CONVEX)
	// PhysicsShapePrimConvex - Physics shape for prim is inherently convex or convexified due to scale
	{
		LLVolumeParams volume_params = volume->getParams();
		S32 detail = get_physics_detail(volume_params, vovolume->getScale());

		LLVolume* phys_volume = LLPrimitive::sVolumeManager->refVolume(volume_params, detail);

		if (phys_volume->mHullPoints && phys_volume->mHullIndices)
			// We have the hull details so just draw them
		{
			// TODO: (Beq) refactor this!! yet another flavour of drawing the same crap. Can we ratioanlise the arguments
			// <FS:Ansariel> Use a vbo for the static LLVertexBuffer::drawArray/Element functions; by Drake Arconis/Shyotl Kuhr
			if (LLGLSLShader::sNoFixedFunction)
			{
				gGL.diffuseColor4fv(line_color.mV);
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				LLVertexBuffer::drawElements(LLRender::TRIANGLES, phys_volume->mNumHullPoints, phys_volume->mHullPoints, NULL, phys_volume->mNumHullIndices, phys_volume->mHullIndices);

				gGL.diffuseColor4fv(color.mV);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				LLVertexBuffer::drawElements(LLRender::TRIANGLES, phys_volume->mNumHullPoints, phys_volume->mHullPoints, NULL, phys_volume->mNumHullIndices, phys_volume->mHullIndices);
			}
			else
			{
				// </FS:Ansariel>
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				llassert(!LLGLSLShader::sNoFixedFunction || LLGLSLShader::sCurBoundShader != 0);
				LLVertexBuffer::unbind();
				glVertexPointer(3, GL_FLOAT, 16, phys_volume->mHullPoints);
				gGL.diffuseColor4fv(line_color.mV);
				gGL.syncMatrices();
				glDrawElements(GL_TRIANGLES, phys_volume->mNumHullIndices, GL_UNSIGNED_SHORT, phys_volume->mHullIndices);

				gGL.diffuseColor4fv(color.mV);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				glDrawElements(GL_TRIANGLES, phys_volume->mNumHullIndices, GL_UNSIGNED_SHORT, phys_volume->mHullIndices);
				// <FS:Ansariel> Use a vbo for the static LLVertexBuffer::drawArray/Element functions; by Drake Arconis/Shyotl Kuhr
			}
			// </FS:Ansariel>
		}
		else
		{
			// The hull data has not been computed yet (it may never be if havok is not installed) draw a magenta box and request it to be built
			// TODO: can we cache the fact Havok is not installed and speed all this up by not bothering?
			gGL.diffuseColor3f(1, 0, 1);
			drawBoxOutline(center, size);
			gMeshRepo.buildHull(volume_params, detail);
		}
		LLPrimitive::sVolumeManager->unrefVolume(phys_volume);
	}
	// PhysicsShapeSculpt - Sculpts are not supported at present (what happened to "you must render something"?)
	else if (physicsShapeType == LLPhysicsShapeBuilderUtil::PhysicsShapeSpecification::SCULPT)
	{
		//TODO: implement sculpted prim physics display
	}
	else
	{
		LL_ERRS() << "Unhandled type" << LL_ENDL;
	}

}
// End Firestorm additions that add the ability to visualise the physics shape in edit mode.
//</FS:Beq> Physics display in edit mode changes

// Draws the selection outlines for the currently selected objects
// Must be called after displayObjects is called, which sets the mGLName parameter
// NOTE: This function gets called 3 times:
//  render_ui_3d: 			FALSE, FALSE, TRUE
//  render_hud_elements:	FALSE, FALSE, FALSE
void LLViewerWindow::renderSelections( BOOL for_gl_pick, BOOL pick_parcel_walls, BOOL for_hud )
{
	LLObjectSelectionHandle selection = LLSelectMgr::getInstance()->getSelection();

	if (!for_hud && !for_gl_pick)
	{
		// Call this once and only once
		LLSelectMgr::getInstance()->updateSilhouettes();
	}
	
	// Draw fence around land selections
	if (for_gl_pick)
	{
		if (pick_parcel_walls)
		{
			LLViewerParcelMgr::getInstance()->renderParcelCollision();
		}
	}
	else if (( for_hud && selection->getSelectType() == SELECT_TYPE_HUD) ||
			 (!for_hud && selection->getSelectType() != SELECT_TYPE_HUD))
	{		
		LLSelectMgr::getInstance()->renderSilhouettes(for_hud);
		
		stop_glerror();
		
		// <FS:Beq> Additions to display/tools in edit mode
		if (LLToolMgr::getInstance()->inEdit() && selection->getSelectType() != SELECT_TYPE_HUD)
		{
			static LLCachedControl<S32> showSpecificLOD(gSavedSettings, "ShowSpecificLODInEdit");
			static LLCachedControl<bool> showPhysicsShapeInEdit(gSavedSettings, "ShowPhysicsShapeInEdit");

			if (-1 != showSpecificLOD) // Note -1 is disabled [no force] so 0 = lowest 1= low etc. 
			{
				struct f : public LLSelectedObjectFunctor
				{
					virtual bool apply(LLViewerObject* object)
					{
						if (object->getPCode() == LL_PCODE_VOLUME)
						{
							LLVOVolume* vol = static_cast<LLVOVolume*>(object);
							if (vol->isMesh())
							{
								vol->forceLOD(showSpecificLOD);
							}
						}
						return true;
					}
				} func;
				LLSelectMgr::getInstance()->getSelection()->applyToObjects(&func);
			}

			if (showPhysicsShapeInEdit)
			{
				// setup opengl common parameters before we iterate over each object
				gGL.pushMatrix();
				//Need to because crash on ATI 3800 (and similar cards) MAINT-5018 
				LLGLDisable multisample(LLPipeline::RenderFSAASamples > 0 ? GL_MULTISAMPLE_ARB : 0);
				LLGLSLShader* shader = LLGLSLShader::sCurBoundShaderPtr;
				if (shader)
				{
					gDebugProgram.bind();
				}
				gGL.getTexUnit(0)->unbind(LLTexUnit::TT_TEXTURE); // no textures needed
				glClearColor(0, 0, 0, 0); // bg black
				gGL.setColorMask(true, true); // write color and alpha info
				gGL.color4f(1.f, 1.f, 1.f, 0.5);
				gGL.matrixMode(LLRender::MM_MODELVIEW);
				LLGLEnable gls_blend(GL_BLEND);
				LLGLEnable gls_cull(GL_CULL_FACE);
				LLGLDepthTest gls_depth(GL_TRUE, GL_FALSE);

				struct f : public LLSelectedObjectFunctor
				{
					virtual bool apply(LLViewerObject* object)
					{
						renderOnePhysicsShape(object);
						return true;
					}
				} func;
				LLSelectMgr::getInstance()->getSelection()->applyToObjects(&func);
				// Restore the original shader program
				if (shader)
				{
					shader->bind();
				}

				gGL.popMatrix();
			}
		}
		// </FS:Beq>

		// setup HUD render
		if (selection->getSelectType() == SELECT_TYPE_HUD && LLSelectMgr::getInstance()->getSelection()->getObjectCount())
		{
			LLBBox hud_bbox = gAgentAvatarp->getHUDBBox();

			// set up transform to encompass bounding box of HUD
			gGL.matrixMode(LLRender::MM_PROJECTION);
			gGL.pushMatrix();
			gGL.loadIdentity();
			F32 depth = llmax(1.f, hud_bbox.getExtentLocal().mV[VX] * 1.1f);
			gGL.ortho(-0.5f * LLViewerCamera::getInstance()->getAspect(), 0.5f * LLViewerCamera::getInstance()->getAspect(), -0.5f, 0.5f, 0.f, depth);
			
			gGL.matrixMode(LLRender::MM_MODELVIEW);
			gGL.pushMatrix();
			gGL.loadIdentity();
			gGL.loadMatrix(OGL_TO_CFR_ROTATION);		// Load Cory's favorite reference frame
			gGL.translatef(-hud_bbox.getCenterLocal().mV[VX] + (depth *0.5f), 0.f, 0.f);
		}

		// Render light for editing
		if (LLSelectMgr::sRenderLightRadius && LLToolMgr::getInstance()->inEdit())
		{
			gGL.getTexUnit(0)->unbind(LLTexUnit::TT_TEXTURE);
			LLGLEnable gls_blend(GL_BLEND);
			LLGLEnable gls_cull(GL_CULL_FACE);
			LLGLDepthTest gls_depth(GL_TRUE, GL_FALSE);
			gGL.matrixMode(LLRender::MM_MODELVIEW);
			gGL.pushMatrix();
			if (selection->getSelectType() == SELECT_TYPE_HUD)
			{
				F32 zoom = gAgentCamera.mHUDCurZoom;
				gGL.scalef(zoom, zoom, zoom);
			}

			struct f : public LLSelectedObjectFunctor
			{
				virtual bool apply(LLViewerObject* object)
				{
					LLDrawable* drawable = object->mDrawable;
					if (drawable && drawable->isLight())
					{
						LLVOVolume* vovolume = drawable->getVOVolume();
						gGL.pushMatrix();

						LLVector3 center = drawable->getPositionAgent();
						gGL.translatef(center[0], center[1], center[2]);
						F32 scale = vovolume->getLightRadius();
						gGL.scalef(scale, scale, scale);

						LLColor4 color(vovolume->getLightColor(), .5f);
						gGL.color4fv(color.mV);
					
						//F32 pixel_area = 100000.f;
						// Render Outside
						gSphere.render();

						// Render Inside
						glCullFace(GL_FRONT);
						gSphere.render();
						glCullFace(GL_BACK);
					
						gGL.popMatrix();
					}
					return true;
				}
			} func;
			LLSelectMgr::getInstance()->getSelection()->applyToObjects(&func);
			
			gGL.popMatrix();
		}				
		
		// NOTE: The average position for the axis arrows of the selected objects should
		// not be recalculated at this time.  If they are, then group rotations will break.

		// Draw arrows at average center of all selected objects
		LLTool* tool = LLToolMgr::getInstance()->getCurrentTool();
		if (tool)
		{
			if(tool->isAlwaysRendered())
			{
				tool->render();
			}
			else
			{
				if( !LLSelectMgr::getInstance()->getSelection()->isEmpty() )
				{
					bool all_selected_objects_move;
					bool all_selected_objects_modify;
					// Note: This might be costly to do on each frame and when a lot of objects are selected
					// we might be better off with some kind of memory for selection and/or states, consider
					// optimizing, perhaps even some kind of selection generation at level of LLSelectMgr to
					// make whole viewer benefit.
					LLSelectMgr::getInstance()->selectGetEditMoveLinksetPermissions(all_selected_objects_move, all_selected_objects_modify);

					BOOL draw_handles = TRUE;

					if (tool == LLToolCompTranslate::getInstance() && !all_selected_objects_move && !LLSelectMgr::getInstance()->isSelfAvatarSelected())
					{
						draw_handles = FALSE;
					}

					if (tool == LLToolCompRotate::getInstance() && !all_selected_objects_move)
					{
						draw_handles = FALSE;
					}

					if ( !all_selected_objects_modify && tool == LLToolCompScale::getInstance() )
					{
						draw_handles = FALSE;
					}
				
					if( draw_handles )
					{
						tool->render();
					}
				}
			}
			if (selection->getSelectType() == SELECT_TYPE_HUD && selection->getObjectCount())
			{
				gGL.matrixMode(LLRender::MM_PROJECTION);
				gGL.popMatrix();

				gGL.matrixMode(LLRender::MM_MODELVIEW);
				gGL.popMatrix();
				stop_glerror();
			}
		}
	}
}

// Return a point near the clicked object representative of the place the object was clicked.
LLVector3d LLViewerWindow::clickPointInWorldGlobal(S32 x, S32 y_from_bot, LLViewerObject* clicked_object) const
{
	// create a normalized vector pointing from the camera center into the 
	// world at the location of the mouse click
	LLVector3 mouse_direction_global = mouseDirectionGlobal( x, y_from_bot );

	LLVector3d relative_object = clicked_object->getPositionGlobal() - gAgentCamera.getCameraPositionGlobal();

	// make mouse vector as long as object vector, so it touchs a point near
	// where the user clicked on the object
	mouse_direction_global *= (F32) relative_object.magVec();

	LLVector3d new_pos;
	new_pos.setVec(mouse_direction_global);
	// transform mouse vector back to world coords
	new_pos += gAgentCamera.getCameraPositionGlobal();

	return new_pos;
}


BOOL LLViewerWindow::clickPointOnSurfaceGlobal(const S32 x, const S32 y, LLViewerObject *objectp, LLVector3d &point_global) const
{
	BOOL intersect = FALSE;

//	U8 shape = objectp->mPrimitiveCode & LL_PCODE_BASE_MASK;
	if (!intersect)
	{
		point_global = clickPointInWorldGlobal(x, y, objectp);
		LL_INFOS() << "approx intersection at " <<  (objectp->getPositionGlobal() - point_global) << LL_ENDL;
	}
	else
	{
		LL_INFOS() << "good intersection at " <<  (objectp->getPositionGlobal() - point_global) << LL_ENDL;
	}

	return intersect;
}

void LLViewerWindow::pickAsync( S32 x,
								S32 y_from_bot,
								MASK mask,
								void (*callback)(const LLPickInfo& info),
								BOOL pick_transparent,
								BOOL pick_rigged,
								BOOL pick_unselectable)
{
	BOOL in_build_mode = LLFloaterReg::instanceVisible("build");
	if (in_build_mode || LLDrawPoolAlpha::sShowDebugAlpha)
	{
		// build mode allows interaction with all transparent objects
		// "Show Debug Alpha" means no object actually transparent
		pick_transparent = TRUE;
	}

	LLPickInfo pick_info(LLCoordGL(x, y_from_bot), mask, pick_transparent, pick_rigged, FALSE, TRUE, pick_unselectable, callback);
	schedulePick(pick_info);
}

void LLViewerWindow::schedulePick(LLPickInfo& pick_info)
{
	if (mPicks.size() >= 1024 || mWindow->getMinimized())
	{ //something went wrong, picks are being scheduled but not processed
		
		if (pick_info.mPickCallback)
		{
			pick_info.mPickCallback(pick_info);
		}
	
		return;
	}
	mPicks.push_back(pick_info);
	
	// delay further event processing until we receive results of pick
	// only do this for async picks so that handleMouseUp won't be called
	// until the pick triggered in handleMouseDown has been processed, for example
	mWindow->delayInputProcessing();
}


void LLViewerWindow::performPick()
{
	if (!mPicks.empty())
	{
		std::vector<LLPickInfo>::iterator pick_it;
		for (pick_it = mPicks.begin(); pick_it != mPicks.end(); ++pick_it)
		{
			pick_it->fetchResults();
		}

		mLastPick = mPicks.back();
		mPicks.clear();
	}
}

void LLViewerWindow::returnEmptyPicks()
{
	std::vector<LLPickInfo>::iterator pick_it;
	for (pick_it = mPicks.begin(); pick_it != mPicks.end(); ++pick_it)
	{
		mLastPick = *pick_it;
		// just trigger callback with empty results
		if (pick_it->mPickCallback)
		{
			pick_it->mPickCallback(*pick_it);
		}
	}
	mPicks.clear();
}

// Performs the GL object/land pick.
LLPickInfo LLViewerWindow::pickImmediate(S32 x, S32 y_from_bot, BOOL pick_transparent, BOOL pick_rigged, BOOL pick_particle)
{
	BOOL in_build_mode = LLFloaterReg::instanceVisible("build");
	if (in_build_mode || LLDrawPoolAlpha::sShowDebugAlpha)
	{
		// build mode allows interaction with all transparent objects
		// "Show Debug Alpha" means no object actually transparent
		pick_transparent = TRUE;
	}
	
	// shortcut queueing in mPicks and just update mLastPick in place
	MASK	key_mask = gKeyboard->currentMask(TRUE);
	mLastPick = LLPickInfo(LLCoordGL(x, y_from_bot), key_mask, pick_transparent, pick_rigged, pick_particle, TRUE, FALSE, NULL);
	mLastPick.fetchResults();

	return mLastPick;
}

LLHUDIcon* LLViewerWindow::cursorIntersectIcon(S32 mouse_x, S32 mouse_y, F32 depth,
										   LLVector4a* intersection)
{
	S32 x = mouse_x;
	S32 y = mouse_y;

	if ((mouse_x == -1) && (mouse_y == -1)) // use current mouse position
	{
		x = getCurrentMouseX();
		y = getCurrentMouseY();
	}

	// world coordinates of mouse
	// VECTORIZE THIS
	LLVector3 mouse_direction_global = mouseDirectionGlobal(x,y);
	LLVector3 mouse_point_global = LLViewerCamera::getInstance()->getOrigin();
	LLVector3 mouse_world_start = mouse_point_global;
	LLVector3 mouse_world_end   = mouse_point_global + mouse_direction_global * depth;

	LLVector4a start, end;
	start.load3(mouse_world_start.mV);
	end.load3(mouse_world_end.mV);
	
	return LLHUDIcon::lineSegmentIntersectAll(start, end, intersection);
}

LLViewerObject* LLViewerWindow::cursorIntersect(S32 mouse_x, S32 mouse_y, F32 depth,
												LLViewerObject *this_object,
												S32 this_face,
												BOOL pick_transparent,
												BOOL pick_rigged,
												S32* face_hit,
												LLVector4a *intersection,
												LLVector2 *uv,
												LLVector4a *normal,
												LLVector4a *tangent,
												LLVector4a* start,
												LLVector4a* end)
{
	S32 x = mouse_x;
	S32 y = mouse_y;

	if ((mouse_x == -1) && (mouse_y == -1)) // use current mouse position
	{
		x = getCurrentMouseX();
		y = getCurrentMouseY();
	}

	// HUD coordinates of mouse
	LLVector3 mouse_point_hud = mousePointHUD(x, y);
	LLVector3 mouse_hud_start = mouse_point_hud - LLVector3(depth, 0, 0);
	LLVector3 mouse_hud_end   = mouse_point_hud + LLVector3(depth, 0, 0);
	
	// world coordinates of mouse
	LLVector3 mouse_direction_global = mouseDirectionGlobal(x,y);
	LLVector3 mouse_point_global = LLViewerCamera::getInstance()->getOrigin();
	
	//get near clip plane
	LLVector3 n = LLViewerCamera::getInstance()->getAtAxis();
	LLVector3 p = mouse_point_global + n * LLViewerCamera::getInstance()->getNear();

	//project mouse point onto plane
	LLVector3 pos;
	line_plane(mouse_point_global, mouse_direction_global, p, n, pos);
	mouse_point_global = pos;

	LLVector3 mouse_world_start = mouse_point_global;
	LLVector3 mouse_world_end   = mouse_point_global + mouse_direction_global * depth;

	if (!LLViewerJoystick::getInstance()->getOverrideCamera())
	{ //always set raycast intersection to mouse_world_end unless
		//flycam is on (for DoF effect)
		gDebugRaycastIntersection.load3(mouse_world_end.mV);
	}

	LLVector4a mw_start;
	mw_start.load3(mouse_world_start.mV);
	LLVector4a mw_end;
	mw_end.load3(mouse_world_end.mV);

	LLVector4a mh_start;
	mh_start.load3(mouse_hud_start.mV);
	LLVector4a mh_end;
	mh_end.load3(mouse_hud_end.mV);

	if (start)
	{
		*start = mw_start;
	}

	if (end)
	{
		*end = mw_end;
	}

	LLViewerObject* found = NULL;

	if (this_object)  // check only this object
	{
		if (this_object->isHUDAttachment()) // is a HUD object?
		{
			if (this_object->lineSegmentIntersect(mh_start, mh_end, this_face, pick_transparent, pick_rigged,
												  face_hit, intersection, uv, normal, tangent))
			{
				found = this_object;
			}
		}
		else // is a world object
		{
			if (this_object->lineSegmentIntersect(mw_start, mw_end, this_face, pick_transparent, pick_rigged,
												  face_hit, intersection, uv, normal, tangent))
			{
				found = this_object;
			}
		}
	}
	else // check ALL objects
	{
		found = gPipeline.lineSegmentIntersectInHUD(mh_start, mh_end, pick_transparent,
													face_hit, intersection, uv, normal, tangent);

// [RLVa:KB] - Checked: 2010-03-31 (RLVa-1.2.0c) | Modified: RLVa-1.2.0c
		if ( (rlv_handler_t::isEnabled()) && (found) &&
			 (LLToolCamera::getInstance()->hasMouseCapture()) && (gKeyboard->currentMask(TRUE) & MASK_ALT) )
		{
			found = NULL;
		}
// [/RLVa:KB]
		if (!found) // if not found in HUD, look in world:
		{
			found = gPipeline.lineSegmentIntersectInWorld(mw_start, mw_end, pick_transparent, pick_rigged,
														  face_hit, intersection, uv, normal, tangent);
			if (found && !pick_transparent)
			{
				gDebugRaycastIntersection = *intersection;
			}
		}

// [RLVa:KB] - Checked: RLVa-1.2.0
		if ( (found) && ((gTeleportDisplay) || ((rlv_handler_t::isEnabled()) && (gRlvHandler.hasBehaviour(RLV_BHVR_INTERACT)))) )
		{
			// Allow picking if:
			//   - the drag-and-drop tool is active (allows inventory offers)
			//   - the camera tool is active
			//   - the pie tool is active *and* we picked our own avie (allows "mouse steering" and the self pie menu)
			LLTool* pCurTool = LLToolMgr::getInstance()->getCurrentTool();
			if ( (LLToolDragAndDrop::getInstance() != pCurTool) && 
			     (!LLToolCamera::getInstance()->hasMouseCapture()) &&
			     ((LLToolPie::getInstance() != pCurTool) || (gAgent.getID() != found->getID())) )
			{
				found = NULL;
			}
		}
// [/RLVa:KB]
	}

	return found;
}

// Returns unit vector relative to camera
// indicating direction of point on screen x,y
LLVector3 LLViewerWindow::mouseDirectionGlobal(const S32 x, const S32 y) const
{
	// find vertical field of view
	F32			fov = LLViewerCamera::getInstance()->getView();

	// find world view center in scaled ui coordinates
	F32			center_x = getWorldViewRectScaled().getCenterX();
	F32			center_y = getWorldViewRectScaled().getCenterY();

	// calculate pixel distance to screen
	F32			distance = ((F32)getWorldViewHeightScaled() * 0.5f) / (tan(fov / 2.f));

	// calculate click point relative to middle of screen
	F32			click_x = x - center_x;
	F32			click_y = y - center_y;

	// compute mouse vector
	LLVector3	mouse_vector =	distance * LLViewerCamera::getInstance()->getAtAxis()
								- click_x * LLViewerCamera::getInstance()->getLeftAxis()
								+ click_y * LLViewerCamera::getInstance()->getUpAxis();

	mouse_vector.normVec();

	return mouse_vector;
}

LLVector3 LLViewerWindow::mousePointHUD(const S32 x, const S32 y) const
{
	// find screen resolution
	S32			height = getWorldViewHeightScaled();

	// find world view center
	F32			center_x = getWorldViewRectScaled().getCenterX();
	F32			center_y = getWorldViewRectScaled().getCenterY();

	// remap with uniform scale (1/height) so that top is -0.5, bottom is +0.5
	F32 hud_x = -((F32)x - center_x)  / height;
	F32 hud_y = ((F32)y - center_y) / height;

	return LLVector3(0.f, hud_x/gAgentCamera.mHUDCurZoom, hud_y/gAgentCamera.mHUDCurZoom);
}

// Returns unit vector relative to camera in camera space
// indicating direction of point on screen x,y
LLVector3 LLViewerWindow::mouseDirectionCamera(const S32 x, const S32 y) const
{
	// find vertical field of view
	F32			fov_height = LLViewerCamera::getInstance()->getView();
	F32			fov_width = fov_height * LLViewerCamera::getInstance()->getAspect();

	// find screen resolution
	S32			height = getWorldViewHeightScaled();
	S32			width = getWorldViewWidthScaled();

	// find world view center
	F32			center_x = getWorldViewRectScaled().getCenterX();
	F32			center_y = getWorldViewRectScaled().getCenterY();

	// calculate click point relative to middle of screen
	F32			click_x = (((F32)x - center_x) / (F32)width) * fov_width * -1.f;
	F32			click_y = (((F32)y - center_y) / (F32)height) * fov_height;

	// compute mouse vector
	LLVector3	mouse_vector =	LLVector3(0.f, 0.f, -1.f);
	LLQuaternion mouse_rotate;
	mouse_rotate.setQuat(click_y, click_x, 0.f);

	mouse_vector = mouse_vector * mouse_rotate;
	// project to z = -1 plane;
	mouse_vector = mouse_vector * (-1.f / mouse_vector.mV[VZ]);

	return mouse_vector;
}



BOOL LLViewerWindow::mousePointOnPlaneGlobal(LLVector3d& point, const S32 x, const S32 y, 
										const LLVector3d &plane_point_global, 
										const LLVector3 &plane_normal_global)
{
	LLVector3d	mouse_direction_global_d;

	mouse_direction_global_d.setVec(mouseDirectionGlobal(x,y));
	LLVector3d	plane_normal_global_d;
	plane_normal_global_d.setVec(plane_normal_global);
	F64 plane_mouse_dot = (plane_normal_global_d * mouse_direction_global_d);
	LLVector3d plane_origin_camera_rel = plane_point_global - gAgentCamera.getCameraPositionGlobal();
	F64	mouse_look_at_scale = (plane_normal_global_d * plane_origin_camera_rel)
								/ plane_mouse_dot;
	if (llabs(plane_mouse_dot) < 0.00001)
	{
		// if mouse is parallel to plane, return closest point on line through plane origin
		// that is parallel to camera plane by scaling mouse direction vector
		// by distance to plane origin, modulated by deviation of mouse direction from plane origin
		LLVector3d plane_origin_dir = plane_origin_camera_rel;
		plane_origin_dir.normVec();
		
		mouse_look_at_scale = plane_origin_camera_rel.magVec() / (plane_origin_dir * mouse_direction_global_d);
	}

	point = gAgentCamera.getCameraPositionGlobal() + mouse_look_at_scale * mouse_direction_global_d;

	return mouse_look_at_scale > 0.0;
}


// Returns global position
BOOL LLViewerWindow::mousePointOnLandGlobal(const S32 x, const S32 y, LLVector3d *land_position_global, BOOL ignore_distance)
{
	LLVector3		mouse_direction_global = mouseDirectionGlobal(x,y);
	F32				mouse_dir_scale;
	BOOL			hit_land = FALSE;
	LLViewerRegion	*regionp;
	F32			land_z;
	const F32	FIRST_PASS_STEP = 1.0f;		// meters
	const F32	SECOND_PASS_STEP = 0.1f;	// meters
	const F32	draw_distance = ignore_distance ? MAX_FAR_CLIP : gAgentCamera.mDrawDistance;
	LLVector3d	camera_pos_global;

	camera_pos_global = gAgentCamera.getCameraPositionGlobal();
	LLVector3d		probe_point_global;
	LLVector3		probe_point_region;

	// walk forwards to find the point
	for (mouse_dir_scale = FIRST_PASS_STEP; mouse_dir_scale < draw_distance; mouse_dir_scale += FIRST_PASS_STEP)
	{
		LLVector3d mouse_direction_global_d;
		mouse_direction_global_d.setVec(mouse_direction_global * mouse_dir_scale);
		probe_point_global = camera_pos_global + mouse_direction_global_d;

		regionp = LLWorld::getInstance()->resolveRegionGlobal(probe_point_region, probe_point_global);

		if (!regionp)
		{
			// ...we're outside the world somehow
			continue;
		}

		S32 i = (S32) (probe_point_region.mV[VX]/regionp->getLand().getMetersPerGrid());
		S32 j = (S32) (probe_point_region.mV[VY]/regionp->getLand().getMetersPerGrid());
		S32 grids_per_edge = (S32) regionp->getLand().mGridsPerEdge;
		if ((i >= grids_per_edge) || (j >= grids_per_edge))
		{
			//LL_INFOS() << "LLViewerWindow::mousePointOnLand probe_point is out of region" << LL_ENDL;
			continue;
		}

		land_z = regionp->getLand().resolveHeightRegion(probe_point_region);

		//LL_INFOS() << "mousePointOnLand initial z " << land_z << LL_ENDL;

		if (probe_point_region.mV[VZ] < land_z)
		{
			// ...just went under land

			// cout << "under land at " << probe_point << " scale " << mouse_vec_scale << endl;

			hit_land = TRUE;
			break;
		}
	}


	if (hit_land)
	{
		// Don't go more than one step beyond where we stopped above.
		// This can't just be "mouse_vec_scale" because floating point error
		// will stop the loop before the last increment.... X - 1.0 + 0.1 + 0.1 + ... + 0.1 != X
		F32 stop_mouse_dir_scale = mouse_dir_scale + FIRST_PASS_STEP;

		// take a step backwards, then walk forwards again to refine position
		for ( mouse_dir_scale -= FIRST_PASS_STEP; mouse_dir_scale <= stop_mouse_dir_scale; mouse_dir_scale += SECOND_PASS_STEP)
		{
			LLVector3d mouse_direction_global_d;
			mouse_direction_global_d.setVec(mouse_direction_global * mouse_dir_scale);
			probe_point_global = camera_pos_global + mouse_direction_global_d;

			regionp = LLWorld::getInstance()->resolveRegionGlobal(probe_point_region, probe_point_global);

			if (!regionp)
			{
				// ...we're outside the world somehow
				continue;
			}

			/*
			i = (S32) (local_probe_point.mV[VX]/regionp->getLand().getMetersPerGrid());
			j = (S32) (local_probe_point.mV[VY]/regionp->getLand().getMetersPerGrid());
			if ((i >= regionp->getLand().mGridsPerEdge) || (j >= regionp->getLand().mGridsPerEdge))
			{
				// LL_INFOS() << "LLViewerWindow::mousePointOnLand probe_point is out of region" << LL_ENDL;
				continue;
			}
			land_z = regionp->getLand().mSurfaceZ[ i + j * (regionp->getLand().mGridsPerEdge) ];
			*/

			land_z = regionp->getLand().resolveHeightRegion(probe_point_region);

			//LL_INFOS() << "mousePointOnLand refine z " << land_z << LL_ENDL;

			if (probe_point_region.mV[VZ] < land_z)
			{
				// ...just went under land again

				*land_position_global = probe_point_global;
				return TRUE;
			}
		}
	}

	return FALSE;
}

// Saves an image to the harddrive as "SnapshotX" where X >= 1.
void LLViewerWindow::saveImageNumbered(LLImageFormatted *image, BOOL force_picker, const snapshot_saved_signal_t::slot_type& success_cb, const snapshot_saved_signal_t::slot_type& failure_cb)
{
	if (!image)
	{
		LL_WARNS() << "No image to save" << LL_ENDL;
		return;
	}
	std::string extension("." + image->getExtension());
	LLImageFormatted* formatted_image = image;
	// Get a base file location if needed.
	if (force_picker || !isSnapshotLocSet())
	{
		std::string proposed_name(sSnapshotBaseName);

		// getSaveFile will append an appropriate extension to the proposed name, based on the ESaveFilter constant passed in.
		LLFilePicker::ESaveFilter pick_type;

		if (extension == ".j2c")
			pick_type = LLFilePicker::FFSAVE_J2C;
		else if (extension == ".bmp")
			pick_type = LLFilePicker::FFSAVE_BMP;
		else if (extension == ".jpg")
			pick_type = LLFilePicker::FFSAVE_JPEG;
		else if (extension == ".png")
			pick_type = LLFilePicker::FFSAVE_PNG;
		else if (extension == ".tga")
			pick_type = LLFilePicker::FFSAVE_TGA;
		else
			pick_type = LLFilePicker::FFSAVE_ALL;

		(new LLFilePickerReplyThread(boost::bind(&LLViewerWindow::onDirectorySelected, this, _1, formatted_image, success_cb, failure_cb), pick_type, proposed_name,
										boost::bind(&LLViewerWindow::onSelectionFailure, this, failure_cb)))->getFile();
	}
	else
	{
		saveImageLocal(formatted_image, success_cb, failure_cb);
	}	
}

void LLViewerWindow::onDirectorySelected(const std::vector<std::string>& filenames, LLImageFormatted *image, const snapshot_saved_signal_t::slot_type& success_cb, const snapshot_saved_signal_t::slot_type& failure_cb)
{
	// Copy the directory + file name
	std::string filepath = filenames[0];

	gSavedPerAccountSettings.setString("SnapshotBaseName", gDirUtilp->getBaseFileName(filepath, true));
	gSavedPerAccountSettings.setString("SnapshotBaseDir", gDirUtilp->getDirName(filepath));
	saveImageLocal(image, success_cb, failure_cb);
}

void LLViewerWindow::onSelectionFailure(const snapshot_saved_signal_t::slot_type& failure_cb)
{
	failure_cb();
}


void LLViewerWindow::saveImageLocal(LLImageFormatted *image, const snapshot_saved_signal_t::slot_type& success_cb, const snapshot_saved_signal_t::slot_type& failure_cb)
{
	std::string lastSnapshotDir = LLViewerWindow::getLastSnapshotDir();
	if (lastSnapshotDir.empty())
	{
		failure_cb();
		return;
	}

// Check if there is enough free space to save snapshot
#ifdef LL_WINDOWS
	boost::filesystem::path b_path(utf8str_to_utf16str(lastSnapshotDir));
#else
	boost::filesystem::path b_path(lastSnapshotDir);
#endif
	if (!boost::filesystem::is_directory(b_path))
	{
		LLSD args;
		args["PATH"] = lastSnapshotDir;
		LLNotificationsUtil::add("SnapshotToLocalDirNotExist", args);
		resetSnapshotLoc();
		failure_cb();
		return;
	}
	boost::filesystem::space_info b_space = boost::filesystem::space(b_path);
	if (b_space.free < image->getDataSize())
	{
		LLSD args;
		args["PATH"] = lastSnapshotDir;

		std::string needM_bytes_string;
		LLResMgr::getInstance()->getIntegerString(needM_bytes_string, (image->getDataSize()) >> 10);
		args["NEED_MEMORY"] = needM_bytes_string;

		std::string freeM_bytes_string;
		LLResMgr::getInstance()->getIntegerString(freeM_bytes_string, (b_space.free) >> 10);
		args["FREE_MEMORY"] = freeM_bytes_string;

		LLNotificationsUtil::add("SnapshotToComputerFailed", args);

		failure_cb();
	}
	
	// Look for an unused file name
	BOOL is_snapshot_name_loc_set = isSnapshotLocSet();
	std::string filepath;
	S32 i = 1;
	S32 err = 0;
	std::string extension("." + image->getExtension());
	do
	{
		filepath = sSnapshotDir;
		filepath += gDirUtilp->getDirDelimiter();
		filepath += sSnapshotBaseName;

		if (is_snapshot_name_loc_set)
		{
			filepath += llformat("_%.3d",i);
		}		

		filepath += extension;

		llstat stat_info;
		err = LLFile::stat( filepath, &stat_info );
		i++;
	}
	while( -1 != err  // Search until the file is not found (i.e., stat() gives an error).
			&& is_snapshot_name_loc_set); // Or stop if we are rewriting.

	LL_INFOS() << "Saving snapshot to " << filepath << LL_ENDL;
	if (image->save(filepath))
	{
		playSnapshotAnimAndSound();
		if (gSavedSettings.getBOOL("FSLogSnapshotsToLocal"))
		{
			LLStringUtil::format_map_t args;
			args["FILENAME"] = filepath;
			report_to_nearby_chat(LLTrans::getString("SnapshotSavedToDisk", args));
		}
		success_cb();
	}
	else
	{
		failure_cb();
	}
}

void LLViewerWindow::resetSnapshotLoc()
{
	gSavedPerAccountSettings.setString("SnapshotBaseDir", std::string());
}

// static
void LLViewerWindow::movieSize(S32 new_width, S32 new_height)
{
	// <FS:TS> FIRE-6182: Set Window Size sets random size each time
	// Don't use LLCoordWindow, since the chosen resolution winds up
	// with position dependent numbers added each time. Instead, we use
	// LLCoordScreen, which avoids this. Fix from Niran's Viewer.
	// LLCoordWindow size;
	// LLCoordWindow new_size(new_width, new_height);
	// gViewerWindow->getWindow()->getSize(&size);
	// if ( size != new_size )
	// {
	//	gViewerWindow->getWindow()->setSize(new_size.convert());
	// }
	U32 nChromeW(0), nChromeH(0);
	gViewerWindow->getWindow()->getWindowChrome( nChromeW, nChromeH );

	LLCoordScreen new_size;
	new_size.mX = new_width + nChromeW;
	new_size.mY = new_height + nChromeH;
	gViewerWindow->getWindow()->setSize(new_size);
	// </FS:TS>

}

BOOL LLViewerWindow::saveSnapshot(const std::string& filepath, S32 image_width, S32 image_height, BOOL show_ui, BOOL show_hud, BOOL do_rebuild, LLSnapshotModel::ESnapshotLayerType type, LLSnapshotModel::ESnapshotFormat format)
{
    LL_INFOS() << "Saving snapshot to: " << filepath << LL_ENDL;

    LLPointer<LLImageRaw> raw = new LLImageRaw;
    BOOL success = rawSnapshot(raw, image_width, image_height, TRUE, FALSE, show_ui, show_hud, do_rebuild);

    if (success)
    {
        U8 image_codec = IMG_CODEC_BMP;
        switch (format)
        {
        case LLSnapshotModel::SNAPSHOT_FORMAT_PNG:
            image_codec = IMG_CODEC_PNG;
            break;
        case LLSnapshotModel::SNAPSHOT_FORMAT_JPEG:
            image_codec = IMG_CODEC_JPEG;
            break;
        default:
            image_codec = IMG_CODEC_BMP;
            break;
        }

        LLPointer<LLImageFormatted> formated_image = LLImageFormatted::createFromType(image_codec);
        success = formated_image->encode(raw, 0.0f);
        if (success)
        {
            success = formated_image->save(filepath);
        }
        else
        {
            LL_WARNS() << "Unable to encode snapshot of format " << format << LL_ENDL;
        }
    }
    else
    {
        LL_WARNS() << "Unable to capture raw snapshot" << LL_ENDL;
    }

    return success;
}


void LLViewerWindow::playSnapshotAnimAndSound()
{
	// <FS:PP> FIRE-8190: Preview function for "UI Sounds" Panel
	// if (gSavedSettings.getBOOL("QuietSnapshotsToDisk"))
	if (gSavedSettings.getBOOL("PlayModeUISndSnapshot"))
	// </FS:PP> FIRE-8190: Preview function for "UI Sounds" Panel
	{
		return;
	}
	gAgent.sendAnimationRequest(ANIM_AGENT_SNAPSHOT, ANIM_REQUEST_START);
	send_sound_trigger(LLUUID(gSavedSettings.getString("UISndSnapshot")), 1.0f);
}

BOOL LLViewerWindow::isSnapshotLocSet() const
{
	std::string snapshot_dir = sSnapshotDir;
	return !snapshot_dir.empty();
}

void LLViewerWindow::resetSnapshotLoc() const
{
	gSavedPerAccountSettings.setString("SnapshotBaseDir", std::string());
}

BOOL LLViewerWindow::thumbnailSnapshot(LLImageRaw *raw, S32 preview_width, S32 preview_height, BOOL show_ui, BOOL show_hud, BOOL do_rebuild, LLSnapshotModel::ESnapshotLayerType type)
{
	return rawSnapshot(raw, preview_width, preview_height, FALSE, FALSE, show_ui, show_hud, do_rebuild, type);
}

// Saves the image from the screen to a raw image
// Since the required size might be bigger than the available screen, this method rerenders the scene in parts (called subimages) and copy
// the results over to the final raw image.
BOOL LLViewerWindow::rawSnapshot(LLImageRaw *raw, S32 image_width, S32 image_height, 
    BOOL keep_window_aspect, BOOL is_texture, BOOL show_ui, BOOL show_hud, BOOL do_rebuild, LLSnapshotModel::ESnapshotLayerType type, S32 max_size)
{
	if (!raw)
	{
		return FALSE;
	}
	//check if there is enough memory for the snapshot image
	if(LLPipeline::sMemAllocationThrottled)
	{
		return FALSE ; //snapshot taking is disabled due to memory restriction.
	}
	if(image_width * image_height > (1 << 22)) //if snapshot image is larger than 2K by 2K
	{
		if(!LLMemory::tryToAlloc(NULL, image_width * image_height * 3))
		{
			LL_WARNS() << "No enough memory to take the snapshot with size (w : h): " << image_width << " : " << image_height << LL_ENDL ;
			return FALSE ; //there is no enough memory for taking this snapshot.
		}
	}

	// PRE SNAPSHOT
	gDisplaySwapBuffers = FALSE;
	
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	setCursor(UI_CURSOR_WAIT);

	// Hide all the UI widgets first and draw a frame
	BOOL prev_draw_ui = gPipeline.hasRenderDebugFeatureMask(LLPipeline::RENDER_DEBUG_FEATURE_UI) ? TRUE : FALSE;

	if ( prev_draw_ui != show_ui)
	{
		LLPipeline::toggleRenderDebugFeature(LLPipeline::RENDER_DEBUG_FEATURE_UI);
	}

    BOOL hide_hud = !show_hud && LLPipeline::sShowHUDAttachments;
	if (hide_hud)
	{
		LLPipeline::sShowHUDAttachments = FALSE;
	}

	// if not showing ui, use full window to render world view
	updateWorldViewRect(!show_ui);

	// Copy screen to a buffer
	// crop sides or top and bottom, if taking a snapshot of different aspect ratio
	// from window
	LLRect window_rect = show_ui ? getWindowRectRaw() : getWorldViewRectRaw(); 

	S32 snapshot_width  = window_rect.getWidth();
	S32 snapshot_height = window_rect.getHeight();
	// SNAPSHOT
	S32 window_width  = snapshot_width;
	S32 window_height = snapshot_height;
	
	// Note: Scaling of the UI is currently *not* supported so we limit the output size if UI is requested
	if (show_ui)
	{
		// If the user wants the UI, limit the output size to the available screen size
		image_width  = llmin(image_width, window_width);
		image_height = llmin(image_height, window_height);
		
		// <FS:CR> Hide currency balance in snapshots
		if (gStatusBar)
		{
			gStatusBar->showBalance((bool)gSavedSettings.getBOOL("FSShowCurrencyBalanceInSnapshots"));
		}
	}

	S32 original_width = 0;
	S32 original_height = 0;
	bool reset_deferred = false;

	LLRenderTarget scratch_space;

	F32 scale_factor = 1.0f ;
	if (!keep_window_aspect || (image_width > window_width) || (image_height > window_height))
	{	
		if ((image_width <= gGLManager.mGLMaxTextureSize && image_height <= gGLManager.mGLMaxTextureSize) && 
			(image_width > window_width || image_height > window_height) && LLPipeline::sRenderDeferred && !show_ui)
		{
			U32 color_fmt = type == LLSnapshotModel::SNAPSHOT_TYPE_DEPTH ? GL_DEPTH_COMPONENT : GL_RGBA;
			if (scratch_space.allocate(image_width, image_height, color_fmt, true, true))
			{
				original_width = gPipeline.mDeferredScreen.getWidth();
				original_height = gPipeline.mDeferredScreen.getHeight();

				if (gPipeline.allocateScreenBuffer(image_width, image_height))
				{
					window_width = image_width;
					window_height = image_height;
					snapshot_width = image_width;
					snapshot_height = image_height;
					reset_deferred = true;
					mWorldViewRectRaw.set(0, image_height, image_width, 0);
					LLViewerCamera::getInstance()->setViewHeightInPixels( mWorldViewRectRaw.getHeight() );
					LLViewerCamera::getInstance()->setAspect( getWorldViewAspectRatio() );
					scratch_space.bindTarget();
				}
				else
				{
					scratch_space.release();
					gPipeline.allocateScreenBuffer(original_width, original_height);
				}
			}
		}

		if (!reset_deferred)
		{
			// if image cropping or need to enlarge the scene, compute a scale_factor
			F32 ratio = llmin( (F32)window_width / image_width , (F32)window_height / image_height) ;
			snapshot_width  = (S32)(ratio * image_width) ;
			snapshot_height = (S32)(ratio * image_height) ;
			scale_factor = llmax(1.0f, 1.0f / ratio) ;
		}
	}
	
	if (show_ui && scale_factor > 1.f)
	{
		// Note: we should never get there...
		LL_WARNS() << "over scaling UI not supported." << LL_ENDL;
	}

	S32 buffer_x_offset = llfloor(((window_width  - snapshot_width)  * scale_factor) / 2.f);
	S32 buffer_y_offset = llfloor(((window_height - snapshot_height) * scale_factor) / 2.f);

	S32 image_buffer_x = llfloor(snapshot_width  * scale_factor) ;
	S32 image_buffer_y = llfloor(snapshot_height * scale_factor) ;

	if ((image_buffer_x > max_size) || (image_buffer_y > max_size)) // boundary check to avoid memory overflow
	{
		scale_factor *= llmin((F32)max_size / image_buffer_x, (F32)max_size / image_buffer_y) ;
		image_buffer_x = llfloor(snapshot_width  * scale_factor) ;
		image_buffer_y = llfloor(snapshot_height * scale_factor) ;
	}
	if ((image_buffer_x > 0) && (image_buffer_y > 0))
	{
		raw->resize(image_buffer_x, image_buffer_y, 3);
	}
	else
	{
		gStatusBar->showBalance(true);	// <FS:CR> Hide currency balance in snapshots
		return FALSE ;
	}
	if (raw->isBufferInvalid())
	{
		gStatusBar->showBalance(true);	// <FS:CR> Hide currency balance in snapshots
		return FALSE ;
	}

	BOOL high_res = scale_factor >= 2.f; // Font scaling is slow, only do so if rez is much higher
	if (high_res && show_ui)
	{
		// Note: we should never get there...
		LL_WARNS() << "High res UI snapshot not supported. " << LL_ENDL;
		/*send_agent_pause();
		//rescale fonts
		initFonts(scale_factor);
		LLHUDObject::reshapeAll();*/
	}

	S32 output_buffer_offset_y = 0;

	F32 depth_conversion_factor_1 = (LLViewerCamera::getInstance()->getFar() + LLViewerCamera::getInstance()->getNear()) / (2.f * LLViewerCamera::getInstance()->getFar() * LLViewerCamera::getInstance()->getNear());
	F32 depth_conversion_factor_2 = (LLViewerCamera::getInstance()->getFar() - LLViewerCamera::getInstance()->getNear()) / (2.f * LLViewerCamera::getInstance()->getFar() * LLViewerCamera::getInstance()->getNear());

	gObjectList.generatePickList(*LLViewerCamera::getInstance());

	// Subimages are in fact partial rendering of the final view. This happens when the final view is bigger than the screen.
	// In most common cases, scale_factor is 1 and there's no more than 1 iteration on x and y
	for (int subimage_y = 0; subimage_y < scale_factor; ++subimage_y)
	{
		S32 subimage_y_offset = llclamp(buffer_y_offset - (subimage_y * window_height), 0, window_height);;
		// handle fractional columns
		U32 read_height = llmax(0, (window_height - subimage_y_offset) -
			llmax(0, (window_height * (subimage_y + 1)) - (buffer_y_offset + raw->getHeight())));

		S32 output_buffer_offset_x = 0;
		for (int subimage_x = 0; subimage_x < scale_factor; ++subimage_x)
		{
			gDisplaySwapBuffers = FALSE;
			gDepthDirty = TRUE;

			S32 subimage_x_offset = llclamp(buffer_x_offset - (subimage_x * window_width), 0, window_width);
			// handle fractional rows
			U32 read_width = llmax(0, (window_width - subimage_x_offset) -
									llmax(0, (window_width * (subimage_x + 1)) - (buffer_x_offset + raw->getWidth())));
			
			// Skip rendering and sampling altogether if either width or height is degenerated to 0 (common in cropping cases)
			if (read_width && read_height)
			{
				const U32 subfield = subimage_x+(subimage_y*llceil(scale_factor));
				display(do_rebuild, scale_factor, subfield, TRUE);
				
				if (!LLPipeline::sRenderDeferred)
				{
					// Required for showing the GUI in snapshots and performing bloom composite overlay
					// Call even if show_ui is FALSE
					LL_RECORD_BLOCK_TIME(FTM_RENDER_UI);
					render_ui(scale_factor, subfield);
					swap();
				}
				
				for (U32 out_y = 0; out_y < read_height ; out_y++)
				{
					S32 output_buffer_offset = ( 
												(out_y * (raw->getWidth())) // ...plus iterated y...
												+ (window_width * subimage_x) // ...plus subimage start in x...
												+ (raw->getWidth() * window_height * subimage_y) // ...plus subimage start in y...
												- output_buffer_offset_x // ...minus buffer padding x...
												- (output_buffer_offset_y * (raw->getWidth()))  // ...minus buffer padding y...
												) * raw->getComponents();
				
					// Ping the watchdog thread every 100 lines to keep us alive (arbitrary number, feel free to change)
					if (out_y % 100 == 0)
					{
						LLAppViewer::instance()->pingMainloopTimeout("LLViewerWindow::rawSnapshot");
					}
					// disable use of glReadPixels when doing nVidia nSight graphics debugging
					if (!LLRender::sNsightDebugSupport)
					{
						if (type == LLSnapshotModel::SNAPSHOT_TYPE_COLOR)
						{
							glReadPixels(
									 subimage_x_offset, out_y + subimage_y_offset,
									 read_width, 1,
									 GL_RGB, GL_UNSIGNED_BYTE,
									 raw->getData() + output_buffer_offset
									 );
						}
						// <FS:Ansariel> FIRE-15667: 24bit depth maps
						else if (type == LLSnapshotModel::SNAPSHOT_TYPE_DEPTH24)
						{
							LLPointer<LLImageRaw> depth_line_buffer = new LLImageRaw(read_width, 1, sizeof(GLfloat)); // need to store floating point values
							glReadPixels(
										 subimage_x_offset, out_y + subimage_y_offset,
										 read_width, 1,
										 GL_DEPTH_COMPONENT, GL_FLOAT,
										 depth_line_buffer->getData()// current output pixel is beginning of buffer...
										 );

							for (S32 i = 0; i < (S32)read_width; i++)
							{
								F32 depth_float = *(F32*)(depth_line_buffer->getData() + (i * sizeof(F32)));
					
								F32 linear_depth_float = 1.f / (depth_conversion_factor_1 - (depth_float * depth_conversion_factor_2));
								U32 RGB24 = F32_to_U32(linear_depth_float, LLViewerCamera::getInstance()->getNear(), LLViewerCamera::getInstance()->getFar());
								//A max value of 16777215 for RGB24 evaluates to black when it shold be white.  The clamp assures that the divisions do not somehow become >=256.
								U8 depth_byteR = (U8)(llclamp(llfloor(RGB24 / 65536.f), 0, 255));
								U8 depth_byteG = (U8)(llclamp(llfloor((RGB24 - depth_byteR * 65536) / 256.f), 0, 255));
								U8 depth_byteB = (U8)(llclamp((RGB24 - depth_byteR * 65536 - depth_byteG * 256), 0u, 255u));
								// write converted scanline out to result image
								*(raw->getData() + output_buffer_offset + (i * raw->getComponents())) = depth_byteR;
								*(raw->getData() + output_buffer_offset + (i * raw->getComponents()) + 1) = depth_byteG;
								*(raw->getData() + output_buffer_offset + (i * raw->getComponents()) + 2) = depth_byteB;
								for (S32 j = 3; j < raw->getComponents(); j++)
								{
									*(raw->getData() + output_buffer_offset + (i * raw->getComponents()) + j) = depth_byteR;
								}
							}
						}
						// </FS:Ansariel>
						else // LLSnapshotModel::SNAPSHOT_TYPE_DEPTH
						{
							// <FS> Fix buffer creation using the wrong type
							//LLPointer<LLImageRaw> depth_line_buffer = new LLImageRaw(read_width, 1, sizeof(GL_FLOAT)); // need to store floating point values
							LLPointer<LLImageRaw> depth_line_buffer = new LLImageRaw(read_width, 1, sizeof(GLfloat)); // need to store floating point values
							// </FS>
							glReadPixels(
										 subimage_x_offset, out_y + subimage_y_offset,
										 read_width, 1,
										 GL_DEPTH_COMPONENT, GL_FLOAT,
										 depth_line_buffer->getData()// current output pixel is beginning of buffer...
										 );

							for (S32 i = 0; i < (S32)read_width; i++)
							{
								F32 depth_float = *(F32*)(depth_line_buffer->getData() + (i * sizeof(F32)));
					
								F32 linear_depth_float = 1.f / (depth_conversion_factor_1 - (depth_float * depth_conversion_factor_2));
								U8 depth_byte = F32_to_U8(linear_depth_float, LLViewerCamera::getInstance()->getNear(), LLViewerCamera::getInstance()->getFar());
								// write converted scanline out to result image
								for (S32 j = 0; j < raw->getComponents(); j++)
								{
									*(raw->getData() + output_buffer_offset + (i * raw->getComponents()) + j) = depth_byte;
								}
							}
						}
					}
				}
			}
			output_buffer_offset_x += subimage_x_offset;
			stop_glerror();
		}
		output_buffer_offset_y += subimage_y_offset;
	}

	gDisplaySwapBuffers = FALSE;
	gDepthDirty = TRUE;

	// POST SNAPSHOT
	if (!gPipeline.hasRenderDebugFeatureMask(LLPipeline::RENDER_DEBUG_FEATURE_UI))
	{
		LLPipeline::toggleRenderDebugFeature(LLPipeline::RENDER_DEBUG_FEATURE_UI);
	}

	if (hide_hud)
	{
		LLPipeline::sShowHUDAttachments = TRUE;
	}

	/*if (high_res)
	{
		initFonts(1.f);
		LLHUDObject::reshapeAll();
	}*/

	// Pre-pad image to number of pixels such that the line length is a multiple of 4 bytes (for BMP encoding)
	// Note: this formula depends on the number of components being 3.  Not obvious, but it's correct.	
	image_width += (image_width * 3) % 4;

	BOOL ret = TRUE ;
	// Resize image
	if(llabs(image_width - image_buffer_x) > 4 || llabs(image_height - image_buffer_y) > 4)
	{
		ret = raw->scale( image_width, image_height );  
	}
	else if(image_width != image_buffer_x || image_height != image_buffer_y)
	{
		ret = raw->scale( image_width, image_height, FALSE );  
	}
	

	setCursor(UI_CURSOR_ARROW);

	if (do_rebuild)
	{
		// If we had to do a rebuild, that means that the lists of drawables to be rendered
		// was empty before we started.
		// Need to reset these, otherwise we call state sort on it again when render gets called the next time
		// and we stand a good chance of crashing on rebuild because the render drawable arrays have multiple copies of
		// objects on them.
		gPipeline.resetDrawOrders();
	}

	if (reset_deferred)
	{
		mWorldViewRectRaw = window_rect;
		LLViewerCamera::getInstance()->setViewHeightInPixels( mWorldViewRectRaw.getHeight() );
		LLViewerCamera::getInstance()->setAspect( getWorldViewAspectRatio() );
		scratch_space.flush();
		scratch_space.release();
		gPipeline.allocateScreenBuffer(original_width, original_height);
		
	}

	if (high_res)
	{
		send_agent_resume();
	}
	
	// <FS:CR> Hide currency balance in snapshots
	if (gStatusBar)
	{
		gStatusBar->showBalance(true);
	}
	
	return ret;
}

void LLViewerWindow::destroyWindow()
{
	if (mWindow)
	{
		LLWindowManager::destroyWindow(mWindow);
	}
	mWindow = NULL;
}


void LLViewerWindow::drawMouselookInstructions()
{
	// Draw instructions for mouselook ("Press ESC to return to World View" partially transparent at the bottom of the screen.)
	const std::string instructions = LLTrans::getString("LeaveMouselook");
	const LLFontGL* font = LLFontGL::getFont(LLFontDescriptor("SansSerif", "Large", LLFontGL::BOLD));
	
	//to be on top of Bottom bar when it is opened
	const S32 INSTRUCTIONS_PAD = 50;

	font->renderUTF8( 
		instructions, 0,
		getWorldViewRectScaled().getCenterX(),
		getWorldViewRectScaled().mBottom + INSTRUCTIONS_PAD,
		LLColor4( 1.0f, 1.0f, 1.0f, 0.5f ),
		LLFontGL::HCENTER, LLFontGL::TOP,
		LLFontGL::NORMAL,LLFontGL::DROP_SHADOW);
}

void* LLViewerWindow::getPlatformWindow() const
{
	return mWindow->getPlatformWindow();
}

void* LLViewerWindow::getMediaWindow() 	const
{
	return mWindow->getMediaWindow();
}

void LLViewerWindow::focusClient()		const
{
	return mWindow->focusClient();
}

LLRootView*	LLViewerWindow::getRootView() const
{
	return mRootView;
}

LLRect LLViewerWindow::getWorldViewRectScaled() const
{
	return mWorldViewRectScaled;
}

S32 LLViewerWindow::getWorldViewHeightScaled() const
{
	return mWorldViewRectScaled.getHeight();
}

S32 LLViewerWindow::getWorldViewWidthScaled() const
{
	return mWorldViewRectScaled.getWidth();
}


S32 LLViewerWindow::getWorldViewHeightRaw() const
{
	return mWorldViewRectRaw.getHeight(); 
}

S32 LLViewerWindow::getWorldViewWidthRaw() const
{
	return mWorldViewRectRaw.getWidth(); 
}

S32	LLViewerWindow::getWindowHeightScaled()	const 	
{ 
	return mWindowRectScaled.getHeight(); 
}

S32	LLViewerWindow::getWindowWidthScaled() const 	
{ 
	return mWindowRectScaled.getWidth(); 
}

S32	LLViewerWindow::getWindowHeightRaw()	const 	
{ 
	return mWindowRectRaw.getHeight(); 
}

S32	LLViewerWindow::getWindowWidthRaw() const 	
{ 
	return mWindowRectRaw.getWidth(); 
}

void LLViewerWindow::setup2DRender()
{
	// setup ortho camera
	gl_state_for_2d(mWindowRectRaw.getWidth(), mWindowRectRaw.getHeight());
	setup2DViewport();
}

void LLViewerWindow::setup2DViewport(S32 x_offset, S32 y_offset)
{
	gGLViewport[0] = mWindowRectRaw.mLeft + x_offset;
	gGLViewport[1] = mWindowRectRaw.mBottom + y_offset;
	gGLViewport[2] = mWindowRectRaw.getWidth();
	gGLViewport[3] = mWindowRectRaw.getHeight();
	glViewport(gGLViewport[0], gGLViewport[1], gGLViewport[2], gGLViewport[3]);
}


void LLViewerWindow::setup3DRender()
{
	// setup perspective camera
	LLViewerCamera::getInstance()->setPerspective(NOT_FOR_SELECTION, mWorldViewRectRaw.mLeft, mWorldViewRectRaw.mBottom,  mWorldViewRectRaw.getWidth(), mWorldViewRectRaw.getHeight(), FALSE, LLViewerCamera::getInstance()->getNear(), MAX_FAR_CLIP*2.f);
	setup3DViewport();
}

void LLViewerWindow::setup3DViewport(S32 x_offset, S32 y_offset)
{
	gGLViewport[0] = mWorldViewRectRaw.mLeft + x_offset;
	gGLViewport[1] = mWorldViewRectRaw.mBottom + y_offset;
	gGLViewport[2] = mWorldViewRectRaw.getWidth();
	gGLViewport[3] = mWorldViewRectRaw.getHeight();
	glViewport(gGLViewport[0], gGLViewport[1], gGLViewport[2], gGLViewport[3]);
}

void LLViewerWindow::revealIntroPanel()
{
	if (mProgressView)
	{
		mProgressView->revealIntroPanel();
	}
}

void LLViewerWindow::setShowProgress(const BOOL show,BOOL fullscreen)
{
	if(show)
	{
		if(fullscreen)
		{
			if(mProgressView)
				mProgressView->fade(TRUE);
		}
		else
		{
			if(mProgressViewMini)
				mProgressViewMini->setVisible(TRUE);
		}
	}
	else
	{
		if(mProgressView && mProgressView->getVisible())
			mProgressView->fade(FALSE);

		if(mProgressViewMini)
			mProgressViewMini->setVisible(FALSE);
	}
}

void LLViewerWindow::setStartupComplete()
{
	if (mProgressView)
	{
		mProgressView->setStartupComplete();
	}
}

BOOL LLViewerWindow::getShowProgress() const
{
	return (mProgressView && mProgressView->getVisible());
}

void LLViewerWindow::setProgressString(const std::string& string)
{
	if (mProgressView)
	{
		mProgressView->setText(string);
	}

	if (mProgressViewMini)
	{
		mProgressViewMini->setText(string);
	}
}

void LLViewerWindow::setProgressMessage(const std::string& msg)
{
	if(mProgressView)
	{
		mProgressView->setMessage(msg);
	}
}

void LLViewerWindow::setProgressPercent(const F32 percent)
{
	if (mProgressView)
	{
		mProgressView->setPercent(percent);
	}

	if (mProgressViewMini)
	{
		mProgressViewMini->setPercent(percent);
	}
}

void LLViewerWindow::setProgressCancelButtonVisible( BOOL b, const std::string& label )
{
	if (mProgressView)
	{
		mProgressView->setCancelButtonVisible( b, label );
	}

	if (mProgressViewMini)
	{
		mProgressViewMini->setCancelButtonVisible( b, label );
	}
}


LLProgressView *LLViewerWindow::getProgressView() const
{
	return mProgressView;
}

void LLViewerWindow::dumpState()
{
	LL_INFOS() << "LLViewerWindow Active " << S32(mActive) << LL_ENDL;
	LL_INFOS() << "mWindow visible " << S32(mWindow->getVisible())
		<< " minimized " << S32(mWindow->getMinimized())
		<< LL_ENDL;
}

void LLViewerWindow::stopGL(BOOL save_state)
{
	//Note: --bao
	//if not necessary, do not change the order of the function calls in this function.
	//if change something, make sure it will not break anything.
	//especially be careful to put anything behind gTextureList.destroyGL(save_state);
	if (!gGLManager.mIsDisabled)
	{
		LL_INFOS() << "Shutting down GL..." << LL_ENDL;

		// Pause texture decode threads (will get unpaused during main loop)
		LLAppViewer::getTextureCache()->pause();
		LLAppViewer::getImageDecodeThread()->pause();
		LLAppViewer::getTextureFetch()->pause();
				
		gSky.destroyGL();
		stop_glerror();		

		LLManipTranslate::destroyGL() ;
		stop_glerror();		

		gBumpImageList.destroyGL();
		stop_glerror();

		LLFontGL::destroyAllGL();
		stop_glerror();

		LLVOAvatar::destroyGL();
		stop_glerror();

		LLVOPartGroup::destroyGL();

		LLViewerDynamicTexture::destroyGL();
		stop_glerror();

		if (gPipeline.isInit())
		{
			gPipeline.destroyGL();
		}
		
		gBox.cleanupGL();
		
		if(gPostProcess)
		{
			gPostProcess->invalidate();
		}

		gTextureList.destroyGL(save_state);
		stop_glerror();
		
		gGLManager.mIsDisabled = TRUE;
		stop_glerror();

		//unload shader's
		while (LLGLSLShader::sInstances.size())
		{
			LLGLSLShader* shader = *(LLGLSLShader::sInstances.begin());
			shader->unload();
		}
		
		LL_INFOS() << "Remaining allocated texture memory: " << LLImageGL::sGlobalTextureMemory.value() << " bytes" << LL_ENDL;
	}
}

void LLViewerWindow::restoreGL(const std::string& progress_message)
{
	//Note: --bao
	//if not necessary, do not change the order of the function calls in this function.
	//if change something, make sure it will not break anything. 
	//especially, be careful to put something before gTextureList.restoreGL();
	if (gGLManager.mIsDisabled)
	{
		LL_INFOS() << "Restoring GL..." << LL_ENDL;
		gGLManager.mIsDisabled = FALSE;
		
		initGLDefaults();
		LLGLState::restoreGL();
		
		gTextureList.restoreGL();
		
		// for future support of non-square pixels, and fonts that are properly stretched
		//LLFontGL::destroyDefaultFonts();
		initFonts();
				
		gSky.restoreGL();
		gPipeline.restoreGL();
		LLDrawPoolWater::restoreGL();
		LLManipTranslate::restoreGL();
		
		gBumpImageList.restoreGL();
		LLViewerDynamicTexture::restoreGL();
		LLVOAvatar::restoreGL();
		LLVOPartGroup::restoreGL();
		
		gResizeScreenTexture = TRUE;
		gWindowResized = TRUE;

		if (isAgentAvatarValid() && gAgentAvatarp->isEditingAppearance())
		{
			LLVisualParamHint::requestHintUpdates();
		}

		if (!progress_message.empty())
		{
			gRestoreGLTimer.reset();
			gRestoreGL = TRUE;
			setShowProgress(TRUE,TRUE);
			setProgressString(progress_message);
		}
		LL_INFOS() << "...Restoring GL done" << LL_ENDL;
		if(!LLAppViewer::instance()->restoreErrorTrap())
		{
			LL_WARNS() << " Someone took over my signal/exception handler (post restoreGL)!" << LL_ENDL;
		}

	}
}

void LLViewerWindow::initFonts(F32 zoom_factor)
{
	LLFontGL::destroyAllGL();
	// Initialize with possibly different zoom factor

	LLFontManager::initClass();

	LLFontGL::initClass( gSavedSettings.getF32("FontScreenDPI"),
								mDisplayScale.mV[VX] * zoom_factor,
								mDisplayScale.mV[VY] * zoom_factor,
								gDirUtilp->getAppRODataDir(),
								gSavedSettings.getString("FSFontSettingsFile"),
								gSavedSettings.getF32("FSFontSizeAdjustment"));
	// Force font reloads, which can be very slow
	LLFontGL::loadDefaultFonts();
}

void LLViewerWindow::requestResolutionUpdate()
{
	mResDirty = true;
}

static LLTrace::BlockTimerStatHandle FTM_WINDOW_CHECK_SETTINGS("Window Settings");

void LLViewerWindow::checkSettings()
{
	LL_RECORD_BLOCK_TIME(FTM_WINDOW_CHECK_SETTINGS);
	if (mStatesDirty)
	{
		gGL.refreshState();
		LLViewerShaderMgr::instance()->setShaders();
		mStatesDirty = false;
	}
	
	// We want to update the resolution AFTER the states getting refreshed not before.
	if (mResDirty)
	{
		reshape(getWindowWidthRaw(), getWindowHeightRaw());
		mResDirty = false;
	}	
}

void LLViewerWindow::restartDisplay(BOOL show_progress_bar)
{
	LL_INFOS() << "Restaring GL" << LL_ENDL;
	stopGL();
	if (show_progress_bar)
	{
		restoreGL(LLTrans::getString("ProgressChangingResolution"));
	}
	else
	{
		restoreGL();
	}
}

BOOL LLViewerWindow::changeDisplaySettings(LLCoordScreen size, BOOL disable_vsync, BOOL show_progress_bar)
{
	//BOOL was_maximized = gSavedSettings.getBOOL("WindowMaximized");

	//gResizeScreenTexture = TRUE;


	//U32 fsaa = gSavedSettings.getU32("RenderFSAASamples");
	//U32 old_fsaa = mWindow->getFSAASamples();

	// if not maximized, use the request size
	if (!mWindow->getMaximized())
	{
		mWindow->setSize(size);
	}

	//if (fsaa == old_fsaa)
	{
		return TRUE;
	}

/*

	// Close floaters that don't handle settings change
	LLFloaterReg::hideInstance("snapshot");
	
	BOOL result_first_try = FALSE;
	BOOL result_second_try = FALSE;

	LLFocusableElement* keyboard_focus = gFocusMgr.getKeyboardFocus();
	send_agent_pause();
	LL_INFOS() << "Stopping GL during changeDisplaySettings" << LL_ENDL;
	stopGL();
	mIgnoreActivate = TRUE;
	LLCoordScreen old_size;
	LLCoordScreen old_pos;
	mWindow->getSize(&old_size);

	//mWindow->setFSAASamples(fsaa);

	result_first_try = mWindow->switchContext(false, size, disable_vsync);
	if (!result_first_try)
	{
		// try to switch back
		//mWindow->setFSAASamples(old_fsaa);
		result_second_try = mWindow->switchContext(false, old_size, disable_vsync);

		if (!result_second_try)
		{
			// we are stuck...try once again with a minimal resolution?
			send_agent_resume();
			mIgnoreActivate = FALSE;
			return FALSE;
		}
	}
	send_agent_resume();

	LL_INFOS() << "Restoring GL during resolution change" << LL_ENDL;
	if (show_progress_bar)
	{
		restoreGL(LLTrans::getString("ProgressChangingResolution"));
	}
	else
	{
		restoreGL();
	}

	if (!result_first_try)
	{
		LLSD args;
		args["RESX"] = llformat("%d",size.mX);
		args["RESY"] = llformat("%d",size.mY);
		LLNotificationsUtil::add("ResolutionSwitchFail", args);
		size = old_size; // for reshape below
	}

	BOOL success = result_first_try || result_second_try;

	if (success)
	{
		// maximize window if was maximized, else reposition
		if (was_maximized)
		{
			mWindow->maximize();
		}
		else
		{
			S32 windowX = gSavedSettings.getS32("WindowX");
			S32 windowY = gSavedSettings.getS32("WindowY");

			mWindow->setPosition(LLCoordScreen ( windowX, windowY ) );
		}
	}

	mIgnoreActivate = FALSE;
	gFocusMgr.setKeyboardFocus(keyboard_focus);
	
	return success;

	*/
}

F32	LLViewerWindow::getWorldViewAspectRatio() const
{
	F32 world_aspect = (F32)mWorldViewRectRaw.getWidth() / (F32)mWorldViewRectRaw.getHeight();
	return world_aspect;
}

void LLViewerWindow::calcDisplayScale()
{
	F32 ui_scale_factor = llclamp(gSavedSettings.getF32("UIScaleFactor") * mWindow->getSystemUISize(), MIN_UI_SCALE, MAX_UI_SCALE);
	LLVector2 display_scale;
	display_scale.setVec(llmax(1.f / mWindow->getPixelAspectRatio(), 1.f), llmax(mWindow->getPixelAspectRatio(), 1.f));
	display_scale *= ui_scale_factor;

	// limit minimum display scale
	if (display_scale.mV[VX] < MIN_DISPLAY_SCALE || display_scale.mV[VY] < MIN_DISPLAY_SCALE)
	{
		display_scale *= MIN_DISPLAY_SCALE / llmin(display_scale.mV[VX], display_scale.mV[VY]);
	}
	
	if (display_scale != mDisplayScale)
	{
		LL_INFOS() << "Setting display scale to " << display_scale << " for ui scale: " << ui_scale_factor << LL_ENDL;

		mDisplayScale = display_scale;
		// Init default fonts
		initFonts();
	}
}

//static
LLRect 	LLViewerWindow::calcScaledRect(const LLRect & rect, const LLVector2& display_scale)
{
	LLRect res = rect;
	res.mLeft = ll_round((F32)res.mLeft / display_scale.mV[VX]);
	res.mRight = ll_round((F32)res.mRight / display_scale.mV[VX]);
	res.mBottom = ll_round((F32)res.mBottom / display_scale.mV[VY]);
	res.mTop = ll_round((F32)res.mTop / display_scale.mV[VY]);

	return res;
}

S32 LLViewerWindow::getChatConsoleBottomPad()
{
	S32 offset = 0;

	if(gToolBarView)
	{
		// <FS:KC> Tie console to legacy snap edge when possible
		static LLUICachedControl<bool> legacy_snap ("FSLegacyEdgeSnap", false);
		if (legacy_snap)
		{
			LLRect snap_rect = gFloaterView->getSnapRect();
			offset = snap_rect.mBottom;
		}// </FS:KC> Tie console to legacy snap edge when possible
		else
		{
			// FS:Ansariel This gets called every frame, so don't call getChild/findChild every time!
			offset += gToolBarView->getBottomToolbar()->getRect().getHeight();
			LLView* chat_stack = gToolBarView->getBottomChatStack();
			if (chat_stack)
			{
				offset = chat_stack->getRect().getHeight();
			}
		}
	}
	// </FS:Ansariel>

	return offset;
}

LLRect LLViewerWindow::getChatConsoleRect()
{
	LLRect full_window(0, getWindowHeightScaled(), getWindowWidthScaled(), 0);
	LLRect console_rect = full_window;

	const S32 CONSOLE_PADDING_TOP = 24;
	const S32 CONSOLE_PADDING_LEFT = 24;
	const S32 CONSOLE_PADDING_RIGHT = 10;

	console_rect.mTop    -= CONSOLE_PADDING_TOP;
	console_rect.mBottom += getChatConsoleBottomPad();

	console_rect.mLeft   += CONSOLE_PADDING_LEFT; 

	// <FS:Ansariel> This also works without relog!
	static LLCachedControl<bool> chatFullWidth(gSavedSettings, "ChatFullWidth");
	if (chatFullWidth)
	// </FS:Ansariel>
	{
		console_rect.mRight -= CONSOLE_PADDING_RIGHT;
	}
	else
	{
		// Make console rect somewhat narrow so having inventory open is
		// less of a problem.

		//AO, Have console reuse/respect the desired nearby popup width set in NearbyToastWidth
		//console_rect.mRight  = console_rect.mLeft + 2 * getWindowWidthScaled() / 3;
		static LLCachedControl<S32> nearbyToastWidth(gSavedSettings, "NearbyToastWidth");
		F32 percentage = nearbyToastWidth / 100.0;
		console_rect.mRight = S32((console_rect.mRight - CONSOLE_PADDING_RIGHT ) * percentage);
		//</AO>
	}

	// <FS:Ansariel> Push the chat console out of the way of the vertical toolbars
	if (gToolBarView)
	{
		// <FS:KC> Tie console to legacy snap edge when possible
		static LLUICachedControl<bool> legacy_snap ("FSLegacyEdgeSnap", false);
		if (legacy_snap)
		{
			LLRect snap_rect = gFloaterView->getSnapRect();
			if (console_rect.mRight > snap_rect.mRight)
			{
				console_rect.mRight = snap_rect.mRight;
			}

			if (console_rect.mLeft < snap_rect.mLeft)
			{
				console_rect.mLeft = snap_rect.mLeft;
			}
		}// </FS:KC> Tie console to legacy snap edge when possible
		else
		{
			LLToolBar* toolbar_left = gToolBarView->getToolbar(LLToolBarEnums::TOOLBAR_LEFT);
			if (toolbar_left && toolbar_left->hasButtons())
			{
				console_rect.mLeft += toolbar_left->getRect().getWidth();
			}

			LLToolBar* toolbar_right = gToolBarView->getToolbar(LLToolBarEnums::TOOLBAR_RIGHT);
			LLRect toolbar_right_screen_rect;
			toolbar_right->localRectToScreen(toolbar_right->getRect(), &toolbar_right_screen_rect);
			if (toolbar_right && toolbar_right->hasButtons() && console_rect.mRight >= toolbar_right_screen_rect.mLeft)
			{
				console_rect.mRight -= toolbar_right->getRect().getWidth();
			}
		}
	}
	// </FS:Ansariel>

	return console_rect;
}
//----------------------------------------------------------------------------


void LLViewerWindow::setUIVisibility(bool visible)
{
	mUIVisible = visible;

	if (!visible)
	{
		gAgentCamera.changeCameraToThirdPerson(FALSE);
		gFloaterView->hideAllFloaters();
	}
	else
	{
		gFloaterView->showHiddenFloaters();
	}

	if (gToolBarView)
	{
		gToolBarView->setToolBarsVisible(visible);
	}

	// <FS:Ansariel> Notification not showing if hiding the UI
	FSNearbyChat::instance().showDefaultChatBar(visible && !gSavedSettings.getBOOL("AutohideChatBar"));
	gSavedSettings.setBOOL("FSInternalShowNavbarNavigationPanel", visible && gSavedSettings.getBOOL("ShowNavbarNavigationPanel"));
	gSavedSettings.setBOOL("FSInternalShowNavbarFavoritesPanel", visible && gSavedSettings.getBOOL("ShowNavbarFavoritesPanel"));
	mRootView->getChildView("chiclet_container")->setVisible(visible);
	// </FS:Ansariel>

	// <FS:Zi> Is done inside XUI now, using visibility_control
	//LLNavigationBar::getInstance()->setVisible(visible ? gSavedSettings.getBOOL("ShowNavbarNavigationPanel") : FALSE);
	LLPanelTopInfoBar::getInstance()->setVisible(visible? gSavedSettings.getBOOL("ShowMiniLocationPanel") : FALSE);
	mRootView->getChildView("status_bar_container")->setVisible(visible);
}

bool LLViewerWindow::getUIVisibility()
{
	return mUIVisible;
}

////////////////////////////////////////////////////////////////////////////
//
// LLPickInfo
//
LLPickInfo::LLPickInfo()
	: mKeyMask(MASK_NONE),
	  mPickCallback(NULL),
	  mPickType(PICK_INVALID),
	  mWantSurfaceInfo(FALSE),
	  mObjectFace(-1),
	  mUVCoords(-1.f, -1.f),
	  mSTCoords(-1.f, -1.f),
	  mXYCoords(-1, -1),
	  mIntersection(),
	  mNormal(),
	  mTangent(),
	  mBinormal(),
	  mHUDIcon(NULL),
	  mPickTransparent(FALSE),
	  mPickRigged(FALSE),
	  mPickParticle(FALSE)
{
}

LLPickInfo::LLPickInfo(const LLCoordGL& mouse_pos, 
		       MASK keyboard_mask, 
		       BOOL pick_transparent,
			   BOOL pick_rigged,
			   BOOL pick_particle,
		       BOOL pick_uv_coords,
			   BOOL pick_unselectable,
		       void (*pick_callback)(const LLPickInfo& pick_info))
	: mMousePt(mouse_pos),
	  mKeyMask(keyboard_mask),
	  mPickCallback(pick_callback),
	  mPickType(PICK_INVALID),
	  mWantSurfaceInfo(pick_uv_coords),
	  mObjectFace(-1),
	  mUVCoords(-1.f, -1.f),
	  mSTCoords(-1.f, -1.f),
	  mXYCoords(-1, -1),
	  mNormal(),
	  mTangent(),
	  mBinormal(),
	  mHUDIcon(NULL),
	  mPickTransparent(pick_transparent),
	  mPickRigged(pick_rigged),
	  mPickParticle(pick_particle),
	  mPickUnselectable(pick_unselectable)
{
}

void LLPickInfo::fetchResults()
{

	S32 face_hit = -1;
	LLVector4a intersection, normal;
	LLVector4a tangent;

	LLVector2 uv;

	LLHUDIcon* hit_icon = gViewerWindow->cursorIntersectIcon(mMousePt.mX, mMousePt.mY, 512.f, &intersection);
	
	LLVector4a origin;
	origin.load3(LLViewerCamera::getInstance()->getOrigin().mV);
	F32 icon_dist = 0.f;
	LLVector4a start;
	LLVector4a end;
	LLVector4a particle_end;

	if (hit_icon)
	{
		LLVector4a delta;
		delta.setSub(intersection, origin);
		icon_dist = delta.getLength3().getF32();
	}
	LLViewerObject* hit_object = gViewerWindow->cursorIntersect(mMousePt.mX, mMousePt.mY, 512.f,
									NULL, -1, mPickTransparent, mPickRigged, &face_hit,
									&intersection, &uv, &normal, &tangent, &start, &end);
	
	mPickPt = mMousePt;

// [RLVa:KB] - Checked: RLVa-2.2 (@setoverlay)
	if ( (gRlvHandler.isEnabled()) && (hit_object) && (!hit_object->isHUDAttachment()) )
	{
		if (gRlvHandler.hitTestOverlay(mMousePt))
		{
			hit_object = nullptr;
		}
	}
// [/RLVa:KB]

	U32 te_offset = face_hit > -1 ? face_hit : 0;

	if (mPickParticle)
	{ //get the end point of line segement to use for particle raycast
		if (hit_object)
		{
			particle_end = intersection;
		}
		else
		{
			particle_end = end;
		}
	}

	LLViewerObject* objectp = hit_object;


	LLVector4a delta;
	delta.setSub(origin, intersection);

	if (hit_icon && 
		(!objectp || 
		icon_dist < delta.getLength3().getF32()))
	{
		// was this name referring to a hud icon?
		mHUDIcon = hit_icon;
		mPickType = PICK_ICON;
		mPosGlobal = mHUDIcon->getPositionGlobal();

	}
	else if (objectp)
	{
		if( objectp->getPCode() == LLViewerObject::LL_VO_SURFACE_PATCH )
		{
			// Hit land
			mPickType = PICK_LAND;
			mObjectID.setNull(); // land has no id

			// put global position into land_pos
			LLVector3d land_pos;
			if (!gViewerWindow->mousePointOnLandGlobal(mPickPt.mX, mPickPt.mY, &land_pos, mPickUnselectable))
			{
				// The selected point is beyond the draw distance or is otherwise 
				// not selectable. Return before calling mPickCallback().
				return;
			}

			// Fudge the land focus a little bit above ground.
			mPosGlobal = land_pos + LLVector3d::z_axis * 0.1f;
		}
		else
		{
			if(isFlora(objectp))
			{
				mPickType = PICK_FLORA;
			}
			else
			{
				mPickType = PICK_OBJECT;
			}

			LLVector3 v_intersection(intersection.getF32ptr());

			mObjectOffset = gAgentCamera.calcFocusOffset(objectp, v_intersection, mPickPt.mX, mPickPt.mY);
			mObjectID = objectp->mID;
			mObjectFace = (te_offset == NO_FACE) ? -1 : (S32)te_offset;

			

			mPosGlobal = gAgent.getPosGlobalFromAgent(v_intersection);
			
			if (mWantSurfaceInfo)
			{
				getSurfaceInfo();
			}
		}
	}
	
	if (mPickParticle)
	{ //search for closest particle to click origin out to intersection point
		S32 part_face = -1;

		LLVOPartGroup* group = gPipeline.lineSegmentIntersectParticle(start, particle_end, NULL, &part_face);
		if (group)
		{
			mParticleOwnerID = group->getPartOwner(part_face);
			mParticleSourceID = group->getPartSource(part_face);
		}
	}

	if (mPickCallback)
	{
		mPickCallback(*this);
	}
}

LLPointer<LLViewerObject> LLPickInfo::getObject() const
{
	return gObjectList.findObject( mObjectID );
}

void LLPickInfo::updateXYCoords()
{
	if (mObjectFace > -1)
	{
		const LLTextureEntry &tep = getObject()->getTEref(mObjectFace);
		LLPointer<LLViewerTexture> imagep = LLViewerTextureManager::getFetchedTexture(tep.getID());
		if(mUVCoords.mV[VX] >= 0.f && mUVCoords.mV[VY] >= 0.f && imagep.notNull())
		{
			mXYCoords.mX = ll_round(mUVCoords.mV[VX] * (F32)imagep->getWidth());
			mXYCoords.mY = ll_round((1.f - mUVCoords.mV[VY]) * (F32)imagep->getHeight());
		}
	}
}

void LLPickInfo::getSurfaceInfo()
{
	// set values to uninitialized - this is what we return if no intersection is found
	mObjectFace   = -1;
	mUVCoords     = LLVector2(-1, -1);
	mSTCoords     = LLVector2(-1, -1);
	mXYCoords	  = LLCoordScreen(-1, -1);
	mIntersection = LLVector3(0,0,0);
	mNormal       = LLVector3(0,0,0);
	mBinormal     = LLVector3(0,0,0);
	mTangent	  = LLVector4(0,0,0,0);
	
	LLVector4a tangent;
	LLVector4a intersection;
	LLVector4a normal;

	tangent.clear();
	normal.clear();
	intersection.clear();
	
	LLViewerObject* objectp = getObject();

	if (objectp)
	{
		if (gViewerWindow->cursorIntersect(ll_round((F32)mMousePt.mX), ll_round((F32)mMousePt.mY), 1024.f,
										   objectp, -1, mPickTransparent, mPickRigged,
										   &mObjectFace,
										   &intersection,
										   &mSTCoords,
										   &normal,
										   &tangent))
		{
			// if we succeeded with the intersect above, compute the texture coordinates:

			if (objectp->mDrawable.notNull() && mObjectFace > -1)
			{
				LLFace* facep = objectp->mDrawable->getFace(mObjectFace);
				if (facep)
				{
					mUVCoords = facep->surfaceToTexture(mSTCoords, intersection, normal);
			}
			}

			mIntersection.set(intersection.getF32ptr());
			mNormal.set(normal.getF32ptr());
			mTangent.set(tangent.getF32ptr());

			//extrapoloate binormal from normal and tangent
			
			LLVector4a binormal;
			binormal.setCross3(normal, tangent);
			binormal.mul(tangent.getF32ptr()[3]);

			mBinormal.set(binormal.getF32ptr());

			mBinormal.normalize();
			mNormal.normalize();
			mTangent.normalize();

			// and XY coords:
			updateXYCoords();
			
		}
	}
}

//static 
bool LLPickInfo::isFlora(LLViewerObject* object)
{
	if (!object) return false;

	LLPCode pcode = object->getPCode();

	if( (LL_PCODE_LEGACY_GRASS == pcode) 
		|| (LL_PCODE_LEGACY_TREE == pcode) 
		|| (LL_PCODE_TREE_NEW == pcode))
	{
		return true;
	}
	return false;
}
