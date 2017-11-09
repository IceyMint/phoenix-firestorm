/**
* @file llsettingsdaycycle.cpp
* @author optional
* @brief A base class for asset based settings groups.
*
* $LicenseInfo:2011&license=viewerlgpl$
* Second Life Viewer Source Code
* Copyright (C) 2017, Linden Research, Inc.
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
#include "llviewercontrol.h"
#include "llsettingsdaycycle.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include "lltrace.h"
#include "llfasttimer.h"
#include "v3colorutil.h"

#include "llglslshader.h"
#include "llviewershadermgr.h"

#include "llenvironment.h"

#include "llagent.h"
#include "pipeline.h"

#include "llsettingssky.h"
#include "llsettingswater.h"

#include "llenvironment.h"

#include "llworld.h"

//=========================================================================
namespace
{
    LLTrace::BlockTimerStatHandle FTM_BLEND_WATERVALUES("Blending Water Environment");
    LLTrace::BlockTimerStatHandle FTM_UPDATE_WATERVALUES("Update Water Environment");

    inline F32 get_wrapping_distance(F32 begin, F32 end)
    {
        if (begin < end)
        {
            return end - begin;
        }
        else if (begin > end)
        {
            return 1.0 - (begin - end);
        }

        return 0;
    }

    LLSettingsDayCycle::CycleTrack_t::iterator get_wrapping_atafter(LLSettingsDayCycle::CycleTrack_t &collection, F32 key)
    {
        if (collection.empty())
            return collection.end();

        LLSettingsDayCycle::CycleTrack_t::iterator it = collection.upper_bound(key);

        if (it == collection.end())
        {   // wrap around
            it = collection.begin();
        }

        return it;
    }

    LLSettingsDayCycle::CycleTrack_t::iterator get_wrapping_atbefore(LLSettingsDayCycle::CycleTrack_t &collection, F32 key)
    {
        if (collection.empty())
            return collection.end();

        LLSettingsDayCycle::CycleTrack_t::iterator it = collection.lower_bound(key);

        if (it == collection.end())
        {   // all keyframes are lower, take the last one.
            --it; // we know the range is not empty
        }
        else if ((*it).first > key)
        {   // the keyframe we are interested in is smaller than the found.
            if (it == collection.begin())
                it = collection.end();
            --it;
        }

        return it;
    }


}

//=========================================================================
const std::string LLSettingsDayCycle::SETTING_DAYLENGTH("day_length");
const std::string LLSettingsDayCycle::SETTING_KEYID("key_id");
const std::string LLSettingsDayCycle::SETTING_KEYNAME("key_name");
const std::string LLSettingsDayCycle::SETTING_KEYKFRAME("key_keyframe");
const std::string LLSettingsDayCycle::SETTING_NAME("name");
const std::string LLSettingsDayCycle::SETTING_TRACKS("tracks");

//const S64 LLSettingsDayCycle::MINIMUM_DAYLENGTH(  300); // 5 mins

const S64 LLSettingsDayCycle::MINIMUM_DAYLENGTH( 14400); // 4 hours
const S64 LLSettingsDayCycle::MAXIMUM_DAYLENGTH(604800); // 7 days

const S32 LLSettingsDayCycle::TRACK_WATER(0);   // water track is 0
const S32 LLSettingsDayCycle::TRACK_MAX(5);     // 5 tracks, 4 skys, 1 water

//=========================================================================
LLSettingsDayCycle::LLSettingsDayCycle(const LLSD &data) :
    LLSettingsBase(data),
    mHasParsed(false)
{
    mDayTracks.resize(TRACK_MAX);
}

LLSettingsDayCycle::LLSettingsDayCycle() :
    LLSettingsBase(),
    mHasParsed(false)
{
    mDayTracks.resize(TRACK_MAX);
}

//=========================================================================
LLSD LLSettingsDayCycle::defaults()
{
    LLSD dfltsetting;

    dfltsetting[SETTING_NAME] = "_default_";
    dfltsetting[SETTING_DAYLENGTH] = static_cast<S32>(MINIMUM_DAYLENGTH);
    dfltsetting[SETTING_TRACKS] = LLSDArray(
        LLSDArray(LLSDMap(SETTING_KEYKFRAME, LLSD::Real(0.0f))(SETTING_KEYNAME, "_default_"))
        (LLSDMap(SETTING_KEYKFRAME, LLSD::Real(0.0f))(SETTING_KEYNAME, "_default_")));

    return dfltsetting;
}

LLSettingsDayCycle::ptr_t LLSettingsDayCycle::buildFromLegacyPreset(const std::string &name, const LLSD &oldsettings)
{
    LLSD newsettings(defaults());

    newsettings[SETTING_NAME] = name;
    newsettings[SETTING_DAYLENGTH] = static_cast<S32>(MINIMUM_DAYLENGTH);

    LLSD watertrack = LLSDArray( 
        LLSDMap ( SETTING_KEYKFRAME, LLSD::Real(0.0f) )
                ( SETTING_KEYNAME, "Default" ));

    LLSD skytrack = LLSD::emptyArray();

    for (LLSD::array_const_iterator it = oldsettings.beginArray(); it != oldsettings.endArray(); ++it)
    {
        LLSD entry = LLSDMap(SETTING_KEYKFRAME, (*it)[0].asReal())
            (SETTING_KEYNAME, (*it)[1].asString());
        skytrack.append(entry);
    }

    newsettings[SETTING_TRACKS] = LLSDArray(watertrack)(skytrack);

    LLSettingsDayCycle::ptr_t dayp = boost::make_shared<LLSettingsDayCycle>(newsettings);
    dayp->parseFromLLSD(dayp->mSettings);

    return dayp;
}

LLSettingsDayCycle::ptr_t LLSettingsDayCycle::buildFromLegacyMessage(const LLUUID &regionId, LLSD daycycle, LLSD skydefs, LLSD waterdef)
{
    LLSettingsWater::ptr_t water = LLSettingsWater::buildFromLegacyPreset("Region", waterdef);
    LLEnvironment::namedSettingMap_t skys;

    for (LLSD::map_iterator itm = skydefs.beginMap(); itm != skydefs.endMap(); ++itm)
    {
        std::string name = (*itm).first;
        LLSettingsSky::ptr_t sky = LLSettingsSky::buildFromLegacyPreset(name, (*itm).second);

        skys[name] = sky;
        LL_WARNS("WindlightCaps") << "created region sky '" << name << "'" << LL_ENDL;
    }

    LLSettingsDayCycle::ptr_t day = buildFromLegacyPreset("Region (legacy)", daycycle);

    day->setWaterAtKeyframe(water, 0.0f);

    for (LLSD::array_iterator ita = daycycle.beginArray(); ita != daycycle.endArray(); ++ita)
    {
        F32 frame = (*ita)[0].asReal();
        std::string name = (*ita)[1].asString();

        LLEnvironment::namedSettingMap_t::iterator it = skys.find(name);

        if (it == skys.end())
            continue;
        day->setSkyAtKeyframe(boost::static_pointer_cast<LLSettingsSky>((*it).second), frame, 1);

        LL_WARNS("WindlightCaps") << "Added '" << name << "' to region day cycle at " << frame << LL_ENDL;
    }

    day->mHasParsed = true;

    return day;
}

LLSettingsDayCycle::ptr_t LLSettingsDayCycle::buildDefaultDayCycle()
{
    LLSD settings = LLSettingsDayCycle::defaults();

    LLSettingsDayCycle::ptr_t dayp = boost::make_shared<LLSettingsDayCycle>(settings);
    dayp->parseFromLLSD(dayp->mSettings);

    return dayp;
}

void LLSettingsDayCycle::parseFromLLSD(LLSD &data)
{
    LLEnvironment &environment(LLEnvironment::instance());
    LLSD tracks = data[SETTING_TRACKS];

    for (S32 i = 0; (i < tracks.size()) && (i < TRACK_MAX); ++i)
    {
        mDayTracks[i].clear();
        LLSD curtrack = tracks[i];
        for (LLSD::array_const_iterator it = curtrack.beginArray(); it != curtrack.endArray(); ++it)
        {
            F32 keyframe = (*it)[SETTING_KEYKFRAME].asReal();
            keyframe = llclamp(keyframe, 0.0f, 1.0f);
            LLSettingsBase::ptr_t setting;

            if ((*it).has(SETTING_KEYNAME))
            {
                if (i == TRACK_WATER)
                    setting = environment.findWaterByName((*it)[SETTING_KEYNAME]);
                else
                    setting = environment.findSkyByName((*it)[SETTING_KEYNAME]);
            }
            else if ((*it).has(SETTING_KEYID))
            {

            }

            if (setting)
                mDayTracks[i][keyframe] = setting;
        }
    }
    mHasParsed = true;
}


LLSettingsDayCycle::ptr_t LLSettingsDayCycle::buildClone()
{
    LLSD settings = cloneSettings();

    LLSettingsDayCycle::ptr_t dayp = boost::make_shared<LLSettingsDayCycle>(settings);

    return dayp;
}

void LLSettingsDayCycle::blend(const LLSettingsBase::ptr_t &other, F32 mix)
{
    LL_ERRS("DAYCYCLE") << "Day cycles are not blendable!" << LL_ENDL;
}

//=========================================================================
F32 LLSettingsDayCycle::secondsToKeyframe(S64Seconds seconds)
{
    S64Seconds daylength = getDayLength();

    return llclamp(static_cast<F32>(seconds.value() % daylength.value()) / static_cast<F32>(daylength.value()), 0.0f, 1.0f);
}

F64Seconds LLSettingsDayCycle::keyframeToSeconds(F32 keyframe)
{
    S64Seconds daylength = getDayLength();

    return F64Seconds(static_cast<S32>(keyframe * static_cast<F32>(daylength.value())));
}

//=========================================================================
void LLSettingsDayCycle::startDayCycle()
{
    F64Seconds now(LLDate::now().secondsSinceEpoch());

    if (!mHasParsed)
        parseFromLLSD(mSettings);

    // water
    if (mDayTracks[0].empty())
    {
        mBlendedWater.reset();
        mWaterBlender.reset();
    }
    else if (mDayTracks[0].size() == 1)
    {
        mBlendedWater = boost::static_pointer_cast<LLSettingsWater>((*(mDayTracks[0].begin())).second);
        mWaterBlender.reset();
    }
    else
    {
        TrackBound_t bounds = getBoundingEntries(mDayTracks[0], now);

        F64Seconds timespan = F64Seconds( getDayLength() * get_wrapping_distance((*bounds.first).first, (*bounds.second).first));

        mBlendedWater = LLSettingsWater::buildDefaultWater();
        mWaterBlender = boost::make_shared<LLSettingsBlender>(mBlendedWater,
            (*bounds.first).second, (*bounds.second).second, timespan);
        mWaterBlender->setOnFinished(boost::bind(&LLSettingsDayCycle::onWaterTransitionDone, this, _1));
    }

    // sky
    if (mDayTracks[1].empty())
    {
        mBlendedSky.reset();
        mSkyBlender.reset();
    }
    else if (mDayTracks[1].size() == 1)
    {
        mBlendedSky = boost::static_pointer_cast<LLSettingsSky>( (*(mDayTracks[1].begin())).second);
        mSkyBlender.reset();
    }
    else
    {
        TrackBound_t bounds = getBoundingEntries(mDayTracks[1], now);
        F64Seconds timespan = F64Seconds(getDayLength() * get_wrapping_distance((*bounds.first).first, (*bounds.second).first));

        mBlendedSky = LLSettingsSky::buildDefaultSky();
        mSkyBlender = boost::make_shared<LLSettingsBlender>(mBlendedSky,
            (*bounds.first).second, (*bounds.second).second, timespan);
        mSkyBlender->setOnFinished(boost::bind(&LLSettingsDayCycle::onSkyTransitionDone, this, 1, _1));
    }
}


void LLSettingsDayCycle::updateSettings()
{
    static LLFrameTimer timer;


    F64Seconds delta(timer.getElapsedTimeAndResetF32());

    if (mSkyBlender)
        mSkyBlender->update(delta);
    if (mWaterBlender)
        mWaterBlender->update(delta);

#if 0
    //F64Seconds time_now(LLWorld::instance().getSpaceTimeUSec());
    F64Seconds time_now(LLDate::now().secondsSinceEpoch());

    // base class clears dirty flag so as to not trigger recursive update
    LLSettingsBase::updateSettings();

    if (!mBlendedWater)
    {
        mBlendedWater = LLEnvironment::instance().getCurrentWater()->buildClone();
        LLEnvironment::instance().selectWater(mBlendedWater);
    }

    if (!mBlendedSky)
    {
        mBlendedSky = LLEnvironment::instance().getCurrentSky()->buildClone();
        LLEnvironment::instance().selectSky(mBlendedSky);
    }


    if ((time_now < mLastUpdateTime) || ((time_now - mLastUpdateTime) > static_cast<F64Seconds>(0.1)))
    {
        F64Seconds daylength = static_cast<F64Seconds>(getDayLength());
        F32 frame = fmod(time_now.value(), daylength.value()) / daylength.value();

        CycleList_t::iterator itTrack = mDayTracks.begin();
        TrackBound_t bounds = getBoundingEntries(*itTrack, frame);

        mBlendedWater->replaceSettings((*bounds.first).second->getSettings());
        if (bounds.first != bounds.second)
        {
            F32 blendf = get_wrapping_distance((*bounds.first).first, frame) / get_wrapping_distance((*bounds.first).first, (*bounds.second).first);

            mBlendedWater->blend((*bounds.second).second, blendf);
        }

        ++itTrack;
        bounds = getBoundingEntries(*itTrack, frame);

        //_WARNS("RIDER") << "Sky blending: frame=" << frame << " start=" << F64Seconds((*bounds.first).first) << " end=" << F64Seconds((*bounds.second).first) << LL_ENDL;

        mBlendedSky->replaceSettings((*bounds.first).second->getSettings());
        if (bounds.first != bounds.second)
        {
            F32 blendf = get_wrapping_distance((*bounds.first).first, frame) / get_wrapping_distance((*bounds.first).first, (*bounds.second).first);
            //_WARNS("RIDER") << "Distance=" << get_wrapping_distance((*bounds.first).first, frame) << "/" << get_wrapping_distance((*bounds.first).first, (*bounds.second).first) << " Blend factor=" << blendf << LL_ENDL;

            mBlendedSky->blend((*bounds.second).second, blendf);
        }

        mLastUpdateTime = time_now;
    }

    // Always mark the day cycle as dirty.So that the blend check can be handled.
    setDirtyFlag(true);
#endif
}

//=========================================================================
void LLSettingsDayCycle::setDayLength(S64Seconds seconds)
{
    S32 val = llclamp(seconds.value(), MINIMUM_DAYLENGTH, MAXIMUM_DAYLENGTH);

    setValue(SETTING_DAYLENGTH, val);
}

LLSettingsDayCycle::KeyframeList_t LLSettingsDayCycle::getTrackKeyframes(S32 trackno)
{
    if ((trackno < 1) || (trackno >= TRACK_MAX))
    {
        LL_WARNS("DAYCYCLE") << "Attempt get track (#" << trackno << ") out of range!" << LL_ENDL;
        return KeyframeList_t();
    }

    KeyframeList_t keyframes;
    CycleTrack_t &track = mDayTracks[trackno];

    keyframes.reserve(track.size());

    for (CycleTrack_t::iterator it = track.begin(); it != track.end(); ++it)
    {
        keyframes.push_back((*it).first);
    }

    return keyframes;
}

LLSettingsDayCycle::TimeList_t LLSettingsDayCycle::getTrackTimes(S32 trackno)
{
    KeyframeList_t keyframes = getTrackKeyframes(trackno);

    if (keyframes.empty())
        return TimeList_t();

    TimeList_t times;

    times.reserve(keyframes.size());
    for (KeyframeList_t::iterator it = keyframes.begin(); it != keyframes.end(); ++it)
    {
        times.push_back(keyframeToSeconds(*it));
    }

    return times;
}

void LLSettingsDayCycle::setWaterAtTime(const LLSettingsWaterPtr_t &water, S64Seconds seconds)
{
    F32 keyframe = secondsToKeyframe(seconds);
    setWaterAtKeyframe(water, keyframe);
}

void LLSettingsDayCycle::setWaterAtKeyframe(const LLSettingsWaterPtr_t &water, F32 keyframe)
{
    mDayTracks[TRACK_WATER][llclamp(keyframe, 0.0f, 1.0f)] = water;
    setDirtyFlag(true);
}


void LLSettingsDayCycle::setSkyAtTime(const LLSettingsSkyPtr_t &sky, S64Seconds seconds, S32 track)
{
    F32 keyframe = secondsToKeyframe(seconds);
    setSkyAtKeyframe(sky, keyframe, track);
}

void LLSettingsDayCycle::setSkyAtKeyframe(const LLSettingsSkyPtr_t &sky, F32 keyframe, S32 track)
{
    if ((track < 1) || (track >= TRACK_MAX))
    {
        LL_WARNS("DAYCYCLE") << "Attempt to set sky track (#" << track << ") out of range!" << LL_ENDL;
        return;
    }

    mDayTracks[track][llclamp(keyframe, 0.0f, 1.0f)] = sky;
    setDirtyFlag(true);
}

LLSettingsDayCycle::TrackBound_t LLSettingsDayCycle::getBoundingEntries(LLSettingsDayCycle::CycleTrack_t &track, F32 keyframe)
{
    return TrackBound_t(get_wrapping_atbefore(track, keyframe), get_wrapping_atafter(track, keyframe));
}

LLSettingsDayCycle::TrackBound_t LLSettingsDayCycle::getBoundingEntries(LLSettingsDayCycle::CycleTrack_t &track, F64Seconds time)
{
    F32 frame = secondsToKeyframe(time);

    return getBoundingEntries(track, frame);
}

//=========================================================================
void LLSettingsDayCycle::onSkyTransitionDone(S32 track, const LLSettingsBlender::ptr_t &blender)
{
    F64Seconds now(LLDate::now().secondsSinceEpoch());
    TrackBound_t bounds = getBoundingEntries(mDayTracks[track], now);

    F32 distance = get_wrapping_distance((*bounds.first).first, (*bounds.second).first);
    F64Seconds timespan = F64Seconds(distance * getDayLength());

    LL_WARNS("RIDER") << "New sky blender. now=" << now <<
        " start=" << (*bounds.first).first << " end=" << (*bounds.second).first <<
        " span=" << timespan << LL_ENDL;

    mSkyBlender = boost::make_shared<LLSettingsBlender>(mBlendedSky,
        (*bounds.first).second, (*bounds.second).second, timespan);
    mSkyBlender->setOnFinished(boost::bind(&LLSettingsDayCycle::onSkyTransitionDone, this, track, _1));
}

void LLSettingsDayCycle::onWaterTransitionDone(const LLSettingsBlender::ptr_t &blender)
{
    F64Seconds now(LLDate::now().secondsSinceEpoch());
    TrackBound_t bounds = getBoundingEntries(mDayTracks[0], now);

    F32 distance = get_wrapping_distance((*bounds.first).first, (*bounds.second).first);
    F64Seconds timespan = F64Seconds(distance * getDayLength());

    mWaterBlender = boost::make_shared<LLSettingsBlender>(mBlendedWater,
        (*bounds.first).second, (*bounds.second).second, timespan);
    mWaterBlender->setOnFinished(boost::bind(&LLSettingsDayCycle::onWaterTransitionDone, this, _1));
}
