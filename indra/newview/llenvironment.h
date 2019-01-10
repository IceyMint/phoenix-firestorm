/**
 * @file llenvmanager.h
 * @brief Declaration of classes managing WindLight and water settings.
 *
 * $LicenseInfo:firstyear=2009&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2011, Linden Research, Inc.
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

#ifndef LL_ENVIRONMENT_H
#define LL_ENVIRONMENT_H

#include "llsingleton.h"
#include "llmemory.h"
#include "llsd.h"

#include "llsettingsbase.h"
#include "llsettingssky.h"
#include "llsettingswater.h"
#include "llsettingsdaycycle.h"

#include "llatmosphere.h"

#include <boost/signals2.hpp>

//-------------------------------------------------------------------------
class LLViewerCamera;
class LLGLSLShader;
class LLParcel;

//-------------------------------------------------------------------------
class LLEnvironment : public LLSingleton<LLEnvironment>
{
    LLSINGLETON(LLEnvironment);
    LOG_CLASS(LLEnvironment);

public:
    static const F32Seconds     TRANSITION_INSTANT;
    static const F32Seconds     TRANSITION_FAST;
    static const F32Seconds     TRANSITION_DEFAULT;
    static const F32Seconds     TRANSITION_SLOW;
    static const F32Seconds     TRANSITION_ALTITUDE;

    static const LLUUID         KNOWN_SKY_SUNRISE;
    static const LLUUID         KNOWN_SKY_MIDDAY;
    static const LLUUID         KNOWN_SKY_SUNSET;
    static const LLUUID         KNOWN_SKY_MIDNIGHT;

    static const S32            NO_TRACK;
    static const S32            NO_VERSION;
    static const S32            VERSION_CLEANUP;

    struct EnvironmentInfo
    {
        EnvironmentInfo();

        typedef std::shared_ptr<EnvironmentInfo>    ptr_t;
        typedef std::array<std::string, 5>          namelist_t;

        S32                     mParcelId;
        LLUUID                  mRegionId;
        S64Seconds              mDayLength;
        S64Seconds              mDayOffset;
        size_t                  mDayHash;
        LLSettingsDay::ptr_t    mDayCycle;
        std::array<F32, 4>      mAltitudes;
        bool                    mIsDefault;
        LLUUID                  mAssetId;
        bool                    mIsLegacy;
        std::string             mDayCycleName;
        namelist_t              mNameList;
        S32                     mEnvVersion;

        static ptr_t            extract(LLSD);
        static ptr_t            extractLegacy(LLSD);
    };

    enum EnvSelection_t
    {
        ENV_EDIT = 0,
        ENV_LOCAL,
        ENV_PUSH,
        ENV_PARCEL,
        ENV_REGION,
        ENV_DEFAULT,
        ENV_END,
        ENV_CURRENT = -1,
        ENV_NONE = -2
    };

    typedef boost::signals2::connection     connection_t;

    typedef std::pair<LLSettingsSky::ptr_t, LLSettingsWater::ptr_t> fixedEnvironment_t;
    typedef std::function<void(S32, EnvironmentInfo::ptr_t)>        environment_apply_fn;
    typedef boost::signals2::signal<void(EnvSelection_t, S32)>           env_changed_signal_t;
    typedef env_changed_signal_t::slot_type                         env_changed_fn;
    typedef std::array<F32, 4>                                      altitude_list_t;
    typedef std::vector<F32>                                        altitudes_vect_t;

    virtual                     ~LLEnvironment();

    bool                        canEdit() const;
    bool                        isExtendedEnvironmentEnabled() const;
    bool                        isInventoryEnabled() const;
    bool                        canAgentUpdateParcelEnvironment() const;
    bool                        canAgentUpdateParcelEnvironment(LLParcel *parcel) const;
    bool                        canAgentUpdateRegionEnvironment() const;

    LLSettingsDay::ptr_t        getCurrentDay() const { return mCurrentEnvironment->getDayCycle(); }
    LLSettingsSky::ptr_t        getCurrentSky() const;
    LLSettingsWater::ptr_t      getCurrentWater() const;

    static void                 getAtmosphericModelSettings(AtmosphericModelSettings& settingsOut, const LLSettingsSky::ptr_t &psky);

    void                        update(const LLViewerCamera * cam);

    static void                 updateGLVariablesForSettings(LLGLSLShader *shader, const LLSettingsBase::ptr_t &psetting);
    void                        updateShaderUniforms(LLGLSLShader *shader);

    void                        setSelectedEnvironment(EnvSelection_t env, LLSettingsBase::Seconds transition = TRANSITION_DEFAULT, bool forced = false);
    EnvSelection_t              getSelectedEnvironment() const                  { return mSelectedEnvironment; }

    bool                        hasEnvironment(EnvSelection_t env);
    void                        setEnvironment(EnvSelection_t env, const LLSettingsDay::ptr_t &pday, LLSettingsDay::Seconds daylength, LLSettingsDay::Seconds dayoffset, S32 env_version = NO_VERSION);
    void                        setEnvironment(EnvSelection_t env, fixedEnvironment_t fixed, S32 env_version = NO_VERSION);
    void                        setEnvironment(EnvSelection_t env, const LLSettingsBase::ptr_t &fixed, S32 env_version = NO_VERSION);
    void                        setEnvironment(EnvSelection_t env, const LLSettingsSky::ptr_t & fixed, S32 env_version = NO_VERSION) { setEnvironment(env, fixedEnvironment_t(fixed, LLSettingsWater::ptr_t()), env_version); }
    void                        setEnvironment(EnvSelection_t env, const LLSettingsWater::ptr_t & fixed, S32 env_version = NO_VERSION) { setEnvironment(env, fixedEnvironment_t(LLSettingsSky::ptr_t(), fixed), env_version); }
    void                        setEnvironment(EnvSelection_t env, const LLSettingsSky::ptr_t & fixeds, const LLSettingsWater::ptr_t & fixedw, S32 env_version = NO_VERSION) { setEnvironment(env, fixedEnvironment_t(fixeds, fixedw), env_version); }
    void                        setEnvironment(EnvSelection_t env, const LLUUID &assetId, LLSettingsDay::Seconds daylength, LLSettingsDay::Seconds dayoffset, S32 env_version = NO_VERSION);
    void                        setEnvironment(EnvSelection_t env, const LLUUID &assetId, S32 env_version = NO_VERSION);

    void                        setSharedEnvironment();

    void                        clearEnvironment(EnvSelection_t env);
    LLSettingsDay::ptr_t        getEnvironmentDay(EnvSelection_t env);
    LLSettingsDay::Seconds      getEnvironmentDayLength(EnvSelection_t env);
    LLSettingsDay::Seconds      getEnvironmentDayOffset(EnvSelection_t env);
    fixedEnvironment_t          getEnvironmentFixed(EnvSelection_t env);
    LLSettingsSky::ptr_t        getEnvironmentFixedSky(EnvSelection_t env)      { return getEnvironmentFixed(env).first; };
    LLSettingsWater::ptr_t      getEnvironmentFixedWater(EnvSelection_t env)    { return getEnvironmentFixed(env).second; };

    void                        updateEnvironment(LLSettingsBase::Seconds transition = TRANSITION_DEFAULT, bool forced = false);

    inline LLVector2            getCloudScrollDelta() const { return mCloudScrollDelta; }
    void                        pauseCloudScroll()          { mCloudScrollPaused = true; }
    void                        resumeCloudScroll()         { mCloudScrollPaused = false; }
    bool                        isCloudScrollPaused() const { return mCloudScrollPaused; }

    F32                         getCamHeight() const;
    F32                         getWaterHeight() const;
    bool                        getIsSunUp() const;
    bool                        getIsMoonUp() const;

    // Returns either sun or moon direction (depending on which is up and stronger)
    // Light direction in +x right, +z up, +y at internal coord sys
    LLVector3                   getLightDirection() const; // returns sun or moon depending on which is up
    LLVector3                   getSunDirection() const;
    LLVector3                   getMoonDirection() const;

    // Returns light direction converted to CFR coord system
    LLVector4                   getLightDirectionCFR() const; // returns sun or moon depending on which is up
    LLVector4                   getSunDirectionCFR() const;
    LLVector4                   getMoonDirectionCFR() const;

    // Returns light direction converted to OGL coord system
    // and clamped above -0.1f in Y to avoid render artifacts in sky shaders
    LLVector4                   getClampedLightNorm() const; // returns sun or moon depending on which is up
    LLVector4                   getClampedSunNorm() const;
    LLVector4                   getClampedMoonNorm() const;

    // Returns light direction converted to OGL coord system
    // and rotated by last cam yaw needed by water rendering shaders
    LLVector4                   getRotatedLightNorm() const;

    static LLSettingsWater::ptr_t   createWaterFromLegacyPreset(const std::string filename);
    static LLSettingsSky::ptr_t createSkyFromLegacyPreset(const std::string filename);
    static LLSettingsDay::ptr_t createDayCycleFromLegacyPreset(const std::string filename);

    // Construct a new day cycle based on the environment.  Replacing either the water or the sky tracks.
    LLSettingsDay::ptr_t        createDayCycleFromEnvironment(EnvSelection_t env, LLSettingsBase::ptr_t settings);

    //-------------------------------------------
    connection_t                setEnvironmentChanged(env_changed_fn cb)    { return mSignalEnvChanged.connect(cb); }

    void                        requestRegion(environment_apply_fn cb = environment_apply_fn());
    void                        updateRegion(const LLUUID &asset_id, std::string display_name, S32 track_num, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        updateRegion(const LLSettingsDay::ptr_t &pday, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        updateRegion(const LLSettingsSky::ptr_t &psky, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        updateRegion(const LLSettingsWater::ptr_t &pwater, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        resetRegion(environment_apply_fn cb = environment_apply_fn());
    void                        requestParcel(S32 parcel_id, environment_apply_fn cb = environment_apply_fn());
    void                        updateParcel(S32 parcel_id, const LLUUID &asset_id, std::string display_name, S32 track_num, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        updateParcel(S32 parcel_id, const LLSettingsDay::ptr_t &pday, S32 track_num, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        updateParcel(S32 parcel_id, const LLSettingsDay::ptr_t &pday, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        updateParcel(S32 parcel_id, const LLSettingsSky::ptr_t &psky, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        updateParcel(S32 parcel_id, const LLSettingsWater::ptr_t &pwater, S32 day_length, S32 day_offset, altitudes_vect_t altitudes = altitudes_vect_t(), environment_apply_fn cb = environment_apply_fn());
    void                        resetParcel(S32 parcel_id, environment_apply_fn cb = environment_apply_fn());

    void                        selectAgentEnvironment();

    S32                         calculateSkyTrackForAltitude(F64 altitude);

    const altitude_list_t &     getRegionAltitudes() const { return mTrackAltitudes; }

    void                        handleEnvironmentPush(LLSD &message);

    class DayInstance
    {
    public:
        enum InstanceType_t
        {
            TYPE_INVALID,
            TYPE_FIXED,
            TYPE_CYCLED
        };
        typedef std::shared_ptr<DayInstance> ptr_t;

                                    DayInstance(EnvSelection_t env);
        virtual                     ~DayInstance() { };

        ptr_t                       clone() const;

        virtual void                applyTimeDelta(const LLSettingsBase::Seconds& delta);

        virtual void                setDay(const LLSettingsDay::ptr_t &pday, LLSettingsDay::Seconds daylength, LLSettingsDay::Seconds dayoffset);
        virtual void                setSky(const LLSettingsSky::ptr_t &psky);
        virtual void                setWater(const LLSettingsWater::ptr_t &pwater);

        void                        initialize();
        bool                        isInitialized();

        void                        clear();

        void                        setSkyTrack(S32 trackno);

        LLSettingsDay::ptr_t        getDayCycle() const     { return mDayCycle; }
        LLSettingsSky::ptr_t        getSky() const          { return mSky; }
        LLSettingsWater::ptr_t      getWater() const        { return mWater; }
        LLSettingsDay::Seconds      getDayLength() const    { return mDayLength; }
        LLSettingsDay::Seconds      getDayOffset() const    { return mDayOffset; }
        S32                         getSkyTrack() const     { return mSkyTrack; }

        virtual void                animate();

        void                        setBlenders(const LLSettingsBlender::ptr_t &skyblend, const LLSettingsBlender::ptr_t &waterblend);

        EnvSelection_t              getEnvironmentSelection() const { return mEnv; }

        void                        setBackup(bool backup);
        bool                        getBackup() const       { return mBackup; }
        bool                        hasBackupSky() const    { return !mBackupSky.isUndefined() || !mBackupWater.isUndefined(); }
        void                        backup();
        void                        restore();

    protected:
        LLSettingsDay::ptr_t        mDayCycle;
        LLSettingsSky::ptr_t        mSky;
        LLSettingsWater::ptr_t      mWater;
        S32                         mSkyTrack;

        bool                        mBackup;
        LLSD                        mBackupSky;
        LLSD                        mBackupWater;

        InstanceType_t              mType;
        bool                        mInitialized;

        LLSettingsDay::Seconds      mDayLength;
        LLSettingsDay::Seconds      mDayOffset;
        S32                         mLastTrackAltitude;

        LLSettingsBlender::ptr_t    mBlenderSky;
        LLSettingsBlender::ptr_t    mBlenderWater;

        EnvSelection_t              mEnv;

        LLSettingsBase::TrackPosition secondsToKeyframe(LLSettingsDay::Seconds seconds);
    };

    DayInstance::ptr_t          getSelectedEnvironmentInstance();
    DayInstance::ptr_t          getSharedEnvironmentInstance();

protected:
    virtual void                initSingleton();

private:
    LLVector4 toCFR(const LLVector3 vec) const;
    LLVector4 toLightNorm(const LLVector3 vec) const;

    typedef std::array<DayInstance::ptr_t, ENV_END> InstanceArray_t;

    struct ExpEnvironmentEntry
    {
        typedef std::shared_ptr<ExpEnvironmentEntry> ptr_t;

        S32Seconds  mTime;
        LLUUID      mExperienceId;
        LLSD        mEnvironmentOverrides;
    };
    typedef std::deque<ExpEnvironmentEntry::ptr_t>  mPushOverrides;

    LLUUID                      mPushEnvironmentExpId;

    class DayTransition : public DayInstance
    {
    public:
                                    DayTransition(const LLSettingsSky::ptr_t &skystart, const LLSettingsWater::ptr_t &waterstart, DayInstance::ptr_t &end, LLSettingsDay::Seconds time);
        virtual                     ~DayTransition() { };

        virtual void                applyTimeDelta(const LLSettingsBase::Seconds& delta);
        virtual void                animate();

    protected:
        LLSettingsSky::ptr_t        mStartSky;
        LLSettingsWater::ptr_t      mStartWater;
        DayInstance::ptr_t          mNextInstance;
        LLSettingsDay::Seconds      mTransitionTime;
    };

    static const F32            SUN_DELTA_YAW;
    F32                         mLastCamYaw = 0.0f;

    LLVector2                   mCloudScrollDelta;  // cumulative cloud delta
    bool                        mCloudScrollPaused;

    InstanceArray_t             mEnvironments;

    EnvSelection_t              mSelectedEnvironment;
    DayInstance::ptr_t          mCurrentEnvironment;

    LLSettingsSky::ptr_t        mSelectedSky;
    LLSettingsWater::ptr_t      mSelectedWater;
    LLSettingsDay::ptr_t        mSelectedDay;

    LLSettingsBlender::ptr_t    mBlenderSky;
    LLSettingsBlender::ptr_t    mBlenderWater;

    env_changed_signal_t        mSignalEnvChanged;

    S32                         mCurrentTrack;
    altitude_list_t             mTrackAltitudes;

    LLSD                        mSkyOverrides;
    LLSD                        mWaterOverrides;
    LLSD                        mSkyOverrideBlends;
    LLSD                        mWaterOverrideBlends;

    DayInstance::ptr_t          getEnvironmentInstance(EnvSelection_t env, bool create = false);

    void                        updateCloudScroll();

    void                        onRegionChange();
    void                        onParcelChange();

    struct UpdateInfo
    {
        typedef std::shared_ptr<UpdateInfo> ptr_t;

        UpdateInfo(LLSettingsDay::ptr_t pday, S32 day_length, S32 day_offset, altitudes_vect_t altitudes):
            mDayp(pday),
            mSettingsAsset(),
            mDayLength(day_length),
            mDayOffset(day_offset),
            mAltitudes(altitudes),
            mDayName()
        {
            if (mDayp)
                mDayName = mDayp->getName();
        }

        UpdateInfo(LLUUID settings_asset, std::string name, S32 day_length, S32 day_offset, altitudes_vect_t altitudes) :
            mDayp(),
            mSettingsAsset(settings_asset),
            mDayLength(day_length),
            mDayOffset(day_offset),
            mAltitudes(altitudes),
            mDayName(name)
        {}

        LLSettingsDay::ptr_t    mDayp; 
        LLUUID                  mSettingsAsset; 
        S32                     mDayLength; 
        S32                     mDayOffset; 
        altitudes_vect_t        mAltitudes;
        std::string             mDayName;
    };

    struct ExpBlendValue
    {
        ExpBlendValue(F32Seconds transition, const std::string &keyname, LLSD value, bool blendin, S32 index = -1) :
            mTransition(transition),
            mTimeRemaining(transition),
            mKeyName(keyname),
            mValue(value),
            mValueInitial(),
            mIndex(index),
            mBlendIn(blendin)
        {}

        F32Seconds      mTransition;
        F32Seconds      mTimeRemaining;
        std::string     mKeyName;
        LLSD            mValue;
        LLSD            mValueInitial;
        S32             mIndex;
        bool            mBlendIn;

        typedef std::shared_ptr<ExpBlendValue>  ptr_t;
    };

    typedef std::deque<ExpBlendValue>   exerienceBlendValues_t;

    void                        coroRequestEnvironment(S32 parcel_id, environment_apply_fn apply);
    void                        coroUpdateEnvironment(S32 parcel_id, S32 track_no, UpdateInfo::ptr_t updates, environment_apply_fn apply);
    void                        coroResetEnvironment(S32 parcel_id, S32 track_no, environment_apply_fn apply);

    void                        recordEnvironment(S32 parcel_id, EnvironmentInfo::ptr_t environment, LLSettingsBase::Seconds transition);

    void                        onAgentPositionHasChanged(const LLVector3 &localpos);

    void                        onSetEnvAssetLoaded(EnvSelection_t env, LLUUID asset_id, LLSettingsBase::ptr_t settings, LLSettingsDay::Seconds daylength, LLSettingsDay::Seconds dayoffset, LLSettingsBase::Seconds transition, S32 status, S32 env_version);
    void                        onUpdateParcelAssetLoaded(LLUUID asset_id, LLSettingsBase::ptr_t settings, S32 status, S32 parcel_id, S32 day_length, S32 day_offset, altitudes_vect_t altitudes);

    void                        handleEnvironmentPushClear(LLUUID experience_id, LLSD &message, F32 transition);
    void                        handleEnvironmentPushFull(LLUUID experience_id, LLSD &message, F32 transition);
    void                        handleEnvironmentPushPartial(LLUUID experience_id, LLSD &message, F32 transition);

    void                        clearExperienceEnvironment(LLUUID experience_id, F32 transition_time);
    void                        setExperienceEnvironment(LLUUID experience_id, LLUUID asset_id, F32 transition_time);
    void                        setExperienceEnvironment(LLUUID experience_id, LLSD environment, F32 transition_time);
    void                        setInstanceBackup(bool dobackup);

    void                        injectSettings(LLUUID experience_id, exerienceBlendValues_t &blends, LLSD injections, LLSettingsBase::Seconds transition, bool blendin);

    void                        applyInjectedSettings(DayInstance::ptr_t environment, F32Seconds delta);
    void                        applyInjectedValues(LLSettingsBase::ptr_t psetting, LLSD injection);
    void                        blendInjectedValues(LLSettingsBase::ptr_t psetting, exerienceBlendValues_t &blends, LLSD &overrides, F32Seconds delta);

    exerienceBlendValues_t      mSkyExperienceBlends;
    exerienceBlendValues_t      mWaterExperienceBlends;
    bool                        mMakeBackups;
};

class LLTrackBlenderLoopingManual : public LLSettingsBlender
{
public:
    LLTrackBlenderLoopingManual(const LLSettingsBase::ptr_t &target, const LLSettingsDay::ptr_t &day, S32 trackno);

    F64                         setPosition(const LLSettingsBase::TrackPosition& position);
    virtual void                switchTrack(S32 trackno, const LLSettingsBase::TrackPosition& position) override;
    S32                         getTrack() const { return mTrackNo; }

    typedef std::shared_ptr<LLTrackBlenderLoopingManual> ptr_t;
protected:
    LLSettingsDay::TrackBound_t getBoundingEntries(F64 position);
    F64                         getSpanLength(const LLSettingsDay::TrackBound_t &bounds) const;

private:
    LLSettingsDay::ptr_t        mDay;
    S32                         mTrackNo;
    F64                         mPosition;

    LLSettingsDay::CycleTrack_t::iterator mEndMarker;
};

#endif // LL_ENVIRONMENT_H

