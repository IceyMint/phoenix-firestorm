/** 
 * @file llfloaterfixedenvironment.cpp
 * @brief Floaters to create and edit fixed settings for sky and water.
 *
 * $LicenseInfo:firstyear=2011&license=viewerlgpl$
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

#include "llviewerprecompiledheaders.h"

#include "llfloaterenvironmentadjust.h"

#include "llslider.h"
#include "llsliderctrl.h"
#include "llcolorswatch.h"
#include "llvirtualtrackball.h"
#include "llenvironment.h"

//=========================================================================
namespace
{
    const std::string FIELD_SKY_AMBIENT_LIGHT("ambient_light");
    const std::string FIELD_SKY_BLUE_HORIZON("blue_horizon");
    const std::string FIELD_SKY_BLUE_DENSITY("blue_density");
    const std::string FIELD_SKY_SUN_COLOR("sun_color");
    const std::string FIELD_SKY_CLOUD_COLOR("cloud_color");
    const std::string FIELD_SKY_HAZE_HORIZON("haze_horizon");
    const std::string FIELD_SKY_HAZE_DENSITY("haze_density");
    const std::string FIELD_SKY_CLOUD_COVERAGE("cloud_coverage");
    const std::string FIELD_SKY_CLOUD_SCALE("cloud_scale");
    const std::string FIELD_SKY_SCENE_GAMMA("scene_gamma");
    const std::string FIELD_SKY_SUN_ROTATION("sun_rotation");
    const std::string FIELD_SKY_SUN_SCALE("sun_scale");
    const std::string FIELD_SKY_GLOW_FOCUS("glow_focus");
    const std::string FIELD_SKY_GLOW_SIZE("glow_size");
    const std::string FIELD_SKY_STAR_BRIGHTNESS("star_brightness");
    const std::string FIELD_SKY_MOON_ROTATION("moon_rotation");

    const F32 SLIDER_SCALE_SUN_AMBIENT(3.0f);
    const F32 SLIDER_SCALE_BLUE_HORIZON_DENSITY(2.0f);
    const F32 SLIDER_SCALE_GLOW_R(20.0f);
    const F32 SLIDER_SCALE_GLOW_B(-5.0f);
    //const F32 SLIDER_SCALE_DENSITY_MULTIPLIER(0.001f);

    const S32 FLOATER_ENVIRONMENT_UPDATE(-2);
}

//=========================================================================
LLFloaterEnvironmentAdjust::LLFloaterEnvironmentAdjust(const LLSD &key):
    LLFloater(key)
{}

LLFloaterEnvironmentAdjust::~LLFloaterEnvironmentAdjust()
{}

//-------------------------------------------------------------------------
BOOL LLFloaterEnvironmentAdjust::postBuild()
{
    getChild<LLUICtrl>(FIELD_SKY_AMBIENT_LIGHT)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onAmbientLightChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_BLUE_HORIZON)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onBlueHorizonChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_BLUE_DENSITY)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onBlueDensityChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_HAZE_HORIZON)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onHazeHorizonChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_HAZE_DENSITY)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onHazeDensityChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_SCENE_GAMMA)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onSceneGammaChanged(); });

    getChild<LLUICtrl>(FIELD_SKY_CLOUD_COLOR)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onCloudColorChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_CLOUD_COVERAGE)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onCloudCoverageChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_CLOUD_SCALE)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onCloudScaleChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_SUN_COLOR)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onSunColorChanged(); });

    getChild<LLUICtrl>(FIELD_SKY_GLOW_FOCUS)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onGlowChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_GLOW_SIZE)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onGlowChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_STAR_BRIGHTNESS)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onStarBrightnessChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_SUN_ROTATION)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onSunRotationChanged(); });
    getChild<LLUICtrl>(FIELD_SKY_SUN_SCALE)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onSunScaleChanged(); });

    getChild<LLUICtrl>(FIELD_SKY_MOON_ROTATION)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onMoonRotationChanged(); });

    refresh();
    return TRUE;
}

void LLFloaterEnvironmentAdjust::onOpen(const LLSD& key)
{
    captureCurrentEnvironment();

    mEventConnection = LLEnvironment::instance().setEnvironmentChanged([this](LLEnvironment::EnvSelection_t env, S32 version){ onEnvironmentUpdated(env, version); });

    LLFloater::onOpen(key);
    refresh();
}

void LLFloaterEnvironmentAdjust::onClose(bool app_quitting)
{
    mEventConnection.disconnect();
    mLiveSky.reset();
    LLFloater::onClose(app_quitting);
}


//-------------------------------------------------------------------------
void LLFloaterEnvironmentAdjust::refresh()
{
    if (!mLiveSky)
    {
        setAllChildrenEnabled(FALSE);
        return;
    }

    setEnabled(TRUE);
    setAllChildrenEnabled(TRUE);

    getChild<LLColorSwatchCtrl>(FIELD_SKY_AMBIENT_LIGHT)->set(mLiveSky->getAmbientColor() / SLIDER_SCALE_SUN_AMBIENT);
    getChild<LLColorSwatchCtrl>(FIELD_SKY_BLUE_HORIZON)->set(mLiveSky->getBlueHorizon() / SLIDER_SCALE_BLUE_HORIZON_DENSITY);
    getChild<LLColorSwatchCtrl>(FIELD_SKY_BLUE_DENSITY)->set(mLiveSky->getBlueDensity() / SLIDER_SCALE_BLUE_HORIZON_DENSITY);
    getChild<LLUICtrl>(FIELD_SKY_HAZE_HORIZON)->setValue(mLiveSky->getHazeHorizon());
    getChild<LLUICtrl>(FIELD_SKY_HAZE_DENSITY)->setValue(mLiveSky->getHazeDensity());
    getChild<LLUICtrl>(FIELD_SKY_SCENE_GAMMA)->setValue(mLiveSky->getGamma());
    getChild<LLColorSwatchCtrl>(FIELD_SKY_CLOUD_COLOR)->set(mLiveSky->getCloudColor());
    getChild<LLUICtrl>(FIELD_SKY_CLOUD_COVERAGE)->setValue(mLiveSky->getCloudShadow());
    getChild<LLUICtrl>(FIELD_SKY_CLOUD_SCALE)->setValue(mLiveSky->getCloudScale());
    getChild<LLColorSwatchCtrl>(FIELD_SKY_SUN_COLOR)->set(mLiveSky->getSunlightColor() / SLIDER_SCALE_SUN_AMBIENT);

    LLColor3 glow(mLiveSky->getGlow());

    // takes 40 - 0.2 range -> 0 - 1.99 UI range
    getChild<LLUICtrl>(FIELD_SKY_GLOW_SIZE)->setValue(2.0 - (glow.mV[0] / SLIDER_SCALE_GLOW_R));
    getChild<LLUICtrl>(FIELD_SKY_GLOW_FOCUS)->setValue(glow.mV[2] / SLIDER_SCALE_GLOW_B);
    getChild<LLUICtrl>(FIELD_SKY_STAR_BRIGHTNESS)->setValue(mLiveSky->getStarBrightness());
    getChild<LLVirtualTrackball>(FIELD_SKY_SUN_ROTATION)->setRotation(mLiveSky->getSunRotation());
    getChild<LLUICtrl>(FIELD_SKY_SUN_SCALE)->setValue(mLiveSky->getSunScale());
    getChild<LLVirtualTrackball>(FIELD_SKY_MOON_ROTATION)->setRotation(mLiveSky->getMoonRotation());

}


void LLFloaterEnvironmentAdjust::captureCurrentEnvironment()
{
    LLEnvironment &environment(LLEnvironment::instance());
    bool updatelocal(false);

    if (environment.hasEnvironment(LLEnvironment::ENV_LOCAL))
    {
        if (environment.getEnvironmentDay(LLEnvironment::ENV_LOCAL))
        {   // We have a full day cycle in the local environment.  Freeze the sky
            mLiveSky = environment.getEnvironmentFixedSky(LLEnvironment::ENV_LOCAL)->buildClone();
            updatelocal = true;
        }
        else
        {   // otherwise we can just use the sky.
            mLiveSky = environment.getEnvironmentFixedSky(LLEnvironment::ENV_LOCAL);
        }
    }
    else
    {
        mLiveSky = environment.getEnvironmentFixedSky(LLEnvironment::ENV_PARCEL, true)->buildClone();
        updatelocal = true;
    }

    if (updatelocal)
    {
        environment.setEnvironment(LLEnvironment::ENV_LOCAL, mLiveSky, FLOATER_ENVIRONMENT_UPDATE);
    }
    environment.setSelectedEnvironment(LLEnvironment::ENV_LOCAL);
    environment.updateEnvironment(LLEnvironment::TRANSITION_INSTANT);

}

//-------------------------------------------------------------------------
void LLFloaterEnvironmentAdjust::onAmbientLightChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setAmbientColor(LLColor3(getChild<LLColorSwatchCtrl>(FIELD_SKY_AMBIENT_LIGHT)->get() * SLIDER_SCALE_SUN_AMBIENT));
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onBlueHorizonChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setBlueHorizon(LLColor3(getChild<LLColorSwatchCtrl>(FIELD_SKY_BLUE_HORIZON)->get() * SLIDER_SCALE_BLUE_HORIZON_DENSITY));
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onBlueDensityChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setBlueDensity(LLColor3(getChild<LLColorSwatchCtrl>(FIELD_SKY_BLUE_DENSITY)->get() * SLIDER_SCALE_BLUE_HORIZON_DENSITY));
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onHazeHorizonChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setHazeHorizon(getChild<LLUICtrl>(FIELD_SKY_HAZE_HORIZON)->getValue().asReal());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onHazeDensityChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setHazeDensity(getChild<LLUICtrl>(FIELD_SKY_HAZE_DENSITY)->getValue().asReal());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onSceneGammaChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setGamma(getChild<LLUICtrl>(FIELD_SKY_SCENE_GAMMA)->getValue().asReal());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onCloudColorChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setCloudColor(LLColor3(getChild<LLColorSwatchCtrl>(FIELD_SKY_CLOUD_COLOR)->get()));
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onCloudCoverageChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setCloudShadow(getChild<LLUICtrl>(FIELD_SKY_CLOUD_COVERAGE)->getValue().asReal());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onCloudScaleChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setCloudScale(getChild<LLUICtrl>(FIELD_SKY_CLOUD_SCALE)->getValue().asReal());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onGlowChanged()
{
    if (!mLiveSky)
        return;
    LLColor3 glow(getChild<LLUICtrl>(FIELD_SKY_GLOW_SIZE)->getValue().asReal(), 0.0f, getChild<LLUICtrl>(FIELD_SKY_GLOW_FOCUS)->getValue().asReal());

    // takes 0 - 1.99 UI range -> 40 -> 0.2 range
    glow.mV[0] = (2.0f - glow.mV[0]) * SLIDER_SCALE_GLOW_R;
    glow.mV[2] *= SLIDER_SCALE_GLOW_B;

    mLiveSky->setGlow(glow);
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onStarBrightnessChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setStarBrightness(getChild<LLUICtrl>(FIELD_SKY_STAR_BRIGHTNESS)->getValue().asReal());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onSunRotationChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setSunRotation(getChild<LLVirtualTrackball>(FIELD_SKY_SUN_ROTATION)->getRotation());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onSunScaleChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setSunScale((getChild<LLUICtrl>(FIELD_SKY_SUN_SCALE)->getValue().asReal()));
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onMoonRotationChanged()
{
    if (!mLiveSky)
        return;
    mLiveSky->setMoonRotation(getChild<LLVirtualTrackball>(FIELD_SKY_MOON_ROTATION)->getRotation());
    mLiveSky->update();
}

void LLFloaterEnvironmentAdjust::onSunColorChanged()
{
    if (!mLiveSky)
        return;
    LLColor3 color(getChild<LLColorSwatchCtrl>(FIELD_SKY_SUN_COLOR)->get());

    color *= SLIDER_SCALE_SUN_AMBIENT;

    mLiveSky->setSunlightColor(color);
    mLiveSky->update();
}


void LLFloaterEnvironmentAdjust::onEnvironmentUpdated(LLEnvironment::EnvSelection_t env, S32 version)
{
    if (env == LLEnvironment::ENV_LOCAL)
    {   // a new local environment has been applied
        if (version != FLOATER_ENVIRONMENT_UPDATE)
        {   // not by this floater
            captureCurrentEnvironment();
            refresh();
        }
    }
}
