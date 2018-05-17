/**
* @file llpaneleditwater.cpp
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

#include "llpaneleditwater.h"

#include "llslider.h"
#include "lltexturectrl.h"
#include "llcolorswatch.h"

namespace
{
    const std::string   FIELD_WATER_FOG_COLOR("water_fog_color");
    const std::string   FIELD_WATER_FOG_DENSITY("water_fog_density");
    const std::string   FIELD_WATER_UNDERWATER_MOD("water_underwater_mod");
    const std::string   FIELD_WATER_NORMAL_MAP("water_normal_map");

    const std::string   FIELD_WATER_WAVE1_X("water_wave1_x");
    const std::string   FIELD_WATER_WAVE1_Y("water_wave1_y");

    const std::string   FIELD_WATER_WAVE2_X("water_wave2_x");
    const std::string   FIELD_WATER_WAVE2_Y("water_wave2_y");

    const std::string   FIELD_WATER_NORMAL_SCALE_X("water_normal_scale_x");
    const std::string   FIELD_WATER_NORMAL_SCALE_Y("water_normal_scale_y");
    const std::string   FIELD_WATER_NORMAL_SCALE_Z("water_normal_scale_z");

    const std::string   FIELD_WATER_FRESNEL_SCALE("water_fresnel_scale");
    const std::string   FIELD_WATER_FRESNEL_OFFSET("water_fresnel_offset");

    const std::string   FIELD_WATER_SCALE_ABOVE("water_scale_above");
    const std::string   FIELD_WATER_SCALE_BELOW("water_scale_below");
    const std::string   FIELD_WATER_BLUR_MULTIP("water_blur_multip");
}

static LLPanelInjector<LLPanelSettingsWaterMainTab> t_settings_water("panel_settings_water");

//==========================================================================
LLPanelSettingsWater::LLPanelSettingsWater() :
    LLSettingsEditPanel(),
    mWaterSettings()
{

}


//==========================================================================
LLPanelSettingsWaterMainTab::LLPanelSettingsWaterMainTab():
    LLPanelSettingsWater(),
    mClrFogColor(nullptr),
    mTxtNormalMap(nullptr)
{
}


BOOL LLPanelSettingsWaterMainTab::postBuild()
{
    mClrFogColor = getChild<LLColorSwatchCtrl>(FIELD_WATER_FOG_COLOR);
    mTxtNormalMap = getChild<LLTextureCtrl>(FIELD_WATER_NORMAL_MAP);

    mClrFogColor->setCommitCallback([this](LLUICtrl *, const LLSD &) { onFogColorChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_FOG_DENSITY)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onFogDensityChanged(); });
//    getChild<LLUICtrl>(FIELD_WATER_FOG_DENSITY)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onFogDensityChanged(getChild<LLUICtrl>(FIELD_WATER_FOG_DENSITY)->getValue().asReal()); });
    getChild<LLUICtrl>(FIELD_WATER_UNDERWATER_MOD)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onFogUnderWaterChanged(); });

    mTxtNormalMap->setDefaultImageAssetID(LLSettingsWater::DEFAULT_WATER_NORMAL_ID);
    mTxtNormalMap->setCommitCallback([this](LLUICtrl *, const LLSD &) { onNormalMapChanged(); });

    getChild<LLUICtrl>(FIELD_WATER_WAVE1_X)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onLargeWaveChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_WAVE1_Y)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onLargeWaveChanged(); });

    getChild<LLUICtrl>(FIELD_WATER_WAVE2_X)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onSmallWaveChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_WAVE2_Y)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onSmallWaveChanged(); });

    getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_X)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onNormalScaleChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_Y)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onNormalScaleChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_Z)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onNormalScaleChanged(); });

    getChild<LLUICtrl>(FIELD_WATER_FRESNEL_SCALE)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onFresnelScaleChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_FRESNEL_OFFSET)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onFresnelOffsetChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_SCALE_ABOVE)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onScaleAboveChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_SCALE_BELOW)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onScaleBelowChanged(); });
    getChild<LLUICtrl>(FIELD_WATER_BLUR_MULTIP)->setCommitCallback([this](LLUICtrl *, const LLSD &) { onBlurMultipChanged(); });

    refresh();

    return TRUE;
}

//==========================================================================
void LLPanelSettingsWaterMainTab::refresh()
{
    if (!mWaterSettings)
    {
        setAllChildrenEnabled(FALSE);
        setEnabled(FALSE);
        return;
    }

    setEnabled(TRUE);
    setAllChildrenEnabled(TRUE);
    mClrFogColor->set(mWaterSettings->getFogColor());
    getChild<LLUICtrl>(FIELD_WATER_FOG_DENSITY)->setValue(mWaterSettings->getFogDensity());
    getChild<LLUICtrl>(FIELD_WATER_UNDERWATER_MOD)->setValue(mWaterSettings->getFogMod());
    mTxtNormalMap->setValue(mWaterSettings->getNormalMapID());
    LLVector2 vect2 = mWaterSettings->getWave1Dir();
    getChild<LLUICtrl>(FIELD_WATER_WAVE1_X)->setValue(vect2[0]);
    getChild<LLUICtrl>(FIELD_WATER_WAVE1_Y)->setValue(vect2[1]);
    vect2 = mWaterSettings->getWave2Dir();
    getChild<LLUICtrl>(FIELD_WATER_WAVE2_X)->setValue(vect2[0]);
    getChild<LLUICtrl>(FIELD_WATER_WAVE2_Y)->setValue(vect2[1]);
    LLVector3 vect3 = mWaterSettings->getNormalScale();
    getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_X)->setValue(vect3[0]);
    getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_Y)->setValue(vect3[1]);
    getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_Z)->setValue(vect3[2]);
    getChild<LLUICtrl>(FIELD_WATER_FRESNEL_SCALE)->setValue(mWaterSettings->getFresnelOffset());
    getChild<LLUICtrl>(FIELD_WATER_FRESNEL_OFFSET)->setValue(mWaterSettings->getFresnelOffset());
    getChild<LLUICtrl>(FIELD_WATER_SCALE_ABOVE)->setValue(mWaterSettings->getScaleAbove());
    getChild<LLUICtrl>(FIELD_WATER_SCALE_BELOW)->setValue(mWaterSettings->getScaleBelow());
    getChild<LLUICtrl>(FIELD_WATER_BLUR_MULTIP)->setValue(mWaterSettings->getBlurMultiplier());
}

//==========================================================================

void LLPanelSettingsWaterMainTab::onFogColorChanged()
{
    mWaterSettings->setFogColor(LLColor3(mClrFogColor->get()));
}

void LLPanelSettingsWaterMainTab::onFogDensityChanged()
{
    mWaterSettings->setFogDensity(getChild<LLUICtrl>(FIELD_WATER_FOG_DENSITY)->getValue().asReal());
}

void LLPanelSettingsWaterMainTab::onFogUnderWaterChanged()
{
    mWaterSettings->setFogMod(getChild<LLUICtrl>(FIELD_WATER_UNDERWATER_MOD)->getValue().asReal());
}

void LLPanelSettingsWaterMainTab::onNormalMapChanged()
{
    mWaterSettings->setNormalMapID(mTxtNormalMap->getImageAssetID());
}


void LLPanelSettingsWaterMainTab::onLargeWaveChanged()
{
    LLVector2 vect(getChild<LLUICtrl>(FIELD_WATER_WAVE1_X)->getValue().asReal(), getChild<LLUICtrl>(FIELD_WATER_WAVE1_Y)->getValue().asReal());
    LL_WARNS("LAPRAS") << "Changing Large Wave from " << mWaterSettings->getWave1Dir() << " -> " << vect << LL_ENDL;
    mWaterSettings->setWave1Dir(vect);
}

void LLPanelSettingsWaterMainTab::onSmallWaveChanged()
{
    LLVector2 vect(getChild<LLUICtrl>(FIELD_WATER_WAVE2_X)->getValue().asReal(), getChild<LLUICtrl>(FIELD_WATER_WAVE2_Y)->getValue().asReal());
    LL_WARNS("LAPRAS") << "Changing Small Wave from " << mWaterSettings->getWave2Dir() << " -> " << vect << LL_ENDL;
    mWaterSettings->setWave2Dir(vect);
}


void LLPanelSettingsWaterMainTab::onNormalScaleChanged()
{
    LLVector3 vect(getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_X)->getValue().asReal(), getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_Y)->getValue().asReal(), getChild<LLUICtrl>(FIELD_WATER_NORMAL_SCALE_Z)->getValue().asReal());
    LL_WARNS("LAPRAS") << "Changing normal scale from " << mWaterSettings->getNormalScale() << " -> " << vect << LL_ENDL;
    mWaterSettings->setNormalScale(vect);
}

void LLPanelSettingsWaterMainTab::onFresnelScaleChanged()
{
    mWaterSettings->setFresnelScale(getChild<LLUICtrl>(FIELD_WATER_FRESNEL_SCALE)->getValue().asReal());
}

void LLPanelSettingsWaterMainTab::onFresnelOffsetChanged()
{
    mWaterSettings->setFresnelOffset(getChild<LLUICtrl>(FIELD_WATER_FRESNEL_OFFSET)->getValue().asReal());
}

void LLPanelSettingsWaterMainTab::onScaleAboveChanged()
{
    mWaterSettings->setScaleAbove(getChild<LLUICtrl>(FIELD_WATER_SCALE_ABOVE)->getValue().asReal());
}

void LLPanelSettingsWaterMainTab::onScaleBelowChanged()
{
    mWaterSettings->setScaleBelow(getChild<LLUICtrl>(FIELD_WATER_SCALE_BELOW)->getValue().asReal());
}

void LLPanelSettingsWaterMainTab::onBlurMultipChanged()
{
    mWaterSettings->setBlurMultiplier(getChild<LLUICtrl>(FIELD_WATER_BLUR_MULTIP)->getValue().asReal());
}
