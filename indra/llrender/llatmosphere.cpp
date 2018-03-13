/** 
 * @file llatmosphere.cpp
 * @brief LLAtmosphere integration impl
 *
 * $LicenseInfo:firstyear=2018&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2018, Linden Research, Inc.
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

#include "linden_common.h"

#include "llatmosphere.h"
#include "llfasttimer.h"
#include "llsys.h"
#include "llglheaders.h"
#include "llrender.h"
#include "llshadermgr.h"
#include "llglslshader.h"

LLAtmosphere* gAtmosphere = nullptr;

// Values from "Reference Solar Spectral Irradiance: ASTM G-173", ETR column
// (see http://rredc.nrel.gov/solar/spectra/am1.5/ASTMG173/ASTMG173.html),
// summed and averaged in each bin (e.g. the value for 360nm is the average
// of the ASTM G-173 values for all wavelengths between 360 and 370nm).
// Values in W.m^-2.
const int kLambdaMin = 360;
const int kLambdaMax = 830;
const double kSolarIrradiance[48] = {
    1.11776, 1.14259, 1.01249, 1.14716, 1.72765, 1.73054, 1.6887, 1.61253,
    1.91198, 2.03474, 2.02042, 2.02212, 1.93377, 1.95809, 1.91686, 1.8298,
    1.8685, 1.8931, 1.85149, 1.8504, 1.8341, 1.8345, 1.8147, 1.78158, 1.7533,
    1.6965, 1.68194, 1.64654, 1.6048, 1.52143, 1.55622, 1.5113, 1.474, 1.4482,
    1.41018, 1.36775, 1.34188, 1.31429, 1.28303, 1.26758, 1.2367, 1.2082,
    1.18737, 1.14683, 1.12362, 1.1058, 1.07124, 1.04992
};

// Values from http://www.iup.uni-bremen.de/gruppen/molspec/databases/
// referencespectra/o3spectra2011/index.html for 233K, summed and averaged in
// each bin (e.g. the value for 360nm is the average of the original values
// for all wavelengths between 360 and 370nm). Values in m^2.
const double kOzoneCrossSection[48] = {
    1.18e-27, 2.182e-28, 2.818e-28, 6.636e-28, 1.527e-27, 2.763e-27, 5.52e-27,
    8.451e-27, 1.582e-26, 2.316e-26, 3.669e-26, 4.924e-26, 7.752e-26, 9.016e-26,
    1.48e-25, 1.602e-25, 2.139e-25, 2.755e-25, 3.091e-25, 3.5e-25, 4.266e-25,
    4.672e-25, 4.398e-25, 4.701e-25, 5.019e-25, 4.305e-25, 3.74e-25, 3.215e-25,
    2.662e-25, 2.238e-25, 1.852e-25, 1.473e-25, 1.209e-25, 9.423e-26, 7.455e-26,
    6.566e-26, 5.105e-26, 4.15e-26, 4.228e-26, 3.237e-26, 2.451e-26, 2.801e-26,
    2.534e-26, 1.624e-26, 1.465e-26, 2.078e-26, 1.383e-26, 7.105e-27
};

// From https://en.wikipedia.org/wiki/Dobson_unit, in molecules.m^-2.
const double kDobsonUnit = 2.687e20;
// Maximum number density of ozone molecules, in m^-3 (computed so at to get
// 300 Dobson units of ozone - for this we divide 300 DU by the integral of
// the ozone density profile defined below, which is equal to 15km).
const double kMaxOzoneNumberDensity = 300.0 * kDobsonUnit / 15000.0;
const double kSunAngularRadius = 0.00935 / 2.0;
const double kBottomRadius = 6360000.0;
const double kTopRadius = 6420000.0;
const double kRayleigh = 1.24062e-6;
const double kRayleighScaleHeight = 8000.0;
const double kMieScaleHeight = 1200.0;
const double kMieAngstromAlpha = 0.0;
const double kMieAngstromBeta = 5.328e-3;
const double kMieSingleScatteringAlbedo = 0.9;
const double kMiePhaseFunctionG = 0.8;
const double max_sun_zenith_angle = F_PI * 2.0 / 3.0;

AtmosphericModelSettings::AtmosphericModelSettings()
    : m_skyBottomRadius(6360.0f)
    , m_skyTopRadius(6420.0f)
    , m_sunArcRadians(0.00045f)
    , m_mieAnisotropy(0.8f)
{
    atmosphere::DensityProfileLayer rayleigh_density(0.0, 1.0, -1.0 / kRayleighScaleHeight, 0.0, 0.0);
    atmosphere::DensityProfileLayer mie_density(0.0, 1.0, -1.0 / kMieScaleHeight, 0.0, 0.0);

    m_rayleighProfile.push_back(rayleigh_density);
    m_mieProfile.push_back(mie_density);

    // Density profile increasing linearly from 0 to 1 between 10 and 25km, and
    // decreasing linearly from 1 to 0 between 25 and 40km. This is an approximate
    // profile from http://www.kln.ac.lk/science/Chemistry/Teaching_Resources/
    // Documents/Introduction%20to%20atmospheric%20chemistry.pdf (page 10).
    m_absorptionProfile.push_back(atmosphere::DensityProfileLayer(25000.0, 0.0, 0.0, 1.0 / 15000.0, -2.0 / 3.0));
    m_absorptionProfile.push_back(atmosphere::DensityProfileLayer(0.0, 0.0, 0.0, -1.0 / 15000.0, 8.0 / 3.0));
}

AtmosphericModelSettings::AtmosphericModelSettings(
    DensityProfile& rayleighProfile,
    DensityProfile& mieProfile,
    DensityProfile& absorptionProfile)
: m_skyBottomRadius(6360.0f)
, m_skyTopRadius(6420.0f)
, m_rayleighProfile(rayleighProfile)
, m_mieProfile(mieProfile)
, m_absorptionProfile(absorptionProfile)
, m_sunArcRadians(0.00045f)
, m_mieAnisotropy(0.8f)
{
}

AtmosphericModelSettings::AtmosphericModelSettings(
    F32             skyBottomRadius,
    F32             skyTopRadius,
    DensityProfile& rayleighProfile,
    DensityProfile& mieProfile,
    DensityProfile& absorptionProfile,
    F32             sunArcRadians,
    F32             mieAniso)
: m_skyBottomRadius(skyBottomRadius)
, m_skyTopRadius(skyTopRadius)
, m_rayleighProfile(rayleighProfile)
, m_mieProfile(mieProfile)
, m_absorptionProfile(absorptionProfile)
, m_sunArcRadians(sunArcRadians)
, m_mieAnisotropy(mieAniso)
{
}

void LLAtmosphere::initClass()
{
    if (!gAtmosphere)
    { 
        gAtmosphere = new LLAtmosphere; 
    }
}

void LLAtmosphere::cleanupClass()
{
    if(gAtmosphere)
    {
        delete gAtmosphere;
    }
    gAtmosphere = NULL;
}

LLAtmosphere::LLAtmosphere()
{
    for (int l = kLambdaMin; l <= kLambdaMax; l += 10)
    {
        double lambda = static_cast<double>(l) * 1e-3;  // micro-meters
        double mie    = kMieAngstromBeta / kMieScaleHeight * pow(lambda, -kMieAngstromAlpha);
        m_wavelengths.push_back(l);
        m_solar_irradiance.push_back(kSolarIrradiance[(l - kLambdaMin) / 10]);
        m_rayleigh_scattering.push_back(kRayleigh * pow(lambda, -4));
        m_mie_scattering.push_back(mie * kMieSingleScatteringAlbedo);
        m_mie_extinction.push_back(mie);
        m_absorption_extinction.push_back(kMaxOzoneNumberDensity * kOzoneCrossSection[(l - kLambdaMin) / 10]);
        m_ground_albedo.push_back(0.6f);
    }

    AtmosphericModelSettings defaults;
    configureAtmosphericModel(defaults);
}

LLAtmosphere::~LLAtmosphere()
{
    // Cease referencing textures from atmosphere::model from our LLGLTextures wrappers for same.
    if (m_transmittance)
    {
        m_transmittance->setTexName(0);
    }

    if (m_scattering)
    {
        m_scattering->setTexName(0);
    }

    if (m_mie_scatter_texture)
    {
        m_mie_scatter_texture->setTexName(0);
    }

    delete m_model;
    m_model = nullptr;
}

bool LLAtmosphere::configureAtmosphericModel(AtmosphericModelSettings& settings)
{
// Advanced Atmospherics TODO
// Make this store a hash of the precomputed data
// and avoid redundant calcs for identical settings

    if (m_model)
    {
        delete m_model;
    }
    m_model = nullptr;
    getTransmittance()->setTexName(0);
    getScattering()->setTexName(0);
    getMieScattering()->setTexName(0);

    // Init libatmosphere model
    m_config.num_scattering_orders = 4;

    m_model = new atmosphere::Model(
                                m_wavelengths,
                                m_solar_irradiance,
                                settings.m_sunArcRadians,
                                settings.m_skyBottomRadius * 1000.0f,
                                settings.m_skyTopRadius * 1000.0f,
                                settings.m_rayleighProfile,
                                m_rayleigh_scattering,
                                settings.m_mieProfile,
                                m_mie_scattering,
                                m_mie_extinction,
                                settings.m_mieAnisotropy,
                                settings.m_absorptionProfile,
                                m_absorption_extinction,
                                m_ground_albedo,
                                max_sun_zenith_angle,
                                1000.0,   
                                15,
                                false,
                                true);

    if (m_model)
    {
        m_model->Init(m_config, m_textures);
        getTransmittance()->setTexName(m_textures.transmittance_texture);
        getScattering()->setTexName(m_textures.transmittance_texture);   
        getMieScattering()->setTexName(m_textures.transmittance_texture);
    }

    return m_model != nullptr;
}

LLGLTexture* LLAtmosphere::getTransmittance()
{
    if (!m_transmittance)
    {
        m_transmittance  = new LLGLTexture;
        m_transmittance->generateGLTexture();
        m_transmittance->setAddressMode(LLTexUnit::eTextureAddressMode::TAM_CLAMP);
        m_transmittance->setFilteringOption(LLTexUnit::eTextureFilterOptions::TFO_BILINEAR);
        m_transmittance->setExplicitFormat(GL_RGB16F_ARB, GL_RGB, GL_FLOAT);
        m_transmittance->setTarget(GL_TEXTURE_2D, LLTexUnit::TT_TEXTURE);
    }
    return m_transmittance;
}

LLGLTexture* LLAtmosphere::getScattering()
{
    if (!m_scattering)
    {
        m_scattering = new LLGLTexture;
        m_scattering->generateGLTexture();
        m_scattering->setAddressMode(LLTexUnit::eTextureAddressMode::TAM_CLAMP);
        m_scattering->setFilteringOption(LLTexUnit::eTextureFilterOptions::TFO_BILINEAR);
        m_scattering->setExplicitFormat(GL_RGB16F_ARB, GL_RGB, GL_FLOAT);
        m_scattering->setTarget(GL_TEXTURE_3D, LLTexUnit::TT_TEXTURE_3D);
    }
    return m_scattering;
}

LLGLTexture* LLAtmosphere::getMieScattering()
{
    if (!m_mie_scatter_texture)
    {
        m_mie_scatter_texture = new LLGLTexture;
        m_mie_scatter_texture->generateGLTexture();
        m_mie_scatter_texture->setAddressMode(LLTexUnit::eTextureAddressMode::TAM_CLAMP);
        m_mie_scatter_texture->setFilteringOption(LLTexUnit::eTextureFilterOptions::TFO_BILINEAR);
        m_mie_scatter_texture->setExplicitFormat(GL_RGB16F_ARB, GL_RGB, GL_FLOAT);
        m_mie_scatter_texture->setTarget(GL_TEXTURE_3D, LLTexUnit::TT_TEXTURE_3D);
    }
    return m_mie_scatter_texture;
}

GLhandleARB LLAtmosphere::getAtmosphericShaderForLink() const
{
    return m_model ? m_model->GetShader() : 0;
}
