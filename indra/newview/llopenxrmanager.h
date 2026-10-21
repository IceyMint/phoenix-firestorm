/**
 * @file llopenxrmanager.h
 * @brief Minimal OpenXR mono-submit manager.
 *
 * This manager is intentionally simple: when enabled, it submits the same
 * rendered backbuffer image to both eye swapchains.
 */

#ifndef LL_LLOPENXRMANAGER_H
#define LL_LLOPENXRMANAGER_H

#include "llrect.h"
#include "v3math.h"

#include <memory>

// P1-01: one located left-eye FOV sample. Cached by submitFrameMono
// after a successful xrLocateViews that passed the P0-06 validity
// checks; locate_generation increments on every valid replacement and
// located_monotonic_us is on the LLOpenXRTrace::nowUs() steady clock.
// The sample is invalidated on shutdown and on locate failure; it is
// never a synthetic substitute. Angles are signed radians in XrFovf
// convention (left/down negative, right/up positive).
struct LLXRLeftEyeFovSample
{
    F32 left = 0.f;
    F32 right = 0.f;
    F32 down = 0.f;
    F32 up = 0.f;
    unsigned long long locate_generation = 0;
    long long located_monotonic_us = 0;
    bool valid = false;
};

struct LLOpenXRManagerImpl;

class LLOpenXRManager
{
public:
    static LLOpenXRManager& instance();

    void submitFrameMono(const LLRect& source_rect);
    bool getHmdForward(LLVector3& out_forward) const;
    bool getHmdUp(LLVector3& out_up) const;
    bool getLeftEyeVerticalFov(F32& out_vertical_fov_radians) const;
    bool getLeftEyeFovSample(LLXRLeftEyeFovSample& out_sample) const;
    bool getLeftEyeAspect(F32& out_aspect) const;
    bool getRecommendedRenderSize(S32& out_width, S32& out_height) const;
    void setMonoEnabled(bool enabled);
    bool isMonoEnabled() const;
    bool isSessionRunning() const;
    void shutdown();

private:
    LLOpenXRManager();
    ~LLOpenXRManager();

    bool initialize();
    void processEvents();
    bool isMonoRequested() const;

private:
    std::unique_ptr<LLOpenXRManagerImpl> mImpl;
    bool mInitializeAttempted;
    bool mDisabled;
    bool mMonoEnabledOverrideActive;
    bool mMonoEnabledOverrideValue;
};

#endif // LL_LLOPENXRMANAGER_H
