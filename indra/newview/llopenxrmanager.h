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

struct LLOpenXRManagerImpl;

class LLOpenXRManager
{
public:
    static LLOpenXRManager& instance();

    void submitFrameMono(const LLRect& source_rect);
    bool getHmdForward(LLVector3& out_forward) const;
    bool getHmdUp(LLVector3& out_up) const;
    bool getLeftEyeVerticalFov(F32& out_vertical_fov_radians) const;
    bool getLeftEyeAspect(F32& out_aspect) const;
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
