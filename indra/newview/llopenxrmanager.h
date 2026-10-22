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
#include <string>

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
    // P2-02: runtime-paced frame loop, gated by
    // FS_OPENXR_P2_FRAME_LOOP=1 (default OFF in this commit; P2-03
    // flips the default). When selected, beginPacedFrame() runs the
    // session events + xrWaitFrame + xrLocateViews for the iteration
    // BEFORE LLAppViewer::idle() and publishes the located pose/FOV
    // snapshot for idle() to consume; submitPacedFrameMono() (called
    // from swap(), on the render path) begins the frame if needed,
    // blits this iteration's own render and ends it;
    // closePacedFrameIfOpen() is the main-loop fallback that closes
    // any still-open waited frame with zero layers. The legacy
    // submitFrameMono() path is untouched when the gate is off, and
    // the 2x2 A/B matrix (loop x optics) is preserved: this gate is
    // independent of coherentOpticsEnabled().
    static bool pacedFrameLoopEnabled();
    bool isPacedFrameLoopSelected() const;
    void beginPacedFrame();
    // P2-02: contract-order begin point -- called by
    // LLAppViewer::doFrame() after idle() and before display():
    // begins the waited frame (resolving pending zero-layer ends).
    void beginPacedFrameAfterIdle();
    void submitPacedFrameMono(const LLRect& source_rect);
    void closePacedFrameIfOpen(const char* status);
    bool hasOpenPacedFrame() const;
    bool getHmdForward(LLVector3& out_forward) const;
    bool getHmdUp(LLVector3& out_up) const;
    bool getLeftEyeVerticalFov(F32& out_vertical_fov_radians) const;
    bool getLeftEyeFovSample(LLXRLeftEyeFovSample& out_sample) const;
    bool getLeftEyeAspect(F32& out_aspect) const;
    bool getRecommendedRenderSize(S32& out_width, S32& out_height) const;
    // P1-03: the LEFT eye's render extent (left swapchain size, or the
    // left view's recommended size before swapchains exist). The mono
    // render and its submission are sized by this in coherent mode --
    // never by the max over both eyes.
    bool getLeftEyeRenderSize(S32& out_width, S32& out_height) const;
    // P1-03: coherent XR optics (off-centre render driven by the
    // per-render FOV snapshot + snapshot submission at the native
    // left-eye extent) are the DEFAULT whenever XR mono is active.
    // FS_OPENXR_P1_OPTICS_EXPERIMENTAL set to a false value
    // ("0"/"false"/"no"/"off") selects the legacy Phase 0 path exactly
    // (symmetric render, centre crop, fabricated symmetric FOV) for
    // A/B comparison; unset, empty, true or unparseable values keep
    // the coherent default.
    static bool coherentOpticsEnabled();
    // P2-01B: diagnostic controls for the pre-surgery experiments.
    // Both are opt-in via environment, default OFF, and change
    // nothing when unset. diagFixedCadenceHz() returns the target
    // rate of FS_OPENXR_DIAG_FIXED_CADENCE_HZ (0 when unset or
    // invalid); diagFrameMarkerEnabled() reads
    // FS_OPENXR_DIAG_FRAME_MARKER. diagPaceFrameStart() is called
    // once per main-loop iteration by LLAppViewer::doFrame() BEFORE
    // the iteration's frame token is allocated; when the cadence
    // lock engages (legacy loop only, XR mono enabled, session
    // running) it sleeps until the iteration's absolute deadline on
    // a fixed monotonic grid and reports the achieved wake-to-wake
    // interval and the deadline miss through the out-params.
    static F32 diagFixedCadenceHz();
    static bool diagFrameMarkerEnabled();
    static bool diagPaceFrameStart(F32& out_target_hz, bool& out_has_achieved_interval,
                                   long long& out_achieved_interval_us, long long& out_deadline_miss_us);
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
    // P2-02: shared initialization for both submit loops (one
    // attempt, then the Linux session-handle check).
    bool ensureInitializedForFrame();
    // P2-02: the common submit body. paced=false is the legacy
    // Phase 0/1 loop exactly (wait/begin/locate after the render);
    // paced=true consumes the XRFrameContext beginPacedFrame()
    // waited on and located before idle().
    void submitFrameMonoImpl(const LLRect& source_rect, bool paced);
    // P2-02: post-idle xrBeginFrame for the paced context (resolves
    // pending zero-layer ends); zero-layer end + trace for a begun
    // paced frame. Both are no-ops off Linux/X11.
    bool beginPacedFrameForRender();
    bool endPacedFrameZeroLayer(const std::string& status);

private:
    std::unique_ptr<LLOpenXRManagerImpl> mImpl;
    bool mInitializeAttempted;
    bool mDisabled;
    bool mMonoEnabledOverrideActive;
    bool mMonoEnabledOverrideValue;
};

#endif // LL_LLOPENXRMANAGER_H
