/**
 * @file llopenxrtrace.h
 * @brief Structured JSONL trace for the OpenXR path (Phase 0 diagnostics).
 *
 * P0-01 infrastructure only: run lifecycle, buffered writing, record
 * framing. Render/XR instrumentation records are added by P0-02/P0-03
 * through writeRecord(). The trace is enabled by the environment
 * variable FS_OPENXR_TRACE=1 and is OFF by default; when disabled,
 * every entry point here is a cheap no-op and viewer behaviour is
 * unchanged.
 *
 * Output: /tmp/firestorm_openxr_trace.jsonl (override with
 * FS_OPENXR_TRACE_FILE), one JSON object per line. Every record
 * carries: schema (integer, currently 1), run_id (per XR session),
 * event, monotonic_us (microseconds on a steady clock since the
 * trace epoch -- never mix with XrTime without a verified
 * conversion), and source_commit (the build's commit when known;
 * overridable at runtime with FS_OPENXR_TRACE_COMMIT so an analysis
 * run can label its traces without recompiling).
 */

#ifndef LL_LLOPENXRTRACE_H
#define LL_LLOPENXRTRACE_H

#include <string>

class LLOpenXRTrace
{
public:
    // True when FS_OPENXR_TRACE enables tracing. Cached after first read.
    static bool enabled();

    // Path of the JSONL trace file (FS_OPENXR_TRACE_FILE or default).
    static std::string traceFilePath();

    // Begin a trace run for an XR session. Writes the "session" record.
    // If a run is already active it is ended first (status "superseded").
    // No-op when tracing is disabled. Returns the run id (0 when disabled).
    // extra_fields_json, when non-empty, is a raw JSON fragment (no
    // leading/trailing comma) appended to the "session" record --
    // P2-01B uses it to state the diagnostic cadence mode.
    static unsigned int beginRun(const std::string& mode, const std::string& extra_fields_json = std::string());

    // End the active run: writes an "end" record with the dropped-record
    // count, flushes, and closes the run. No-op without an active run.
    static void endRun(const std::string& status);

    static bool runActive();

    // Frame a record: wraps caller-supplied JSON fields (a fragment like
    // "\"frame_id\":123,\"near\":0.01" -- no leading/trailing comma, may be
    // empty) with the standard envelope and buffers it for writing.
    // Buffer is bounded; records beyond the bound are dropped and counted.
    static void writeRecord(const std::string& event, const std::string& fields_json);

    // Flush buffered records if at least one second has passed since the
    // last flush. Intended to be called once per XR frame; cheap when
    // disabled or when no run is active.
    static void flushIfDue();

    // Flush immediately (used by endRun and shutdown paths).
    static void flush();

    // Monotonic microseconds since the trace epoch (steady clock).
    static long long nowUs();

    // Monotonic per-run sequence for XR frame records (P0-03 assigns
    // xr_frame_id with this). Returns 0 when no run is active.
    static unsigned long long nextXrFrameSeq();

    // Monotonic per-run sequence for render records (P0-02 assigns
    // render_seq with this). Returns 0 when no run is active.
    static unsigned long long nextRenderSeq();

    // P1-01: FOV provenance of the most recently completed render,
    // transferred explicitly to the submission that submits that
    // render's image (association is NEVER inferred from frame
    // numbers). P1-03: the channel is FUNCTIONAL, not trace-only --
    // display_update_camera() stashes every main-world render's
    // snapshot whether or not a trace run is active (with render_seq
    // 0 when untraced; record_render_trace() re-stashes with the real
    // render_seq when a run is active), and takeRenderProvenance() is
    // called once by the submit path and clears the stash (single
    // use: a stale snapshot can never be submitted twice). fov_rad
    // order is the XrFovf order [left, right, down, up], signed
    // radians. offcenter_engaged records whether the render actually
    // installed the XR off-centre projection with this FOV.
    struct RenderProvenance
    {
        bool present = false;
        unsigned long long render_seq = 0;
        bool fov_valid = false;
        unsigned long long fov_generation = 0;
        float fov_rad[4] = {0.f, 0.f, 0.f, 0.f};
        bool offcenter_engaged = false;
        // P2-01: identity of the producing main-loop iteration and the
        // render's true GL state, transferred with the image so the
        // xr_submit record can state exactly which rendered state it
        // submitted (Amendment 4). Matrices are glm column-major
        // (value_ptr order), matching the render record's
        // world_gl_projection / world_gl_view fields.
        unsigned long long app_frame_token = 0;
        bool has_world_gl = false;
        float world_gl_projection[16] = {};
        float world_gl_view[16] = {};
        bool has_gl_rects = false;
        int gl_viewport[4] = {0, 0, 0, 0};
        int gl_scissor[4] = {0, 0, 0, 0};
        bool gl_scissor_enabled = false;
        // Placement fingerprint of the rendered state (see
        // placementFingerprint()); empty when not computed.
        std::string render_fingerprint;
    };
    static void stashRenderProvenance(const RenderProvenance& provenance);
    static bool takeRenderProvenance(RenderProvenance& out);

    // P2-01: functional identity + stage timestamps of the current
    // main-loop iteration. LLAppViewer::doFrame() allocates one
    // app_frame_token per iteration; nextAppFrameToken() is NOT gated
    // on a trace run -- like the render-provenance channel, this is
    // functional frame identity that later phases build on, and the
    // token is process-monotonic so it stays unique across runs in
    // one trace file. The stage marks record when idle(), the world
    // render, and display() ran inside the iteration; the "render",
    // "xr_submit" and per-iteration "app_frame" records carry the
    // token so one iteration's records join exactly, never by
    // sequence-number inference.
    struct AppFrameContext
    {
        bool present = false;
        unsigned long long app_frame_token = 0;
        long long t_mainloop_frame_start_us = 0;
        long long t_idle_start_us = 0;
        long long t_idle_end_us = 0;
        long long t_world_render_start_us = 0;
        long long t_render_end_us = 0;
        long long t_display_return_us = 0;
        // P2-02: which frame loop produced this iteration --
        // "legacy_after_render" (the Phase 0/1 loop, the default) or
        // "paced_pre_idle" (FS_OPENXR_P2_FRAME_LOOP=1). Set by
        // LLOpenXRManager::beginPacedFrame() through
        // setAppFrameLoopMode(); the app_frame, render and xr_submit
        // records all read the label from here.
        std::string frame_loop_mode = "legacy_after_render";
        // P2-01B: diagnostic fixed-cadence state of this iteration,
        // stamped by LLAppViewer::doFrame() when the cadence lock
        // paced the iteration's start (see
        // LLOpenXRManager::diagPaceFrameStart()). The achieved
        // interval is the wake-to-wake spacing of paced frame
        // starts; the first paced iteration has none.
        bool diag_cadence_active = false;
        float diag_cadence_target_hz = 0.f;
        bool has_diag_achieved_interval = false;
        long long diag_achieved_interval_us = 0;
        long long diag_deadline_miss_us = 0;
    };
    static unsigned long long nextAppFrameToken();
    static void beginAppFrame(unsigned long long app_frame_token);
    static void markAppFrameIdleStart();
    static void markAppFrameIdleEnd();
    static void markAppFrameWorldRenderStart();
    static void markAppFrameRenderEnd();
    static void markAppFrameDisplayReturn();
    static void setAppFrameDiagCadence(float target_hz, bool has_achieved_interval,
                                       long long achieved_interval_us, long long deadline_miss_us);
    static void setAppFrameLoopMode(const std::string& mode);
    static bool peekAppFrameContext(AppFrameContext& out);
    static unsigned long long currentAppFrameToken();
    // P2-02: functional (not trace-gated) world-render completion
    // marker: display() records, on its main present path after
    // render_ui(), the app_frame_token whose world render +
    // post-processing completed, so the paced submit can
    // prove the image it is about to submit was actually rendered
    // by the current frame -- the camera-step provenance stash
    // alone (written before the render) cannot prove that.
    static void markWorldRenderCompleted(unsigned long long app_frame_token);
    static unsigned long long lastWorldRenderCompletedToken();

    // P2-01: placement fingerprint -- FNV-1a 64 over a canonical
    // string of numeric state values (floats serialized with the
    // trace's own float32-reproducible formatting, 9 significant
    // digits), returned as 16 lowercase hex digits. Fingerprints
    // group identical placements for A/B analysis; the raw values
    // are always recorded in full alongside, never replaced.
    static std::string placementFingerprint(const std::string& canonical_values);
};

#endif // LL_LLOPENXRTRACE_H
