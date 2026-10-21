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
    static unsigned int beginRun(const std::string& mode);

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
};

#endif // LL_LLOPENXRTRACE_H
