#include "llviewerprecompiledheaders.h"

#include "llopenxrtrace.h"

#include "llstring.h"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>

namespace
{
constexpr const char* OPENXR_TRACE_ENV = "FS_OPENXR_TRACE";
constexpr const char* OPENXR_TRACE_FILE_ENV = "FS_OPENXR_TRACE_FILE";
constexpr const char* OPENXR_TRACE_COMMIT_ENV = "FS_OPENXR_TRACE_COMMIT";
constexpr const char* OPENXR_TRACE_DEFAULT_FILE = "/tmp/firestorm_openxr_trace.jsonl";
constexpr int TRACE_SCHEMA = 1;
constexpr std::size_t TRACE_BUFFER_MAX_RECORDS = 16384;
constexpr long long TRACE_FLUSH_INTERVAL_US = 1000000; // 1 second

struct TraceState
{
    std::mutex mutex;
    bool enabled_cached = false;
    bool enabled_known = false;
    bool run_active = false;
    unsigned int run_id = 0;
    unsigned int next_run_id = 1;
    unsigned long long xr_frame_seq = 0;
    unsigned long long render_seq = 0;
    LLOpenXRTrace::RenderProvenance render_provenance;
    unsigned long long app_frame_token = 0;
    LLOpenXRTrace::AppFrameContext app_frame_context;
    unsigned long long last_world_render_completed_token = 0;
    unsigned long long dropped_records = 0;
    unsigned long long written_records = 0;
    long long last_flush_us = 0;
    std::chrono::steady_clock::time_point epoch = std::chrono::steady_clock::now();
    std::deque<std::string> buffer;
};

TraceState& trace_state()
{
    static TraceState state;
    return state;
}

long long now_us_unlocked(TraceState& state)
{
    const auto elapsed = std::chrono::steady_clock::now() - state.epoch;
    return std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
}

std::string json_escape(const std::string& value)
{
    std::string out;
    out.reserve(value.size() + 2);
    for (char c : value)
    {
        switch (c)
        {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20)
            {
                out += ' ';
            }
            else
            {
                out += c;
            }
            break;
        }
    }
    return out;
}

std::string source_commit()
{
    std::string from_env = LLStringUtil::getenv(OPENXR_TRACE_COMMIT_ENV);
    if (!from_env.empty())
    {
        return from_env;
    }
    return "unspecified";
}

// Caller must hold state.mutex.
void buffer_record_unlocked(TraceState& state, const std::string& line)
{
    if (state.buffer.size() >= TRACE_BUFFER_MAX_RECORDS)
    {
        state.dropped_records++;
        return;
    }
    state.buffer.push_back(line);
}

// Caller must hold state.mutex.
void flush_unlocked(TraceState& state)
{
    if (state.buffer.empty())
    {
        state.last_flush_us = now_us_unlocked(state);
        return;
    }

    std::ofstream out(LLOpenXRTrace::traceFilePath(), std::ios::app);
    if (out.is_open())
    {
        for (const std::string& line : state.buffer)
        {
            out << line << '\n';
            state.written_records++;
        }
    }
    else
    {
        state.dropped_records += state.buffer.size();
    }
    state.buffer.clear();
    state.last_flush_us = now_us_unlocked(state);
}

// Caller must hold state.mutex.
std::string frame_record_unlocked(TraceState& state, const std::string& event,
                                  const std::string& fields_json)
{
    std::string line = "{\"schema\":";
    line += std::to_string(TRACE_SCHEMA);
    line += ",\"run_id\":";
    line += std::to_string(state.run_id);
    line += ",\"event\":\"";
    line += json_escape(event);
    line += "\",\"monotonic_us\":";
    line += std::to_string(now_us_unlocked(state));
    if (!fields_json.empty())
    {
        line += ",";
        line += fields_json;
    }
    line += "}";
    return line;
}
} // namespace

/*static*/ bool LLOpenXRTrace::enabled()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.enabled_known)
    {
        std::string value = LLStringUtil::getenv(OPENXR_TRACE_ENV);
        bool parsed = false;
        if (!value.empty() && LLStringUtil::convertToBOOL(value, parsed))
        {
            state.enabled_cached = parsed;
        }
        else
        {
            state.enabled_cached = !value.empty();
        }
        state.enabled_known = true;
    }
    return state.enabled_cached;
}

/*static*/ std::string LLOpenXRTrace::traceFilePath()
{
    std::string configured = LLStringUtil::getenv(OPENXR_TRACE_FILE_ENV);
    if (!configured.empty())
    {
        return configured;
    }
    return OPENXR_TRACE_DEFAULT_FILE;
}

/*static*/ unsigned int LLOpenXRTrace::beginRun(const std::string& mode, const std::string& extra_fields_json)
{
    if (!enabled())
    {
        return 0;
    }

    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);

    if (state.run_active)
    {
        // Close the previous run inline (endRun() would re-take the mutex).
        const std::string end_line = frame_record_unlocked(
            state, "end", "\"status\":\"superseded\",\"dropped_records\":" +
                              std::to_string(state.dropped_records));
        buffer_record_unlocked(state, end_line);
        flush_unlocked(state);
        state.run_active = false;
    }

    state.run_id = state.next_run_id++;
    state.run_active = true;
    state.xr_frame_seq = 0;
    state.render_seq = 0;
    state.render_provenance = RenderProvenance();
    state.dropped_records = 0;
    state.written_records = 0;

    std::string fields = "\"mode\":\"" + json_escape(mode) +
                         "\",\"source_commit\":\"" + json_escape(source_commit()) +
                         "\",\"trace_file\":\"" + json_escape(traceFilePath()) + "\"";
    if (!extra_fields_json.empty())
    {
        fields += "," + extra_fields_json;
    }
    buffer_record_unlocked(state, frame_record_unlocked(state, "session", fields));
    flush_unlocked(state);
    return state.run_id;
}

/*static*/ void LLOpenXRTrace::endRun(const std::string& status)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.run_active)
    {
        return;
    }

    std::string fields = "\"status\":\"" + json_escape(status) +
                         "\",\"dropped_records\":" + std::to_string(state.dropped_records) +
                         ",\"written_records\":" + std::to_string(state.written_records);
    buffer_record_unlocked(state, frame_record_unlocked(state, "end", fields));
    state.run_active = false;
    flush_unlocked(state);
}

/*static*/ bool LLOpenXRTrace::runActive()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.run_active;
}

/*static*/ void LLOpenXRTrace::writeRecord(const std::string& event, const std::string& fields_json)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.run_active)
    {
        return;
    }
    buffer_record_unlocked(state, frame_record_unlocked(state, event, fields_json));
}

/*static*/ void LLOpenXRTrace::flushIfDue()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.run_active || state.buffer.empty())
    {
        return;
    }
    if (now_us_unlocked(state) - state.last_flush_us >= TRACE_FLUSH_INTERVAL_US)
    {
        flush_unlocked(state);
    }
}

/*static*/ void LLOpenXRTrace::flush()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    flush_unlocked(state);
}

/*static*/ long long LLOpenXRTrace::nowUs()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    return now_us_unlocked(state);
}

/*static*/ unsigned long long LLOpenXRTrace::nextRenderSeq()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.run_active)
    {
        return 0;
    }
    return ++state.render_seq;
}

/*static*/ void LLOpenXRTrace::stashRenderProvenance(const RenderProvenance& provenance)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    // P1-03: no run_active gate -- the coherent submit path consumes
    // this snapshot functionally, with or without a trace run.
    state.render_provenance = provenance;
    state.render_provenance.present = true;
}

/*static*/ bool LLOpenXRTrace::takeRenderProvenance(RenderProvenance& out)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.render_provenance.present)
    {
        return false;
    }
    out = state.render_provenance;
    state.render_provenance = RenderProvenance();
    return true;
}

/*static*/ unsigned long long LLOpenXRTrace::nextAppFrameToken()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    // Process-monotonic and not reset by beginRun(): a token names
    // one main-loop iteration uniquely for the whole trace file.
    return ++state.app_frame_token;
}

/*static*/ void LLOpenXRTrace::beginAppFrame(unsigned long long app_frame_token)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.app_frame_context = AppFrameContext();
    state.app_frame_context.present = true;
    state.app_frame_context.app_frame_token = app_frame_token;
    state.app_frame_context.t_mainloop_frame_start_us = now_us_unlocked(state);
}

/*static*/ void LLOpenXRTrace::markAppFrameIdleStart()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.app_frame_context.present)
    {
        state.app_frame_context.t_idle_start_us = now_us_unlocked(state);
    }
}

/*static*/ void LLOpenXRTrace::markAppFrameIdleEnd()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.app_frame_context.present)
    {
        state.app_frame_context.t_idle_end_us = now_us_unlocked(state);
    }
}

/*static*/ void LLOpenXRTrace::markAppFrameWorldRenderStart()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.app_frame_context.present)
    {
        state.app_frame_context.t_world_render_start_us = now_us_unlocked(state);
    }
}

/*static*/ void LLOpenXRTrace::markAppFrameRenderEnd()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.app_frame_context.present)
    {
        state.app_frame_context.t_render_end_us = now_us_unlocked(state);
    }
}

/*static*/ void LLOpenXRTrace::markAppFrameDisplayReturn()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.app_frame_context.present)
    {
        state.app_frame_context.t_display_return_us = now_us_unlocked(state);
    }
}

/*static*/ void LLOpenXRTrace::setAppFrameDiagCadence(float target_hz, bool has_achieved_interval,
                                                      long long achieved_interval_us, long long deadline_miss_us)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.app_frame_context.present)
    {
        state.app_frame_context.diag_cadence_active = true;
        state.app_frame_context.diag_cadence_target_hz = target_hz;
        state.app_frame_context.has_diag_achieved_interval = has_achieved_interval;
        state.app_frame_context.diag_achieved_interval_us = achieved_interval_us;
        state.app_frame_context.diag_deadline_miss_us = deadline_miss_us;
    }
}

/*static*/ void LLOpenXRTrace::setAppFrameLoopMode(const std::string& mode)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.app_frame_context.present)
    {
        state.app_frame_context.frame_loop_mode = mode;
    }
}

/*static*/ void LLOpenXRTrace::markWorldRenderCompleted(unsigned long long app_frame_token)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    state.last_world_render_completed_token = app_frame_token;
}

/*static*/ unsigned long long LLOpenXRTrace::lastWorldRenderCompletedToken()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.last_world_render_completed_token;
}

/*static*/ bool LLOpenXRTrace::peekAppFrameContext(AppFrameContext& out)
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.app_frame_context.present)
    {
        return false;
    }
    out = state.app_frame_context;
    return true;
}

/*static*/ unsigned long long LLOpenXRTrace::currentAppFrameToken()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    return state.app_frame_context.present ? state.app_frame_context.app_frame_token : 0;
}

/*static*/ std::string LLOpenXRTrace::placementFingerprint(const std::string& canonical_values)
{
    // FNV-1a 64-bit over the canonical value string.
    unsigned long long hash = 1469598103934665603ULL;
    for (char c : canonical_values)
    {
        hash ^= (unsigned long long)(unsigned char)c;
        hash *= 1099511628211ULL;
    }
    std::ostringstream oss;
    oss << std::hex << std::setfill('0') << std::setw(16) << hash;
    return oss.str();
}

/*static*/ unsigned long long LLOpenXRTrace::nextXrFrameSeq()
{
    TraceState& state = trace_state();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (!state.run_active)
    {
        return 0;
    }
    return ++state.xr_frame_seq;
}
