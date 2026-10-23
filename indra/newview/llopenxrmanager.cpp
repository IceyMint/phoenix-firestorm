#include "llviewerprecompiledheaders.h"

#include "llopenxrmanager.h"

#include "llopenxrtrace.h"

#include "llerror.h"
#include "llrect.h"
#include "llstring.h"

#include "llglheaders.h"

#include <chrono>
#include <cstdlib>
#include <thread>

#if LL_LINUX && LL_X11
#define XR_USE_GRAPHICS_API_OPENGL
#define XR_USE_PLATFORM_XLIB
#include <openxr/openxr_platform.h>

#include "llviewerdisplay.h"
#include "llagentcamera.h"
#include "llviewercamera.h"
#include "pipeline.h"
#include "llviewerwindow.h"
#include "llwindowsdl.h"

#include <GL/glx.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#endif

namespace
{
constexpr const char* OPENXR_MONO_ENV = "FS_OPENXR_MONO";
constexpr const char* OPENXR_LOG_ENV = "FS_OPENXR_LOG_FILE";
constexpr const char* OPENXR_P1_OPTICS_ENV = "FS_OPENXR_P1_OPTICS_EXPERIMENTAL";
constexpr const char* OPENXR_DIAG_CADENCE_ENV = "FS_OPENXR_DIAG_FIXED_CADENCE_HZ";
constexpr const char* OPENXR_DIAG_MARKER_ENV = "FS_OPENXR_DIAG_FRAME_MARKER";
constexpr const char* OPENXR_P2_FRAME_LOOP_ENV = "FS_OPENXR_P2_FRAME_LOOP";

bool is_openxr_mono_requested()
{
    std::string value = LLStringUtil::getenv(OPENXR_MONO_ENV);
    if (value.empty())
    {
        return false;
    }

    bool result = false;
    if (LLStringUtil::convertToBOOL(value, result))
    {
        return result;
    }

    return true;
}

bool xr_coherent_optics_enabled()
{
    // P1-03: coherent optics are the default. Only an explicit,
    // parseable false disables them (the legacy Phase 0 A/B override)
    // -- the same default-on convention as the FS_OPENXR_DECOUPLED_
    // RENDER / FS_OPENXR_NATIVE_RES readers.
    std::string value = LLStringUtil::getenv(OPENXR_P1_OPTICS_ENV);
    if (value.empty())
    {
        return true;
    }

    bool requested = true;
    if (LLStringUtil::convertToBOOL(value, requested))
    {
        return requested;
    }

    return true;
}

// P2-01B: FS_OPENXR_DIAG_FIXED_CADENCE_HZ parser. The whole value
// must be a single float in (0, 1000]; anything else (empty,
// trailing junk, out of range) leaves the lock disabled.
bool parse_diag_cadence_hz(const std::string& raw, F32& out_hz)
{
    if (raw.empty())
    {
        return false;
    }
    char* end = nullptr;
    const float parsed = std::strtof(raw.c_str(), &end);
    if (end == raw.c_str() || *end != '\0' || !(parsed > 0.f) || parsed > 1000.f)
    {
        return false;
    }
    out_hz = parsed;
    return true;
}

// P2-01B: FS_OPENXR_DIAG_FRAME_MARKER reader (same convention as
// the mono request: unset/empty = off, unparseable non-empty = on).
bool diag_frame_marker_requested()
{
    std::string value = LLStringUtil::getenv(OPENXR_DIAG_MARKER_ENV);
    if (value.empty())
    {
        return false;
    }

    bool result = false;
    if (LLStringUtil::convertToBOOL(value, result))
    {
        return result;
    }

    return true;
}

// P2-02: FS_OPENXR_P2_FRAME_LOOP gate reader. P2-03b: the paced
// loop is the default -- unset, empty or unparseable select it;
// an explicit parseable value wins, so FS_OPENXR_P2_FRAME_LOOP=0
// remains the legacy escape hatch. Launch-time configuration,
// read on each call like the other FS_OPENXR_* readers.
bool xr_paced_frame_loop_requested()
{
    std::string value = LLStringUtil::getenv(OPENXR_P2_FRAME_LOOP_ENV);
    if (value.empty())
    {
        return true;
    }

    bool result = false;
    if (LLStringUtil::convertToBOOL(value, result))
    {
        return result;
    }

    return true;
}

// P2-01B: legacy-loop gate hook for the diagnostic cadence lock,
// updated for P2-02: the lock engages only when the paced loop is
// NOT selected -- the paced loop is paced by its own xrWaitFrame,
// and the lock is a legacy-loop diagnostic by contract.
bool xr_diag_frame_loop_is_legacy()
{
    // Aligned with isPacedFrameLoopSelected(): the paced loop runs
    // only when the gate is on AND mono XR is requested; in every
    // other case the legacy loop is what actually runs and the
    // cadence lock may engage.
    return !LLOpenXRManager::instance().isPacedFrameLoopSelected();
}
}

/*static*/ bool LLOpenXRManager::coherentOpticsEnabled()
{
    return xr_coherent_optics_enabled();
}

/*static*/ bool LLOpenXRManager::pacedFrameLoopEnabled()
{
    return xr_paced_frame_loop_requested();
}

/*static*/ F32 LLOpenXRManager::diagFixedCadenceHz()
{
    // Parsed once: the diagnostic is launch-time configuration.
    static const F32 cached_hz = [] {
        F32 hz = 0.f;
        parse_diag_cadence_hz(LLStringUtil::getenv(OPENXR_DIAG_CADENCE_ENV), hz);
        return hz;
    }();
    return cached_hz;
}

/*static*/ bool LLOpenXRManager::diagFrameMarkerEnabled()
{
    return diag_frame_marker_requested();
}

#if LL_LINUX && LL_X11
namespace
{
constexpr XrViewConfigurationType XR_VIEW_CONFIG = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

// P2-01B: diagnostic cadence-lock grid state. Main thread only
// (diagPaceFrameStart() is called from LLAppViewer::doFrame()).
// Deadlines live on an absolute grid anchored at engagement:
// deadline_k = anchor + k * period. Slots lost to an overrun are
// skipped -- the lock never bursts to catch up.
struct DiagCadenceGrid
{
    bool anchored = false;
    std::chrono::steady_clock::time_point next_deadline{};
    std::chrono::steady_clock::time_point last_wake{};
    bool has_last_wake = false;
};
DiagCadenceGrid g_diag_cadence_grid;

std::string openxr_log_file_path()
{
    std::string configured_path = LLStringUtil::getenv(OPENXR_LOG_ENV);
    if (!configured_path.empty())
    {
        return configured_path;
    }
    return "/tmp/firestorm_openxr.log";
}

void write_openxr_file_log(const char* level, const std::string& message)
{
    static std::mutex log_mutex;
    std::lock_guard<std::mutex> lock(log_mutex);

    std::ofstream log_file(openxr_log_file_path(), std::ios::app);
    if (!log_file.is_open())
    {
        return;
    }

    std::time_t now = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);
    char timestamp[32] = {};
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &local_tm);
    log_file << timestamp << " [" << level << "] " << message << '\n';
}

void openxr_info(const std::string& message)
{
    LL_INFOS("OpenXR") << message << LL_ENDL;
    write_openxr_file_log("INFO", message);
}

void openxr_warn(const std::string& message)
{
    LL_WARNS("OpenXR") << message << LL_ENDL;
    write_openxr_file_log("WARN", message);
}

std::string xr_session_state_name(XrSessionState state)
{
    switch (state)
    {
    case XR_SESSION_STATE_UNKNOWN: return "UNKNOWN";
    case XR_SESSION_STATE_IDLE: return "IDLE";
    case XR_SESSION_STATE_READY: return "READY";
    case XR_SESSION_STATE_SYNCHRONIZED: return "SYNCHRONIZED";
    case XR_SESSION_STATE_VISIBLE: return "VISIBLE";
    case XR_SESSION_STATE_FOCUSED: return "FOCUSED";
    case XR_SESSION_STATE_STOPPING: return "STOPPING";
    case XR_SESSION_STATE_LOSS_PENDING: return "LOSS_PENDING";
    case XR_SESSION_STATE_EXITING: return "EXITING";
    case XR_SESSION_STATE_MAX_ENUM: return "MAX_ENUM";
    default: return "UNRECOGNIZED";
    }
}

std::string xr_reference_space_type_name(XrReferenceSpaceType space_type)
{
    switch (space_type)
    {
    case XR_REFERENCE_SPACE_TYPE_VIEW: return "VIEW";
    case XR_REFERENCE_SPACE_TYPE_LOCAL: return "LOCAL";
    case XR_REFERENCE_SPACE_TYPE_STAGE: return "STAGE";
    case XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR: return "LOCAL_FLOOR";
    case XR_REFERENCE_SPACE_TYPE_UNBOUNDED_MSFT: return "UNBOUNDED_MSFT";
    case XR_REFERENCE_SPACE_TYPE_COMBINED_EYE_VARJO: return "COMBINED_EYE_VARJO";
    case XR_REFERENCE_SPACE_TYPE_MAX_ENUM: return "MAX_ENUM";
    default: return "UNRECOGNIZED";
    }
}

std::string format_xr_result(XrInstance instance, XrResult result)
{
    if (instance != XR_NULL_HANDLE)
    {
        std::array<char, XR_MAX_RESULT_STRING_SIZE> buffer{};
        if (XR_SUCCEEDED(xrResultToString(instance, result, buffer.data())))
        {
            return std::string(buffer.data());
        }
    }
    return "XrResult(" + std::to_string(result) + ")";
}

template <typename T>
void init_xr_struct(T& value, XrStructureType type)
{
    value = {};
    value.type = type;
    value.next = nullptr;
}

bool has_swapchain_format(const std::vector<int64_t>& available_formats, int64_t desired_format)
{
    return std::find(available_formats.begin(), available_formats.end(), desired_format) != available_formats.end();
}

int64_t choose_swapchain_format(const std::vector<int64_t>& available_formats)
{
    const std::array<int64_t, 4> preferred_formats =
    {
        GL_SRGB8_ALPHA8,
        GL_RGBA8,
        GL_RGB10_A2,
        GL_RGBA16F
    };

    for (int64_t format : preferred_formats)
    {
        if (has_swapchain_format(available_formats, format))
        {
            return format;
        }
    }

    if (!available_formats.empty())
    {
        return available_formats.front();
    }

    return GL_RGBA8;
}

LLVector3 xr_forward_from_orientation(const XrQuaternionf& orientation)
{
    const LLQuaternion rotation(orientation.x, orientation.y, orientation.z, orientation.w);
    LLVector3 forward = LLVector3(0.f, 0.f, -1.f) * rotation;
    forward.normalize();
    return forward;
}

LLVector3 xr_up_from_orientation(const XrQuaternionf& orientation)
{
    const LLQuaternion rotation(orientation.x, orientation.y, orientation.z, orientation.w);
    LLVector3 up = LLVector3(0.f, 1.f, 0.f) * rotation;
    up.normalize();
    return up;
}

XrPosef xr_center_pose_from_views(const XrView& left_view, const XrView& right_view)
{
    XrPosef pose{};

    // Average positions between eyes to approximate a center-eye pose.
    pose.position.x = 0.5f * (left_view.pose.position.x + right_view.pose.position.x);
    pose.position.y = 0.5f * (left_view.pose.position.y + right_view.pose.position.y);
    pose.position.z = 0.5f * (left_view.pose.position.z + right_view.pose.position.z);

    // Average orientation quaternions with sign correction to avoid antipodal cancelation.
    F32 qx0 = left_view.pose.orientation.x;
    F32 qy0 = left_view.pose.orientation.y;
    F32 qz0 = left_view.pose.orientation.z;
    F32 qw0 = left_view.pose.orientation.w;
    F32 qx1 = right_view.pose.orientation.x;
    F32 qy1 = right_view.pose.orientation.y;
    F32 qz1 = right_view.pose.orientation.z;
    F32 qw1 = right_view.pose.orientation.w;

    const F32 dot = qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1;
    if (dot < 0.f)
    {
        qx1 = -qx1;
        qy1 = -qy1;
        qz1 = -qz1;
        qw1 = -qw1;
    }

    F32 qx = qx0 + qx1;
    F32 qy = qy0 + qy1;
    F32 qz = qz0 + qz1;
    F32 qw = qw0 + qw1;
    const F32 len_sq = qx * qx + qy * qy + qz * qz + qw * qw;
    if (len_sq > 0.f)
    {
        const F32 inv_len = 1.f / sqrtf(len_sq);
        qx *= inv_len;
        qy *= inv_len;
        qz *= inv_len;
        qw *= inv_len;
    }
    else
    {
        qx = qx0;
        qy = qy0;
        qz = qz0;
        qw = qw0;
    }

    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

bool xr_fov_aspect(const XrFovf& fov, F32& out_aspect)
{
    const F32 horizontal_extent = tanf(fov.angleRight) + tanf(-fov.angleLeft);
    const F32 vertical_extent = tanf(fov.angleUp) + tanf(-fov.angleDown);
    if (!std::isfinite(horizontal_extent) || !std::isfinite(vertical_extent) ||
        horizontal_extent <= 0.f || vertical_extent <= 0.f)
    {
        return false;
    }

    out_aspect = horizontal_extent / vertical_extent;
    return std::isfinite(out_aspect) && out_aspect > 0.f;
}

// P1-03: the Phase 0 centre-crop and FOV-fabrication helpers survive
// VERBATIM under legacy_* names, called ONLY from submitFrameMono's
// legacy branch (the FS_OPENXR_P1_OPTICS_EXPERIMENTAL=0 A/B override).
// The coherent path never crops and never fabricates an FOV.
LLRect legacy_center_crop_rect_to_aspect(const LLRect& source_rect, F32 target_aspect)
{
    if (!std::isfinite(target_aspect) || target_aspect <= 0.f)
    {
        return source_rect;
    }

    const S32 source_width = llmax(source_rect.getWidth(), 1);
    const S32 source_height = llmax(source_rect.getHeight(), 1);
    const F32 source_aspect = (F32)source_width / (F32)source_height;
    if (fabsf(source_aspect - target_aspect) < 0.0001f)
    {
        return source_rect;
    }

    LLRect cropped_rect = source_rect;
    if (source_aspect > target_aspect)
    {
        const S32 target_width = llclamp((S32)((F32)source_height * target_aspect + 0.5f), 1, source_width);
        const S32 inset_x = (source_width - target_width) / 2;
        cropped_rect.mLeft = source_rect.mLeft + inset_x;
        cropped_rect.mRight = cropped_rect.mLeft + target_width;
    }
    else
    {
        const S32 target_height = llclamp((S32)((F32)source_width / target_aspect + 0.5f), 1, source_height);
        const S32 inset_y = (source_height - target_height) / 2;
        cropped_rect.mBottom = source_rect.mBottom + inset_y;
        cropped_rect.mTop = cropped_rect.mBottom + target_height;
    }

    return cropped_rect;
}

XrFovf legacy_xr_centered_mono_fov_for_source_crop(const XrFovf& reference_view_fov,
                                            const LLRect& full_source_rect,
                                            const LLRect& cropped_source_rect)
{
    const F32 full_width = llmax((F32)full_source_rect.getWidth(), 1.f);
    const F32 full_height = llmax((F32)full_source_rect.getHeight(), 1.f);
    const F32 crop_width = llclamp((F32)cropped_source_rect.getWidth(), 1.f, full_width);
    const F32 crop_height = llclamp((F32)cropped_source_rect.getHeight(), 1.f, full_height);
    const F32 full_aspect = full_width / full_height;
    const F32 crop_x = llclamp(crop_width / full_width, 0.01f, 1.f);
    const F32 crop_y = llclamp(crop_height / full_height, 0.01f, 1.f);
    const F32 source_vertical_fov = llclamp(reference_view_fov.angleUp - reference_view_fov.angleDown, 0.02f, 3.12f);
    const F32 source_vertical_half = llclamp(source_vertical_fov * 0.5f, 0.01f, 1.56f);
    const F32 source_horizontal_half = llclamp(
        atanf(tanf(source_vertical_half) * llmax(full_aspect, 0.01f)),
        0.01f,
        1.56f);
    const F32 mono_vertical_half = llclamp(atanf(tanf(source_vertical_half) * crop_y), 0.01f, 1.56f);
    const F32 mono_horizontal_half = llclamp(atanf(tanf(source_horizontal_half) * crop_x), 0.01f, 1.56f);

    XrFovf fov{};
    fov.angleLeft = -mono_horizontal_half;
    fov.angleRight = mono_horizontal_half;
    fov.angleUp = mono_vertical_half;
    fov.angleDown = -mono_vertical_half;
    return fov;
}

bool get_openxr_cursor_position_raw(S32& out_x_raw, S32& out_y_raw)
{
    if (!gViewerWindow || gViewerWindow->getCursorHidden())
    {
        return false;
    }

    const S32 window_width_scaled = gViewerWindow->getWindowWidthScaled();
    const S32 window_height_scaled = gViewerWindow->getWindowHeightScaled();
    const S32 window_width_raw = gViewerWindow->getWindowWidthRaw();
    const S32 window_height_raw = gViewerWindow->getWindowHeightRaw();
    if (window_width_scaled <= 0 || window_height_scaled <= 0 || window_width_raw <= 0 || window_height_raw <= 0)
    {
        return false;
    }

    const LLCoordGL mouse_scaled = gViewerWindow->getCurrentMouse();
    out_x_raw = (S32)std::lround((F32)mouse_scaled.mX * (F32)window_width_raw / (F32)window_width_scaled);
    out_y_raw = (S32)std::lround((F32)mouse_scaled.mY * (F32)window_height_raw / (F32)window_height_scaled);
    return true;
}

void clear_scissored_rect(S32 framebuffer_width,
                          S32 framebuffer_height,
                          S32 left,
                          S32 bottom,
                          S32 right,
                          S32 top,
                          F32 r,
                          F32 g,
                          F32 b,
                          F32 a)
{
    left = llclamp(left, 0, framebuffer_width);
    right = llclamp(right, 0, framebuffer_width);
    bottom = llclamp(bottom, 0, framebuffer_height);
    top = llclamp(top, 0, framebuffer_height);
    if (right <= left || top <= bottom)
    {
        return;
    }

    glScissor(left, bottom, right - left, top - bottom);
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT);
}

void draw_openxr_cursor_overlay(const LLRect& source_rect, uint32_t framebuffer_width, uint32_t framebuffer_height)
{
    const S32 dst_width = llmax(0, (S32)framebuffer_width);
    const S32 dst_height = llmax(0, (S32)framebuffer_height);
    if (dst_width <= 0 || dst_height <= 0)
    {
        return;
    }

    S32 cursor_x_raw = 0;
    S32 cursor_y_raw = 0;
    if (!get_openxr_cursor_position_raw(cursor_x_raw, cursor_y_raw))
    {
        return;
    }

    const S32 src_width = llmax(source_rect.getWidth(), 1);
    const S32 src_height = llmax(source_rect.getHeight(), 1);
    if (cursor_x_raw < source_rect.mLeft || cursor_x_raw >= source_rect.mRight ||
        cursor_y_raw < source_rect.mBottom || cursor_y_raw >= source_rect.mTop)
    {
        return;
    }

    const F32 u = ((F32)cursor_x_raw - (F32)source_rect.mLeft) / (F32)src_width;
    const F32 v = ((F32)cursor_y_raw - (F32)source_rect.mBottom) / (F32)src_height;
    const S32 cursor_x = llclamp((S32)std::lround(u * (F32)dst_width), 0, dst_width - 1);
    const S32 cursor_y = llclamp((S32)std::lround(v * (F32)dst_height), 0, dst_height - 1);

    const S32 short_dim = llmax(1, (S32)std::min(framebuffer_width, framebuffer_height));
    const S32 inner_half = llclamp((S32)std::lround((F32)short_dim * 0.0125f), 6, 18);
    const S32 inner_thickness = llclamp((S32)std::lround((F32)short_dim * 0.0025f), 1, 3);
    const S32 outer_half = inner_half + 2;
    const S32 outer_thickness = inner_thickness + 2;

    GLboolean scissor_was_enabled = glIsEnabled(GL_SCISSOR_TEST);
    GLint previous_scissor_box[4] = {};
    glGetIntegerv(GL_SCISSOR_BOX, previous_scissor_box);
    GLboolean previous_color_mask[4] = {};
    glGetBooleanv(GL_COLOR_WRITEMASK, previous_color_mask);
    GLfloat previous_clear_color[4] = {};
    glGetFloatv(GL_COLOR_CLEAR_VALUE, previous_clear_color);

    glEnable(GL_SCISSOR_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Draw a high-contrast crosshair cursor (black outline, white center).
    clear_scissored_rect(dst_width, dst_height,
                         cursor_x - outer_thickness / 2, cursor_y - outer_half,
                         cursor_x + (outer_thickness + 1) / 2, cursor_y + outer_half + 1,
                         0.f, 0.f, 0.f, 1.f);
    clear_scissored_rect(dst_width, dst_height,
                         cursor_x - outer_half, cursor_y - outer_thickness / 2,
                         cursor_x + outer_half + 1, cursor_y + (outer_thickness + 1) / 2,
                         0.f, 0.f, 0.f, 1.f);
    clear_scissored_rect(dst_width, dst_height,
                         cursor_x - inner_thickness / 2, cursor_y - inner_half,
                         cursor_x + (inner_thickness + 1) / 2, cursor_y + inner_half + 1,
                         1.f, 1.f, 1.f, 1.f);
    clear_scissored_rect(dst_width, dst_height,
                         cursor_x - inner_half, cursor_y - inner_thickness / 2,
                         cursor_x + inner_half + 1, cursor_y + (inner_thickness + 1) / 2,
                         1.f, 1.f, 1.f, 1.f);

    glScissor(previous_scissor_box[0], previous_scissor_box[1], previous_scissor_box[2], previous_scissor_box[3]);
    glColorMask(previous_color_mask[0], previous_color_mask[1], previous_color_mask[2], previous_color_mask[3]);
    glClearColor(previous_clear_color[0], previous_clear_color[1], previous_clear_color[2], previous_clear_color[3]);
    if (!scissor_was_enabled)
    {
        glDisable(GL_SCISSOR_TEST);
    }
}

// P2-01B: diagnostic frame marker (FS_OPENXR_DIAG_FRAME_MARKER=1).
// Draws a chunky Gray-coded encoding of a 10-bit marker value into
// the swapchain image currently attached to the blit FBO. It is
// called at the submission blit point in submitFrameMono, AFTER the
// world blit and after the cursor overlay (so nothing overdraws
// it), once per eye; it never touches the desktop backbuffer.
// Layout, in GL bottom-left-origin image coordinates:
//   - cell size 40x40 px; grid 14 columns x 2 rows (560x80 px);
//   - grid centred horizontally; its bottom edge sits 160 px above
//     the image's bottom edge (inside the lens-visible area, away
//     from the corners the circular view crops);
//   - row 0 (lower): [1][0][g9..g0][parity][1] -- a white sync
//     cell, a black sync cell, the 10 Gray-code bits of the value
//     (g = v ^ (v >> 1)) MSB first, one even-parity bit over those
//     10 bits, and a white sync cell;
//   - row 1 (upper): the exact inverse of row 0, as a consistency
//     check under compression.
// Cells are pure white (1) or pure black (0), drawn with the same
// scissored clears as the cursor overlay. At the Quest recording's
// 720 px width (source 3096 px, ~0.23x) one cell is ~9 recording
// px, big enough to sample at cell centres. Decoder: read row 0's
// cell centres left to right; verify the sync cells (1, 0, ..., 1),
// row 1 == NOT row 0, and even parity; un-Gray (b9 = g9,
// bi = b(i+1) ^ gi) to recover v; match v against the xr_submit
// records' diagnostic_marker_value in the same run.
void draw_openxr_diag_frame_marker(uint32_t framebuffer_width, uint32_t framebuffer_height, unsigned int marker_value)
{
    const S32 dst_width = llmax(0, (S32)framebuffer_width);
    const S32 dst_height = llmax(0, (S32)framebuffer_height);
    constexpr S32 cell_size = 40;
    constexpr S32 columns = 14;
    constexpr S32 rows = 2;
    constexpr S32 bottom_inset = 160;
    const S32 grid_width = cell_size * columns;
    const S32 grid_height = cell_size * rows;
    if (dst_width < grid_width || dst_height < bottom_inset + grid_height)
    {
        return;
    }

    const unsigned int value = marker_value & 0x3FF;
    const unsigned int gray = value ^ (value >> 1);
    int cells[columns];
    cells[0] = 1;
    cells[1] = 0;
    int parity = 0;
    for (int i = 0; i < 10; i++)
    {
        const int bit = (int)((gray >> (9 - i)) & 1u);
        cells[2 + i] = bit;
        parity ^= bit;
    }
    cells[12] = parity;
    cells[13] = 1;

    const S32 origin_x = (dst_width - grid_width) / 2;
    const S32 origin_y = bottom_inset;

    GLboolean scissor_was_enabled = glIsEnabled(GL_SCISSOR_TEST);
    GLint previous_scissor_box[4] = {};
    glGetIntegerv(GL_SCISSOR_BOX, previous_scissor_box);
    GLboolean previous_color_mask[4] = {};
    glGetBooleanv(GL_COLOR_WRITEMASK, previous_color_mask);
    GLfloat previous_clear_color[4] = {};
    glGetFloatv(GL_COLOR_CLEAR_VALUE, previous_clear_color);

    glEnable(GL_SCISSOR_TEST);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    for (S32 row = 0; row < rows; row++)
    {
        for (S32 col = 0; col < columns; col++)
        {
            const int bit = cells[col] ^ (row == 1 ? 1 : 0);
            const F32 level = bit ? 1.f : 0.f;
            clear_scissored_rect(dst_width, dst_height,
                                 origin_x + col * cell_size, origin_y + row * cell_size,
                                 origin_x + (col + 1) * cell_size, origin_y + (row + 1) * cell_size,
                                 level, level, level, 1.f);
        }
    }

    glScissor(previous_scissor_box[0], previous_scissor_box[1], previous_scissor_box[2], previous_scissor_box[3]);
    glColorMask(previous_color_mask[0], previous_color_mask[1], previous_color_mask[2], previous_color_mask[3]);
    glClearColor(previous_clear_color[0], previous_clear_color[1], previous_clear_color[2], previous_clear_color[3]);
    if (!scissor_was_enabled)
    {
        glDisable(GL_SCISSOR_TEST);
    }
}
}

struct OpenXRSwapchain
{
    XrSwapchain handle = XR_NULL_HANDLE;
    uint32_t width = 0;
    uint32_t height = 0;
    int64_t format = 0;
    uint32_t sample_count = 0;
    std::vector<XrSwapchainImageOpenGLKHR> images;
};

struct LLOpenXRManagerImpl
{
    XrInstance instance = XR_NULL_HANDLE;
    XrSystemId system_id = XR_NULL_SYSTEM_ID;
    XrSession session = XR_NULL_HANDLE;
    XrSpace app_space = XR_NULL_HANDLE;
    XrSessionState session_state = XR_SESSION_STATE_UNKNOWN;
    XrEnvironmentBlendMode blend_mode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    PFN_xrGetOpenGLGraphicsRequirementsKHR get_gl_requirements = nullptr;
    std::vector<XrViewConfigurationView> config_views;
    std::vector<XrView> views;
    std::vector<OpenXRSwapchain> swapchains;
    GLuint blit_fbo = 0;
    GLuint source_read_fbo = 0;
    bool session_running = false;
    bool submit_ready_logged = false;
    uint64_t frame_submit_count = 0;
    LLVector3 latest_hmd_forward = LLVector3::x_axis;
    bool has_hmd_forward = false;
    LLVector3 latest_hmd_up = LLVector3::y_axis;
    bool has_hmd_up = false;
    F32 latest_left_eye_vertical_fov = 0.f;
    bool has_left_eye_vertical_fov = false;
    F32 latest_left_eye_aspect = 0.f;
    bool has_left_eye_aspect = false;
    LLXRLeftEyeFovSample left_eye_fov_sample;
    // P2-01: THE locate generation counter. Increments exactly once
    // per successful xrLocateViews whose views pass the P0-06
    // validity gates and are adopted as the manager's current pose /
    // FOV sample. The FOV sample, the head's latest pose, and every
    // trace generation field derive from this one counter, so a
    // generation number names one locate unambiguously. (This
    // replaces the P1-01 FOV-only counter; it mints at the same
    // adoption point, so sample.locate_generation is unchanged in
    // meaning.)
    unsigned long long locate_generation = 0;
    // Generation of the locate that produced latest_hmd_forward/up.
    unsigned long long latest_pose_generation = 0;
    // P2-02: the manager-owned frame context of the runtime-paced
    // loop (FS_OPENXR_P2_FRAME_LOOP=1). One main-loop iteration's XR
    // frame lives here from beginPacedFrame() (session events,
    // xrWaitFrame, xrLocateViews, snapshot publish -- all before
    // idle()) through xrBeginFrame (after idle) to the submit on
    // the render path or the main-loop fallback close. It replaces
    // the trace stash as the functional owner of frame identity on
    // the paced path: token, frame state, located views and their
    // generation, lifecycle flags, and the stage timestamps the
    // xr_submit record is built from. The legacy loop never
    // touches it.
    struct XRFrameContext
    {
        bool active = false;
        unsigned long long token = 0;
        XrFrameState frame_state{};
        XrViewState view_state{};
        std::vector<XrView> located_views;
        uint32_t located_view_count = 0;
        bool locate_attempted = false;
        unsigned long long located_generation = 0;
        bool wait_succeeded = false;
        bool begin_attempted = false;
        bool begun = false;
        bool ended = false;
        bool views_valid = false;
        // Zero-layer end owed at begin time (shouldRender=false,
        // locate_failed, invalid_view_state); empty when the frame
        // is a normal render frame.
        std::string pending_end_status;
        // xr_submit trace scalars (LLOpenXRTrace::nowUs() domain).
        bool tracing = false;
        long long t_entry = 0;
        long long t_wait_start = 0;
        long long t_wait_end = 0;
        long long t_begin_start = 0;
        long long t_begin_end = 0;
        long long t_locate_start = 0;
        long long t_locate_end = 0;
        int wait_result = 0;
        int begin_result = 0;
        int locate_result = 0;
        int session_state = 0;
        // P2-03a: the frozen AvatarEyeFrame + the frame-owned
        // render receipt (see llopenxrmanager.h). eye_frame_frozen
        // states the freeze ran (the struct's valid flag states
        // whether it succeeded); render_receipt is registered by
        // the display path and consumed by the paced submit.
        LLXRAvatarEyeFrame eye_frame;
        bool eye_frame_frozen = false;
        LLXRWorldRenderReceipt render_receipt;
    };
    XRFrameContext paced_frame;
    // P2-01: the head path's most recent consumption of the pose
    // (stamped by getHmdForward): which generation it used, and the
    // app_frame_token of the iteration that used it.
    unsigned long long head_consumed_pose_generation = 0;
    unsigned long long head_consumed_app_frame_token = 0;
};
#endif

#if !(LL_LINUX && LL_X11)
struct LLOpenXRManagerImpl
{
};
#endif

LLOpenXRManager::LLOpenXRManager()
: mInitializeAttempted(false),
  mDisabled(false),
  mMonoEnabledOverrideActive(false),
  mMonoEnabledOverrideValue(false)
{
}

LLOpenXRManager::~LLOpenXRManager()
{
    shutdown();
}

LLOpenXRManager& LLOpenXRManager::instance()
{
#if LL_LINUX && LL_X11
    static bool instance_logged = false;
    if (!instance_logged)
    {
        write_openxr_file_log("INFO", "LLOpenXRManager::instance() called.");
        instance_logged = true;
    }
#endif

    static LLOpenXRManager manager;
    return manager;
}

bool LLOpenXRManager::isMonoRequested() const
{
    if (mMonoEnabledOverrideActive)
    {
        return mMonoEnabledOverrideValue;
    }

    return is_openxr_mono_requested();
}

void LLOpenXRManager::setMonoEnabled(bool enabled)
{
    const bool previous_requested = isMonoRequested();
    mMonoEnabledOverrideActive = true;
    mMonoEnabledOverrideValue = enabled;

    if (enabled)
    {
        mDisabled = false;
        mInitializeAttempted = false;
#if LL_LINUX && LL_X11
        if (!previous_requested || !isSessionRunning())
        {
            openxr_info("OpenXR mono enabled via /xr command.");
        }
#endif
        return;
    }

#if LL_LINUX && LL_X11
    if (previous_requested || isSessionRunning())
    {
        openxr_info("OpenXR mono disabled via /xr command.");
    }
#endif
    shutdown();
    mDisabled = false;
    mInitializeAttempted = false;
}

bool LLOpenXRManager::isMonoEnabled() const
{
    return isMonoRequested();
}

bool LLOpenXRManager::isSessionRunning() const
{
#if LL_LINUX && LL_X11
    return mImpl && mImpl->session_running;
#else
    return false;
#endif
}

// P2-01B: the diagnostic cadence lock's per-iteration pacing point.
// Called by LLAppViewer::doFrame() before the iteration's frame
// token is allocated. Engages only when the lock is configured, the
// active frame loop is the legacy one, XR mono is enabled and a
// session is running; otherwise it is a no-op and (when XR is not
// submitting) the grid is dropped so re-engagement re-anchors.
/*static*/ bool LLOpenXRManager::diagPaceFrameStart(F32& out_target_hz, bool& out_has_achieved_interval,
                                                    long long& out_achieved_interval_us, long long& out_deadline_miss_us)
{
    out_target_hz = 0.f;
    out_has_achieved_interval = false;
    out_achieved_interval_us = 0;
    out_deadline_miss_us = 0;
#if LL_LINUX && LL_X11
    const F32 target_hz = diagFixedCadenceHz();
    if (target_hz <= 0.f)
    {
        return false;
    }
    if (!xr_diag_frame_loop_is_legacy())
    {
        return false;
    }
    if (!instance().isMonoEnabled() || !instance().isSessionRunning())
    {
        g_diag_cadence_grid = DiagCadenceGrid();
        return false;
    }

    DiagCadenceGrid& grid = g_diag_cadence_grid;
    const std::chrono::steady_clock::duration period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / (double)target_hz));
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (!grid.anchored)
    {
        grid.next_deadline = now;
        grid.anchored = true;
        grid.has_last_wake = false;
    }
    const std::chrono::steady_clock::time_point deadline = grid.next_deadline;
    long long miss_us = 0;
    if (now > deadline)
    {
        miss_us = (long long)std::chrono::duration_cast<std::chrono::microseconds>(now - deadline).count();
    }
    else if (now < deadline)
    {
        std::this_thread::sleep_until(deadline);
    }
    const std::chrono::steady_clock::time_point wake = std::chrono::steady_clock::now();
    if (grid.has_last_wake)
    {
        out_has_achieved_interval = true;
        out_achieved_interval_us =
            (long long)std::chrono::duration_cast<std::chrono::microseconds>(wake - grid.last_wake).count();
    }
    grid.last_wake = wake;
    grid.has_last_wake = true;
    grid.next_deadline = deadline + period;
    if (grid.next_deadline <= wake)
    {
        // Slots were lost to an overrun or a long block elsewhere:
        // skip them. The lock never bursts to catch up.
        grid.next_deadline = wake + period;
    }
    out_target_hz = target_hz;
    out_deadline_miss_us = miss_us;
    return true;
#else
    return false;
#endif
}

bool LLOpenXRManager::initialize()
{
    if (mDisabled)
    {
        return false;
    }

    if (!isMonoRequested())
    {
        return false;
    }

#if !(LL_LINUX && LL_X11)
    LL_WARNS("OpenXR") << "OpenXR mono path is only implemented for Linux currently." << LL_ENDL;
    mDisabled = true;
    return false;
#else
    {
        std::ostringstream init_message;
        init_message << "OpenXR initialize requested. " << OPENXR_MONO_ENV << "="
                     << LLStringUtil::getenv(OPENXR_MONO_ENV)
                     << ", XR_RUNTIME_JSON=" << LLStringUtil::getenv("XR_RUNTIME_JSON")
                     << ", log_file=" << openxr_log_file_path();
        openxr_info(init_message.str());
    }

    mImpl = std::make_unique<LLOpenXRManagerImpl>();

    uint32_t extension_count = 0;
    XrResult result = xrEnumerateInstanceExtensionProperties(nullptr, 0, &extension_count, nullptr);
    if (XR_FAILED(result))
    {
        openxr_warn("Failed to enumerate OpenXR extension count: " + std::to_string(static_cast<int>(result)));
        mDisabled = true;
        return false;
    }

    std::vector<XrExtensionProperties> extension_props(extension_count);
    for (auto& extension : extension_props)
    {
        init_xr_struct(extension, XR_TYPE_EXTENSION_PROPERTIES);
    }

    result = xrEnumerateInstanceExtensionProperties(nullptr, extension_count, &extension_count, extension_props.data());
    if (XR_FAILED(result))
    {
        openxr_warn("Failed to enumerate OpenXR extensions: " + std::to_string(static_cast<int>(result)));
        mDisabled = true;
        return false;
    }

    bool has_opengl_extension = false;
    for (const auto& extension : extension_props)
    {
        if (std::strcmp(extension.extensionName, XR_KHR_OPENGL_ENABLE_EXTENSION_NAME) == 0)
        {
            has_opengl_extension = true;
            break;
        }
    }

    if (!has_opengl_extension)
    {
        openxr_warn(std::string("Runtime does not expose ") + XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);
        mDisabled = true;
        return false;
    }

    const char* enabled_extensions[] = { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME };

    XrInstanceCreateInfo instance_info{};
    instance_info.type = XR_TYPE_INSTANCE_CREATE_INFO;
    std::strncpy(instance_info.applicationInfo.applicationName, "firestorm-openxr-mono", XR_MAX_APPLICATION_NAME_SIZE - 1);
    std::strncpy(instance_info.applicationInfo.engineName, "firestorm", XR_MAX_ENGINE_NAME_SIZE - 1);
    instance_info.applicationInfo.applicationVersion = 1;
    instance_info.applicationInfo.engineVersion = 1;
    instance_info.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    instance_info.enabledExtensionCount = 1;
    instance_info.enabledExtensionNames = enabled_extensions;

    result = xrCreateInstance(&instance_info, &mImpl->instance);
    if (XR_FAILED(result))
    {
        openxr_warn("xrCreateInstance failed: " + std::to_string(static_cast<int>(result)));
        mDisabled = true;
        return false;
    }

    openxr_info("xrCreateInstance succeeded.");

    XrSystemGetInfo system_info{};
    system_info.type = XR_TYPE_SYSTEM_GET_INFO;
    system_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

    result = xrGetSystem(mImpl->instance, &system_info, &mImpl->system_id);
    if (XR_FAILED(result))
    {
        openxr_warn("xrGetSystem failed: " + format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    openxr_info("xrGetSystem succeeded.");

    result = xrGetInstanceProcAddr(
        mImpl->instance,
        "xrGetOpenGLGraphicsRequirementsKHR",
        reinterpret_cast<PFN_xrVoidFunction*>(&mImpl->get_gl_requirements));
    if (XR_FAILED(result) || mImpl->get_gl_requirements == nullptr)
    {
        openxr_warn("xrGetInstanceProcAddr(xrGetOpenGLGraphicsRequirementsKHR) failed: " +
                    format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    XrGraphicsRequirementsOpenGLKHR requirements{};
    requirements.type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR;
    result = mImpl->get_gl_requirements(mImpl->instance, mImpl->system_id, &requirements);
    if (XR_FAILED(result))
    {
        openxr_warn("xrGetOpenGLGraphicsRequirementsKHR failed: " + format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    {
        std::ostringstream requirement_msg;
        requirement_msg << "OpenGL requirements: min="
                        << XR_VERSION_MAJOR(requirements.minApiVersionSupported) << '.'
                        << XR_VERSION_MINOR(requirements.minApiVersionSupported) << '.'
                        << XR_VERSION_PATCH(requirements.minApiVersionSupported)
                        << " max="
                        << XR_VERSION_MAJOR(requirements.maxApiVersionSupported) << '.'
                        << XR_VERSION_MINOR(requirements.maxApiVersionSupported) << '.'
                        << XR_VERSION_PATCH(requirements.maxApiVersionSupported);
        openxr_info(requirement_msg.str());
    }

    Display* x_display = LLWindowSDL::get_SDL_Display();
    GLXDrawable glx_drawable = glXGetCurrentDrawable();
    GLXContext glx_context = glXGetCurrentContext();
    if (!x_display || !glx_drawable || !glx_context)
    {
        openxr_warn("Missing current X11/GLX context for OpenXR session creation.");
        mDisabled = true;
        shutdown();
        return false;
    }

    XrGraphicsBindingOpenGLXlibKHR graphics_binding{};
    graphics_binding.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR;
    graphics_binding.xDisplay = x_display;
    graphics_binding.glxDrawable = glx_drawable;
    graphics_binding.glxContext = glx_context;

    // Fill all GLX binding fields expected by OpenXR runtimes (not just context/drawable).
    int fbconfig_id = 0;
    if (glXQueryContext(x_display, glx_context, GLX_FBCONFIG_ID, &fbconfig_id) == 0 || fbconfig_id == 0)
    {
        unsigned int drawable_fbconfig_id = 0;
        glXQueryDrawable(x_display, glx_drawable, GLX_FBCONFIG_ID, &drawable_fbconfig_id);
        fbconfig_id = static_cast<int>(drawable_fbconfig_id);
    }

    GLXFBConfig resolved_fbconfig = nullptr;
    if (fbconfig_id != 0)
    {
        int attrs[] = { GLX_FBCONFIG_ID, fbconfig_id, None };
        int fbconfig_count = 0;
        GLXFBConfig* configs = glXChooseFBConfig(x_display, DefaultScreen(x_display), attrs, &fbconfig_count);
        if (configs && fbconfig_count > 0)
        {
            resolved_fbconfig = configs[0];
        }
        if (configs)
        {
            XFree(configs);
        }
    }

    if (!resolved_fbconfig)
    {
        openxr_warn("Failed to resolve GLXFBConfig from current context/drawable.");
        mDisabled = true;
        shutdown();
        return false;
    }

    graphics_binding.glxFBConfig = resolved_fbconfig;

    VisualID visual_id = 0;
    XVisualInfo* visual_info = glXGetVisualFromFBConfig(x_display, resolved_fbconfig);
    if (visual_info)
    {
        visual_id = visual_info->visualid;
        XFree(visual_info);
    }
    if (visual_id == 0)
    {
        unsigned int drawable_visual_id = 0;
        glXQueryDrawable(x_display, glx_drawable, GLX_VISUAL_ID, &drawable_visual_id);
        visual_id = static_cast<VisualID>(drawable_visual_id);
    }
    if (visual_id == 0)
    {
        openxr_warn("Failed to resolve GLX visual ID from current context/drawable.");
        mDisabled = true;
        shutdown();
        return false;
    }

    graphics_binding.visualid = visual_id;

    {
        std::ostringstream binding_msg;
        binding_msg << "GLX binding resolved: fbconfig_id=" << fbconfig_id
                    << ", visualid=" << static_cast<unsigned long>(graphics_binding.visualid);
        openxr_info(binding_msg.str());
    }

    XrSessionCreateInfo session_info{};
    session_info.type = XR_TYPE_SESSION_CREATE_INFO;
    session_info.next = &graphics_binding;
    session_info.systemId = mImpl->system_id;

    result = xrCreateSession(mImpl->instance, &session_info, &mImpl->session);
    if (XR_FAILED(result))
    {
        openxr_warn("xrCreateSession failed: " + format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    openxr_info("xrCreateSession succeeded.");

    XrReferenceSpaceCreateInfo space_info{};
    space_info.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO;
    space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    space_info.poseInReferenceSpace.orientation.w = 1.f;

    result = xrCreateReferenceSpace(mImpl->session, &space_info, &mImpl->app_space);
    if (XR_FAILED(result))
    {
        openxr_warn("xrCreateReferenceSpace failed: " + format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    uint32_t view_count = 0;
    result = xrEnumerateViewConfigurationViews(mImpl->instance, mImpl->system_id, XR_VIEW_CONFIG, 0, &view_count, nullptr);
    if (XR_FAILED(result) || view_count < 2)
    {
        openxr_warn("xrEnumerateViewConfigurationViews failed or returned fewer than two views: " +
                    format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    mImpl->config_views.resize(view_count);
    for (auto& view : mImpl->config_views)
    {
        init_xr_struct(view, XR_TYPE_VIEW_CONFIGURATION_VIEW);
    }

    result = xrEnumerateViewConfigurationViews(
        mImpl->instance, mImpl->system_id, XR_VIEW_CONFIG, view_count, &view_count, mImpl->config_views.data());
    if (XR_FAILED(result))
    {
        openxr_warn("xrEnumerateViewConfigurationViews(list) failed: " +
                    format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    mImpl->views.resize(view_count);
    for (auto& view : mImpl->views)
    {
        init_xr_struct(view, XR_TYPE_VIEW);
    }

    uint32_t blend_mode_count = 0;
    result = xrEnumerateEnvironmentBlendModes(mImpl->instance, mImpl->system_id, XR_VIEW_CONFIG, 0, &blend_mode_count, nullptr);
    if (XR_SUCCEEDED(result) && blend_mode_count > 0)
    {
        std::vector<XrEnvironmentBlendMode> blend_modes(blend_mode_count);
        result = xrEnumerateEnvironmentBlendModes(
            mImpl->instance, mImpl->system_id, XR_VIEW_CONFIG, blend_mode_count, &blend_mode_count, blend_modes.data());
        if (XR_SUCCEEDED(result))
        {
            auto opaque_it = std::find(blend_modes.begin(), blend_modes.end(), XR_ENVIRONMENT_BLEND_MODE_OPAQUE);
            mImpl->blend_mode = (opaque_it != blend_modes.end()) ? *opaque_it : blend_modes.front();
        }
    }

    uint32_t format_count = 0;
    result = xrEnumerateSwapchainFormats(mImpl->session, 0, &format_count, nullptr);
    if (XR_FAILED(result) || format_count == 0)
    {
        openxr_warn("xrEnumerateSwapchainFormats failed: " + format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    std::vector<int64_t> swapchain_formats(format_count);
    result = xrEnumerateSwapchainFormats(mImpl->session, format_count, &format_count, swapchain_formats.data());
    if (XR_FAILED(result))
    {
        openxr_warn("xrEnumerateSwapchainFormats(list) failed: " +
                    format_xr_result(mImpl->instance, result));
        mDisabled = true;
        shutdown();
        return false;
    }

    int64_t chosen_format = choose_swapchain_format(swapchain_formats);

    mImpl->swapchains.resize(2);
    for (uint32_t eye = 0; eye < 2; ++eye)
    {
        const XrViewConfigurationView& view = mImpl->config_views[eye];
        OpenXRSwapchain& swapchain = mImpl->swapchains[eye];

        XrSwapchainCreateInfo swapchain_info{};
        swapchain_info.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
        swapchain_info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
        swapchain_info.format = chosen_format;
        // P0-07/V3: submission copies into swapchain images with a
        // GL_TEXTURE_2D blit; a multisampled swapchain would be
        // incompatible with that copy. Force 1, log the runtime values.
        swapchain_info.sampleCount = 1;
        openxr_info("OpenXR swapchain eye " + std::to_string(eye) +
                    ": sampleCount forced to 1 for the GL blit path (recommended=" +
                    std::to_string(view.recommendedSwapchainSampleCount) + ", max=" +
                    std::to_string(view.maxSwapchainSampleCount) + ").");
        swapchain_info.width = view.recommendedImageRectWidth;
        swapchain_info.height = view.recommendedImageRectHeight;
        swapchain_info.faceCount = 1;
        swapchain_info.arraySize = 1;
        swapchain_info.mipCount = 1;

        result = xrCreateSwapchain(mImpl->session, &swapchain_info, &swapchain.handle);
        if (XR_FAILED(result))
        {
            openxr_warn("xrCreateSwapchain failed: " + format_xr_result(mImpl->instance, result));
            mDisabled = true;
            shutdown();
            return false;
        }

        swapchain.width = swapchain_info.width;
        swapchain.height = swapchain_info.height;
        swapchain.format = swapchain_info.format;
        swapchain.sample_count = swapchain_info.sampleCount;

        uint32_t image_count = 0;
        result = xrEnumerateSwapchainImages(swapchain.handle, 0, &image_count, nullptr);
        if (XR_FAILED(result) || image_count == 0)
        {
            openxr_warn("xrEnumerateSwapchainImages(count) failed: " +
                        format_xr_result(mImpl->instance, result));
            mDisabled = true;
            shutdown();
            return false;
        }

        swapchain.images.resize(image_count);
        for (auto& image : swapchain.images)
        {
            init_xr_struct(image, XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR);
        }

        result = xrEnumerateSwapchainImages(
            swapchain.handle,
            image_count,
            &image_count,
            reinterpret_cast<XrSwapchainImageBaseHeader*>(swapchain.images.data()));
        if (XR_FAILED(result))
        {
            openxr_warn("xrEnumerateSwapchainImages(list) failed: " +
                        format_xr_result(mImpl->instance, result));
            mDisabled = true;
            shutdown();
            return false;
        }
    }

    glGenFramebuffers(1, &mImpl->blit_fbo);

    openxr_info(std::string("OpenXR mono initialized. Runtime OpenGL requirements are available; set ") +
                OPENXR_MONO_ENV + "=0 to disable.");

    // Re-evaluate internal screen target sizing now that runtime view sizes are known.
    gResizeScreenTexture = true;

    return true;
#endif
}

void LLOpenXRManager::processEvents()
{
#if LL_LINUX && LL_X11
    if (!mImpl || mImpl->instance == XR_NULL_HANDLE)
    {
        return;
    }

    XrEventDataBuffer event{};
    event.type = XR_TYPE_EVENT_DATA_BUFFER;

    while (xrPollEvent(mImpl->instance, &event) == XR_SUCCESS)
    {
        switch (event.type)
        {
        case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
            openxr_warn("OpenXR runtime instance loss pending; disabling VR submit.");
            mDisabled = true;
            return;

        case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED:
        {
            const auto* session_event = reinterpret_cast<const XrEventDataSessionStateChanged*>(&event);
            const XrSessionState previous_state = mImpl->session_state;
            mImpl->session_state = session_event->state;
            openxr_info("Session state changed: " +
                        xr_session_state_name(previous_state) + " -> " +
                        xr_session_state_name(mImpl->session_state));

            if (mImpl->session_state == XR_SESSION_STATE_READY && !mImpl->session_running)
            {
                XrSessionBeginInfo begin_info{};
                begin_info.type = XR_TYPE_SESSION_BEGIN_INFO;
                begin_info.primaryViewConfigurationType = XR_VIEW_CONFIG;
                XrResult begin_result = xrBeginSession(mImpl->session, &begin_info);
                if (XR_SUCCEEDED(begin_result))
                {
                    mImpl->session_running = true;
                    openxr_info("OpenXR session started.");
                    if (LLOpenXRTrace::enabled())
                    {
                        // P2-01B: the session record states the
                        // diagnostic cadence target when the lock is
                        // configured for this process.
                        std::string trace_extra_fields;
                        const F32 diag_cadence_hz = LLOpenXRManager::diagFixedCadenceHz();
                        if (diag_cadence_hz > 0.f)
                        {
                            std::ostringstream hz_stream;
                            hz_stream << diag_cadence_hz;
                            trace_extra_fields = "\"diag_cadence_target_hz\":" + hz_stream.str();
                        }
                        const unsigned int trace_run = LLOpenXRTrace::beginRun(
                            LLOpenXRManager::pacedFrameLoopEnabled() ? "paced_pre_idle_mono" : "legacy_mono",
                            trace_extra_fields);
                        openxr_info("OpenXR trace run " + std::to_string(trace_run) +
                                    " -> " + LLOpenXRTrace::traceFilePath());
                    }
                }
                else
                {
                    openxr_warn("xrBeginSession failed: " + format_xr_result(mImpl->instance, begin_result));
                }
            }
            else if (mImpl->session_state == XR_SESSION_STATE_STOPPING && mImpl->session_running)
            {
                xrEndSession(mImpl->session);
                mImpl->session_running = false;
                mImpl->has_hmd_forward = false;
                mImpl->has_left_eye_vertical_fov = false;
                mImpl->has_left_eye_aspect = false;
                mImpl->left_eye_fov_sample.valid = false;
                gResizeScreenTexture = true;
                openxr_info("OpenXR session stopped.");
                LLOpenXRTrace::endRun("stopping");
            }
            else if (mImpl->session_state == XR_SESSION_STATE_EXITING ||
                     mImpl->session_state == XR_SESSION_STATE_LOSS_PENDING)
            {
                openxr_warn("OpenXR runtime requested exit; disabling VR submit.");
                LLOpenXRTrace::endRun("exiting");
                mImpl->has_hmd_forward = false;
                mImpl->has_left_eye_vertical_fov = false;
                mImpl->has_left_eye_aspect = false;
                mImpl->left_eye_fov_sample.valid = false;
                gResizeScreenTexture = true;
                mDisabled = true;
                return;
            }
            break;
        }
        case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
        {
            // P2-04a: provenance probe only. Reference-space change
            // events are recorded for the recenter provenance
            // catalogue; nothing in the viewer reacts to them. The
            // full field set also goes to the viewer log because
            // trace records only exist inside an active run, while
            // these events can arrive across session transitions.
            const auto* space_event = reinterpret_cast<const XrEventDataReferenceSpaceChangePending*>(&event);
            const XrPosef& prev_pose = space_event->poseInPreviousSpace;
            std::ostringstream fields;
            fields << "\"reference_space_type\":" << static_cast<int>(space_event->referenceSpaceType)
                   << ",\"reference_space_type_name\":\"" << xr_reference_space_type_name(space_event->referenceSpaceType) << "\""
                   << ",\"change_time_ns\":" << static_cast<long long>(space_event->changeTime)
                   << ",\"pose_valid\":" << (space_event->poseValid ? "true" : "false");
            if (space_event->poseValid)
            {
                // poseInPreviousSpace is undefined when poseValid
                // is false (OpenXR spec); serialize null instead.
                fields << ",\"prev_space_pos\":[" << prev_pose.position.x << "," << prev_pose.position.y << "," << prev_pose.position.z << "]"
                       << ",\"prev_space_quat\":[" << prev_pose.orientation.x << "," << prev_pose.orientation.y << "," << prev_pose.orientation.z << "," << prev_pose.orientation.w << "]";
            }
            else
            {
                fields << ",\"prev_space_pos\":null,\"prev_space_quat\":null";
            }
            fields << ",\"event_session\":" << static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(space_event->session))
                   << ",\"app_session\":" << static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(mImpl->session))
                   << ",\"session_state\":\"" << xr_session_state_name(mImpl->session_state) << "\""
                   << ",\"app_frame_token\":" << LLOpenXRTrace::currentAppFrameToken();
            openxr_info("Reference space change pending: {" + fields.str() + "}");
            if (LLOpenXRTrace::enabled())
            {
                LLOpenXRTrace::writeRecord("xr_recenter_probe", fields.str());
            }
            break;
        }
        default:
            break;
        }

        event = {};
        event.type = XR_TYPE_EVENT_DATA_BUFFER;
    }
#endif
}


#if LL_LINUX && LL_X11
namespace
{
// P0-03: collects one xr_submit trace record across submitFrameMono's
// many exit paths. Emits exactly once, from finish() or the destructor,
// and only when a trace run is active. All timestamps are
// LLOpenXRTrace::nowUs() (steady clock); OpenXR times are recorded raw
// and labelled, never mixed with the steady clock.
struct XrSubmitTrace
{
    bool tracing = false;
    bool emitted = false;
    std::string status = "no_session";
    long long t_entry = 0;
    long long t_wait_end = 0;
    long long t_begin_end = 0;
    long long t_locate_end = 0;
    long long t_end_end = 0;
    int wait_result = 0;
    int begin_result = 0;
    int locate_result = 0;
    int end_result = 0;
    long long predicted_display_time = 0;
    long long predicted_display_period = 0;
    bool should_render = false;
    uint32_t located_view_count = 0;
    unsigned long long view_state_flags = 0;
    bool has_eye[2] = {false, false};
    float eye_pos[2][3] = {};
    float eye_quat[2][4] = {};
    float eye_fov[2][4] = {};
    bool using_pipeline_source = false;
    int input_rect[4] = {0, 0, 0, 0};
    int cropped_rect[4] = {0, 0, 0, 0};
    float mono_fov[4] = {0.f, 0.f, 0.f, 0.f};
    float mono_pos[3] = {0.f, 0.f, 0.f};
    float mono_quat[4] = {0.f, 0.f, 0.f, 0.f};
    uint32_t eye_swap_w[2] = {0, 0};
    uint32_t eye_swap_h[2] = {0, 0};
    long long eye_swap_format[2] = {0, 0};
    uint32_t eye_swap_sample[2] = {0, 0};
    bool eye_attempted[2] = {false, false};
    int eye_acquire_result[2] = {0, 0};
    int eye_wait_result[2] = {0, 0};
    int eye_release_result[2] = {0, 0};
    bool eye_fbo_complete[2] = {false, false};
    bool eye_blit_done[2] = {false, false};
    bool eye_submitted[2] = {false, false};
    uint32_t submitted_view_count = 0;
    uint32_t layer_count = 0;
    // P1-01: render provenance transferred with the submitted image.
    bool has_render_source = false;
    unsigned long long render_seq_source = 0;
    bool render_fov_valid = false;
    unsigned long long render_fov_generation = 0;
    float render_fov_rad[4] = {0.f, 0.f, 0.f, 0.f};
    // P1-01: manager's latest located left-eye sample at submit time.
    bool has_latest_located = false;
    float latest_located_fov_rad[4] = {0.f, 0.f, 0.f, 0.f};
    // P1-03: optics mode of this submission and strict-coherence
    // evidence. blit_scale is [eye][x,y] = swapchain extent / source
    // extent of the blit that copied the submitted image; the right
    // eye's scale is recorded separately, never folded into the left.
    bool optics_coherent = false;
    bool has_coherence_delta = false;
    float coherence_max_delta_deg = 0.f;
    bool has_blit_scale[2] = {false, false};
    float blit_scale[2][2] = {};
    // P2-01: per-call stage start timestamps (the *_end stamps above
    // are P0-03). All on the LLOpenXRTrace::nowUs() host monotonic
    // clock; 0 = stage not reached.
    long long t_wait_start = 0;
    long long t_begin_start = 0;
    long long t_locate_start = 0;
    long long t_end_start = 0;
    // P2-01: the XrTime the locate was performed at (the wait-returned
    // frame's predictedDisplayTime, in the runtime's clock domain --
    // never mixed with the host timestamps above).
    bool has_locate_display_time = false;
    long long locate_display_time = 0;
    // P2-01: session state at this call (after processEvents).
    bool has_session_state = false;
    int session_state = 0;
    bool has_pose_sample_valid = false;
    bool pose_sample_valid = false;
    // P2-01: pose generations (manager's single locate counter).
    bool has_located_generation = false;
    unsigned long long located_generation = 0;
    bool has_head_generation = false;
    unsigned long long head_generation = 0;
    bool has_render_generation = false;
    unsigned long long render_generation = 0;
    bool has_layer_generation = false;
    unsigned long long layer_generation = 0;
    // P2-01: source identity/state transferred with the image.
    unsigned long long source_app_frame_token = 0;
    bool has_source_gl = false;
    float source_world_gl_projection[16] = {};
    float source_world_gl_view[16] = {};
    bool has_source_gl_rects = false;
    int source_gl_viewport[4] = {0, 0, 0, 0};
    int source_gl_scissor[4] = {0, 0, 0, 0};
    bool source_gl_scissor_enabled = false;
    std::string source_render_fingerprint;
    // P2-01B: diagnostic source-image marker state for this
    // submission (FS_OPENXR_DIAG_FRAME_MARKER). The value is the
    // 10-bit value the marker encoded (the low 10 bits of the
    // producing iteration's app_frame_token); has_ stays false when
    // no eye's blit completed, i.e. the marker never reached an
    // image.
    bool diagnostic_marker_enabled = false;
    bool has_diagnostic_marker_value = false;
    unsigned int diagnostic_marker_value = 0;
    // P2-01: swapchain image index per eye (at acquire) and the blit
    // rectangles per eye (source [left,bottom,w,h], dest [x,y,w,h];
    // the filter is the glBlitFramebuffer literal, GL_LINEAR).
    bool has_eye_image_index[2] = {false, false};
    uint32_t eye_image_index[2] = {0, 0};
    bool has_blit_rects[2] = {false, false};
    int blit_src_rect[2][4] = {};
    int blit_dst_rect[2][4] = {};
    // P2-01 / Amendment 4: the exact submitted state, captured by
    // captureEndFrame() immediately before each xrEndFrame.
    bool has_submitted_state = false;
    int submitted_layer_count = 0;
    long long submitted_end_display_time = 0;
    bool has_projection_layer = false;
    std::string submitted_layer_space_type;
    unsigned long long submitted_layer_space_id = 0;
    uint32_t submitted_view_count_captured = 0;
    bool submitted_view_present[2] = {false, false};
    float submitted_view_pos[2][3] = {};
    float submitted_view_quat[2][4] = {};
    float submitted_view_fov[2][4] = {};
    int submitted_view_rect[2][4] = {};
    unsigned long long submitted_view_swapchain[2] = {0, 0};
    bool has_submitted_image_index[2] = {false, false};
    uint32_t submitted_view_image_index[2] = {0, 0};
    bool has_submitted_pose_generation = false;
    unsigned long long submitted_pose_generation = 0;
    bool has_submitted_fov_generation = false;
    unsigned long long submitted_fov_generation = 0;
    std::string fp_layer;

    // P2-03a: frozen AvatarEyeFrame + render-receipt state
    // (paced records only; has_ stays false on legacy records and
    // none of these fields are emitted there). The runtime views'
    // FOVs already ride in eye_fov[][] above.
    bool has_avatar_eye = false;
    unsigned long long avatar_eye_frame_token = 0;
    bool avatar_eye_valid = false;
    std::string avatar_eye_invalid_reason;
    unsigned long long avatar_eye_locate_generation = 0;
    unsigned long long avatar_eye_head_pose_generation = 0;
    long long avatar_eye_predicted_display_time = 0;
    bool avatar_eye_left_cornea_valid = false;
    float avatar_eye_left_cornea[3] = {0.f, 0.f, 0.f};
    bool avatar_eye_right_cornea_valid = false;
    float avatar_eye_right_cornea[3] = {0.f, 0.f, 0.f};
    std::string avatar_eye_right_cornea_source;
    std::string avatar_eye_left_cornea_point;
    float avatar_eye_resolved_head_quat[4] = {0.f, 0.f, 0.f, 1.f};
    float avatar_eye_resolved_camera_quat[4] = {0.f, 0.f, 0.f, 1.f};
    bool avatar_eye_gaze_verified = false;
    float avatar_eye_requested_near_far[2] = {0.f, 0.f};
    int avatar_eye_target_extent[2] = {0, 0};
    bool has_render_receipt = false;
    unsigned long long receipt_token = 0;
    unsigned long long receipt_locate_generation = 0;
    bool receipt_consumed = false;
    bool receipt_completed = false;
    std::string receipt_validation_status;
    // P2-01 / Amendment 4: capture the exact state about to be handed
    // to xrEndFrame. Called IMMEDIATELY before each xrEndFrame in
    // submitFrameMono; also stamps t_end_start. proj_layer is nullptr
    // for zero-layer ends. Values are copied verbatim from the
    // structures the runtime will receive -- nothing is recomputed,
    // and layer values are never read back from the manager's caches.
    void captureEndFrame(const XrFrameEndInfo& info, const XrCompositionLayerProjection* proj_layer,
                         const char* space_type, unsigned long long space_id)
    {
        if (tracing)
        {
            t_end_start = LLOpenXRTrace::nowUs();
        }
        has_submitted_state = true;
        submitted_layer_count = (int)info.layerCount;
        submitted_end_display_time = (long long)info.displayTime;
        if (info.layerCount == 0 || !proj_layer)
        {
            return;
        }
        has_projection_layer = true;
        submitted_layer_space_type = space_type ? space_type : "";
        submitted_layer_space_id = space_id;
        const uint32_t count = std::min<uint32_t>(proj_layer->viewCount, 2);
        // Layer views were appended in eye order for the eyes whose
        // submission succeeded; map them back by that same order.
        uint32_t view_index = 0;
        for (uint32_t eye = 0; eye < 2 && view_index < count; eye++)
        {
            if (!eye_submitted[eye])
            {
                continue;
            }
            const XrCompositionLayerProjectionView& view = proj_layer->views[view_index];
            submitted_view_present[view_index] = true;
            submitted_view_pos[view_index][0] = view.pose.position.x;
            submitted_view_pos[view_index][1] = view.pose.position.y;
            submitted_view_pos[view_index][2] = view.pose.position.z;
            submitted_view_quat[view_index][0] = view.pose.orientation.x;
            submitted_view_quat[view_index][1] = view.pose.orientation.y;
            submitted_view_quat[view_index][2] = view.pose.orientation.z;
            submitted_view_quat[view_index][3] = view.pose.orientation.w;
            submitted_view_fov[view_index][0] = view.fov.angleLeft;
            submitted_view_fov[view_index][1] = view.fov.angleRight;
            submitted_view_fov[view_index][2] = view.fov.angleDown;
            submitted_view_fov[view_index][3] = view.fov.angleUp;
            submitted_view_rect[view_index][0] = view.subImage.imageRect.offset.x;
            submitted_view_rect[view_index][1] = view.subImage.imageRect.offset.y;
            submitted_view_rect[view_index][2] = view.subImage.imageRect.extent.width;
            submitted_view_rect[view_index][3] = view.subImage.imageRect.extent.height;
            submitted_view_swapchain[view_index] = (unsigned long long)(uintptr_t)view.subImage.swapchain;
            if (has_eye_image_index[eye])
            {
                has_submitted_image_index[view_index] = true;
                submitted_view_image_index[view_index] = eye_image_index[eye];
            }
            view_index++;
        }
        submitted_view_count_captured = view_index;
        if (tracing)
        {
            std::ostringstream canon;
            canon << submitted_layer_space_type << "#" << submitted_layer_space_id;
            for (uint32_t i = 0; i < view_index; i++)
            {
                canon << "|" << f32(submitted_view_pos[i][0]) << "," << f32(submitted_view_pos[i][1])
                      << "," << f32(submitted_view_pos[i][2]) << "|" << f32(submitted_view_quat[i][0])
                      << "," << f32(submitted_view_quat[i][1]) << "," << f32(submitted_view_quat[i][2])
                      << "," << f32(submitted_view_quat[i][3]) << "|" << f32(submitted_view_fov[i][0])
                      << "," << f32(submitted_view_fov[i][1]) << "," << f32(submitted_view_fov[i][2])
                      << "," << f32(submitted_view_fov[i][3]) << "|" << submitted_view_rect[i][0] << ","
                      << submitted_view_rect[i][1] << "," << submitted_view_rect[i][2] << ","
                      << submitted_view_rect[i][3];
            }
            fp_layer = LLOpenXRTrace::placementFingerprint(canon.str());
        }
    }

    static std::string f32(float v)
    {
        std::ostringstream oss;
        oss << std::setprecision(9) << v;
        return oss.str();
    }

    // P2-01: a duration on the nowUs() clock, or "null" when either
    // endpoint was never stamped (the stage was not reached).
    static std::string dur_or_null(long long start, long long end)
    {
        if (start <= 0 || end < start)
        {
            return "null";
        }
        return std::to_string(end - start);
    }

    static std::string us_or_null(long long v)
    {
        return v > 0 ? std::to_string(v) : std::string("null");
    }

    static std::string gen_or_null(bool has, unsigned long long v)
    {
        return has ? std::to_string(v) : std::string("null");
    }

    void finish(const std::string& final_status)
    {
        if (emitted)
        {
            return;
        }
        emitted = true;
        status = final_status;
        if (!tracing)
        {
            return;
        }

        std::ostringstream f;
        f << "\"xr_frame_seq\":" << LLOpenXRTrace::nextXrFrameSeq();
        f << ",\"status\":\"" << status << "\"";
        // P2-01: the frame's outcome under its contract name, the
        // loop mode, and the join to this iteration's main-loop
        // context (token + the stage stamps that exist by submit
        // time; t_display_return_us lives on the app_frame record,
        // which is written after display() returns).
        f << ",\"frame_outcome\":\"" << status << "\"";
        {
            LLOpenXRTrace::AppFrameContext mode_ctx;
            std::string loop_mode = "legacy_after_render";
            if (LLOpenXRTrace::peekAppFrameContext(mode_ctx) && !mode_ctx.frame_loop_mode.empty())
            {
                loop_mode = mode_ctx.frame_loop_mode;
            }
            f << ",\"frame_loop_mode\":\"" << loop_mode << "\"";
        }
        {
            LLOpenXRTrace::AppFrameContext app_ctx;
            if (LLOpenXRTrace::peekAppFrameContext(app_ctx))
            {
                f << ",\"app_frame_token\":" << app_ctx.app_frame_token;
                f << ",\"t_mainloop_frame_start_us\":" << app_ctx.t_mainloop_frame_start_us;
                f << ",\"t_idle_start_us\":" << us_or_null(app_ctx.t_idle_start_us);
                f << ",\"t_idle_end_us\":" << us_or_null(app_ctx.t_idle_end_us);
                f << ",\"t_world_render_start_us\":" << us_or_null(app_ctx.t_world_render_start_us);
                f << ",\"t_render_end_us\":" << us_or_null(app_ctx.t_render_end_us);
            }
            else
            {
                f << ",\"app_frame_token\":null,\"t_mainloop_frame_start_us\":null";
                f << ",\"t_idle_start_us\":null,\"t_idle_end_us\":null";
                f << ",\"t_world_render_start_us\":null,\"t_render_end_us\":null";
            }
        }
        if (has_session_state)
        {
            f << ",\"session_state\":" << session_state;
            f << ",\"session_state_name\":\"" << xr_session_state_name((XrSessionState)session_state) << "\"";
        }
        else
        {
            f << ",\"session_state\":null,\"session_state_name\":null";
        }
        f << ",\"pose_sample_valid\":"
          << (has_pose_sample_valid ? (pose_sample_valid ? "true" : "false") : "null");
        f << ",\"t_entry_us\":" << t_entry;
        f << ",\"t_wait_end_us\":" << t_wait_end;
        f << ",\"t_begin_end_us\":" << t_begin_end;
        f << ",\"t_locate_end_us\":" << t_locate_end;
        f << ",\"t_end_end_us\":" << t_end_end;
        // P2-01: stage start stamps + derived call durations (all
        // nowUs() host monotonic; null when a stage was not reached).
        f << ",\"t_wait_start_us\":" << t_wait_start;
        f << ",\"t_begin_start_us\":" << t_begin_start;
        f << ",\"t_locate_start_us\":" << t_locate_start;
        f << ",\"t_end_start_us\":" << t_end_start;
        f << ",\"wait_block_us\":" << dur_or_null(t_wait_start, t_wait_end);
        f << ",\"begin_call_us\":" << dur_or_null(t_begin_start, t_begin_end);
        f << ",\"locate_call_us\":" << dur_or_null(t_locate_start, t_locate_end);
        f << ",\"end_call_us\":" << dur_or_null(t_end_start, t_end_end);
        f << ",\"locate_to_end_us\":" << dur_or_null(t_locate_end, t_end_end);
        f << ",\"wait_result\":" << wait_result;
        f << ",\"begin_result\":" << begin_result;
        f << ",\"locate_result\":" << locate_result;
        f << ",\"end_result\":" << end_result;
        f << ",\"predicted_display_time_xr_ns\":" << predicted_display_time;
        f << ",\"predicted_display_period_xr_ns\":" << predicted_display_period;
        // P2-01: the locate ran at the wait-returned frame's
        // predictedDisplayTime (runtime clock domain). Cross-domain
        // deltas between host us and XrTime ns require
        // XR_KHR_convert_timespec_time, which is NOT enabled in this
        // codebase (only XR_KHR_OPENGL_ENABLE is) -- they are null,
        // never estimated, and no conversion extension is negotiated
        // in this commit.
        f << ",\"locate_display_time_xr_ns\":"
          << (has_locate_display_time ? std::to_string(locate_display_time) : std::string("null"));
        f << ",\"clock_domain_conversion_available\":false";
        f << ",\"wait_return_xr_ns\":null,\"end_start_xr_ns\":null";
        f << ",\"predicted_minus_wait_return_us\":null,\"end_start_minus_predicted_us\":null";
        f << ",\"should_render\":" << (should_render ? "true" : "false");
        f << ",\"located_view_count\":" << located_view_count;
        f << ",\"view_state_flags\":" << view_state_flags;
        f << ",\"view_orientation_valid\":" << ((view_state_flags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) ? "true" : "false");
        f << ",\"view_position_valid\":" << ((view_state_flags & XR_VIEW_STATE_POSITION_VALID_BIT) ? "true" : "false");
        f << ",\"view_orientation_tracked\":" << ((view_state_flags & XR_VIEW_STATE_ORIENTATION_TRACKED_BIT) ? "true" : "false");
        f << ",\"view_position_tracked\":" << ((view_state_flags & XR_VIEW_STATE_POSITION_TRACKED_BIT) ? "true" : "false");
        for (int eye = 0; eye < 2; eye++)
        {
            if (!has_eye[eye])
            {
                continue;
            }
            f << ",\"eye" << eye << "_pos\":[" << f32(eye_pos[eye][0]) << "," << f32(eye_pos[eye][1]) << "," << f32(eye_pos[eye][2]) << "]";
            f << ",\"eye" << eye << "_quat\":[" << f32(eye_quat[eye][0]) << "," << f32(eye_quat[eye][1]) << "," << f32(eye_quat[eye][2]) << "," << f32(eye_quat[eye][3]) << "]";
            f << ",\"eye" << eye << "_fov\":[" << f32(eye_fov[eye][0]) << "," << f32(eye_fov[eye][1]) << "," << f32(eye_fov[eye][2]) << "," << f32(eye_fov[eye][3]) << "]";
        }
        for (int eye = 0; eye < 2; eye++)
        {
            if (!eye_attempted[eye])
            {
                continue;
            }
            f << ",\"eye" << eye << "_swap_w\":" << eye_swap_w[eye];
            f << ",\"eye" << eye << "_swap_h\":" << eye_swap_h[eye];
            f << ",\"eye" << eye << "_swap_format\":" << eye_swap_format[eye];
            f << ",\"eye" << eye << "_swap_sample\":" << eye_swap_sample[eye];
            f << ",\"eye" << eye << "_acquire_result\":" << eye_acquire_result[eye];
            f << ",\"eye" << eye << "_wait_result\":" << eye_wait_result[eye];
            f << ",\"eye" << eye << "_release_result\":" << eye_release_result[eye];
            f << ",\"eye" << eye << "_fbo_complete\":" << (eye_fbo_complete[eye] ? "true" : "false");
            f << ",\"eye" << eye << "_blit_done\":" << (eye_blit_done[eye] ? "true" : "false");
            f << ",\"eye" << eye << "_submitted\":" << (eye_submitted[eye] ? "true" : "false");
            if (has_eye_image_index[eye])
            {
                f << ",\"eye" << eye << "_image_index\":" << eye_image_index[eye];
            }
        }
        f << ",\"using_pipeline_source\":" << (using_pipeline_source ? "true" : "false");
        f << ",\"input_rect\":[" << input_rect[0] << "," << input_rect[1] << "," << input_rect[2] << "," << input_rect[3] << "]";
        f << ",\"cropped_rect\":[" << cropped_rect[0] << "," << cropped_rect[1] << "," << cropped_rect[2] << "," << cropped_rect[3] << "]";
        f << ",\"mono_fov\":[" << f32(mono_fov[0]) << "," << f32(mono_fov[1]) << "," << f32(mono_fov[2]) << "," << f32(mono_fov[3]) << "]";
        f << ",\"mono_pos\":[" << f32(mono_pos[0]) << "," << f32(mono_pos[1]) << "," << f32(mono_pos[2]) << "]";
        f << ",\"mono_quat\":[" << f32(mono_quat[0]) << "," << f32(mono_quat[1]) << "," << f32(mono_quat[2]) << "," << f32(mono_quat[3]) << "]";
        if (has_render_source)
        {
            f << ",\"render_seq_source\":" << render_seq_source;
            f << ",\"xr_render_fov_generation\":"
              << (render_fov_valid ? std::to_string(render_fov_generation) : std::string("null"));
            f << ",\"xr_render_fov_rad\":";
            if (render_fov_valid)
            {
                f << "[" << f32(render_fov_rad[0]) << "," << f32(render_fov_rad[1]) << ","
                  << f32(render_fov_rad[2]) << "," << f32(render_fov_rad[3]) << "]";
            }
            else
            {
                f << "null";
            }
        }
        else
        {
            f << ",\"render_seq_source\":null,\"xr_render_fov_generation\":null,\"xr_render_fov_rad\":null";
        }
        f << ",\"xr_submitted_fov_rad\":[" << f32(mono_fov[0]) << "," << f32(mono_fov[1]) << ","
          << f32(mono_fov[2]) << "," << f32(mono_fov[3]) << "]";
        f << ",\"xr_latest_located_left_fov_rad\":";
        if (has_latest_located)
        {
            f << "[" << f32(latest_located_fov_rad[0]) << "," << f32(latest_located_fov_rad[1]) << ","
              << f32(latest_located_fov_rad[2]) << "," << f32(latest_located_fov_rad[3]) << "]";
        }
        else
        {
            f << "null";
        }
        if (has_latest_located && submitted_view_count > 0)
        {
            float max_delta_rad = 0.f;
            for (int i = 0; i < 4; i++)
            {
                max_delta_rad = std::max(max_delta_rad, std::fabs(mono_fov[i] - latest_located_fov_rad[i]));
            }
            f << ",\"xr_fov_delta_current_deg\":" << f32(max_delta_rad * 57.29577951308232f);
        }
        else
        {
            f << ",\"xr_fov_delta_current_deg\":null";
        }
        f << ",\"xr_optics_source_valid\":" << ((has_render_source && render_fov_valid) ? "true" : "false");
        // P2-01: pose generations, all from the manager's single
        // locate counter. On the old loop they expose the
        // architecture directly: the layer carries THIS call's
        // locate, while the head and the render consumed the
        // previous one (delta 1).
        f << ",\"pose_generation_located\":" << gen_or_null(has_located_generation, located_generation);
        f << ",\"pose_generation_used_by_head\":" << gen_or_null(has_head_generation, head_generation);
        f << ",\"pose_generation_used_by_render\":" << gen_or_null(has_render_generation, render_generation);
        f << ",\"pose_generation_used_by_layer\":" << gen_or_null(has_layer_generation, layer_generation);
        f << ",\"render_generation_delta\":"
          << ((has_located_generation && has_render_generation)
                  ? std::to_string((long long)located_generation - (long long)render_generation)
                  : std::string("null"));
        f << ",\"layer_generation_delta\":"
          << ((has_located_generation && has_layer_generation)
                  ? std::to_string((long long)located_generation - (long long)layer_generation)
                  : std::string("null"));
        f << ",\"submitted_pose_generation\":" << gen_or_null(has_submitted_pose_generation, submitted_pose_generation);
        f << ",\"submitted_fov_generation\":" << gen_or_null(has_submitted_fov_generation, submitted_fov_generation);
        // The AvatarEyeFrame does not exist until P2-03; its token
        // is null here -- app_frame_token (above) is the linkage
        // available in P2-01.
        f << ",\"avatar_eye_frame_token\":null";
        // P2-01B: diagnostic marker join for the headset-recording
        // A/B analysis. Emitted only when the marker diagnostic is
        // enabled, so non-diagnostic records are unchanged.
        if (diagnostic_marker_enabled)
        {
            f << ",\"diagnostic_marker_enabled\":true";
            f << ",\"diagnostic_marker_value\":"
              << (has_diagnostic_marker_value ? std::to_string(diagnostic_marker_value) : std::string("null"));
        }
        f << ",\"submitted_view_count\":" << submitted_view_count;
        f << ",\"layer_count\":" << layer_count;
        f << ",\"xr_optics_mode\":\"" << (optics_coherent ? "coherent" : "legacy") << "\"";
        if (optics_coherent)
        {
            f << ",\"xr_coherence_max_delta_deg\":";
            if (has_coherence_delta)
            {
                f << f32(coherence_max_delta_deg);
            }
            else
            {
                f << "null";
            }
        }
        for (int eye = 0; eye < 2; eye++)
        {
            if (has_blit_scale[eye])
            {
                f << ",\"eye" << eye << "_blit_scale\":[" << f32(blit_scale[eye][0]) << "," << f32(blit_scale[eye][1]) << "]";
            }
        }
        // P2-01 / Amendment 4: the exact submitted state (captured
        // immediately before the xrEndFrame that consumed it), the
        // source render's transferred GL state, the blit rectangles,
        // and the four placement fingerprints. Raw values are
        // recorded in full; fingerprints only group them.
        f << ",\"has_submitted_state\":" << (has_submitted_state ? "true" : "false");
        if (has_submitted_state)
        {
            f << ",\"submitted_layer_count\":" << submitted_layer_count;
            f << ",\"submitted_end_display_time_xr_ns\":" << submitted_end_display_time;
            f << ",\"submitted_has_projection_layer\":" << (has_projection_layer ? "true" : "false");
            if (has_projection_layer)
            {
                f << ",\"layer_space_type\":\"" << submitted_layer_space_type << "\"";
                f << ",\"layer_space_id\":" << submitted_layer_space_id;
                for (uint32_t i = 0; i < submitted_view_count_captured; i++)
                {
                    f << ",\"submitted_view" << i << "_pos\":[" << f32(submitted_view_pos[i][0]) << "," << f32(submitted_view_pos[i][1]) << "," << f32(submitted_view_pos[i][2]) << "]";
                    f << ",\"submitted_view" << i << "_quat\":[" << f32(submitted_view_quat[i][0]) << "," << f32(submitted_view_quat[i][1]) << "," << f32(submitted_view_quat[i][2]) << "," << f32(submitted_view_quat[i][3]) << "]";
                    f << ",\"submitted_view" << i << "_fov\":[" << f32(submitted_view_fov[i][0]) << "," << f32(submitted_view_fov[i][1]) << "," << f32(submitted_view_fov[i][2]) << "," << f32(submitted_view_fov[i][3]) << "]";
                    f << ",\"submitted_view" << i << "_rect\":[" << submitted_view_rect[i][0] << "," << submitted_view_rect[i][1] << "," << submitted_view_rect[i][2] << "," << submitted_view_rect[i][3] << "]";
                    f << ",\"submitted_view" << i << "_swapchain\":" << submitted_view_swapchain[i];
                    f << ",\"submitted_view" << i << "_image_index\":"
                      << (has_submitted_image_index[i] ? std::to_string(submitted_view_image_index[i]) : std::string("null"));
                }
            }
        }
        f << ",\"source_app_frame_token\":" << source_app_frame_token;
        if (has_source_gl)
        {
            f << ",\"source_world_gl_projection\":[";
            for (int i = 0; i < 16; i++)
            {
                if (i > 0)
                {
                    f << ",";
                }
                f << f32(source_world_gl_projection[i]);
            }
            f << "]";
            f << ",\"source_world_gl_view\":[";
            for (int i = 0; i < 16; i++)
            {
                if (i > 0)
                {
                    f << ",";
                }
                f << f32(source_world_gl_view[i]);
            }
            f << "]";
        }
        else
        {
            f << ",\"source_world_gl_projection\":null,\"source_world_gl_view\":null";
        }
        if (has_source_gl_rects)
        {
            f << ",\"source_gl_viewport\":[" << source_gl_viewport[0] << "," << source_gl_viewport[1] << "," << source_gl_viewport[2] << "," << source_gl_viewport[3] << "]";
            f << ",\"source_gl_scissor\":[" << source_gl_scissor[0] << "," << source_gl_scissor[1] << "," << source_gl_scissor[2] << "," << source_gl_scissor[3] << "]";
            f << ",\"source_gl_scissor_enabled\":" << (source_gl_scissor_enabled ? "true" : "false");
        }
        else
        {
            f << ",\"source_gl_viewport\":null,\"source_gl_scissor\":null,\"source_gl_scissor_enabled\":null";
        }
        for (int eye = 0; eye < 2; eye++)
        {
            if (has_blit_rects[eye])
            {
                f << ",\"eye" << eye << "_blit_src_rect\":[" << blit_src_rect[eye][0] << "," << blit_src_rect[eye][1] << "," << blit_src_rect[eye][2] << "," << blit_src_rect[eye][3] << "]";
                f << ",\"eye" << eye << "_blit_dst_rect\":[" << blit_dst_rect[eye][0] << "," << blit_dst_rect[eye][1] << "," << blit_dst_rect[eye][2] << "," << blit_dst_rect[eye][3] << "]";
                f << ",\"eye" << eye << "_blit_filter\":\"LINEAR\"";
            }
        }
        {
            std::ostringstream src_canon;
            src_canon << source_app_frame_token << "#"
                      << (has_render_source ? std::to_string(render_seq_source) : std::string("null"))
                      << "#" << input_rect[0] << "," << input_rect[1] << "," << input_rect[2] << "," << input_rect[3]
                      << "#" << cropped_rect[0] << "," << cropped_rect[1] << "," << cropped_rect[2] << "," << cropped_rect[3]
                      << "#";
            if (has_source_gl_rects)
            {
                src_canon << source_gl_viewport[0] << "," << source_gl_viewport[1] << "," << source_gl_viewport[2] << "," << source_gl_viewport[3]
                          << "|" << source_gl_scissor[0] << "," << source_gl_scissor[1] << "," << source_gl_scissor[2] << "," << source_gl_scissor[3];
            }
            else
            {
                src_canon << "null";
            }
            src_canon << "#";
            for (int eye = 0; eye < 2; eye++)
            {
                if (eye > 0)
                {
                    src_canon << "|";
                }
                src_canon << (has_eye_image_index[eye] ? std::to_string(eye_image_index[eye]) : std::string("-"))
                          << "@" << eye_swap_w[eye] << "x" << eye_swap_h[eye];
            }
            const std::string fp_source = LLOpenXRTrace::placementFingerprint(src_canon.str());
            const std::string fp_submission = LLOpenXRTrace::placementFingerprint(
                (source_render_fingerprint.empty() ? std::string("null") : source_render_fingerprint) +
                "|" + (fp_layer.empty() ? std::string("null") : fp_layer) + "|" + fp_source);
            f << ",\"placement_fingerprint_render\":";
            if (source_render_fingerprint.empty())
            {
                f << "null";
            }
            else
            {
                f << "\"" << source_render_fingerprint << "\"";
            }
            f << ",\"placement_fingerprint_layer\":";
            if (fp_layer.empty())
            {
                f << "null";
            }
            else
            {
                f << "\"" << fp_layer << "\"";
            }
            f << ",\"placement_fingerprint_source\":\"" << fp_source << "\"";
            f << ",\"placement_fingerprint_submission\":\"" << fp_submission << "\"";
        }
        if (has_avatar_eye)
        {
            f << ",\"avatar_eye_frame_token\":" << avatar_eye_frame_token;
            f << ",\"avatar_eye_frame_valid\":" << (avatar_eye_valid ? "true" : "false");
            f << ",\"avatar_eye_frame_invalid_reason\":\"" << avatar_eye_invalid_reason << "\"";
            f << ",\"avatar_eye_locate_generation\":" << avatar_eye_locate_generation;
            f << ",\"avatar_eye_head_pose_generation\":" << avatar_eye_head_pose_generation;
            f << ",\"avatar_eye_predicted_display_time_xr_ns\":" << avatar_eye_predicted_display_time;
            f << ",\"avatar_eye_left_cornea_agent\":[" << avatar_eye_left_cornea[0]
              << "," << avatar_eye_left_cornea[1] << "," << avatar_eye_left_cornea[2] << "]";
            f << ",\"avatar_eye_left_cornea_valid\":" << (avatar_eye_left_cornea_valid ? "true" : "false");
            if (avatar_eye_right_cornea_valid)
            {
                f << ",\"avatar_eye_right_cornea_agent\":[" << avatar_eye_right_cornea[0]
                  << "," << avatar_eye_right_cornea[1] << "," << avatar_eye_right_cornea[2] << "]";
            }
            else
            {
                f << ",\"avatar_eye_right_cornea_agent\":null";
            }
            f << ",\"avatar_eye_right_cornea_valid\":" << (avatar_eye_right_cornea_valid ? "true" : "false");
            f << ",\"avatar_eye_right_cornea_source\":\"" << avatar_eye_right_cornea_source << "\"";
            f << ",\"avatar_eye_left_cornea_point\":\"" << avatar_eye_left_cornea_point << "\"";
            f << ",\"avatar_eye_resolved_head_quat\":[" << avatar_eye_resolved_head_quat[0]
              << "," << avatar_eye_resolved_head_quat[1] << "," << avatar_eye_resolved_head_quat[2]
              << "," << avatar_eye_resolved_head_quat[3] << "]";
            f << ",\"avatar_eye_resolved_camera_quat\":[" << avatar_eye_resolved_camera_quat[0]
              << "," << avatar_eye_resolved_camera_quat[1] << "," << avatar_eye_resolved_camera_quat[2]
              << "," << avatar_eye_resolved_camera_quat[3] << "]";
            f << ",\"avatar_eye_gaze_composition_verified\":" << (avatar_eye_gaze_verified ? "true" : "false");
            f << ",\"avatar_eye_requested_near_far\":[" << avatar_eye_requested_near_far[0]
              << "," << avatar_eye_requested_near_far[1] << "]";
            f << ",\"avatar_eye_target_extent\":[" << avatar_eye_target_extent[0]
              << "," << avatar_eye_target_extent[1] << "]";
        }
        if (has_render_receipt)
        {
            f << ",\"render_receipt_token\":" << receipt_token;
            f << ",\"render_receipt_locate_generation\":" << receipt_locate_generation;
            f << ",\"render_receipt_completed\":" << (receipt_completed ? "true" : "false");
            f << ",\"render_receipt_consumed\":" << (receipt_consumed ? "true" : "false");
            f << ",\"render_receipt_validation_status\":\"" << receipt_validation_status << "\"";
        }
        LLOpenXRTrace::writeRecord("xr_submit", f.str());
    }

    ~XrSubmitTrace()
    {
        finish(status);
    }
};

// P2-02: adopt one located view set as the manager's current pose /
// FOV sample -- generation mint on the one locate counter, FOV
// caches + provenance sample, head forward/up. Extracted verbatim
// from submitFrameMono's legacy adoption block so the paced loop's
// pre-idle locate adopts identically to the legacy loop's
// after-render one. Callers have already applied the P0-06 validity
// gates. Returns the minted locate generation (0 when no view was
// usable).
unsigned long long adopt_located_views(LLOpenXRManagerImpl& impl,
                                       const std::vector<XrView>& views,
                                       uint32_t usable_view_count)
{
    unsigned long long this_locate_generation = 0;
    if (usable_view_count > 0)
    {
        const XrFovf& left_eye_fov = views[0].fov;

        // The locate passed the validity gates and its views are
        // adopted as the manager's current pose/FOV sample -- mint
        // its generation on the one counter.
        this_locate_generation = ++impl.locate_generation;
        const F32 vertical_fov = left_eye_fov.angleUp - left_eye_fov.angleDown;
        if (std::isfinite(vertical_fov) && vertical_fov > 0.f)
        {
            impl.latest_left_eye_vertical_fov = vertical_fov;
            impl.has_left_eye_vertical_fov = true;
        }
        else
        {
            impl.has_left_eye_vertical_fov = false;
        }

        const F32 horizontal_extent = tanf(left_eye_fov.angleRight) + tanf(-left_eye_fov.angleLeft);
        const F32 vertical_extent = tanf(left_eye_fov.angleUp) + tanf(-left_eye_fov.angleDown);
        const F32 aspect = horizontal_extent / llmax(vertical_extent, 0.00001f);
        if (std::isfinite(aspect) && aspect > 0.f)
        {
            impl.latest_left_eye_aspect = aspect;
            impl.has_left_eye_aspect = true;
        }
        else
        {
            impl.has_left_eye_aspect = false;
        }

        // Cache the full located left-eye FOV as a provenance
        // sample. Only a locate that passed the P0-06 validity gate
        // and yields finite angles with positive extent counts as a
        // valid replacement (locate_generation increments); anything
        // else invalidates the sample. Never synthetic.
        const bool fov_angles_finite =
            std::isfinite(left_eye_fov.angleLeft) && std::isfinite(left_eye_fov.angleRight) &&
            std::isfinite(left_eye_fov.angleDown) && std::isfinite(left_eye_fov.angleUp);
        if (fov_angles_finite && std::isfinite(vertical_fov) && vertical_fov > 0.f &&
            (left_eye_fov.angleRight - left_eye_fov.angleLeft) > 0.f)
        {
            LLXRLeftEyeFovSample sample;
            sample.left = left_eye_fov.angleLeft;
            sample.right = left_eye_fov.angleRight;
            sample.down = left_eye_fov.angleDown;
            sample.up = left_eye_fov.angleUp;
            sample.locate_generation = this_locate_generation;
            sample.located_monotonic_us = LLOpenXRTrace::nowUs();
            sample.valid = true;
            impl.left_eye_fov_sample = sample;
        }
        else
        {
            impl.left_eye_fov_sample.valid = false;
        }
    }
    else
    {
        impl.has_left_eye_vertical_fov = false;
        impl.has_left_eye_aspect = false;
        impl.left_eye_fov_sample.valid = false;
    }

    LLVector3 accumulated_forward;
    LLVector3 accumulated_up;
    for (uint32_t i = 0; i < usable_view_count; ++i)
    {
        accumulated_forward += xr_forward_from_orientation(views[i].pose.orientation);
        accumulated_up += xr_up_from_orientation(views[i].pose.orientation);
    }

    if (usable_view_count > 0 && accumulated_forward.normalize() > 0.f)
    {
        impl.latest_hmd_forward = accumulated_forward;
        impl.has_hmd_forward = true;
        impl.latest_pose_generation = this_locate_generation;
    }
    else
    {
        impl.has_hmd_forward = false;
    }

    if (usable_view_count > 0 && accumulated_up.normalize() > 0.f)
    {
        impl.latest_hmd_up = accumulated_up;
        impl.has_hmd_up = true;
    }
    else
    {
        impl.has_hmd_up = false;
    }

    return this_locate_generation;
}

// P2-02: build the xr_submit trace collector for a paced frame from
// its XRFrameContext (used by the zero-layer ends and the begin /
// wait failure records; the paced submit path itself fills its
// collector inline so the render provenance taken at its entry is
// preserved). The head generation counts only when the head
// consumed the pose during the context's own iteration (token
// match), exactly as on the legacy path.
// P2-03a: copy the frozen eye frame + receipt state into an
// xr_submit collector (paced records only).
static void fill_avatar_eye_trace(XrSubmitTrace& tr,
                                  const LLXRAvatarEyeFrame& eye,
                                  bool has_eye,
                                  const LLXRWorldRenderReceipt& receipt,
                                  bool has_receipt)
{
    tr.has_avatar_eye = has_eye;
    if (has_eye)
    {
        tr.avatar_eye_frame_token = eye.app_frame_token;
        tr.avatar_eye_valid = eye.valid;
        tr.avatar_eye_invalid_reason = eye.invalid_reason;
        tr.avatar_eye_locate_generation = eye.locate_generation;
        tr.avatar_eye_head_pose_generation = eye.head_consumed_pose_generation;
        tr.avatar_eye_predicted_display_time = eye.predicted_display_time_xr_ns;
        tr.avatar_eye_left_cornea_valid = eye.left_cornea_valid;
        tr.avatar_eye_left_cornea[0] = eye.left_cornea_agent.mV[0];
        tr.avatar_eye_left_cornea[1] = eye.left_cornea_agent.mV[1];
        tr.avatar_eye_left_cornea[2] = eye.left_cornea_agent.mV[2];
        tr.avatar_eye_right_cornea_valid = eye.right_cornea_valid;
        tr.avatar_eye_right_cornea[0] = eye.right_cornea_agent.mV[0];
        tr.avatar_eye_right_cornea[1] = eye.right_cornea_agent.mV[1];
        tr.avatar_eye_right_cornea[2] = eye.right_cornea_agent.mV[2];
        tr.avatar_eye_right_cornea_source = eye.right_cornea_source;
        tr.avatar_eye_left_cornea_point = eye.left_cornea_point;
        tr.avatar_eye_resolved_head_quat[0] = eye.resolved_head_world.mQ[0];
        tr.avatar_eye_resolved_head_quat[1] = eye.resolved_head_world.mQ[1];
        tr.avatar_eye_resolved_head_quat[2] = eye.resolved_head_world.mQ[2];
        tr.avatar_eye_resolved_head_quat[3] = eye.resolved_head_world.mQ[3];
        tr.avatar_eye_resolved_camera_quat[0] = eye.resolved_camera_world.mQ[0];
        tr.avatar_eye_resolved_camera_quat[1] = eye.resolved_camera_world.mQ[1];
        tr.avatar_eye_resolved_camera_quat[2] = eye.resolved_camera_world.mQ[2];
        tr.avatar_eye_resolved_camera_quat[3] = eye.resolved_camera_world.mQ[3];
        tr.avatar_eye_gaze_verified = eye.gaze_composition_verified;
        tr.avatar_eye_requested_near_far[0] = eye.requested_near;
        tr.avatar_eye_requested_near_far[1] = eye.requested_far;
        tr.avatar_eye_target_extent[0] = eye.target_width;
        tr.avatar_eye_target_extent[1] = eye.target_height;
    }
    tr.has_render_receipt = has_receipt;
    if (has_receipt)
    {
        tr.receipt_token = receipt.app_frame_token;
        tr.receipt_locate_generation = receipt.source_locate_generation;
        tr.receipt_consumed = receipt.consumed;
    }
}

XrSubmitTrace make_paced_trace(const LLOpenXRManagerImpl& impl,
                               const LLOpenXRManagerImpl::XRFrameContext& frame_ctx)
{
    XrSubmitTrace tr;
    tr.tracing = frame_ctx.tracing;
    tr.t_entry = frame_ctx.t_entry;
    tr.t_wait_start = frame_ctx.t_wait_start;
    tr.t_wait_end = frame_ctx.t_wait_end;
    tr.t_begin_start = frame_ctx.t_begin_start;
    tr.t_begin_end = frame_ctx.t_begin_end;
    tr.t_locate_start = frame_ctx.t_locate_start;
    tr.t_locate_end = frame_ctx.t_locate_end;
    tr.wait_result = frame_ctx.wait_result;
    tr.begin_result = frame_ctx.begin_result;
    tr.locate_result = frame_ctx.locate_result;
    tr.predicted_display_time = (long long)frame_ctx.frame_state.predictedDisplayTime;
    tr.predicted_display_period = (long long)frame_ctx.frame_state.predictedDisplayPeriod;
    tr.should_render = frame_ctx.frame_state.shouldRender;
    // Only claim a locate display time when a locate was actually
    // attempted for this frame (never on shouldRender=false or
    // wait_failed records).
    tr.has_locate_display_time = frame_ctx.locate_attempted;
    tr.locate_display_time = (long long)frame_ctx.frame_state.predictedDisplayTime;
    tr.located_view_count = frame_ctx.located_view_count;
    tr.view_state_flags = (unsigned long long)frame_ctx.view_state.viewStateFlags;
    tr.has_session_state = true;
    tr.session_state = frame_ctx.session_state;
    tr.has_pose_sample_valid = true;
    // P2-03a (review C3): fields describing what this paced frame
    // consumed are sourced from the frame-owned frozen snapshot
    // when the freeze ran -- the mutable cache may already hold
    // a later locate by the time the record is written.
    const LLXRLeftEyeFovSample& trace_sample =
        frame_ctx.eye_frame_frozen ? frame_ctx.eye_frame.fov_sample
                                   : impl.left_eye_fov_sample;
    tr.pose_sample_valid = trace_sample.valid;
    tr.optics_coherent = LLOpenXRManager::coherentOpticsEnabled();
    if (frame_ctx.located_generation > 0)
    {
        tr.has_located_generation = true;
        tr.located_generation = frame_ctx.located_generation;
    }
    if (frame_ctx.token != 0 && impl.head_consumed_app_frame_token == frame_ctx.token)
    {
        tr.has_head_generation = true;
        tr.head_generation = impl.head_consumed_pose_generation;
    }
    if (trace_sample.valid)
    {
        tr.has_latest_located = true;
        tr.latest_located_fov_rad[0] = trace_sample.left;
        tr.latest_located_fov_rad[1] = trace_sample.right;
        tr.latest_located_fov_rad[2] = trace_sample.down;
        tr.latest_located_fov_rad[3] = trace_sample.up;
    }
    const uint32_t usable_view_count = std::min<uint32_t>(frame_ctx.located_view_count,
                                                          (uint32_t)frame_ctx.located_views.size());
    for (uint32_t i = 0; i < usable_view_count && i < 2; ++i)
    {
        tr.has_eye[i] = true;
        tr.eye_pos[i][0] = frame_ctx.located_views[i].pose.position.x;
        tr.eye_pos[i][1] = frame_ctx.located_views[i].pose.position.y;
        tr.eye_pos[i][2] = frame_ctx.located_views[i].pose.position.z;
        tr.eye_quat[i][0] = frame_ctx.located_views[i].pose.orientation.x;
        tr.eye_quat[i][1] = frame_ctx.located_views[i].pose.orientation.y;
        tr.eye_quat[i][2] = frame_ctx.located_views[i].pose.orientation.z;
        tr.eye_quat[i][3] = frame_ctx.located_views[i].pose.orientation.w;
        tr.eye_fov[i][0] = frame_ctx.located_views[i].fov.angleLeft;
        tr.eye_fov[i][1] = frame_ctx.located_views[i].fov.angleRight;
        tr.eye_fov[i][2] = frame_ctx.located_views[i].fov.angleDown;
        tr.eye_fov[i][3] = frame_ctx.located_views[i].fov.angleUp;
    }
    fill_avatar_eye_trace(tr, frame_ctx.eye_frame, frame_ctx.eye_frame_frozen,
                          frame_ctx.render_receipt, frame_ctx.render_receipt.present);
    return tr;
}
} // namespace
#endif // LL_LINUX && LL_X11

bool LLOpenXRManager::ensureInitializedForFrame()
{
    if (mDisabled)
    {
        return false;
    }

    if (!mInitializeAttempted)
    {
        mInitializeAttempted = true;
        if (!initialize())
        {
#if LL_LINUX && LL_X11
            if (isMonoRequested())
            {
                openxr_warn("OpenXR initialize() returned false; VR submit disabled.");
            }
            else
            {
                write_openxr_file_log("INFO", std::string(OPENXR_MONO_ENV) + " not enabled; OpenXR path skipped.");
            }
#endif
            return false;
        }
    }

#if LL_LINUX && LL_X11
    return mImpl && mImpl->session != XR_NULL_HANDLE;
#else
    return false;
#endif
}

void LLOpenXRManager::submitFrameMono(const LLRect& source_rect)
{
    submitFrameMonoImpl(source_rect, false);
}

void LLOpenXRManager::submitPacedFrameMono(const LLRect& source_rect)
{
#if LL_LINUX && LL_X11
    if (!isPacedFrameLoopSelected() || !mImpl)
    {
        return;
    }
    const LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
    if (!frame_ctx.active || !frame_ctx.wait_succeeded)
    {
        return;
    }
    if (!frame_ctx.begun || frame_ctx.ended)
    {
        // The frame must already be begun: beginPacedFrameAfterIdle()
        // runs between idle() and display() (contract order). A frame
        // reaching the submit unbegun -- or already ended by its
        // pending zero-layer resolution -- is never submitted here;
        // the main-loop fallback closes an unbegun one with zero
        // layers after display().
        return;
    }
    submitFrameMonoImpl(source_rect, true);
    // The impl ends the frame on every exit path and marks the
    // context ended; if anything ever leaves it open, close it here
    // with zero layers rather than leaking an open frame into the
    // next iteration's wait.
    if (mImpl && mImpl->paced_frame.active)
    {
        if (mImpl->paced_frame.begun && !mImpl->paced_frame.ended)
        {
            closePacedFrameIfOpen("fallback_zero_layer");
        }
        if (mImpl && mImpl->paced_frame.active)
        {
            mImpl->paced_frame = LLOpenXRManagerImpl::XRFrameContext();
        }
    }
#else
    (void)source_rect;
#endif
}

void LLOpenXRManager::submitFrameMonoImpl(const LLRect& source_rect, bool paced)
{
#if LL_LINUX && LL_X11
    LLOpenXRTrace::flushIfDue();
    XrSubmitTrace xr_trace;
    xr_trace.tracing = LLOpenXRTrace::runActive();
    if (xr_trace.tracing)
    {
        xr_trace.t_entry = LLOpenXRTrace::nowUs();
    }
    // P1-01/P1-03: the image this call submits is the most recently
    // completed render; take the provenance stashed for that render.
    // The take is single-use and happens whether or not a trace run
    // is active: in coherent mode the snapshot is the submission's
    // source of truth, not just trace evidence. Never inferred from
    // frame ids.
    LLOpenXRTrace::RenderProvenance render_provenance;
    bool has_render_provenance = false;
    // P2-03a: on the paced path the display path registers a
    // frame-owned render receipt instead of stashing provenance;
    // translate it into the provenance shape so every downstream
    // consumer (trace population, readiness, strict coherence,
    // submitted FOV) reads one structure. The legacy path takes
    // the stash exactly as before.
    LLXRWorldRenderReceipt frame_receipt;
    bool has_frame_receipt = false;
    LLXRAvatarEyeFrame frame_eye;
    bool has_frame_eye = false;
    if (paced)
    {
        has_frame_eye = getAvatarEyeFrame(frame_eye);
        if (peekRenderReceipt(frame_receipt) &&
            frame_receipt.app_frame_token == LLOpenXRTrace::currentAppFrameToken())
        {
            has_frame_receipt = true;
            render_provenance.present = true;
            render_provenance.render_seq = frame_receipt.render_seq;
            render_provenance.fov_valid = frame_receipt.fov_valid;
            render_provenance.fov_generation = frame_receipt.source_locate_generation;
            for (int i = 0; i < 4; i++)
            {
                render_provenance.fov_rad[i] = frame_receipt.fov_rad[i];
            }
            render_provenance.offcenter_engaged = frame_receipt.offcenter_engaged;
            render_provenance.app_frame_token = frame_receipt.app_frame_token;
            render_provenance.has_world_gl = frame_receipt.has_world_gl;
            for (int i = 0; i < 16; i++)
            {
                render_provenance.world_gl_projection[i] = frame_receipt.world_gl_projection[i];
                render_provenance.world_gl_view[i] = frame_receipt.world_gl_view[i];
            }
            render_provenance.has_gl_rects = frame_receipt.has_gl_rects;
            for (int i = 0; i < 4; i++)
            {
                render_provenance.gl_viewport[i] = frame_receipt.gl_viewport[i];
                render_provenance.gl_scissor[i] = frame_receipt.gl_scissor[i];
            }
            render_provenance.gl_scissor_enabled = frame_receipt.gl_scissor_enabled;
            render_provenance.render_fingerprint = frame_receipt.render_fingerprint;
            has_render_provenance = true;
        }
        fill_avatar_eye_trace(xr_trace, frame_eye, has_frame_eye,
                              frame_receipt, has_frame_receipt);
    }
    else
    {
        has_render_provenance = LLOpenXRTrace::takeRenderProvenance(render_provenance);
    }
    const bool coherent_optics = LLOpenXRManager::coherentOpticsEnabled();
    xr_trace.optics_coherent = coherent_optics;
    if (xr_trace.tracing && has_render_provenance)
    {
        xr_trace.has_render_source = true;
        xr_trace.render_seq_source = render_provenance.render_seq;
        xr_trace.render_fov_valid = render_provenance.fov_valid;
        xr_trace.render_fov_generation = render_provenance.fov_generation;
        for (int i = 0; i < 4; i++)
        {
            xr_trace.render_fov_rad[i] = render_provenance.fov_rad[i];
        }
        // P2-01: the generation the render consumed, and the source
        // state transferred with its image (Amendment 4).
        if (render_provenance.fov_valid && render_provenance.fov_generation > 0)
        {
            xr_trace.has_render_generation = true;
            xr_trace.render_generation = render_provenance.fov_generation;
        }
        xr_trace.source_app_frame_token = render_provenance.app_frame_token;
        xr_trace.source_render_fingerprint = render_provenance.render_fingerprint;
        if (render_provenance.has_world_gl)
        {
            xr_trace.has_source_gl = true;
            for (int i = 0; i < 16; i++)
            {
                xr_trace.source_world_gl_projection[i] = render_provenance.world_gl_projection[i];
                xr_trace.source_world_gl_view[i] = render_provenance.world_gl_view[i];
            }
        }
        if (render_provenance.has_gl_rects)
        {
            xr_trace.has_source_gl_rects = true;
            for (int i = 0; i < 4; i++)
            {
                xr_trace.source_gl_viewport[i] = render_provenance.gl_viewport[i];
                xr_trace.source_gl_scissor[i] = render_provenance.gl_scissor[i];
            }
            xr_trace.source_gl_scissor_enabled = render_provenance.gl_scissor_enabled;
        }
    }
    static bool submit_entry_logged = false;
    if (!submit_entry_logged)
    {
        write_openxr_file_log(
            "INFO",
            std::string("submitFrameMono entered. ") + OPENXR_MONO_ENV + "=" +
            LLStringUtil::getenv(OPENXR_MONO_ENV) + ", XR_RUNTIME_JSON=" +
            LLStringUtil::getenv("XR_RUNTIME_JSON"));
        submit_entry_logged = true;
    }
#endif

    if (!paced)
    {
        // Legacy loop: this call owns initialization and session
        // events. The paced loop did both in beginPacedFrame()
        // before idle(); its submit only consumes the frame context.
        if (mDisabled)
        {
            return;
        }

        if (!ensureInitializedForFrame())
        {
            return;
        }
    }

#if LL_LINUX && LL_X11
    if (!mImpl || mImpl->session == XR_NULL_HANDLE)
    {
        return;
    }

    if (!paced)
    {
        processEvents();
    }
    // P2-01: iteration context for this call's record. The head
    // generation counts only when the head consumed the pose during
    // THIS main-loop iteration (token match); a stale stamp from an
    // earlier iteration is not this frame's head data.
    xr_trace.has_session_state = true;
    xr_trace.session_state = (int)mImpl->session_state;
    // P2-03a (review C3): on the paced path the record's sample
    // validity describes the frozen frame, not the live cache.
    xr_trace.pose_sample_valid =
        (paced && has_frame_eye) ? frame_eye.fov_sample.valid
                                 : mImpl->left_eye_fov_sample.valid;
    xr_trace.has_pose_sample_valid = true;
    if (mImpl->head_consumed_app_frame_token != 0 &&
        mImpl->head_consumed_app_frame_token == LLOpenXRTrace::currentAppFrameToken())
    {
        xr_trace.has_head_generation = true;
        xr_trace.head_generation = mImpl->head_consumed_pose_generation;
    }
    if (mDisabled || !mImpl->session_running)
    {
        return;
    }

    if (!mImpl->submit_ready_logged)
    {
        openxr_info("OpenXR submit loop active.");
        mImpl->submit_ready_logged = true;
    }

    // P2-02: frame state shared by the two loop modes. The legacy
    // branch below runs wait/begin/locate itself (the Phase 0/1
    // order: after the render); the paced branch transfers the
    // XRFrameContext that beginPacedFrame() waited on and located
    // before idle(). frame_views names the located views the
    // submission is built from in either mode.
    XrFrameState frame_state{};
    frame_state.type = XR_TYPE_FRAME_STATE;
    XrViewState view_state{};
    view_state.type = XR_TYPE_VIEW_STATE;
    uint32_t located_view_count = 0;
    unsigned long long this_locate_generation = 0;
    XrResult result = XR_SUCCESS;
    const std::vector<XrView>* frame_views = &mImpl->views;

    if (!paced)
    {
        XrFrameWaitInfo frame_wait_info{};
        frame_wait_info.type = XR_TYPE_FRAME_WAIT_INFO;

        // frame_state is declared above the loop-mode branch.

        if (xr_trace.tracing)
        {
            xr_trace.t_wait_start = LLOpenXRTrace::nowUs();
        }
        result = xrWaitFrame(mImpl->session, &frame_wait_info, &frame_state);
        xr_trace.wait_result = (int)result;
        xr_trace.predicted_display_time = (long long)frame_state.predictedDisplayTime;
        xr_trace.predicted_display_period = (long long)frame_state.predictedDisplayPeriod;
        xr_trace.should_render = frame_state.shouldRender;
        if (xr_trace.tracing)
        {
            xr_trace.t_wait_end = LLOpenXRTrace::nowUs();
        }
        if (XR_FAILED(result))
        {
            openxr_warn("xrWaitFrame failed: " + format_xr_result(mImpl->instance, result));
            xr_trace.finish("wait_failed");
            return;
        }

        XrFrameBeginInfo frame_begin_info{};
        frame_begin_info.type = XR_TYPE_FRAME_BEGIN_INFO;
        if (xr_trace.tracing)
        {
            xr_trace.t_begin_start = LLOpenXRTrace::nowUs();
        }
        result = xrBeginFrame(mImpl->session, &frame_begin_info);
        xr_trace.begin_result = (int)result;
        if (xr_trace.tracing)
        {
            xr_trace.t_begin_end = LLOpenXRTrace::nowUs();
        }
        if (XR_FAILED(result))
        {
            openxr_warn("xrBeginFrame failed: " + format_xr_result(mImpl->instance, result));
            xr_trace.finish("begin_failed");
            return;
        }

        if (!frame_state.shouldRender)
        {
            XrFrameEndInfo frame_end_info{};
            frame_end_info.type = XR_TYPE_FRAME_END_INFO;
            frame_end_info.displayTime = frame_state.predictedDisplayTime;
            frame_end_info.environmentBlendMode = mImpl->blend_mode;
            frame_end_info.layerCount = 0;
            frame_end_info.layers = nullptr;
            xr_trace.captureEndFrame(frame_end_info, nullptr, nullptr, 0);
            xr_trace.end_result = (int)xrEndFrame(mImpl->session, &frame_end_info);
            xr_trace.layer_count = 0;
            if (xr_trace.tracing)
            {
                xr_trace.t_end_end = LLOpenXRTrace::nowUs();
            }
            xr_trace.finish("skipped_should_render");
            return;
        }

        XrViewLocateInfo view_locate_info{};
        view_locate_info.type = XR_TYPE_VIEW_LOCATE_INFO;
        view_locate_info.viewConfigurationType = XR_VIEW_CONFIG;
        view_locate_info.displayTime = frame_state.predictedDisplayTime;
        view_locate_info.space = mImpl->app_space;

        // view_state is declared above the loop-mode branch.

        if (xr_trace.tracing)
        {
            xr_trace.t_locate_start = LLOpenXRTrace::nowUs();
        }
        xr_trace.has_locate_display_time = true;
        xr_trace.locate_display_time = (long long)view_locate_info.displayTime;
        result = xrLocateViews(
            mImpl->session,
            &view_locate_info,
            &view_state,
            (uint32_t)mImpl->views.size(),
            &located_view_count,
            mImpl->views.data());
        xr_trace.locate_result = (int)result;
        xr_trace.located_view_count = located_view_count;
        xr_trace.view_state_flags = (unsigned long long)view_state.viewStateFlags;
        if (xr_trace.tracing)
        {
            xr_trace.t_locate_end = LLOpenXRTrace::nowUs();
        }
        if (XR_FAILED(result) || located_view_count < 2)
        {
            openxr_warn("xrLocateViews failed: " + format_xr_result(mImpl->instance, result));
            mImpl->has_hmd_forward = false;
            mImpl->has_hmd_up = false;
            mImpl->has_left_eye_vertical_fov = false;
            mImpl->has_left_eye_aspect = false;
            mImpl->left_eye_fov_sample.valid = false;
            xr_trace.pose_sample_valid = false;
            XrFrameEndInfo frame_end_info{};
            frame_end_info.type = XR_TYPE_FRAME_END_INFO;
            frame_end_info.displayTime = frame_state.predictedDisplayTime;
            frame_end_info.environmentBlendMode = mImpl->blend_mode;
            frame_end_info.layerCount = 0;
            frame_end_info.layers = nullptr;
            // P2-01: finish() moves after this zero-layer end so the
            // record carries the submitted state and the end timestamps
            // (trace-only reorder; the end call itself is unchanged).
            xr_trace.captureEndFrame(frame_end_info, nullptr, nullptr, 0);
            xr_trace.end_result = (int)xrEndFrame(mImpl->session, &frame_end_info);
            if (xr_trace.tracing)
            {
                xr_trace.t_end_end = LLOpenXRTrace::nowUs();
            }
            xr_trace.finish("locate_failed");
            return;
        }

        // P0-06/V2: respect the located view state's validity flags. An
        // invalid pose must drive neither the avatar head nor a submitted
        // view; end the frame with no layers instead. Nothing here
        // recalibrates or reuses a stale pose.
        const bool orientation_valid =
            (view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0;
        const bool position_valid = (view_state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) != 0;
        if (!orientation_valid || !position_valid)
        {
            if (!orientation_valid)
            {
                mImpl->has_hmd_forward = false;
                mImpl->has_hmd_up = false;
            }
            mImpl->has_left_eye_vertical_fov = false;
            mImpl->has_left_eye_aspect = false;
            XrFrameEndInfo frame_end_info{};
            frame_end_info.type = XR_TYPE_FRAME_END_INFO;
            frame_end_info.displayTime = frame_state.predictedDisplayTime;
            frame_end_info.environmentBlendMode = mImpl->blend_mode;
            frame_end_info.layerCount = 0;
            frame_end_info.layers = nullptr;
            xr_trace.captureEndFrame(frame_end_info, nullptr, nullptr, 0);
            xr_trace.end_result = (int)xrEndFrame(mImpl->session, &frame_end_info);
            if (xr_trace.tracing)
            {
                xr_trace.t_end_end = LLOpenXRTrace::nowUs();
            }
            mImpl->left_eye_fov_sample.valid = false;
            xr_trace.pose_sample_valid = false;
            xr_trace.finish("invalid_view_state");
            return;
        }

        const uint32_t usable_view_count = std::min<uint32_t>(located_view_count, (uint32_t)mImpl->views.size());
        // P2-01: the generation this call's locate mints once its views
        // are adopted below (this_locate_generation is declared above
        // the loop-mode branch at function scope: the submitted-layer
        // generation fields at the final xrEndFrame derive from it).
        // P2-02: adoption (generation mint, FOV caches + sample, head
        // forward/up) lives in adopt_located_views() so the paced
        // loop's pre-idle locate adopts identically to this one.
        this_locate_generation = adopt_located_views(*mImpl, mImpl->views, usable_view_count);
        if (this_locate_generation > 0)
        {
            xr_trace.has_located_generation = true;
            xr_trace.located_generation = this_locate_generation;
        }

        // P2-01: mirror the sample's post-locate validity for the record.
        xr_trace.pose_sample_valid = mImpl->left_eye_fov_sample.valid;
        xr_trace.has_pose_sample_valid = true;

        // P1-01: the manager's latest located left-eye sample, for the
        // xr_submit record's render-vs-latest-located comparison.
        if (mImpl->left_eye_fov_sample.valid)
        {
            xr_trace.has_latest_located = true;
            xr_trace.latest_located_fov_rad[0] = mImpl->left_eye_fov_sample.left;
            xr_trace.latest_located_fov_rad[1] = mImpl->left_eye_fov_sample.right;
            xr_trace.latest_located_fov_rad[2] = mImpl->left_eye_fov_sample.down;
            xr_trace.latest_located_fov_rad[3] = mImpl->left_eye_fov_sample.up;
        }

        for (uint32_t i = 0; i < usable_view_count && i < 2; ++i)
        {
            xr_trace.has_eye[i] = true;
            xr_trace.eye_pos[i][0] = mImpl->views[i].pose.position.x;
            xr_trace.eye_pos[i][1] = mImpl->views[i].pose.position.y;
            xr_trace.eye_pos[i][2] = mImpl->views[i].pose.position.z;
            xr_trace.eye_quat[i][0] = mImpl->views[i].pose.orientation.x;
            xr_trace.eye_quat[i][1] = mImpl->views[i].pose.orientation.y;
            xr_trace.eye_quat[i][2] = mImpl->views[i].pose.orientation.z;
            xr_trace.eye_quat[i][3] = mImpl->views[i].pose.orientation.w;
            xr_trace.eye_fov[i][0] = mImpl->views[i].fov.angleLeft;
            xr_trace.eye_fov[i][1] = mImpl->views[i].fov.angleRight;
            xr_trace.eye_fov[i][2] = mImpl->views[i].fov.angleDown;
            xr_trace.eye_fov[i][3] = mImpl->views[i].fov.angleUp;
        }
    }
    else
    {
        // P2-02 paced loop: session events, xrWaitFrame and
        // xrLocateViews for THIS frame already ran in
        // beginPacedFrame() before idle(), and
        // beginPacedFrameAfterIdle() (called by doFrame between
        // idle() and display()) already began the frame. This
        // branch only transfers the frame context's state into the
        // submit path and enforces per-token source readiness.
        const LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
        frame_state = frame_ctx.frame_state;
        view_state = frame_ctx.view_state;
        located_view_count = frame_ctx.located_view_count;
        this_locate_generation = frame_ctx.located_generation;
        frame_views = &frame_ctx.located_views;
        xr_trace.t_entry = frame_ctx.t_entry;
        xr_trace.t_wait_start = frame_ctx.t_wait_start;
        xr_trace.t_wait_end = frame_ctx.t_wait_end;
        xr_trace.t_begin_start = frame_ctx.t_begin_start;
        xr_trace.t_begin_end = frame_ctx.t_begin_end;
        xr_trace.t_locate_start = frame_ctx.t_locate_start;
        xr_trace.t_locate_end = frame_ctx.t_locate_end;
        xr_trace.wait_result = frame_ctx.wait_result;
        xr_trace.begin_result = frame_ctx.begin_result;
        xr_trace.locate_result = frame_ctx.locate_result;
        xr_trace.predicted_display_time = (long long)frame_ctx.frame_state.predictedDisplayTime;
        xr_trace.predicted_display_period = (long long)frame_ctx.frame_state.predictedDisplayPeriod;
        xr_trace.should_render = frame_ctx.frame_state.shouldRender;
        xr_trace.has_locate_display_time = true;
        xr_trace.locate_display_time = (long long)frame_ctx.frame_state.predictedDisplayTime;
        xr_trace.located_view_count = frame_ctx.located_view_count;
        xr_trace.view_state_flags = (unsigned long long)frame_ctx.view_state.viewStateFlags;
        if (frame_ctx.located_generation > 0)
        {
            xr_trace.has_located_generation = true;
            xr_trace.located_generation = frame_ctx.located_generation;
        }
        // P2-03a (review C3): frozen sourcing, as in
        // make_paced_trace -- these fields describe the sample
        // this frame froze and consumed.
        const LLXRLeftEyeFovSample& submit_sample =
            has_frame_eye ? frame_eye.fov_sample
                          : mImpl->left_eye_fov_sample;
        xr_trace.pose_sample_valid = submit_sample.valid;
        xr_trace.has_pose_sample_valid = true;
        if (submit_sample.valid)
        {
            xr_trace.has_latest_located = true;
            xr_trace.latest_located_fov_rad[0] = submit_sample.left;
            xr_trace.latest_located_fov_rad[1] = submit_sample.right;
            xr_trace.latest_located_fov_rad[2] = submit_sample.down;
            xr_trace.latest_located_fov_rad[3] = submit_sample.up;
        }
        {
            const uint32_t usable_view_count = std::min<uint32_t>(located_view_count,
                                                                  (uint32_t)frame_ctx.located_views.size());
            for (uint32_t i = 0; i < usable_view_count && i < 2; ++i)
            {
                xr_trace.has_eye[i] = true;
                xr_trace.eye_pos[i][0] = frame_ctx.located_views[i].pose.position.x;
                xr_trace.eye_pos[i][1] = frame_ctx.located_views[i].pose.position.y;
                xr_trace.eye_pos[i][2] = frame_ctx.located_views[i].pose.position.z;
                xr_trace.eye_quat[i][0] = frame_ctx.located_views[i].pose.orientation.x;
                xr_trace.eye_quat[i][1] = frame_ctx.located_views[i].pose.orientation.y;
                xr_trace.eye_quat[i][2] = frame_ctx.located_views[i].pose.orientation.z;
                xr_trace.eye_quat[i][3] = frame_ctx.located_views[i].pose.orientation.w;
                xr_trace.eye_fov[i][0] = frame_ctx.located_views[i].fov.angleLeft;
                xr_trace.eye_fov[i][1] = frame_ctx.located_views[i].fov.angleRight;
                xr_trace.eye_fov[i][2] = frame_ctx.located_views[i].fov.angleDown;
                xr_trace.eye_fov[i][3] = frame_ctx.located_views[i].fov.angleUp;
            }
        }
        // Per-token source readiness: the only image this frame may
        // submit is the render produced by THIS iteration. Token
        // equality alone would not prove that -- the camera-step
        // provenance stash is written BEFORE the render, already
        // carrying this frame's token -- so the check also requires
        // the world-render completion marker (set by display() on
        // its main present path, after render_ui()/renderFinalize
        // for this token) to match. Anything
        // else -- no render this iteration, a stale stash, a camera
        // snapshot without a completed render -- ends the frame
        // with zero layers; a previous frame's image is never
        // reused.
        if (!has_render_provenance || render_provenance.app_frame_token != frame_ctx.token ||
            frame_receipt.consumed ||
            frame_receipt.source_locate_generation != frame_ctx.located_generation ||
            LLOpenXRTrace::lastWorldRenderCompletedToken() != frame_ctx.token ||
            !consumeRenderReceipt(frame_ctx.token))
        {
            XrFrameEndInfo frame_end_info{};
            frame_end_info.type = XR_TYPE_FRAME_END_INFO;
            frame_end_info.displayTime = frame_state.predictedDisplayTime;
            frame_end_info.environmentBlendMode = mImpl->blend_mode;
            frame_end_info.layerCount = 0;
            frame_end_info.layers = nullptr;
            xr_trace.captureEndFrame(frame_end_info, nullptr, nullptr, 0);
            xr_trace.end_result = (int)xrEndFrame(mImpl->session, &frame_end_info);
            if (XR_FAILED((XrResult)xr_trace.end_result))
            {
                // End-failure policy: record (end_result), warn,
                // retire -- never retry a failed end (see
                // endPacedFrameZeroLayer).
                openxr_warn("xrEndFrame failed (paced source-not-ready close): " +
                            format_xr_result(mImpl->instance, (XrResult)xr_trace.end_result));
            }
            xr_trace.layer_count = 0;
            if (xr_trace.tracing)
            {
                xr_trace.t_end_end = LLOpenXRTrace::nowUs();
            }
            mImpl->paced_frame.ended = true;
            xr_trace.receipt_validation_status = "source_not_ready";
            xr_trace.finish("paced_source_not_ready");
            return;
        }
        // Readiness proven for THIS frame's own render: the
        // receipt was consumed atomically by the checked API in
        // the condition above, so it can never authorize a
        // second submit. Validation status is NOT set here --
        // the coherent checks below decide it (review m2).
        frame_receipt.consumed = true;
        xr_trace.receipt_consumed = true;
        xr_trace.receipt_completed = true;
    }

    GLint previous_read_fbo = 0;
    GLint previous_draw_fbo = 0;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previous_read_fbo);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previous_draw_fbo);

    std::array<XrCompositionLayerProjectionView, 2> layer_views;
    for (auto& layer_view : layer_views)
    {
        init_xr_struct(layer_view, XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW);
    }

    LLRect submit_input_rect = source_rect;
    bool using_pipeline_submit_source = false;
    const LLRenderTarget* xr_submit_source = nullptr;
    if (LLRenderTarget::sUseFBO)
    {
        xr_submit_source = gPipeline.getXRSubmitSource();
        if (xr_submit_source && xr_submit_source->isComplete() &&
            xr_submit_source->getNumTextures() > 0 &&
            xr_submit_source->getWidth() > 0 && xr_submit_source->getHeight() > 0)
        {
            if (mImpl->source_read_fbo == 0)
            {
                glGenFramebuffers(1, &mImpl->source_read_fbo);
            }

            if (mImpl->source_read_fbo != 0)
            {
                glBindFramebuffer(GL_READ_FRAMEBUFFER, mImpl->source_read_fbo);
                glFramebufferTexture2D(
                    GL_READ_FRAMEBUFFER,
                    GL_COLOR_ATTACHMENT0,
                    GL_TEXTURE_2D,
                    xr_submit_source->getTexture(0),
                    0);
                glReadBuffer(GL_COLOR_ATTACHMENT0);

                if (glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
                {
                    using_pipeline_submit_source = true;
                    submit_input_rect.mLeft = 0;
                    submit_input_rect.mBottom = 0;
                    submit_input_rect.mRight = (S32)xr_submit_source->getWidth();
                    submit_input_rect.mTop = (S32)xr_submit_source->getHeight();
                }
                else
                {
                    openxr_warn("OpenXR pipeline submit source framebuffer incomplete; falling back to desktop backbuffer.");
                }
            }
        }
    }

    if (coherent_optics)
    {
        // P1-03 strict coherence: the submitted image must be the
        // native-extent pipeline render whose off-centre projection
        // used the snapshot FOV, and that FOV must still agree with
        // the currently located left-eye FOV (the sample this call's
        // own xrLocateViews just cached). Any doubt ends the frame
        // with ZERO layers -- never the desktop backbuffer, never a
        // fabricated or freshly located substitute FOV. The rejection
        // is the frame's trace status, so it is visible in evidence.
        const char* strict_status = nullptr;
        if (!has_render_provenance)
        {
            strict_status = "strict_no_snapshot";
        }
        else if (!render_provenance.fov_valid || render_provenance.fov_generation == 0)
        {
            strict_status = "strict_snapshot_invalid";
        }
        else if (!render_provenance.offcenter_engaged)
        {
            strict_status = "strict_render_not_offcenter";
        }
        else if (paced ? !(has_frame_eye && frame_eye.fov_sample.valid)
                       : !mImpl->left_eye_fov_sample.valid)
        {
            strict_status = "strict_no_current_sample";
        }
        else
        {
            const LLXRLeftEyeFovSample& current_sample =
                paced ? frame_eye.fov_sample : mImpl->left_eye_fov_sample;
            const float current_rad[4] = {current_sample.left, current_sample.right,
                                          current_sample.down, current_sample.up};
            float max_delta_rad = 0.f;
            for (int i = 0; i < 4; i++)
            {
                max_delta_rad = std::max(max_delta_rad,
                                         std::fabs(render_provenance.fov_rad[i] - current_rad[i]));
            }
            xr_trace.has_coherence_delta = true;
            xr_trace.coherence_max_delta_deg = max_delta_rad * 57.29577951308232f;
            // P2-03a: on the paced path the delta (receipt FOV
            // vs this frame's frozen located sample) is RECORDED
            // but no longer rejects -- there is no second locate
            // for it to drift against (design review Q2).
            if (!paced && xr_trace.coherence_max_delta_deg > 0.05f)
            {
                strict_status = "strict_fov_drift";
            }
        }
        if (!strict_status && paced)
        {
            // P2-03a independent-fact check (fix 1): the near plane
            // the render ACTUALLY installed is derived from the
            // receipt's captured GL projection -- read from real GL
            // state after setup3DRender(), not the camera's near
            // field copied twice -- and must equal the frozen
            // requested near. Far is recorded, not enforced --
            // display legitimately adjusts it. Column-major glm
            // order: m[10] = -(f+n)/(f-n), m[14] = -2fn/(f-n),
            // so near = m[14] / (m[10] - 1).
            bool near_agrees = false;
            if (has_frame_receipt && has_frame_eye && frame_receipt.has_world_gl)
            {
                const float gl_m10 = frame_receipt.world_gl_projection[10];
                const float gl_m14 = frame_receipt.world_gl_projection[14];
                const float denom = gl_m10 - 1.0f;
                if (std::fabs(denom) > 1e-6f)
                {
                    const float derived_near = gl_m14 / denom;
                    const float near_tolerance = std::max(0.0001f,
                        frame_eye.requested_near * 0.01f);
                    near_agrees = (derived_near > 0.f) &&
                        (std::fabs(derived_near - frame_eye.requested_near) <=
                            near_tolerance);
                }
            }
            if (!near_agrees)
            {
                strict_status = "avatar_eye_projection_mismatch";
            }
        }
        if (!strict_status && paced)
        {
            // P2-03a source-extent agreement (design review M2):
            // the receipt's measured source extent (the world-
            // render view rect swap() submits from, captured at
            // registration) must equal the frozen target extent.
            // The swapchain extent check below is retained.
            if (!has_frame_receipt || !has_frame_eye ||
                frame_receipt.source_width != frame_eye.target_width ||
                frame_receipt.source_height != frame_eye.target_height)
            {
                strict_status = "avatar_eye_source_extent_mismatch";
            }
        }
        if (!strict_status && !using_pipeline_submit_source)
        {
            strict_status = "strict_no_pipeline_source";
        }
        if (!strict_status && !mImpl->swapchains.empty())
        {
            const OpenXRSwapchain& left_swapchain = mImpl->swapchains[0];
            if (submit_input_rect.mLeft != 0 || submit_input_rect.mBottom != 0 ||
                submit_input_rect.getWidth() != (S32)left_swapchain.width ||
                submit_input_rect.getHeight() != (S32)left_swapchain.height)
            {
                strict_status = "strict_extent_mismatch";
            }
        }
        if (strict_status)
        {
            openxr_warn(std::string("OpenXR coherent submit rejected (") + strict_status +
                        "); submitting no layer.");
            XrFrameEndInfo frame_end_info{};
            frame_end_info.type = XR_TYPE_FRAME_END_INFO;
            frame_end_info.displayTime = frame_state.predictedDisplayTime;
            frame_end_info.environmentBlendMode = mImpl->blend_mode;
            frame_end_info.layerCount = 0;
            frame_end_info.layers = nullptr;
            xr_trace.captureEndFrame(frame_end_info, nullptr, nullptr, 0);
            xr_trace.end_result = (int)xrEndFrame(mImpl->session, &frame_end_info);
            if (paced)
            {
                mImpl->paced_frame.ended = true;
                xr_trace.receipt_validation_status = strict_status;
            }
            if (xr_trace.tracing)
            {
                xr_trace.t_end_end = LLOpenXRTrace::nowUs();
            }
            xr_trace.finish(strict_status);
            return;
        }
        if (paced)
        {
            // Coherent validation passed on the paced path
            // (review m2): only NOW is the receipt's
            // validation status "ok".
            xr_trace.receipt_validation_status = "ok";
        }
    }
    else if (!using_pipeline_submit_source)
    {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

        // P0-08/D13: framebuffer 0 is the desktop backbuffer, sized by
        // the window -- not by the XR render rect. Clamp the source
        // rect to the real drawable bounds and log any clamp; never
        // blit from outside the backbuffer.
        const S32 backbuffer_w = gViewerWindow ? gViewerWindow->getWindowWidthRaw() : 0;
        const S32 backbuffer_h = gViewerWindow ? gViewerWindow->getWindowHeightRaw() : 0;
        LLRect clamped_rect = submit_input_rect;
        clamped_rect.mLeft = llclamp(clamped_rect.mLeft, 0, backbuffer_w);
        clamped_rect.mRight = llclamp(clamped_rect.mRight, 0, backbuffer_w);
        clamped_rect.mBottom = llclamp(clamped_rect.mBottom, 0, backbuffer_h);
        clamped_rect.mTop = llclamp(clamped_rect.mTop, 0, backbuffer_h);
        const bool rect_clamped = clamped_rect.mLeft != submit_input_rect.mLeft ||
                                  clamped_rect.mRight != submit_input_rect.mRight ||
                                  clamped_rect.mBottom != submit_input_rect.mBottom ||
                                  clamped_rect.mTop != submit_input_rect.mTop;
        if (rect_clamped)
        {
            openxr_warn("OpenXR backbuffer fallback: source rect " +
                        std::to_string(submit_input_rect.getWidth()) + "x" +
                        std::to_string(submit_input_rect.getHeight()) +
                        " exceeds framebuffer 0 (" + std::to_string(backbuffer_w) + "x" +
                        std::to_string(backbuffer_h) + "); clamped.");
            submit_input_rect = clamped_rect;
        }
        if (submit_input_rect.getWidth() <= 0 || submit_input_rect.getHeight() <= 0)
        {
            openxr_warn("OpenXR backbuffer fallback: no valid source area; submitting no layer.");
            XrFrameEndInfo frame_end_info{};
            frame_end_info.type = XR_TYPE_FRAME_END_INFO;
            frame_end_info.displayTime = frame_state.predictedDisplayTime;
            frame_end_info.environmentBlendMode = mImpl->blend_mode;
            frame_end_info.layerCount = 0;
            frame_end_info.layers = nullptr;
            xr_trace.captureEndFrame(frame_end_info, nullptr, nullptr, 0);
            xr_trace.end_result = (int)xrEndFrame(mImpl->session, &frame_end_info);
            if (paced)
            {
                mImpl->paced_frame.ended = true;
            }
            if (xr_trace.tracing)
            {
                xr_trace.t_end_end = LLOpenXRTrace::nowUs();
            }
            xr_trace.finish("d13_empty_source");
            return;
        }
    }

    // Mono submit uses one rendered viewpoint duplicated to both eyes
    // (the averaged centre pose is unchanged in both modes).
    const XrPosef mono_pose = xr_center_pose_from_views((*frame_views)[0], (*frame_views)[1]);
    LLRect submit_source_rect;
    XrFovf mono_fov{};
    if (coherent_optics)
    {
        // P1-03: no crop, no fabricated FOV. The source is the full
        // native left-eye extent (the strict gates above verified it
        // matches the left swapchain, so input_rect == cropped_rect
        // and the left blit is 1.000 by construction), and the
        // submitted FOV is the render's own snapshot, exactly. Both
        // eyes receive this FOV (mono-right duplication policy).
        submit_source_rect = submit_input_rect;
        mono_fov.angleLeft = render_provenance.fov_rad[0];
        mono_fov.angleRight = render_provenance.fov_rad[1];
        mono_fov.angleDown = render_provenance.fov_rad[2];
        mono_fov.angleUp = render_provenance.fov_rad[3];
    }
    else
    {
        // Legacy Phase 0 path (FS_OPENXR_P1_OPTICS_EXPERIMENTAL=0):
        // keep submit projection independent from desktop window size
        // by center-cropping the source image to runtime aspect when
        // needed, with a symmetric FOV fabricated for the cropped
        // region. Preserved verbatim for A/B comparison.
        const XrFovf& left_eye_fov = (*frame_views)[0].fov;
        F32 runtime_aspect = 0.f;
        if (!xr_fov_aspect(left_eye_fov, runtime_aspect))
        {
            const F32 source_width = llmax((F32)submit_input_rect.getWidth(), 1.f);
            const F32 source_height = llmax((F32)submit_input_rect.getHeight(), 1.f);
            runtime_aspect = source_width / source_height;
        }
        submit_source_rect = legacy_center_crop_rect_to_aspect(submit_input_rect, runtime_aspect);
        mono_fov = legacy_xr_centered_mono_fov_for_source_crop(left_eye_fov, submit_input_rect, submit_source_rect);
    }

    xr_trace.using_pipeline_source = using_pipeline_submit_source;
    xr_trace.input_rect[0] = submit_input_rect.mLeft;
    xr_trace.input_rect[1] = submit_input_rect.mBottom;
    xr_trace.input_rect[2] = submit_input_rect.getWidth();
    xr_trace.input_rect[3] = submit_input_rect.getHeight();
    xr_trace.cropped_rect[0] = submit_source_rect.mLeft;
    xr_trace.cropped_rect[1] = submit_source_rect.mBottom;
    xr_trace.cropped_rect[2] = submit_source_rect.getWidth();
    xr_trace.cropped_rect[3] = submit_source_rect.getHeight();
    xr_trace.mono_fov[0] = mono_fov.angleLeft;
    xr_trace.mono_fov[1] = mono_fov.angleRight;
    xr_trace.mono_fov[2] = mono_fov.angleDown;
    xr_trace.mono_fov[3] = mono_fov.angleUp;
    xr_trace.mono_pos[0] = mono_pose.position.x;
    xr_trace.mono_pos[1] = mono_pose.position.y;
    xr_trace.mono_pos[2] = mono_pose.position.z;
    xr_trace.mono_quat[0] = mono_pose.orientation.x;
    xr_trace.mono_quat[1] = mono_pose.orientation.y;
    xr_trace.mono_quat[2] = mono_pose.orientation.z;
    xr_trace.mono_quat[3] = mono_pose.orientation.w;

    // P2-01B: diagnostic source-image marker state for this
    // submission. The encoded value is the low 10 bits of the
    // producing iteration's app_frame_token (maintained whether or
    // not a trace run is active); it is drawn into each eye image
    // whose blit completes, and recorded on the xr_submit record so
    // a headset recording decodes straight back to this submission.
    const bool diag_marker_enabled = LLOpenXRManager::diagFrameMarkerEnabled();
    const unsigned int diag_marker_value = diag_marker_enabled
        ? (unsigned int)(LLOpenXRTrace::currentAppFrameToken() & 0x3FFULL)
        : 0;
    xr_trace.diagnostic_marker_enabled = diag_marker_enabled;

    uint32_t submitted_view_count = 0;
    bool any_eye_failed = false;
    for (uint32_t eye = 0; eye < 2 && eye < (uint32_t)mImpl->swapchains.size(); ++eye)
    {
        OpenXRSwapchain& swapchain = mImpl->swapchains[eye];
        if (swapchain.handle == XR_NULL_HANDLE || swapchain.images.empty())
        {
            continue;
        }

        XrSwapchainImageAcquireInfo acquire_info{};
        acquire_info.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO;

        uint32_t image_index = 0;
        xr_trace.eye_attempted[eye] = true;
        xr_trace.eye_swap_w[eye] = swapchain.width;
        xr_trace.eye_swap_h[eye] = swapchain.height;
        xr_trace.eye_swap_format[eye] = (long long)swapchain.format;
        xr_trace.eye_swap_sample[eye] = swapchain.sample_count;
        result = xrAcquireSwapchainImage(swapchain.handle, &acquire_info, &image_index);
        xr_trace.eye_acquire_result[eye] = (int)result;
        if (XR_SUCCEEDED(result))
        {
            xr_trace.has_eye_image_index[eye] = true;
            xr_trace.eye_image_index[eye] = image_index;
        }
        xr_trace.eye_submitted[eye] = false;
        if (XR_FAILED(result))
        {
            openxr_warn("xrAcquireSwapchainImage failed: " + format_xr_result(mImpl->instance, result));
            any_eye_failed = true;
            continue;
        }

        XrSwapchainImageWaitInfo wait_info{};
        wait_info.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO;
        wait_info.timeout = XR_INFINITE_DURATION;
        result = xrWaitSwapchainImage(swapchain.handle, &wait_info);
        xr_trace.eye_wait_result[eye] = (int)result;
        if (XR_FAILED(result))
        {
            openxr_warn("xrWaitSwapchainImage failed: " + format_xr_result(mImpl->instance, result));
            XrSwapchainImageReleaseInfo release_info{};
            release_info.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
            xr_trace.eye_release_result[eye] = (int)xrReleaseSwapchainImage(swapchain.handle, &release_info);
            any_eye_failed = true;
            continue;
        }

        const GLuint color_texture = swapchain.images[image_index].image;

        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mImpl->blit_fbo);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture, 0);

        xr_trace.eye_fbo_complete[eye] =
            (glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
        if (xr_trace.eye_fbo_complete[eye])
        {
            xr_trace.eye_blit_done[eye] = true;
            // P1-03: record the copy's scale per eye (swapchain extent
            // over source extent). The left eye is 1.000 by
            // construction in coherent mode; a differently sized right
            // swapchain shows its own scale here, never folded into
            // the left figure.
            xr_trace.has_blit_scale[eye] = true;
            xr_trace.blit_scale[eye][0] = (F32)swapchain.width / (F32)llmax(submit_source_rect.getWidth(), 1);
            xr_trace.blit_scale[eye][1] = (F32)swapchain.height / (F32)llmax(submit_source_rect.getHeight(), 1);
            // P2-01: the blit actually performed for this eye
            // (source rect, full-swapchain dest, GL_LINEAR filter).
            xr_trace.has_blit_rects[eye] = true;
            xr_trace.blit_src_rect[eye][0] = submit_source_rect.mLeft;
            xr_trace.blit_src_rect[eye][1] = submit_source_rect.mBottom;
            xr_trace.blit_src_rect[eye][2] = submit_source_rect.getWidth();
            xr_trace.blit_src_rect[eye][3] = submit_source_rect.getHeight();
            xr_trace.blit_dst_rect[eye][0] = 0;
            xr_trace.blit_dst_rect[eye][1] = 0;
            xr_trace.blit_dst_rect[eye][2] = (int)swapchain.width;
            xr_trace.blit_dst_rect[eye][3] = (int)swapchain.height;
            glBlitFramebuffer(
                submit_source_rect.mLeft,
                submit_source_rect.mBottom,
                submit_source_rect.mRight,
                submit_source_rect.mTop,
                0,
                0,
                (GLint)swapchain.width,
                (GLint)swapchain.height,
                GL_COLOR_BUFFER_BIT,
                GL_LINEAR);

            draw_openxr_cursor_overlay(submit_source_rect, swapchain.width, swapchain.height);

            // P2-01B: diagnostic frame marker, drawn LAST (after
            // the cursor overlay) so nothing overdraws it, into
            // this eye's swapchain image only.
            if (diag_marker_enabled)
            {
                draw_openxr_diag_frame_marker(swapchain.width, swapchain.height, diag_marker_value);
                xr_trace.has_diagnostic_marker_value = true;
                xr_trace.diagnostic_marker_value = diag_marker_value;
            }
        }
        else
        {
            openxr_warn("OpenXR swapchain framebuffer incomplete, skipping eye " + std::to_string(eye));
            any_eye_failed = true;
        }

        XrSwapchainImageReleaseInfo release_info{};
        release_info.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
        const XrResult release_result = xrReleaseSwapchainImage(swapchain.handle, &release_info);
        xr_trace.eye_release_result[eye] = (int)release_result;

        // P0-05/V1: per-eye success means the copy actually completed and
        // the image was released cleanly. Anything less fails the frame.
        const bool eye_ok = xr_trace.eye_fbo_complete[eye] && xr_trace.eye_blit_done[eye] &&
                            XR_SUCCEEDED(release_result);
        xr_trace.eye_submitted[eye] = eye_ok;
        if (!eye_ok)
        {
            any_eye_failed = true;
            continue;
        }

        XrCompositionLayerProjectionView& projection_view = layer_views[submitted_view_count];
        projection_view.pose = mono_pose;
        projection_view.fov = mono_fov;
        projection_view.subImage.swapchain = swapchain.handle;
        projection_view.subImage.imageRect.offset = {0, 0};
        projection_view.subImage.imageRect.extent = {(int32_t)swapchain.width, (int32_t)swapchain.height};
        projection_view.subImage.imageArrayIndex = 0;
        submitted_view_count++;
    }

    glBindFramebuffer(GL_READ_FRAMEBUFFER, previous_read_fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, previous_draw_fbo);

    XrCompositionLayerProjection projection_layer{};
    projection_layer.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION;
    projection_layer.space = mImpl->app_space;
    projection_layer.viewCount = submitted_view_count;
    projection_layer.views = layer_views.data();

    const XrCompositionLayerBaseHeader* layers[] =
    {
        reinterpret_cast<const XrCompositionLayerBaseHeader*>(&projection_layer)
    };

    XrFrameEndInfo frame_end_info{};
    frame_end_info.type = XR_TYPE_FRAME_END_INFO;
    frame_end_info.displayTime = frame_state.predictedDisplayTime;
    frame_end_info.environmentBlendMode = mImpl->blend_mode;
    const bool submit_layer = (submitted_view_count == 2 && !any_eye_failed);
    frame_end_info.layerCount = submit_layer ? 1 : 0;
    frame_end_info.layers = submit_layer ? layers : nullptr;

    xr_trace.submitted_view_count = submitted_view_count;
    xr_trace.layer_count = frame_end_info.layerCount;
    // P2-01 / Amendment 4: capture the exact submitted state
    // immediately before xrEndFrame hands it to the runtime. The
    // submitted pose is this call's locate (mono_pose is built from
    // this call's views); the submitted FOV is the render snapshot's
    // generation in coherent mode and this locate's in legacy mode
    // (fabricated from this locate's views).
    if (submit_layer)
    {
        xr_trace.has_submitted_pose_generation = true;
        xr_trace.submitted_pose_generation = this_locate_generation;
        xr_trace.has_layer_generation = true;
        xr_trace.layer_generation = this_locate_generation;
        if (coherent_optics)
        {
            if (has_render_provenance && render_provenance.fov_valid)
            {
                xr_trace.has_submitted_fov_generation = true;
                xr_trace.submitted_fov_generation = render_provenance.fov_generation;
            }
        }
        else
        {
            xr_trace.has_submitted_fov_generation = true;
            xr_trace.submitted_fov_generation = this_locate_generation;
        }
    }
    // app_space is the session's XR_REFERENCE_SPACE_TYPE_LOCAL
    // reference space (created in initialize()); its handle value is
    // the space identity within the run.
    xr_trace.captureEndFrame(frame_end_info, submit_layer ? &projection_layer : nullptr,
                             "LOCAL", (unsigned long long)(uintptr_t)mImpl->app_space);
    result = xrEndFrame(mImpl->session, &frame_end_info);
    xr_trace.end_result = (int)result;
    if (paced)
    {
        mImpl->paced_frame.ended = true;
        if (XR_FAILED(result))
        {
            // End-failure policy: recorded in end_result above,
            // warned here, retired without retry (see
            // endPacedFrameZeroLayer).
            openxr_warn("xrEndFrame failed (paced submit): " + format_xr_result(mImpl->instance, result));
        }
    }
    if (xr_trace.tracing)
    {
        xr_trace.t_end_end = LLOpenXRTrace::nowUs();
    }
    if (XR_FAILED(result))
    {
        openxr_warn("xrEndFrame failed: " + format_xr_result(mImpl->instance, result));
        xr_trace.finish("end_failed");
        return;
    }
    xr_trace.finish(submit_layer ? "submitted"
                                 : ((submitted_view_count > 0 || any_eye_failed) ? "partial_dropped"
                                                                                  : "no_views"));

    mImpl->frame_submit_count++;
    if (mImpl->frame_submit_count == 1 || (mImpl->frame_submit_count % 300) == 0)
    {
        openxr_info("Submitted OpenXR frames: " + std::to_string(mImpl->frame_submit_count));
    }

#else
    (void)source_rect;
#endif
}

bool LLOpenXRManager::isPacedFrameLoopSelected() const
{
    return pacedFrameLoopEnabled() && isMonoRequested();
}

bool LLOpenXRManager::hasOpenPacedFrame() const
{
#if LL_LINUX && LL_X11
    return mImpl && mImpl->paced_frame.active && mImpl->paced_frame.begun &&
           !mImpl->paced_frame.ended;
#else
    return false;
#endif
}

// P2-02: the paced loop's pre-idle frame begin. Called by
// LLAppViewer::doFrame() immediately before idle(). Session events
// first, then the synchronous xrWaitFrame (the runtime paces the
// loop here; contract section 4: no bound, no watchdog, no skip --
// telemetry only), then xrLocateViews at the frame's predicted
// display time, then adoption + publication of the pose/FOV
// snapshot, so idle()'s updateLookAt and animation resolve ONCE
// against this frame's locate. Outcomes that end the frame with
// zero layers (shouldRender=false, locate failure, invalid view
// state) are recorded on the context and resolved at
// xrBeginFrame time, after idle.
void LLOpenXRManager::beginPacedFrame()
{
#if LL_LINUX && LL_X11
    if (!isPacedFrameLoopSelected())
    {
        return;
    }
    LLOpenXRTrace::setAppFrameLoopMode("paced_pre_idle");

    if (mImpl && mImpl->paced_frame.active)
    {
        // A frame left open by a previous iteration (its fallback
        // close should have handled it) is closed before a new one
        // is waited on -- the runtime forbids a second wait while a
        // frame is open.
        closePacedFrameIfOpen("fallback_zero_layer");
        if (mImpl && mImpl->paced_frame.active)
        {
            mImpl->paced_frame = LLOpenXRManagerImpl::XRFrameContext();
        }
    }

    if (mDisabled)
    {
        return;
    }

    if (!mInitializeAttempted)
    {
        // initialize() binds the session to the current GLX context
        // and disables the manager permanently when none is current.
        // The legacy loop first initializes from swap(), where a
        // context is guaranteed; at this pre-idle point a
        // context-less iteration simply retries on the next one
        // instead of disabling VR.
        if (!glXGetCurrentContext() || !glXGetCurrentDrawable() ||
            !LLWindowSDL::get_SDL_Display())
        {
            return;
        }
    }

    if (!ensureInitializedForFrame())
    {
        return;
    }

    processEvents();
    if (mDisabled || !mImpl->session_running)
    {
        return;
    }

    LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
    frame_ctx = LLOpenXRManagerImpl::XRFrameContext();
    frame_ctx.active = true;
    frame_ctx.token = LLOpenXRTrace::currentAppFrameToken();
    frame_ctx.tracing = LLOpenXRTrace::runActive();
    frame_ctx.t_entry = LLOpenXRTrace::nowUs();
    frame_ctx.session_state = (int)mImpl->session_state;

    XrFrameWaitInfo frame_wait_info{};
    frame_wait_info.type = XR_TYPE_FRAME_WAIT_INFO;
    frame_ctx.frame_state.type = XR_TYPE_FRAME_STATE;
    frame_ctx.t_wait_start = LLOpenXRTrace::nowUs();
    const XrResult wait_result = xrWaitFrame(mImpl->session, &frame_wait_info, &frame_ctx.frame_state);
    frame_ctx.wait_result = (int)wait_result;
    frame_ctx.t_wait_end = LLOpenXRTrace::nowUs();
    if (XR_FAILED(wait_result))
    {
        openxr_warn("xrWaitFrame failed (paced loop): " + format_xr_result(mImpl->instance, wait_result));
        XrSubmitTrace xr_trace = make_paced_trace(*mImpl, frame_ctx);
        xr_trace.finish("wait_failed");
        frame_ctx = LLOpenXRManagerImpl::XRFrameContext();
        return;
    }
    frame_ctx.wait_succeeded = true;

    // Blocking telemetry (contract section 4): warn past two
    // predicted periods, severe past 100 ms. A returned long wait
    // is processed normally either way.
    {
        const long long wait_us = frame_ctx.t_wait_end - frame_ctx.t_wait_start;
        const long long period_us = (long long)frame_ctx.frame_state.predictedDisplayPeriod / 1000;
        if (wait_us > 100000 || (period_us > 0 && wait_us > 2 * period_us))
        {
            openxr_warn("xrWaitFrame blocked " + std::to_string(wait_us) + " us (paced loop" +
                        (wait_us > 100000 ? ", severe" : "") + ").");
        }
    }

    if (!frame_ctx.frame_state.shouldRender)
    {
        // No locate will happen for this frame: explicitly retire
        // the published snapshot so idle() does not consume a
        // previous frame's pose/FOV as if it were current. (In the
        // legacy loop locate ran after the render, so this staleness
        // was invisible; pre-idle it must be explicit.) Mirrors the
        // locate-failure invalidation below.
        mImpl->has_hmd_forward = false;
        mImpl->has_hmd_up = false;
        mImpl->has_left_eye_vertical_fov = false;
        mImpl->has_left_eye_aspect = false;
        mImpl->left_eye_fov_sample.valid = false;
        frame_ctx.pending_end_status = "skipped_should_render";
        return;
    }

    if (mImpl->views.empty())
    {
        frame_ctx.pending_end_status = "locate_failed";
        return;
    }

    XrViewLocateInfo view_locate_info{};
    view_locate_info.type = XR_TYPE_VIEW_LOCATE_INFO;
    view_locate_info.viewConfigurationType = XR_VIEW_CONFIG;
    view_locate_info.displayTime = frame_ctx.frame_state.predictedDisplayTime;
    view_locate_info.space = mImpl->app_space;

    frame_ctx.view_state.type = XR_TYPE_VIEW_STATE;
    frame_ctx.located_views.assign(mImpl->views.size(), XrView{});
    for (auto& view : frame_ctx.located_views)
    {
        view.type = XR_TYPE_VIEW;
        view.next = nullptr;
    }
    frame_ctx.locate_attempted = true;
    frame_ctx.t_locate_start = LLOpenXRTrace::nowUs();
    uint32_t located_view_count = 0;
    const XrResult locate_result = xrLocateViews(
        mImpl->session,
        &view_locate_info,
        &frame_ctx.view_state,
        (uint32_t)frame_ctx.located_views.size(),
        &located_view_count,
        frame_ctx.located_views.data());
    frame_ctx.locate_result = (int)locate_result;
    frame_ctx.located_view_count = located_view_count;
    frame_ctx.t_locate_end = LLOpenXRTrace::nowUs();
    if (XR_FAILED(locate_result) || located_view_count < 2)
    {
        openxr_warn("xrLocateViews failed (paced loop): " + format_xr_result(mImpl->instance, locate_result));
        mImpl->has_hmd_forward = false;
        mImpl->has_hmd_up = false;
        mImpl->has_left_eye_vertical_fov = false;
        mImpl->has_left_eye_aspect = false;
        mImpl->left_eye_fov_sample.valid = false;
        frame_ctx.pending_end_status = "locate_failed";
        return;
    }

    // P0-06/V2 validity gates, identical to the legacy loop: an
    // invalid pose drives neither the avatar head nor a submitted
    // view. Nothing here recalibrates or reuses a stale pose.
    const bool orientation_valid =
        (frame_ctx.view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0;
    const bool position_valid =
        (frame_ctx.view_state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) != 0;
    if (!orientation_valid || !position_valid)
    {
        if (!orientation_valid)
        {
            mImpl->has_hmd_forward = false;
            mImpl->has_hmd_up = false;
        }
        mImpl->has_left_eye_vertical_fov = false;
        mImpl->has_left_eye_aspect = false;
        mImpl->left_eye_fov_sample.valid = false;
        frame_ctx.pending_end_status = "invalid_view_state";
        return;
    }

    // Publish the snapshot: adopt this locate as the manager's
    // current pose / FOV sample BEFORE idle() resolves look-at and
    // animation against it. The context keeps its own copy of the
    // located views for the submission.
    const uint32_t usable_view_count = std::min<uint32_t>(located_view_count,
                                                          (uint32_t)frame_ctx.located_views.size());
    frame_ctx.located_generation = adopt_located_views(*mImpl, frame_ctx.located_views, usable_view_count);
    frame_ctx.views_valid = frame_ctx.located_generation > 0;
#endif
}

#if LL_LINUX && LL_X11
// P2-03a: freeze the AvatarEyeFrame for a waited paced frame.
// Runs at the post-idle seam (from beginPacedFrameAfterIdle),
// after idle()'s single look-at/animation resolve and before
// xrBeginFrame. Everything the paced render and submit will
// consume is copied OUT of the manager's mutable caches and the
// avatar's live state here, exactly once. Failures are named and
// fail closed: the caller routes them into the existing pending
// zero-layer mechanism. Nothing here recalibrates, reuses a stale
// pose, or touches head control.
static void freeze_avatar_eye_frame(LLOpenXRManagerImpl& impl,
                                    LLOpenXRManagerImpl::XRFrameContext& frame_ctx)
{
    LLXRAvatarEyeFrame& eye = frame_ctx.eye_frame;
    eye = LLXRAvatarEyeFrame();
    eye.app_frame_token = frame_ctx.token;
    eye.locate_generation = frame_ctx.located_generation;
    eye.head_consumed_pose_generation =
        (impl.head_consumed_app_frame_token == frame_ctx.token)
            ? impl.head_consumed_pose_generation
            : 0;
    eye.predicted_display_time_xr_ns =
        (long long)frame_ctx.frame_state.predictedDisplayTime;
    eye.view_state_flags = (unsigned int)frame_ctx.view_state.viewStateFlags;
    eye.coherent_optics = LLOpenXRManager::coherentOpticsEnabled();

    if (!frame_ctx.pending_end_status.empty())
    {
        // shouldRender=false / locate failed / invalid view state:
        // the frame already owes a zero-layer end; the snapshot
        // records why and nothing further is consumed.
        eye.invalid_reason = frame_ctx.pending_end_status;
        frame_ctx.eye_frame_frozen = true;
        return;
    }

    // FOV: this frame's located sample, copied at the seam.
    eye.fov_sample = impl.left_eye_fov_sample;
    eye.vertical_fov_valid = impl.has_left_eye_vertical_fov;
    if (eye.vertical_fov_valid)
    {
        eye.vertical_fov_rad = impl.latest_left_eye_vertical_fov;
    }
    if (!eye.fov_sample.valid)
    {
        eye.invalid_reason = "avatar_eye_left_fov_invalid";
        frame_ctx.eye_frame_frozen = true;
        return;
    }

    // Consumer-generation validation (design review M1): the
    // frozen sample must BE this frame's locate; a head
    // consumption stamped for this iteration must be of the same
    // generation (a zero stamp means the head path did not
    // consume this iteration -- recorded, not a failure).
    if (frame_ctx.located_generation == 0 ||
        eye.fov_sample.locate_generation != frame_ctx.located_generation)
    {
        eye.invalid_reason = "avatar_eye_generation_mismatch";
        frame_ctx.eye_frame_frozen = true;
        return;
    }
    if (eye.head_consumed_pose_generation != 0 &&
        eye.head_consumed_pose_generation != frame_ctx.located_generation)
    {
        eye.invalid_reason = "avatar_eye_head_generation_mismatch";
        frame_ctx.eye_frame_frozen = true;
        return;
    }

    // Avatar anatomy + resolved head rotation: ONE const accessor
    // call returning a value object (no pointers escape).
    LLAgentCamera::ViewpointOverrideEyeState eye_state;
    if (!gAgentCamera.getViewpointOverrideEyeState(eye_state))
    {
        eye.invalid_reason = "avatar_eye_avatar_invalid";
        frame_ctx.eye_frame_frozen = true;
        return;
    }
    eye.left_cornea_valid = eye_state.left_valid;
    eye.left_cornea_agent = eye_state.left_pos_agent;
    eye.left_cornea_point = eye_state.left_point_name;
    eye.right_cornea_valid = eye_state.right_valid;
    eye.right_cornea_agent = eye_state.right_pos_agent;
    switch (eye_state.right_source)
    {
    case LLAgentCamera::EViewpointOverrideRightSource::ATTACHMENT_MARKER:
        eye.right_cornea_source = "attachment_marker";
        break;
    case LLAgentCamera::EViewpointOverrideRightSource::AMBIGUOUS:
        eye.right_cornea_source = "ambiguous";
        break;
    case LLAgentCamera::EViewpointOverrideRightSource::WRONG_ATTACH_POINT:
        eye.right_cornea_source = "wrong_attach_point";
        break;
    default:
        eye.right_cornea_source = "none";
        break;
    }
    eye.head_valid = eye_state.head_valid;
    eye.resolved_head_world = eye_state.head_world;
    if (!eye.left_cornea_valid)
    {
        eye.invalid_reason = "avatar_eye_left_anchor_missing";
        frame_ctx.eye_frame_frozen = true;
        return;
    }
    if (!eye.head_valid)
    {
        eye.invalid_reason = "avatar_eye_orientation_invalid";
        frame_ctx.eye_frame_frozen = true;
        return;
    }

    // Camera orientation as resolved by idle(), kept distinct
    // from the head rotation (gaze composition is unproven).
    LLViewerCamera* camera = LLViewerCamera::getInstance();
    eye.resolved_camera_world = camera->getQuaternion();
    // Orientation validation (design review M3): a captured
    // quaternion is evidence only if every component is finite
    // and its magnitude is nondegenerate. Anything else fails
    // closed with the named orientation reason.
    {
        auto quat_usable = [](const LLQuaternion& q) {
            F32 sumsq = 0.f;
            for (int i = 0; i < 4; i++)
            {
                if (!std::isfinite(q.mQ[i]))
                {
                    return false;
                }
                sumsq += q.mQ[i] * q.mQ[i];
            }
            return sumsq > 0.25f && sumsq < 4.f;
        };
        if (!quat_usable(eye.resolved_head_world) ||
            !quat_usable(eye.resolved_camera_world))
        {
            eye.invalid_reason = "avatar_eye_orientation_invalid";
            frame_ctx.eye_frame_frozen = true;
            return;
        }
    }
    eye.camera_valid = true;
    eye.requested_near = camera->getNear();
    eye.requested_far = gAgentCamera.mDrawDistance;

    S32 target_w = 0;
    S32 target_h = 0;
    if (!LLOpenXRManager::instance().getLeftEyeRenderSize(target_w, target_h) ||
        target_w <= 0 || target_h <= 0)
    {
        eye.invalid_reason = "avatar_eye_extent_invalid";
        frame_ctx.eye_frame_frozen = true;
        return;
    }
    eye.target_width = (int)target_w;
    eye.target_height = (int)target_h;

    eye.valid = true;
    frame_ctx.eye_frame_frozen = true;
}
#endif // LL_LINUX && LL_X11

bool LLOpenXRManager::getAvatarEyeFrame(LLXRAvatarEyeFrame& out) const
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->paced_frame.eye_frame_frozen)
    {
        return false;
    }
    out = mImpl->paced_frame.eye_frame;
    return true;
#else
    (void)out;
    return false;
#endif
}

void LLOpenXRManager::noteRenderReceipt(const LLXRWorldRenderReceipt& receipt)
{
#if LL_LINUX && LL_X11
    if (!mImpl)
    {
        return;
    }
    LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
    // Lifecycle ownership (design review M4): registration only
    // for the active, begun, unended frame the receipt names,
    // and never after that frame's receipt was consumed.
    if (!frame_ctx.active || !frame_ctx.begun || frame_ctx.ended ||
        receipt.app_frame_token != frame_ctx.token ||
        frame_ctx.render_receipt.consumed)
    {
        return;
    }
    LLXRWorldRenderReceipt& stored = frame_ctx.render_receipt;
    if (!stored.present)
    {
        stored = receipt;
        stored.present = true;
        return;
    }
    if (receipt.base_complete && !stored.base_complete)
    {
        // The authoritative base registration replaces a partial
        // trace-only one, preserving any render_seq /
        // fingerprint the trace update already merged.
        const unsigned long long kept_seq = stored.render_seq;
        const std::string kept_fingerprint = stored.render_fingerprint;
        stored = receipt;
        stored.present = true;
        if (stored.render_seq == 0)
        {
            stored.render_seq = kept_seq;
        }
        if (stored.render_fingerprint.empty())
        {
            stored.render_fingerprint = kept_fingerprint;
        }
        return;
    }
    // Merge the record_render_trace() update for the same token:
    // render_seq, fingerprint and any GL state it carries.
    if (receipt.render_seq != 0)
    {
        stored.render_seq = receipt.render_seq;
    }
    if (!receipt.render_fingerprint.empty())
    {
        stored.render_fingerprint = receipt.render_fingerprint;
    }
    if (receipt.has_world_gl)
    {
        stored.has_world_gl = true;
        std::memcpy(stored.world_gl_projection, receipt.world_gl_projection,
                    sizeof(stored.world_gl_projection));
        std::memcpy(stored.world_gl_view, receipt.world_gl_view,
                    sizeof(stored.world_gl_view));
    }
    if (receipt.has_gl_rects)
    {
        stored.has_gl_rects = true;
        std::memcpy(stored.gl_viewport, receipt.gl_viewport, sizeof(stored.gl_viewport));
        std::memcpy(stored.gl_scissor, receipt.gl_scissor, sizeof(stored.gl_scissor));
        stored.gl_scissor_enabled = receipt.gl_scissor_enabled;
    }
    if (receipt.actual_near != 0.f)
    {
        stored.actual_near = receipt.actual_near;
    }
    if (receipt.actual_far != 0.f)
    {
        stored.actual_far = receipt.actual_far;
    }
#endif
}

bool LLOpenXRManager::peekRenderReceipt(LLXRWorldRenderReceipt& out) const
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->paced_frame.render_receipt.present)
    {
        return false;
    }
    out = mImpl->paced_frame.render_receipt;
    return true;
#else
    (void)out;
    return false;
#endif
}

bool LLOpenXRManager::consumeRenderReceipt(unsigned long long token)
{
#if LL_LINUX && LL_X11
    if (!mImpl)
    {
        return false;
    }
    LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
    if (!frame_ctx.active || !frame_ctx.begun || frame_ctx.ended ||
        frame_ctx.token != token ||
        !frame_ctx.render_receipt.present ||
        !frame_ctx.render_receipt.base_complete ||
        frame_ctx.render_receipt.consumed)
    {
        return false;
    }
    frame_ctx.render_receipt.consumed = true;
    return true;
#else
    (void)token;
    return false;
#endif
}

void LLOpenXRManager::beginPacedFrameAfterIdle()
{
#if LL_LINUX && LL_X11
    if (!isPacedFrameLoopSelected())
    {
        return;
    }
    // Contract order (Phase 2 digest): the waited, pre-idle-located
    // frame is begun HERE -- after idle() resolved look-at and
    // animation against its snapshot, before display() renders --
    // so the runtime knows the frame has begun while the render
    // happens. Pending zero-layer outcomes (shouldRender=false,
    // locate failure, invalid view state) are resolved at this
    // point too, so such frames never reach the render path.
    // P2-03a: freeze the AvatarEyeFrame first (post-resolve,
    // pre-BeginFrame). A freeze failure on an otherwise renderable
    // frame joins the pending zero-layer outcomes so the waited
    // frame still gets its balanced begin + zero-layer end here.
    if (mImpl)
    {
        LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
        if (frame_ctx.active && !frame_ctx.ended && frame_ctx.wait_succeeded &&
            !frame_ctx.eye_frame_frozen)
        {
            freeze_avatar_eye_frame(*mImpl, frame_ctx);
            if (!frame_ctx.eye_frame.valid && frame_ctx.pending_end_status.empty())
            {
                frame_ctx.pending_end_status = frame_ctx.eye_frame.invalid_reason;
            }
        }
    }
    beginPacedFrameForRender();
#endif
}

bool LLOpenXRManager::beginPacedFrameForRender()
{
#if LL_LINUX && LL_X11
    if (!mImpl)
    {
        return false;
    }
    LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
    if (!frame_ctx.active || frame_ctx.ended || !frame_ctx.wait_succeeded)
    {
        return false;
    }
    if (frame_ctx.begun)
    {
        return true;
    }
    if (frame_ctx.begin_attempted)
    {
        return false;
    }
    frame_ctx.begin_attempted = true;

    XrFrameBeginInfo frame_begin_info{};
    frame_begin_info.type = XR_TYPE_FRAME_BEGIN_INFO;
    frame_ctx.t_begin_start = LLOpenXRTrace::nowUs();
    const XrResult begin_result = xrBeginFrame(mImpl->session, &frame_begin_info);
    frame_ctx.begin_result = (int)begin_result;
    frame_ctx.t_begin_end = LLOpenXRTrace::nowUs();
    if (XR_FAILED(begin_result))
    {
        openxr_warn("xrBeginFrame failed (paced loop): " + format_xr_result(mImpl->instance, begin_result));
        XrSubmitTrace xr_trace = make_paced_trace(*mImpl, frame_ctx);
        xr_trace.finish("begin_failed");
        frame_ctx = LLOpenXRManagerImpl::XRFrameContext();
        return false;
    }
    frame_ctx.begun = true;

    if (!frame_ctx.pending_end_status.empty())
    {
        // shouldRender=false / locate failed / invalid view state:
        // the waited frame still gets its orderly zero-layer end,
        // before display() -- there is nothing to render for it.
        const std::string status = frame_ctx.pending_end_status;
        endPacedFrameZeroLayer(status);
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool LLOpenXRManager::endPacedFrameZeroLayer(const std::string& status)
{
#if LL_LINUX && LL_X11
    if (!mImpl)
    {
        return false;
    }
    LLOpenXRManagerImpl::XRFrameContext& frame_ctx = mImpl->paced_frame;
    if (!frame_ctx.active || !frame_ctx.begun || frame_ctx.ended)
    {
        return false;
    }
    XrSubmitTrace xr_trace = make_paced_trace(*mImpl, frame_ctx);
    XrFrameEndInfo frame_end_info{};
    frame_end_info.type = XR_TYPE_FRAME_END_INFO;
    frame_end_info.displayTime = frame_ctx.frame_state.predictedDisplayTime;
    frame_end_info.environmentBlendMode = mImpl->blend_mode;
    frame_end_info.layerCount = 0;
    frame_end_info.layers = nullptr;
    xr_trace.captureEndFrame(frame_end_info, nullptr, nullptr, 0);
    xr_trace.end_result = (int)xrEndFrame(mImpl->session, &frame_end_info);
    xr_trace.layer_count = 0;
    if (xr_trace.tracing)
    {
        xr_trace.t_end_end = LLOpenXRTrace::nowUs();
    }
    const bool ended_ok = XR_SUCCEEDED((XrResult)xr_trace.end_result);
    if (!ended_ok)
    {
        openxr_warn("xrEndFrame failed (paced loop, zero-layer close): " +
                    format_xr_result(mImpl->instance, (XrResult)xr_trace.end_result));
    }
    // End-failure policy: a failed xrEndFrame is recorded on the
    // xr_submit record (end_result, "end_failed" status) and warned
    // above; the context is then retired WITHOUT retrying the end --
    // after a failed end the runtime owns the frame's state, and a
    // blind second end risks compounding the error. Pacing resumes
    // at the next iteration's xrWaitFrame, and the failure stays
    // visible in the trace rather than being silently absorbed.
    xr_trace.finish(ended_ok ? status : "end_failed");
    frame_ctx = LLOpenXRManagerImpl::XRFrameContext();
    return ended_ok;
#else
    (void)status;
    return false;
#endif
}

// P2-02: main-loop fallback close. Any waited frame that never
// reached its submit (display() skipped, early return, snapshot or
// cube path) is closed here with zero layers; a frame that was
// never begun is begun only to be closed, so every successful
// xrWaitFrame reaches an orderly xrEndFrame. No-op when no paced
// frame is open.
void LLOpenXRManager::closePacedFrameIfOpen(const char* status)
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->paced_frame.active)
    {
        return;
    }
    if (!mImpl->paced_frame.wait_succeeded)
    {
        mImpl->paced_frame = LLOpenXRManagerImpl::XRFrameContext();
        return;
    }
    if (!mImpl->paced_frame.begun)
    {
        beginPacedFrameForRender();
        if (!mImpl || !mImpl->paced_frame.active)
        {
            return;
        }
    }
    if (mImpl->paced_frame.begun && !mImpl->paced_frame.ended)
    {
        endPacedFrameZeroLayer(status ? status : "fallback_zero_layer");
    }
#else
    (void)status;
#endif
}

bool LLOpenXRManager::getHmdForward(LLVector3& out_forward) const
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->session_running || !mImpl->has_hmd_forward)
    {
        return false;
    }

    out_forward = mImpl->latest_hmd_forward;
    if (out_forward.normalize() > 0.f)
    {
        // P2-01: record the head path's consumption of the current
        // pose sample (which locate generation it used, in which
        // main-loop iteration). Trace evidence only; the returned
        // value and its normalization are unchanged.
        mImpl->head_consumed_pose_generation = mImpl->latest_pose_generation;
        mImpl->head_consumed_app_frame_token = LLOpenXRTrace::currentAppFrameToken();
        return true;
    }
    return false;
#else
    (void)out_forward;
    return false;
#endif
}

bool LLOpenXRManager::getHmdUp(LLVector3& out_up) const
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->session_running || !mImpl->has_hmd_up)
    {
        return false;
    }

    out_up = mImpl->latest_hmd_up;
    return out_up.normalize() > 0.f;
#else
    (void)out_up;
    return false;
#endif
}

bool LLOpenXRManager::getLeftEyeVerticalFov(F32& out_vertical_fov_radians) const
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->session_running || !mImpl->has_left_eye_vertical_fov)
    {
        return false;
    }

    out_vertical_fov_radians = mImpl->latest_left_eye_vertical_fov;
    return std::isfinite(out_vertical_fov_radians) && out_vertical_fov_radians > 0.f;
#else
    (void)out_vertical_fov_radians;
    return false;
#endif
}

bool LLOpenXRManager::getLeftEyeFovSample(LLXRLeftEyeFovSample& out_sample) const
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->session_running || !mImpl->left_eye_fov_sample.valid)
    {
        out_sample = LLXRLeftEyeFovSample();
        return false;
    }

    out_sample = mImpl->left_eye_fov_sample;
    return true;
#else
    out_sample = LLXRLeftEyeFovSample();
    return false;
#endif
}

bool LLOpenXRManager::getLeftEyeAspect(F32& out_aspect) const
{
#if LL_LINUX && LL_X11
    if (!mImpl || !mImpl->session_running || !mImpl->has_left_eye_aspect)
    {
        return false;
    }

    out_aspect = mImpl->latest_left_eye_aspect;
    return std::isfinite(out_aspect) && out_aspect > 0.f;
#else
    (void)out_aspect;
    return false;
#endif
}

bool LLOpenXRManager::getRecommendedRenderSize(S32& out_width, S32& out_height) const
{
#if LL_LINUX && LL_X11
    if (!mImpl)
    {
        return false;
    }

    S32 max_width = 0;
    S32 max_height = 0;

    if (!mImpl->swapchains.empty())
    {
        for (const auto& swapchain : mImpl->swapchains)
        {
            max_width = llmax(max_width, (S32)swapchain.width);
            max_height = llmax(max_height, (S32)swapchain.height);
        }
    }
    else
    {
        for (const auto& view : mImpl->config_views)
        {
            max_width = llmax(max_width, (S32)view.recommendedImageRectWidth);
            max_height = llmax(max_height, (S32)view.recommendedImageRectHeight);
        }
    }

    if (max_width <= 0 || max_height <= 0)
    {
        return false;
    }

    out_width = max_width;
    out_height = max_height;
    return true;
#else
    (void)out_width;
    (void)out_height;
    return false;
#endif
}

bool LLOpenXRManager::getLeftEyeRenderSize(S32& out_width, S32& out_height) const
{
#if LL_LINUX && LL_X11
    if (!mImpl)
    {
        return false;
    }

    // P1-03: the LEFT eye's extent -- the left swapchain once it
    // exists, else the left view's recommended size. Never the max
    // over both eyes (that is getRecommendedRenderSize, legacy only).
    if (!mImpl->swapchains.empty())
    {
        out_width = (S32)mImpl->swapchains[0].width;
        out_height = (S32)mImpl->swapchains[0].height;
    }
    else if (!mImpl->config_views.empty())
    {
        out_width = (S32)mImpl->config_views[0].recommendedImageRectWidth;
        out_height = (S32)mImpl->config_views[0].recommendedImageRectHeight;
    }
    else
    {
        return false;
    }

    return out_width > 0 && out_height > 0;
#else
    (void)out_width;
    (void)out_height;
    return false;
#endif
}

void LLOpenXRManager::shutdown()
{
#if LL_LINUX && LL_X11
    // P2-02: a paced frame still open at shutdown is closed with
    // zero layers before the session is torn down.
    closePacedFrameIfOpen("shutdown_close");
    LLOpenXRTrace::endRun("shutdown");
    if (!mImpl)
    {
        return;
    }

    gResizeScreenTexture = true;
    openxr_info("OpenXR shutdown started.");

    if (mImpl->blit_fbo != 0)
    {
        glDeleteFramebuffers(1, &mImpl->blit_fbo);
        mImpl->blit_fbo = 0;
    }

    if (mImpl->source_read_fbo != 0)
    {
        glDeleteFramebuffers(1, &mImpl->source_read_fbo);
        mImpl->source_read_fbo = 0;
    }

    if (mImpl->session_running && mImpl->session != XR_NULL_HANDLE)
    {
        xrEndSession(mImpl->session);
        mImpl->session_running = false;
    }
    mImpl->has_hmd_forward = false;
    mImpl->has_left_eye_vertical_fov = false;
    mImpl->has_left_eye_aspect = false;
    mImpl->left_eye_fov_sample.valid = false;

    for (auto& swapchain : mImpl->swapchains)
    {
        if (swapchain.handle != XR_NULL_HANDLE)
        {
            xrDestroySwapchain(swapchain.handle);
            swapchain.handle = XR_NULL_HANDLE;
        }
        swapchain.images.clear();
    }
    mImpl->swapchains.clear();

    if (mImpl->app_space != XR_NULL_HANDLE)
    {
        xrDestroySpace(mImpl->app_space);
        mImpl->app_space = XR_NULL_HANDLE;
    }

    if (mImpl->session != XR_NULL_HANDLE)
    {
        xrDestroySession(mImpl->session);
        mImpl->session = XR_NULL_HANDLE;
    }

    if (mImpl->instance != XR_NULL_HANDLE)
    {
        xrDestroyInstance(mImpl->instance);
        mImpl->instance = XR_NULL_HANDLE;
    }

    openxr_info("OpenXR shutdown complete.");
#endif

    mImpl.reset();
}
