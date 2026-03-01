#include "llviewerprecompiledheaders.h"

#include "llopenxrmanager.h"

#include "llerror.h"
#include "llrect.h"
#include "llstring.h"

#include "llglheaders.h"

#if LL_LINUX && LL_X11
#define XR_USE_GRAPHICS_API_OPENGL
#define XR_USE_PLATFORM_XLIB
#include <openxr/openxr_platform.h>

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
}

#if LL_LINUX && LL_X11
namespace
{
constexpr XrViewConfigurationType XR_VIEW_CONFIG = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

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

XrFovf xr_centered_mono_fov(F32 vertical_fov_radians, F32 aspect)
{
    XrFovf fov{};
    const F32 vertical_half = llclamp(vertical_fov_radians * 0.5f, 0.01f, 1.56f);
    const F32 horizontal_half = llclamp(atanf(tanf(vertical_half) * llmax(aspect, 0.01f)), 0.01f, 1.56f);

    fov.angleLeft = -horizontal_half;
    fov.angleRight = horizontal_half;
    fov.angleUp = vertical_half;
    fov.angleDown = -vertical_half;
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
}

struct OpenXRSwapchain
{
    XrSwapchain handle = XR_NULL_HANDLE;
    uint32_t width = 0;
    uint32_t height = 0;
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
        swapchain_info.sampleCount = llmax(1u, view.recommendedSwapchainSampleCount);
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
                openxr_info("OpenXR session stopped.");
            }
            else if (mImpl->session_state == XR_SESSION_STATE_EXITING ||
                     mImpl->session_state == XR_SESSION_STATE_LOSS_PENDING)
            {
                openxr_warn("OpenXR runtime requested exit; disabling VR submit.");
                mImpl->has_hmd_forward = false;
                mImpl->has_left_eye_vertical_fov = false;
                mImpl->has_left_eye_aspect = false;
                mDisabled = true;
                return;
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

void LLOpenXRManager::submitFrameMono(const LLRect& source_rect)
{
#if LL_LINUX && LL_X11
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

    if (mDisabled)
    {
        return;
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
            return;
        }
    }

#if LL_LINUX && LL_X11
    if (!mImpl || mImpl->session == XR_NULL_HANDLE)
    {
        return;
    }

    processEvents();
    if (mDisabled || !mImpl->session_running)
    {
        return;
    }

    if (!mImpl->submit_ready_logged)
    {
        openxr_info("OpenXR submit loop active.");
        mImpl->submit_ready_logged = true;
    }

    XrFrameWaitInfo frame_wait_info{};
    frame_wait_info.type = XR_TYPE_FRAME_WAIT_INFO;

    XrFrameState frame_state{};
    frame_state.type = XR_TYPE_FRAME_STATE;

    XrResult result = xrWaitFrame(mImpl->session, &frame_wait_info, &frame_state);
    if (XR_FAILED(result))
    {
        openxr_warn("xrWaitFrame failed: " + format_xr_result(mImpl->instance, result));
        return;
    }

    XrFrameBeginInfo frame_begin_info{};
    frame_begin_info.type = XR_TYPE_FRAME_BEGIN_INFO;
    result = xrBeginFrame(mImpl->session, &frame_begin_info);
    if (XR_FAILED(result))
    {
        openxr_warn("xrBeginFrame failed: " + format_xr_result(mImpl->instance, result));
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
        xrEndFrame(mImpl->session, &frame_end_info);
        return;
    }

    XrViewLocateInfo view_locate_info{};
    view_locate_info.type = XR_TYPE_VIEW_LOCATE_INFO;
    view_locate_info.viewConfigurationType = XR_VIEW_CONFIG;
    view_locate_info.displayTime = frame_state.predictedDisplayTime;
    view_locate_info.space = mImpl->app_space;

    XrViewState view_state{};
    view_state.type = XR_TYPE_VIEW_STATE;

    uint32_t located_view_count = 0;
    result = xrLocateViews(
        mImpl->session,
        &view_locate_info,
        &view_state,
        (uint32_t)mImpl->views.size(),
        &located_view_count,
        mImpl->views.data());
    if (XR_FAILED(result) || located_view_count < 2)
    {
        openxr_warn("xrLocateViews failed: " + format_xr_result(mImpl->instance, result));
        mImpl->has_hmd_forward = false;
        mImpl->has_hmd_up = false;
        mImpl->has_left_eye_vertical_fov = false;
        mImpl->has_left_eye_aspect = false;
        XrFrameEndInfo frame_end_info{};
        frame_end_info.type = XR_TYPE_FRAME_END_INFO;
        frame_end_info.displayTime = frame_state.predictedDisplayTime;
        frame_end_info.environmentBlendMode = mImpl->blend_mode;
        frame_end_info.layerCount = 0;
        frame_end_info.layers = nullptr;
        xrEndFrame(mImpl->session, &frame_end_info);
        return;
    }

    const uint32_t usable_view_count = std::min<uint32_t>(located_view_count, (uint32_t)mImpl->views.size());
    if (usable_view_count > 0)
    {
        const XrFovf& left_eye_fov = mImpl->views[0].fov;
        const F32 vertical_fov = left_eye_fov.angleUp - left_eye_fov.angleDown;
        if (std::isfinite(vertical_fov) && vertical_fov > 0.f)
        {
            mImpl->latest_left_eye_vertical_fov = vertical_fov;
            mImpl->has_left_eye_vertical_fov = true;
        }
        else
        {
            mImpl->has_left_eye_vertical_fov = false;
        }

        const F32 horizontal_extent = tanf(left_eye_fov.angleRight) + tanf(-left_eye_fov.angleLeft);
        const F32 vertical_extent = tanf(left_eye_fov.angleUp) + tanf(-left_eye_fov.angleDown);
        const F32 aspect = horizontal_extent / llmax(vertical_extent, 0.00001f);
        if (std::isfinite(aspect) && aspect > 0.f)
        {
            mImpl->latest_left_eye_aspect = aspect;
            mImpl->has_left_eye_aspect = true;
        }
        else
        {
            mImpl->has_left_eye_aspect = false;
        }
    }
    else
    {
        mImpl->has_left_eye_vertical_fov = false;
        mImpl->has_left_eye_aspect = false;
    }

    LLVector3 accumulated_forward;
    LLVector3 accumulated_up;
    for (uint32_t i = 0; i < usable_view_count; ++i)
    {
        accumulated_forward += xr_forward_from_orientation(mImpl->views[i].pose.orientation);
        accumulated_up += xr_up_from_orientation(mImpl->views[i].pose.orientation);
    }

    if (usable_view_count > 0 && accumulated_forward.normalize() > 0.f)
    {
        mImpl->latest_hmd_forward = accumulated_forward;
        mImpl->has_hmd_forward = true;
    }
    else
    {
        mImpl->has_hmd_forward = false;
    }

    if (usable_view_count > 0 && accumulated_up.normalize() > 0.f)
    {
        mImpl->latest_hmd_up = accumulated_up;
        mImpl->has_hmd_up = true;
    }
    else
    {
        mImpl->has_hmd_up = false;
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

    // Mono submit uses one rendered viewpoint duplicated to both eyes.
    // Use centered mono pose/FOV to keep compositor projection aligned with that image.
    const F32 source_width = llmax((F32)source_rect.getWidth(), 1.f);
    const F32 source_height = llmax((F32)source_rect.getHeight(), 1.f);
    const F32 source_aspect = source_width / source_height;
    const F32 left_eye_vertical_fov = mImpl->views[0].fov.angleUp - mImpl->views[0].fov.angleDown;
    const XrPosef mono_pose = xr_center_pose_from_views(mImpl->views[0], mImpl->views[1]);
    const XrFovf mono_fov = xr_centered_mono_fov(left_eye_vertical_fov, source_aspect);

    uint32_t submitted_view_count = 0;
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
        result = xrAcquireSwapchainImage(swapchain.handle, &acquire_info, &image_index);
        if (XR_FAILED(result))
        {
            openxr_warn("xrAcquireSwapchainImage failed: " + format_xr_result(mImpl->instance, result));
            continue;
        }

        XrSwapchainImageWaitInfo wait_info{};
        wait_info.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO;
        wait_info.timeout = XR_INFINITE_DURATION;
        result = xrWaitSwapchainImage(swapchain.handle, &wait_info);
        if (XR_FAILED(result))
        {
            openxr_warn("xrWaitSwapchainImage failed: " + format_xr_result(mImpl->instance, result));
            XrSwapchainImageReleaseInfo release_info{};
            release_info.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
            xrReleaseSwapchainImage(swapchain.handle, &release_info);
            continue;
        }

        const GLuint color_texture = swapchain.images[image_index].image;

        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mImpl->blit_fbo);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture, 0);

        if (glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
        {
            glBlitFramebuffer(
                source_rect.mLeft,
                source_rect.mBottom,
                source_rect.mRight,
                source_rect.mTop,
                0,
                0,
                (GLint)swapchain.width,
                (GLint)swapchain.height,
                GL_COLOR_BUFFER_BIT,
                GL_LINEAR);

            draw_openxr_cursor_overlay(source_rect, swapchain.width, swapchain.height);
        }
        else
        {
            openxr_warn("OpenXR swapchain framebuffer incomplete, skipping eye " + std::to_string(eye));
        }

        XrSwapchainImageReleaseInfo release_info{};
        release_info.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO;
        xrReleaseSwapchainImage(swapchain.handle, &release_info);

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
    frame_end_info.layerCount = (submitted_view_count > 0) ? 1 : 0;
    frame_end_info.layers = (submitted_view_count > 0) ? layers : nullptr;

    result = xrEndFrame(mImpl->session, &frame_end_info);
    if (XR_FAILED(result))
    {
        openxr_warn("xrEndFrame failed: " + format_xr_result(mImpl->instance, result));
        return;
    }

    mImpl->frame_submit_count++;
    if (mImpl->frame_submit_count == 1 || (mImpl->frame_submit_count % 300) == 0)
    {
        openxr_info("Submitted OpenXR frames: " + std::to_string(mImpl->frame_submit_count));
    }

#else
    (void)source_rect;
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
    return out_forward.normalize() > 0.f;
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

void LLOpenXRManager::shutdown()
{
#if LL_LINUX && LL_X11
    if (!mImpl)
    {
        return;
    }

    openxr_info("OpenXR shutdown started.");

    if (mImpl->blit_fbo != 0)
    {
        glDeleteFramebuffers(1, &mImpl->blit_fbo);
        mImpl->blit_fbo = 0;
    }

    if (mImpl->session_running && mImpl->session != XR_NULL_HANDLE)
    {
        xrEndSession(mImpl->session);
        mImpl->session_running = false;
    }
    mImpl->has_hmd_forward = false;
    mImpl->has_left_eye_vertical_fov = false;
    mImpl->has_left_eye_aspect = false;

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
