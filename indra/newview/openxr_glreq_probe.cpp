#define XR_USE_GRAPHICS_API_OPENGL
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace
{
std::string format_version(XrVersion version)
{
    return std::to_string(XR_VERSION_MAJOR(version)) + "." +
           std::to_string(XR_VERSION_MINOR(version)) + "." +
           std::to_string(XR_VERSION_PATCH(version));
}

std::string format_result(XrInstance instance, XrResult result)
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
}

int main()
{
    constexpr const char* kRequiredExtension = XR_KHR_OPENGL_ENABLE_EXTENSION_NAME;

    std::cout << "Step 1: checking for extension " << kRequiredExtension << std::endl;

    uint32_t extension_count = 0;
    XrResult result = xrEnumerateInstanceExtensionProperties(nullptr, 0, &extension_count, nullptr);
    if (XR_FAILED(result))
    {
        std::cerr << "xrEnumerateInstanceExtensionProperties(count) failed: " << static_cast<int>(result) << '\n';
        return 1;
    }

    std::vector<XrExtensionProperties> extensions(extension_count);
    for (auto& extension : extensions)
    {
        extension.type = XR_TYPE_EXTENSION_PROPERTIES;
        extension.next = nullptr;
    }

    result = xrEnumerateInstanceExtensionProperties(nullptr, extension_count, &extension_count, extensions.data());
    if (XR_FAILED(result))
    {
        std::cerr << "xrEnumerateInstanceExtensionProperties(list) failed: " << static_cast<int>(result) << '\n';
        return 1;
    }

    const bool has_required_extension = std::any_of(
        extensions.begin(),
        extensions.end(),
        [](const XrExtensionProperties& extension)
        {
            return std::strcmp(extension.extensionName, XR_KHR_OPENGL_ENABLE_EXTENSION_NAME) == 0;
        });

    if (!has_required_extension)
    {
        std::cerr << "Required extension is not reported by the runtime: " << kRequiredExtension << '\n';
        return 2;
    }

    const char* enabled_extensions[] = {kRequiredExtension};

    XrInstanceCreateInfo instance_info{};
    instance_info.type = XR_TYPE_INSTANCE_CREATE_INFO;
    std::strncpy(instance_info.applicationInfo.applicationName, "openxr_glreq_probe", XR_MAX_APPLICATION_NAME_SIZE - 1);
    std::strncpy(instance_info.applicationInfo.engineName, "firestorm", XR_MAX_ENGINE_NAME_SIZE - 1);
    instance_info.applicationInfo.applicationVersion = 1;
    instance_info.applicationInfo.engineVersion = 1;
    instance_info.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    instance_info.enabledExtensionCount = 1;
    instance_info.enabledExtensionNames = enabled_extensions;

    XrInstance instance = XR_NULL_HANDLE;
    result = xrCreateInstance(&instance_info, &instance);
    if (XR_FAILED(result))
    {
        std::cerr << "xrCreateInstance failed: " << static_cast<int>(result) << '\n';
        return 3;
    }

    auto destroy_instance = [&instance]()
    {
        if (instance != XR_NULL_HANDLE)
        {
            xrDestroyInstance(instance);
            instance = XR_NULL_HANDLE;
        }
    };

    XrInstanceProperties instance_properties{};
    instance_properties.type = XR_TYPE_INSTANCE_PROPERTIES;
    result = xrGetInstanceProperties(instance, &instance_properties);
    if (XR_SUCCEEDED(result))
    {
        std::cout << "Runtime: " << instance_properties.runtimeName
                  << " " << format_version(instance_properties.runtimeVersion) << '\n';
    }

    XrSystemGetInfo system_info{};
    system_info.type = XR_TYPE_SYSTEM_GET_INFO;
    system_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

    XrSystemId system_id = XR_NULL_SYSTEM_ID;
    result = xrGetSystem(instance, &system_info, &system_id);
    if (XR_FAILED(result))
    {
        std::cerr << "xrGetSystem failed: " << format_result(instance, result)
                  << " (is a headset/runtime available?)\n";
        destroy_instance();
        return 4;
    }

    PFN_xrGetOpenGLGraphicsRequirementsKHR get_gl_requirements = nullptr;
    result = xrGetInstanceProcAddr(
        instance,
        "xrGetOpenGLGraphicsRequirementsKHR",
        reinterpret_cast<PFN_xrVoidFunction*>(&get_gl_requirements));
    if (XR_FAILED(result) || get_gl_requirements == nullptr)
    {
        std::cerr << "xrGetInstanceProcAddr(xrGetOpenGLGraphicsRequirementsKHR) failed: "
                  << format_result(instance, result) << '\n';
        destroy_instance();
        return 5;
    }

    std::cout << "Step 2: querying OpenGL graphics requirements before session creation" << std::endl;

    XrGraphicsRequirementsOpenGLKHR requirements{};
    requirements.type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR;
    result = get_gl_requirements(instance, system_id, &requirements);
    if (XR_FAILED(result))
    {
        std::cerr << "xrGetOpenGLGraphicsRequirementsKHR failed: " << format_result(instance, result) << '\n';
        destroy_instance();
        return 6;
    }

    std::cout << "OpenGL min API version: " << format_version(requirements.minApiVersionSupported) << '\n';
    std::cout << "OpenGL max API version: " << format_version(requirements.maxApiVersionSupported) << '\n';
    std::cout << "Probe completed successfully.\n";

    destroy_instance();
    return 0;
}
