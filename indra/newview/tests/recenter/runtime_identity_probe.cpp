#include "fsvropenxrruntime.h"
#include <openxr/openxr.h>
#include <cstring>
#include <iostream>
int main() {
    XrInstanceCreateInfo info{};
    info.type = XR_TYPE_INSTANCE_CREATE_INFO;
    std::strcpy(info.applicationInfo.applicationName, "Firestorm identity probe");
    info.applicationInfo.apiVersion = XR_MAKE_VERSION(1,0,0);
    XrInstance instance = XR_NULL_HANDLE;
    const XrResult create = xrCreateInstance(&info, &instance);
    std::cout << "xrCreateInstance=" << create << '\n';
    if (XR_FAILED(create)) return 2;
    XrInstanceProperties props{};
    props.type = XR_TYPE_INSTANCE_PROPERTIES;
    const auto result = xrGetInstanceProperties(instance, &props);
    std::string detail;
    const bool manifest = fsvrVerifyRuntimeManifest(detail);
    const bool accepted = XR_SUCCEEDED(result) && fsvrSupportedRuntimeName(props.runtimeName) && manifest;
    std::cout << "XR_RUNTIME_JSON=" << (std::getenv("XR_RUNTIME_JSON") ? "set" : "UNSET")
              << "\nruntime_name=" << props.runtimeName << "\nmanifest_verified=" << manifest
              << "\naccepted=" << accepted << "\n" << detail << '\n';
    xrDestroyInstance(instance);
    return accepted ? 0 : 1;
}
