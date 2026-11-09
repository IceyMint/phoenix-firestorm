// Linux OpenXR loader selection. Explicit overrides never fall through.
#ifndef FS_VR_OPENXR_RUNTIME_H
#define FS_VR_OPENXR_RUNTIME_H
#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <limits.h>
#include <link.h>
#include <boost/json.hpp>
inline bool fsvrSupportedRuntimeName(const std::string& name)
{
    return name == "WiVRn" || name.compare(0, 6, "WiVRn ") == 0;
}
inline std::vector<std::string> fsvrRuntimeManifestCandidates()
{
    auto env = [](const char* key) { const char* v = std::getenv(key); return v ? std::string(v) : std::string(); };
    const auto pin = env("XR_RUNTIME_JSON");
    if (!pin.empty()) return {pin};
    std::vector<std::string> roots;
    auto home = env("XDG_CONFIG_HOME");
    if (home.empty() && !env("HOME").empty()) home = env("HOME") + "/.config";
    if (!home.empty()) roots.push_back(home);
    auto dirs = env("XDG_CONFIG_DIRS");
    if (dirs.empty()) dirs = "/etc/xdg";
    size_t begin = 0;
    do {
        auto end = dirs.find(':', begin);
        const auto root = dirs.substr(begin, end == std::string::npos ? end : end - begin);
        if (!root.empty() && root[0] == '/') roots.push_back(root);
        if (end == std::string::npos) break;
        begin = end + 1;
    } while (true);
    roots.push_back("/etc");
    std::vector<std::string> paths;
    for (const auto& root : roots) {
        paths.push_back(root + "/openxr/1/active_runtime.x86_64.json");
        paths.push_back(root + "/openxr/1/active_runtime.json");
    }
    return paths;
}
// Selected manifest plus an actually loaded library, independent of prefix.
// This is deployment evidence, not cryptographic authentication.
inline bool fsvrVerifyRuntimeManifest(std::string& out_detail)
{
    std::string selected;
    for (const auto& candidate : fsvrRuntimeManifestCandidates())
    {
        std::ifstream file(candidate);
        if (file.good()) { selected = candidate; break; }
    }
    char manifest_path[PATH_MAX] = {};
    if (selected.empty() || !realpath(selected.c_str(), manifest_path))
    { out_detail = "selected runtime manifest unavailable"; return false; }
    std::ifstream file(manifest_path);
    std::ostringstream contents; contents << file.rdbuf();
    std::string name, library;
    boost::system::error_code parse_error;
    const auto json = boost::json::parse(contents.str(), parse_error);
    const auto* runtime = !parse_error && json.is_object() ? json.as_object().if_contains("runtime") : nullptr;
    const auto* manifest_name = runtime && runtime->is_object() ? runtime->as_object().if_contains("name") : nullptr;
    const auto* manifest_library = runtime && runtime->is_object() ? runtime->as_object().if_contains("library_path") : nullptr;
    if (manifest_name && manifest_name->is_string()) name = manifest_name->as_string().c_str();
    if (manifest_library && manifest_library->is_string()) library = manifest_library->as_string().c_str();
    if (name != "WiVRn" || library.empty())
    { out_detail = "selected manifest is not WiVRn"; return false; }
    const std::string path(manifest_path);
    if (library[0] != '/') library = path.substr(0, path.find_last_of('/') + 1) + library;
    char resolved[PATH_MAX] = {};
    if (!realpath(library.c_str(), resolved))
    { out_detail = "manifest library does not resolve"; return false; }
    struct Loaded { std::string path; bool found = false; } loaded{resolved, false};
    dl_iterate_phdr([](dl_phdr_info* info, size_t, void* data) -> int {
        auto& match = *static_cast<Loaded*>(data);
        char canonical[PATH_MAX] = {};
        if (info->dlpi_name && realpath(info->dlpi_name, canonical) && match.path == canonical)
            match.found = true;
        return 0;
    }, &loaded);
    out_detail = std::string("selected manifest ") + manifest_path +
        ", library " + resolved + (loaded.found ? " loaded" : " NOT loaded");
    return loaded.found;
}

#endif
