// Regression for the settings registry consumed by LLKeyframeMotion.
#include "linden_common.h"
#include "llcontrol.h"

#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: head_animation_settings_test settings.xml override.xml\n";
        return 2;
    }
    LLControlGroup settings("HeadAnimationSettingsTest");
    auto require = [](bool passed, const char* message)
    {
        if (!passed) std::cerr << "FAIL: " << message << '\n';
        return passed;
    };
    if (!require(settings.loadFromFile(argv[1], true) > 0, "load default registry") ||
        !require(settings.controlExists("AOMLocalHeadAnims"), "head animation control declared"))
        return 1;
    auto control = settings.getControl("AOMLocalHeadAnims");
    if (!require(control->isType(TYPE_BOOLEAN), "Boolean type") ||
        !require(control->isPersisted(), "user preference persists") ||
        !require(control->getDefault().asBoolean(), "animations enabled by default"))
        return 1;

    // Same strict two-argument constructor as LLKeyframeMotion::applyKeyframes.
    LLCachedControl<bool> enabled(settings, "AOMLocalHeadAnims");
    if (!require(enabled, "fresh profile enables animations")) return 1;
    settings.setBOOL("AOMLocalHeadAnims", false);
    if (!require(!enabled, "cache observes disabling animations")) return 1;
    settings.setBOOL("AOMLocalHeadAnims", true);
    if (!require(enabled, "cache observes enabling animations")) return 1;
    if (!require(settings.loadFromFile(argv[2], false) > 0, "load user override") ||
        !require(!enabled, "saved false overrides true default") ||
        !require(control->getDefault().asBoolean(), "user override preserves default"))
        return 1;
    std::cout << "PASS: registry, strict cache, live changes and saved override\n";
    return 0;
}
