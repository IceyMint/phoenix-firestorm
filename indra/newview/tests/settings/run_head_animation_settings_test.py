#!/usr/bin/env python3
"""Run with an existing Linux Makefiles viewer build (LL_TESTS may be off).

Uses production LLControlGroup/LLCachedControl and the build's dependency flags.
--settings can target an extracted package registry or a baseline fixture.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--build-dir", type=Path, required=True)
parser.add_argument("--settings", type=Path)
args = parser.parse_args()
build = args.build_dir.resolve()
source = Path(__file__).resolve().parent
settings = (args.settings or source.parents[1] / "app_settings/settings.xml").resolve()
flags = {}
for line in (build / "llxml/CMakeFiles/llxml.dir/flags.make").read_text().splitlines():
    if " = " in line:
        key, value = line.split(" = ", 1)
        flags[key] = shlex.split(value)
link = shlex.split((build / "newview/CMakeFiles/firestorm-bin.dir/link.txt").read_text())
# Reuse dependency libraries, excluding viewer objects and viewer entry point.
libraries = link[link.index("-o") + 2:]
with tempfile.TemporaryDirectory(prefix="head-animation-settings-") as scratch:
    scratch = Path(scratch)
    obj = scratch / "test.o"
    binary = scratch / "test"
    subprocess.run([link[0], *flags["CXX_DEFINES"], *flags["CXX_INCLUDES"],
                    "-I" + str(source.parents[2] / "llxml"), *flags["CXX_FLAGS"],
                    "-c", str(source / "head_animation_settings_test.cpp"), "-o", str(obj)],
                   check=True)
    subprocess.run([link[0], str(obj), "-o", str(binary), *libraries],
                   cwd=build / "newview", check=True)
    override = scratch / "override.xml"
    override.write_text('''<llsd><map><key>AOMLocalHeadAnims</key><map>
<key>Type</key><string>Boolean</string><key>Value</key><integer>0</integer>
</map></map></llsd>''')
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = str(build / "packages/lib/release") + ":" + env.get("LD_LIBRARY_PATH", "")
    subprocess.run([str(binary), str(settings), str(override)], env=env, check=True)
