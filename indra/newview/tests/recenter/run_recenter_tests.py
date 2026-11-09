#!/usr/bin/env python3
"""Compile production recenter/calibration code with an existing Linux CMake package set.

Example:
  python3 indra/newview/tests/recenter/run_recenter_tests.py \
    --build-dir /path/to/build-linux-x86_64 --compile-viewer

The existing build is read only. All objects use this checkout's source, including
llmath; no old viewer objects, math libraries, mocks or assertions under NDEBUG
are used. This is a focused regression/object build, not a linked viewer build.
"""
import argparse
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', required=True, type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--cxx', default='c++')
    parser.add_argument('--compile-viewer', action='store_true')
    parser.add_argument('--sanitize', action='store_true')
    options = parser.parse_args()
    repo = Path(__file__).resolve().parents[4]
    build = options.build_dir.resolve()
    cache = (build / 'CMakeCache.txt').read_text()
    old_source = re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$', cache, re.M).group(1)
    old_repo = str(Path(old_source).parent)
    old_build = re.search(r'^CMAKE_CACHEFILE_DIR:INTERNAL=(.+)$', cache, re.M).group(1)
    make_flags = (build / 'newview/CMakeFiles/firestorm-bin.dir/flags.make').read_text()
    flags = []
    for line in make_flags.splitlines():
        if line.startswith(('CXX_DEFINES =', 'CXX_INCLUDES =', 'CXX_FLAGS =')):
            flags.extend(shlex.split(line.split('=', 1)[1]))
    flags = [x.replace(old_build, str(build)).replace(old_repo, str(repo)) for x in flags]
    flags = [x for x in flags if not x.startswith(('-O', '-g', '-D_FORTIFY_SOURCE='))]
    flags += ['-O1', '-Werror', '-ffunction-sections', '-fdata-sections',
              '-I' + str(repo / 'indra/newview')]
    if options.sanitize:
        flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    with tempfile.TemporaryDirectory(prefix='firestorm-recenter-') as temporary:
        output = options.output_dir.resolve() if options.output_dir else Path(temporary)
        output.mkdir(parents=True, exist_ok=True)

        def compile_source(source, name, extra=()):
            target = output / (name + '.o')
            subprocess.run([options.cxx, *flags, *extra, '-c', str(repo / source),
                            '-o', str(target)], check=True)
            return str(target)

        pure = ['-DFSVR_CONTINUITY_PURE_ONLY', '-DFSVR_CALIBRATION_PURE_ONLY']
        objects = []
        for name in ['llvrcontinuity', 'llvrcalibration']:
            objects.append(compile_source('indra/newview/' + name + '.cpp', name, pure))
        for name in ['m4math', 'v3math', 'v4math', 'm3math', 'llquaternion']:
            objects.append(compile_source('indra/llmath/' + name + '.cpp', name))
        executable = output / 'recenter_test'
        subprocess.run([options.cxx, *flags,
                        str(repo / 'indra/newview/tests/recenter/recenter_test.cpp'),
                        *objects, '-Wl,--gc-sections', '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
        if options.compile_viewer:
            for name in ['llvrcontinuity', 'llvrcalibration', 'llopenxrmanager',
                         'llagentcamera', 'llvradapterapply']:
                compile_source('indra/newview/' + name + '.cpp', name + '_viewer')
                print(name + ': production object compiled', flush=True)


if __name__ == '__main__':
    main()
