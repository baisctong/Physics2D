#!/usr/bin/env bash
# Linux/macOS build wrapper. Mirrors scripts/build.ps1 so that CI and local
# Linux developers use the same command sequence.
#
#   ./scripts/build.sh debug --tests
#   ./scripts/build.sh sanitize --tests
#
# Prerequisites: cmake >= 3.25, ninja, a C++20 compiler (GCC 12+ / Clang 15+),
# and on Linux the OpenGL/X11 development packages for the testbed:
#   sudo apt install build-essential cmake ninja-build libgl1-mesa-dev xorg-dev
set -euo pipefail

cd "$(dirname "$0")/.."

preset="${1:-debug}"
shift || true

run_tests=0
for arg in "$@"; do
    case "${arg}" in
        --tests) run_tests=1 ;;
        *) echo "unknown argument: ${arg}" >&2; exit 2 ;;
    esac
done

case "${preset}" in
    debug|release|sanitize|headless) ;;
    *) echo "unknown preset '${preset}' (expected debug|release|sanitize|headless)" >&2; exit 2 ;;
esac

echo "[build] configure ${preset}"
cmake --preset "${preset}"

echo "[build] build ${preset}"
cmake --build --preset "${preset}"

if [ "${run_tests}" -eq 1 ]; then
    echo "[build] ctest"
    case "${preset}" in
        debug)    ctest --preset all --output-on-failure ;;
        release)  ctest --preset release --output-on-failure ;;
        sanitize) ctest --preset sanitize --output-on-failure ;;
        headless) ctest --preset headless --output-on-failure ;;
    esac
fi
