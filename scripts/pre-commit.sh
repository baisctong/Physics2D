#!/usr/bin/env bash
# Local pre-commit gate (Section 2.6 of the design document).
#
# Install as a hook once per clone:
#   ln -sf ../../scripts/pre-commit.sh .git/hooks/pre-commit
#
# On Windows use the PowerShell equivalent:
#   powershell -File scripts/build.ps1 -Preset debug -RunTests
set -euo pipefail

cd "$(dirname "$0")/.."

echo '[pre-commit] format check'
if command -v clang-format >/dev/null 2>&1; then
    files=$(git ls-files 'include/**/*.h' 'src/**/*.cpp' 'tests/**/*.cpp' 'testbed/**/*.cpp' 'bench/**/*.cpp' 2>/dev/null || true)
    if [ -n "${files}" ]; then
        clang-format --style=file --dry-run --Werror ${files}
    fi
else
    echo '[pre-commit] clang-format not found, skipping format check'
fi

echo '[pre-commit] configure + build + test'
cmake --preset debug
cmake --build --preset debug
ctest --preset all --output-on-failure

echo '[pre-commit] OK'
