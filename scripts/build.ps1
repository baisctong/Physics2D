<#
.SYNOPSIS
    Configure / build / test Physics2D with the correct MSVC environment.

.DESCRIPTION
    Windows users cannot run `cmake --preset debug` from a plain PowerShell
    window because cl.exe is only on PATH inside a Visual Studio developer
    prompt. This script imports the vcvars64 environment into the current
    process, adds the project-local Ninja (downloaded by bootstrap.ps1) to PATH
    and then drives CMake exactly like the presets do.

    On Linux/macOS the script degrades to a thin preset wrapper.

.PARAMETER Preset
    One of the Ninja presets: debug, release, sanitize, headless.

.PARAMETER BuildDir
    Optional workspace-relative build directory. Used by contributors working
    in parallel (build/math, build/core, ...) and by CI cache layout. When set,
    the script configures that directory directly instead of using a preset
    binary dir.

.PARAMETER Target
    Optional single cmake target to build (e.g. physics2d_tests).

.PARAMETER ConfigureOnly
    Stop after the configure step.

.PARAMETER RunTests
    Run ctest after a successful build.

.EXAMPLE
    ./scripts/build.ps1 -Preset debug -RunTests
    ./scripts/build.ps1 -Preset debug -BuildDir build/core -NoTestbed -RunTests
#>
[CmdletBinding()]
param(
    [ValidateSet('debug', 'release', 'sanitize', 'headless')]
    [string]$Preset = 'debug',

    [string]$BuildDir = '',

    [switch]$NoTestbed,

    [switch]$ConfigureOnly,

    [switch]$RunTests,

    # Force a full rebuild (cmake --build --clean-first). On Chinese-locale MSVC
    # the Ninja dependency graph misses header-only changes (docs/dev-setup.md
    # section 6), so use this after editing a header before trusting an
    # incremental build. Canonical clean builds (deleted build dir) never need it.
    [switch]$Clean,

    [string]$Target = '',

    [int]$Jobs = 0
)

$ErrorActionPreference = 'Stop'

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
Push-Location $repoRoot
try {
    $isWindows = ($env:OS -eq 'Windows_NT')

    # ------------------------------------------------------------------
    # Toolchain environment
    # ------------------------------------------------------------------
    function Get-VsInstallPath {
        $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
        if (-not (Test-Path $vswhere)) { return $null }
        return (& $vswhere -latest -products * `
                -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
                -property installationPath) | Select-Object -First 1
    }

    function Import-MsvcEnvironment {
        if ($env:VSCMD_VER) { return }  # already inside a developer prompt
        $vsPath = Get-VsInstallPath
        if (-not $vsPath) {
            throw 'Visual Studio with the "Desktop development with C++" workload was not found. ' +
                  'Install VS 2022 (or Build Tools) and re-run, or use a VS "x64 Native Tools" prompt.'
        }
        $vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'
        if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found at $vcvars" }
        Write-Host "[build] importing MSVC environment from $vcvars" -ForegroundColor DarkGray
        $lines = & cmd.exe /c "`"$vcvars`" >nul 2>&1 && set"
        foreach ($line in $lines) {
            $idx = $line.IndexOf('=')
            if ($idx -le 0) { continue }
            $name = $line.Substring(0, $idx)
            $value = $line.Substring($idx + 1)
            [Environment]::SetEnvironmentVariable($name, $value, 'Process')
        }
        if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
            throw 'cl.exe still not available after importing vcvars64 environment.'
        }
    }

    # Locates a usable Ninja. Order matters:
    #   1. the Ninja bundled with Visual Studio (no download, lives outside the repo);
    #   2. the user-local copy installed by scripts/bootstrap.ps1
    #      (%LOCALAPPDATA%\Physics2D\tools) - deliberately OUTSIDE the repository,
    #      because some sandboxed agent environments deny file writes to child
    #      processes whose executable lives inside the workspace;
    #   3. whatever is on PATH.
    function Find-Ninja {
        $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
        if (Test-Path $vswhere) {
            $vsPath = (& $vswhere -latest -products * -property installationPath) | Select-Object -First 1
            if ($vsPath) {
                $candidates = @(
                    (Join-Path $vsPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe'),
                    (Join-Path $vsPath 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ninja.exe')
                )
                foreach ($candidate in $candidates) {
                    if (Test-Path $candidate) { return $candidate }
                }
            }
        }
        $localNinja = Join-Path $env:LOCALAPPDATA 'Physics2D\tools\ninja.exe'
        if (Test-Path $localNinja) { return $localNinja }
        $onPath = Get-Command ninja -ErrorAction SilentlyContinue
        if ($onPath) { return $onPath.Source }
        return $null
    }

    function Ensure-Ninja {
        $ninja = Find-Ninja
        if (-not $ninja) {
            throw 'Ninja not found. Run scripts/bootstrap.ps1 once (it installs Ninja to %LOCALAPPDATA%\Physics2D\tools), ' +
                  'or use the vs2022-* presets (cmake --preset vs2022-debug && cmake --build --preset vs2022-debug).'
        }
        $ninjaDir = Split-Path -Parent $ninja
        $env:PATH = "$ninjaDir$([IO.Path]::PathSeparator)$env:PATH"
        Write-Host "[build] ninja: $ninja" -ForegroundColor DarkGray
    }

    if ($isWindows) {
        Import-MsvcEnvironment
        Ensure-Ninja
        # KNOWN LIMITATION (documented in docs/dev-setup.md section 6): on a
        # Chinese-locale Visual Studio, cl.exe prints `/showIncludes` lines as
        # "注意: 包含文件:" while CMake writes the UTF-8 form of that prefix into
        # CMakeFiles/rules.ninja. Ninja compares raw bytes, so the localized
        # prefix never matches and the recorded dependency list is EMPTY: editing
        # a header does not trigger a recompile on an incremental build (VSLANG=
        # 1033 does NOT help in this VS install - verified). Canonical clean
        # builds are unaffected; for incremental work after header edits use
        # `-Clean`. This is an environment limitation, not a project one.
    }
    elseif (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
        throw 'Ninja not found on PATH. Install it (apt install ninja-build) or use a generator preset.'
    }

    # ------------------------------------------------------------------
    # Configure
    # ------------------------------------------------------------------
    $configMap = @{
        debug    = 'Debug'
        release  = 'RelWithDebInfo'
        sanitize = 'Debug'
        headless = 'RelWithDebInfo'
    }
    $buildType = $configMap[$Preset]

    $buildPath = if ($BuildDir) { Join-Path $repoRoot $BuildDir } else { Join-Path $repoRoot "build/$Preset" }

    if ($BuildDir) {
        Write-Host "[build] configure: raw dir $buildPath ($buildType)" -ForegroundColor Cyan
        $configureArgs = @(
            '-S', $repoRoot,
            '-B', $buildPath,
            '-G', 'Ninja',
            "-DCMAKE_BUILD_TYPE=$buildType",
            '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
        )
        if ($NoTestbed -or $Preset -in @('sanitize', 'headless')) {
            $configureArgs += '-DPHYSICS2D_BUILD_TESTBED=OFF'
        }
        if ($Preset -eq 'sanitize') {
            $configureArgs += @('-DPHYSICS2D_ENABLE_ASAN=ON', '-DPHYSICS2D_ENABLE_UBSAN=ON')
        }
        if ($Preset -eq 'headless') {
            $configureArgs += '-DPHYSICS2D_HEADLESS_ONLY=ON'
        }
        & cmake @configureArgs
    }
    else {
        Write-Host "[build] configure preset: $Preset" -ForegroundColor Cyan
        & cmake --preset $Preset
    }
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (exit $LASTEXITCODE)" }

    if ($ConfigureOnly) { return }

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------
    Write-Host "[build] build" -ForegroundColor Cyan
    $buildArgs = @('--build', $buildPath)

    # Stale-object guard (Windows only, see docs/dev-setup.md section 6).
    # Ninja's depfile is EMPTY on this machine because the localized
    # /showIncludes prefix never matches CMake's UTF-8 copy, so editing a header
    # does not trigger a recompile. Mixing objects compiled against different
    # header layouts corrupts the heap (changed class size -> mismatched
    # new/delete), which is exactly the "HEAP CORRUPTION DETECTED" crash seen in
    # Week 7. Rather than trusting the user to remember -Clean, detect it: if any
    # header in the repository is newer than the newest object file in this build
    # directory, force a clean-first build. This costs one rebuild only when it
    # matters and makes an incremental run as trustworthy as a clean one.
    if ($isWindows -and -not $Clean) {
        $newestHeader = Get-ChildItem -Path (Join-Path $repoRoot 'include'), (Join-Path $repoRoot 'testbed'),
                (Join-Path $repoRoot 'tests'), (Join-Path $repoRoot 'bench') `
            -Recurse -Filter '*.h' -File -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        $newestObject = Get-ChildItem -Path $buildPath -Recurse -Include '*.obj', '*.o' -File `
            -ErrorAction SilentlyContinue |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        if ($null -ne $newestHeader -and $null -ne $newestObject -and
            $newestHeader.LastWriteTime -gt $newestObject.LastWriteTime) {
            Write-Host ("[build] WARNING: header '{0}' is newer than the newest object file; " -f
                $newestHeader.Name) -ForegroundColor Yellow
            Write-Host ("[build]   forcing a clean-first rebuild (localized /showIncludes makes " +
                "Ninja header dependencies empty - docs/dev-setup.md section 6)") -ForegroundColor Yellow
            $buildArgs += '--clean-first'
        }
    } elseif ($Clean) {
        $buildArgs += '--clean-first'
    }
    if ($Target) { $buildArgs += @('--target', $Target) }
    if ($Jobs -gt 0) { $buildArgs += @('--parallel', $Jobs) }
    & cmake @buildArgs
    if ($LASTEXITCODE -ne 0) { throw "CMake build failed (exit $LASTEXITCODE)" }

    # ------------------------------------------------------------------
    # Test
    # ------------------------------------------------------------------
    if ($RunTests -and $Preset -ne 'headless') {
        Write-Host "[build] ctest in $buildPath" -ForegroundColor Cyan
        & ctest --test-dir $buildPath --output-on-failure
        if ($LASTEXITCODE -ne 0) { throw "ctest failed (exit $LASTEXITCODE)" }
    }
}
finally {
    Pop-Location
}
