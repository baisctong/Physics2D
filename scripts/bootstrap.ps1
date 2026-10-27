<#
.SYNOPSIS
    One-time developer setup: verify the toolchain, fetch a project-local Ninja
    and (optionally) clang-format.

.DESCRIPTION
    The design document assumes CMake + Ninja + vcpkg. This repository replaces
    vcpkg with CMake FetchContent, but Ninja is still the generator the presets
    use. To avoid requiring a system-wide install, this script places binaries
    under tools/ (git-ignored) so that a fresh clone is self-contained.

    Downloaded tools:
      tools/ninja/ninja.exe            (required - presets use the Ninja generator)
      tools/clang-format/clang-format.exe  (optional - scripts/format.ps1)

    If Python is unavailable, the script falls back to winget on Windows and
    otherwise prints manual instructions.
#>
[CmdletBinding()]
param(
    [switch]$Force
)

$ErrorActionPreference = 'Stop'

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
Push-Location $repoRoot
try {
    Write-Host '=== Physics2D bootstrap ===' -ForegroundColor Cyan
    $failed = $false

    # ------------------------------------------------------------------
    # Required: CMake
    # ------------------------------------------------------------------
    $cmake = Get-Command cmake -ErrorAction SilentlyContinue
    if (-not $cmake) {
        Write-Host '[x] CMake not found on PATH. Install CMake >= 3.25.' -ForegroundColor Red
        $failed = $true
    }
    else {
        $cmakeVersion = (& cmake --version | Select-Object -First 1)
        Write-Host "[ok] $cmakeVersion ($($cmake.Source))"
    }

    # ------------------------------------------------------------------
    # Required: git
    # ------------------------------------------------------------------
    if (Get-Command git -ErrorAction SilentlyContinue) {
        Write-Host "[ok] $(git --version)"
    }
    else {
        Write-Host '[x] git not found on PATH.' -ForegroundColor Red
        $failed = $true
    }

    # ------------------------------------------------------------------
    # Required on Windows: MSVC toolset (via Visual Studio)
    # ------------------------------------------------------------------
    if ($env:OS -eq 'Windows_NT') {
        $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
        $vsPath = $null
        if (Test-Path $vswhere) {
            $vsPath = (& $vswhere -latest -products * `
                    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
                    -property installationPath) | Select-Object -First 1
        }
        if ($vsPath) {
            Write-Host "[ok] Visual Studio C++ toolset: $vsPath"
        }
        else {
            Write-Host '[x] Visual Studio with "Desktop development with C++" not found.' -ForegroundColor Red
            Write-Host '    Install VS 2022 Community/Build Tools, or use a machine with GCC/Clang.'
            $failed = $true
        }
    }

    # ------------------------------------------------------------------
    # Helper: extract a Python wheel into tools/<name>
    # ------------------------------------------------------------------
    function Get-PythonExecutable {
        $candidates = @('python', 'python3', 'py')
        foreach ($candidate in $candidates) {
            $cmd = Get-Command $candidate -ErrorAction SilentlyContinue
            if ($cmd) { return $cmd.Source }
        }
        return $null
    }

    function Install-ToolFromPipWheel {
        param(
            [Parameter(Mandatory = $true)][string]$WheelName,
            [Parameter(Mandatory = $true)][string]$ExecutableName,
            [Parameter(Mandatory = $true)][string]$DestinationDir
        )
        $python = Get-PythonExecutable
        if (-not $python) { return $false }

        $tempRoot = Join-Path $env:TEMP ("p2-bootstrap-" + [guid]::NewGuid().ToString('N'))
        try {
            New-Item -ItemType Directory -Force -Path $tempRoot | Out-Null
            # Capture both streams: PowerShell 5.1 turns native-command stderr into
            # error records, which would otherwise abort the script on pip notices.
            $pipOutput = & $python -m pip download $WheelName --no-deps -d $tempRoot --quiet 2>&1
            if ($LASTEXITCODE -ne 0) {
                Write-Host ($pipOutput | Out-String) -ForegroundColor DarkGray
                return $false
            }
            $wheel = Get-ChildItem $tempRoot -Filter '*.whl' | Select-Object -First 1
            if (-not $wheel) { return $false }
            $zipCopy = Join-Path $tempRoot 'package.zip'
            Copy-Item $wheel.FullName $zipCopy -Force
            $extractDir = Join-Path $tempRoot 'unpacked'
            Expand-Archive -Path $zipCopy -DestinationPath $extractDir -Force
            $exe = Get-ChildItem $extractDir -Recurse -Filter $ExecutableName | Select-Object -First 1
            if (-not $exe) { return $false }
            New-Item -ItemType Directory -Force -Path $DestinationDir | Out-Null
            Copy-Item $exe.FullName (Join-Path $DestinationDir $ExecutableName) -Force
            return $true
        }
        finally {
            Remove-Item $tempRoot -Recurse -Force -ErrorAction SilentlyContinue
        }
    }

    # ------------------------------------------------------------------
    # Ninja. Deliberately installed OUTSIDE the repository:
    # some sandboxed agent/CI environments deny file writes to child
    # processes whose executable image lives inside the workspace, which
    # makes a repo-local ninja.exe fail with "Permission denied" on its own
    # lock/log files and on behalf of its children (cl.exe).
    # Preference: Visual Studio bundled Ninja -> %LOCALAPPDATA% copy -> PATH.
    # ------------------------------------------------------------------
    function Find-Ninja {
        $vswhereLocal = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
        if (Test-Path $vswhereLocal) {
            $vsLocal = (& $vswhereLocal -latest -products * -property installationPath) | Select-Object -First 1
            if ($vsLocal) {
                $bundleCandidates = @(
                    (Join-Path $vsLocal 'Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe'),
                    (Join-Path $vsLocal 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ninja.exe')
                )
                foreach ($candidate in $bundleCandidates) {
                    if (Test-Path $candidate) { return $candidate }
                }
            }
        }
        $localCopy = Join-Path $env:LOCALAPPDATA 'Physics2D\tools\ninja.exe'
        if (Test-Path $localCopy) { return $localCopy }
        $onPath = Get-Command ninja -ErrorAction SilentlyContinue
        if ($onPath) { return $onPath.Source }
        return $null
    }

    $ninjaPath = Find-Ninja
    if ($ninjaPath -and -not $Force) {
        Write-Host "[ok] ninja $(& $ninjaPath --version) ($ninjaPath)"
    }
    elseif ($env:OS -eq 'Windows_NT') {
        $toolDir = Join-Path $env:LOCALAPPDATA 'Physics2D\tools'
        New-Item -ItemType Directory -Force -Path $toolDir | Out-Null
        $target = Join-Path $toolDir 'ninja.exe'
        $zipPath = Join-Path $env:TEMP 'physics2d-ninja.zip'
        $downloadUrl = 'https://github.com/ninja-build/ninja/releases/download/v1.12.1/ninja-win.zip'
        Write-Host "[..] downloading official Ninja to $target ..."
        $downloaded = $false
        if (Get-Command curl.exe -ErrorAction SilentlyContinue) {
            & curl.exe -L --silent --show-error --max-time 180 -o $zipPath $downloadUrl
            $downloaded = ($LASTEXITCODE -eq 0)
        }
        if (-not $downloaded) {
            try {
                Invoke-WebRequest -Uri $downloadUrl -OutFile $zipPath -TimeoutSec 180
                $downloaded = $true
            }
            catch {
                Write-Host "[!] download failed: $($_.Exception.Message)" -ForegroundColor Yellow
            }
        }
        if ($downloaded -and (Test-Path $zipPath)) {
            $extractDir = Join-Path $env:TEMP 'physics2d-ninja-extract'
            Remove-Item $extractDir -Recurse -Force -ErrorAction SilentlyContinue
            Expand-Archive -Path $zipPath -DestinationPath $extractDir -Force
            $exe = Get-ChildItem $extractDir -Recurse -Filter 'ninja.exe' | Select-Object -First 1
            if ($exe) {
                Copy-Item $exe.FullName $target -Force
                Write-Host "[ok] ninja $(& $target --version) installed at $target"
            }
            else {
                Write-Host '[x] ninja.exe not found inside the downloaded archive.' -ForegroundColor Red
                $failed = $true
            }
        }
        else {
            Write-Host '[x] Could not download Ninja automatically.' -ForegroundColor Red
            Write-Host '    Install Ninja manually (winget install Ninja-build.Ninja), or use the vs2022-* presets.'
            $failed = $true
        }
    }
    elseif (Get-Command ninja -ErrorAction SilentlyContinue) {
        Write-Host "[ok] ninja $(ninja --version)"
    }
    else {
        Write-Host '[x] Ninja not found. Install it with your package manager.' -ForegroundColor Red
        $failed = $true
    }

    # ------------------------------------------------------------------
    # Optional: clang-format (used by scripts/format.ps1 and pre-commit hook).
    # Installed next to the user-local Ninja, i.e. outside the repository.
    # ------------------------------------------------------------------
    $clangFormatExe = Join-Path $env:LOCALAPPDATA 'Physics2D\tools\clang-format.exe'
    if (Test-Path $clangFormatExe) {
        Write-Host '[ok] clang-format (user-local)'
    }
    elseif (Get-Command clang-format -ErrorAction SilentlyContinue) {
        Write-Host "[ok] $(clang-format --version)"
    }
    elseif ($env:OS -eq 'Windows_NT') {
        Write-Host '[..] installing optional clang-format into %LOCALAPPDATA%\Physics2D\tools ...'
        $ok = Install-ToolFromPipWheel -WheelName 'clang-format' -ExecutableName 'clang-format.exe' `
            -DestinationDir (Join-Path $env:LOCALAPPDATA 'Physics2D\tools')
        if ($ok) { Write-Host '[ok] clang-format installed (optional)' }
        else { Write-Host '[!] clang-format not installed (optional; formatting must be done manually)' -ForegroundColor Yellow }
    }

    Write-Host ''
    if ($failed) {
        Write-Host 'Bootstrap finished WITH ERRORS - fix the items marked [x].' -ForegroundColor Red
        exit 1
    }

    Write-Host 'Bootstrap complete. Next steps:' -ForegroundColor Green
    Write-Host '  powershell -File scripts/build.ps1 -Preset debug -RunTests'
    Write-Host '  powershell -File scripts/build.ps1 -Preset release'
    Write-Host '  cmake --preset vs2022-debug   # alternative without Ninja'
}
finally {
    Pop-Location
}
