<#
.SYNOPSIS
    Run clang-format over every first-party C++ source in the repository.

.PARAMETER Check
    Do not modify files; exit 1 when any file is not formatted (CI mode).

.PARAMETER Paths
    Optional subset of files/directories to format (default: the whole tree).
#>
[CmdletBinding()]
param(
    [switch]$Check,
    [string[]]$Paths = @()
)

$ErrorActionPreference = 'Stop'

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
Push-Location $repoRoot
try {
    $localTool = Join-Path $env:LOCALAPPDATA 'Physics2D\tools\clang-format.exe'
    $tool = if (Test-Path $localTool) { $localTool } else { (Get-Command clang-format -ErrorAction SilentlyContinue).Source }
    if (-not $tool) {
        Write-Host '[format] clang-format not found. Run scripts/bootstrap.ps1 or install clang-format 17+.' -ForegroundColor Yellow
        exit 1
    }

    if ($Paths.Count -eq 0) {
        $Paths = @('include', 'src', 'tests', 'testbed', 'bench')
    }

    $files = @()
    foreach ($p in $Paths) {
        $full = if ([IO.Path]::IsPathRooted($p)) { $p } else { Join-Path $repoRoot $p }
        if (Test-Path $full -PathType Leaf) {
            $files += $full
        }
        elseif (Test-Path $full -PathType Container) {
            $files += Get-ChildItem $full -Recurse -File -Include *.h, *.cpp, *.hpp | Select-Object -ExpandProperty FullName
        }
    }

    if ($files.Count -eq 0) {
        Write-Host '[format] no source files found.'
        exit 0
    }

    Write-Host "[format] $($files.Count) files ($(if ($Check) { 'check' } else { 'in-place' }))"
    if ($Check) {
        $bad = @()
        foreach ($file in $files) {
            $output = & $tool --style=file --dry-run --Werror $file 2>&1
            if ($LASTEXITCODE -ne 0) {
                $bad += $file
                Write-Host "  needs formatting: $file"
                $output | Select-Object -First 3 | ForEach-Object { Write-Host "    $_" }
            }
        }
        if ($bad.Count -gt 0) { exit 1 }
        Write-Host '[format] OK'
    }
    else {
        & $tool --style=file -i @files
        Write-Host '[format] done'
    }
}
finally {
    Pop-Location
}
