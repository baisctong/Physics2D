# scripts/record_gifs.ps1 - record the README demo GIFs from the deterministic testbed.
#
# Week 8 (delivery week). This machine has no ffmpeg, so the pipeline is:
#   1. physics2d_testbed --smoke N --hidden --capture-dir ... : the smoke loop renders
#      exactly one fixed 1/60 s step per frame, and the host reads the finished
#      framebuffer back (world + ImGui panels) as binary PPM before glfwSwapBuffers.
#   2. A generated Python + Pillow encoder (written into the gitignored build/capture/
#      directory by this script, so this .ps1 is the only file that has to be kept)
#      downscales the frames, quantises one global palette, and writes the GIF.
#      Intermediates are deleted after each GIF; nothing large is committed.
#
# The recording is deterministic in physics: the same command always replays the same
# 1/60 s step sequence. Only the host-timing readouts inside the panels (FPS / Step ms)
# naturally differ per machine, which is why the panel numbers are never asserted here -
# the smoke run's [smoke] summary remains the machine-checkable verdict.
#
# Usage (repository root, Windows PowerShell 5.1 like the other scripts):
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1 -Only dominoes,island_sleep
#   powershell -NoProfile -ExecutionPolicy Bypass -File scripts/record_gifs.ps1 `
#       -Testbed build/w8tbrel/testbed/physics2d_testbed.exe
#
# Requirements: a built physics2d_testbed.exe (debug or release), a desktop session with
# OpenGL 3.3, and Python 3 with Pillow. The GDI/GL path is the same one --smoke uses; the
# window stays hidden (recording works on this machine's NVIDIA driver), so no window
# flashes on screen.

param(
    [string]$Testbed = '',
    [string]$PythonExe = 'python',
    [string[]]$Only = @(),
    [string]$OutputDirectory = 'docs/images',
    [string]$CaptureRoot = 'build/capture',
    [double]$Fps = 12.5,
    [int]$Width = 720,
    [int]$MaxBytesPerGif = 600000,
    [int]$MaxTotalBytes = 2621440
)

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$captureRootFull = Join-Path $repoRoot $CaptureRoot
$outputFull = Join-Path $repoRoot $OutputDirectory

# One entry per GIF. Field meaning:
#  Smoke/Every/Start  deterministic smoke sampling (frames == fixed 1/60 s steps)
#  HideUi             omit the ImGui panels (the domino row is 20 m wide; the floating
#                     panels would cover its two ends)
#  Display            --capture-display overrides (see testbed --help)
#  Crop               native-pixel "top:bottom" crop turned into a wide banner GIF
#  Colors             palette ladder, largest first; the encoder steps down until the
#                     file fits MaxBytesPerGif
# The line "why this window" is repeated in docs/demos.md next to the file names.
$specs = @(
    @{ Name = 'dominoes'; Demo = 'Dominoes'; Smoke = 700; Every = 7; Start = 20
       HideUi = $true; Display = 'axes=off'; Crop = '240:560'; Colors = '128,96,72'
       Note = '20-domino chain: sleep -> push at t=1 s -> whole-island wake wave' },
    @{ Name = 'box_stack'; Demo = 'BoxStack'; Smoke = 900; Every = 8; Start = 1
       HideUi = $false; Display = ''; Crop = ''; Colors = '128,96,72,48'
       Note = '10-box tower: no visible jitter, contact points, penetration readout' },
    @{ Name = 'sap_stress'; Demo = 'StressBenchmark'; Smoke = 400; Every = 2; Start = 250
       HideUi = $false; Display = 'contacts=off,axes=off'; Crop = ''; Colors = '16,14,12'
       Note = '400 random OBBs settling, panel: broadphase=SAP / candidates / Step ms' },
    @{ Name = 'island_sleep'; Demo = 'IslandSleep500'; Smoke = 600; Every = 8; Start = 1
       HideUi = $false; Display = ''; Crop = ''; Colors = '128,96,72,48'
       Note = '500 boxes / 100 islands: awake 500 -> 0, live Step-time comparison' },
    @{ Name = 'contact_debug'; Demo = 'BoxStack'; Smoke = 300; Every = 4; Start = 1
       HideUi = $false; Display = 'normals=on'; Crop = ''; Colors = '128,96,72,48'
       Note = 'contact point + normal overlay and the Contacts impulse table' }
)

function Format-Kilobytes([long]$Bytes) {
    return ('{0:N1} KB' -f ($Bytes / 1024.0))
}

function Assert-UnderCaptureRoot([string]$Path) {
    # Guard the only destructive step (removing the temporary PPM frames): refuse any
    # path that a bad parameter could point outside build/capture.
    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $fullRoot = [System.IO.Path]::GetFullPath($captureRootFull)
    if (-not $fullPath.StartsWith($fullRoot + [System.IO.Path]::DirectorySeparatorChar,
                                  [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "refusing to touch '$fullPath': it is outside the capture root '$fullRoot'"
    }
}

# ---- locate inputs -----------------------------------------------------------
$testbedCandidates = @()
if ($Testbed -ne '') {
    $testbedCandidates += $Testbed
} else {
    # Canonical build layout first (docs/testing.md): build/release, then debug.
    $testbedCandidates += (Join-Path $repoRoot 'build/release/testbed/physics2d_testbed.exe')
    $testbedCandidates += (Join-Path $repoRoot 'build/debug/testbed/physics2d_testbed.exe')
}
# The capture flags were added in Week 8. A stale binary from an earlier week silently
# rejects them ("unknown argument: --capture-dir"), so probe --help and skip candidates
# that predate the capture feature - this turns a confusing mid-recording failure into an
# up-front "rebuild first" message (and makes the canonical build/debug fallback safe).
$testbedPath = $null
foreach ($candidate in ($testbedCandidates | Where-Object { Test-Path $_ })) {
    $resolvedCandidate = (Resolve-Path $candidate).Path
    $helpOutput = & $resolvedCandidate --help 2>&1
    if (($helpOutput -join "`n") -match '--capture-dir') {
        $testbedPath = $resolvedCandidate
        break
    }
    Write-Host "[record] skipping '$resolvedCandidate': it predates the Week 8 --capture-dir support"
}
if (-not $testbedPath) {
    throw ("no physics2d_testbed.exe with Week 8 frame-capture support was found. Build it first, e.g.`n" +
           "  powershell -NoProfile -ExecutionPolicy Bypass -File scripts/build.ps1 -Preset release`n" +
           "or pass -Testbed build/<dir>/testbed/physics2d_testbed.exe.")
}

try {
    $encoderProbe = & $PythonExe -c "import sys, PIL; print(sys.version.split()[0] + ' / Pillow ' + PIL.__version__)" 2>&1
} catch {
    throw "cannot run Python '$PythonExe': $_. Pass the Pillow-enabled interpreter with -PythonExe (on the DSH machine: the bundled Python from load_workspace_dependencies)."
}
if ($LASTEXITCODE -ne 0) {
    throw ("Python '$PythonExe' cannot import Pillow. Install Pillow or pass -PythonExe <python with Pillow>.`n" +
           "Probe output: $encoderProbe")
}

if ($Only.Count -gt 0) {
    # `powershell -File record_gifs.ps1 -Only a,b` passes the list as ONE string, while a
    # direct PowerShell call passes an array; accept both so the documented command works.
    $onlyNames = @()
    foreach ($item in $Only) {
        foreach ($piece in ($item -split ',')) {
            $trimmed = $piece.Trim()
            if ($trimmed -ne '') {
                $onlyNames += $trimmed
            }
        }
    }
} else {
    $onlyNames = @()
}
if ($onlyNames.Count -gt 0) {
    foreach ($requested in $onlyNames) {
        if (-not ($specs | Where-Object { $_.Name -eq $requested })) {
            throw "unknown GIF name '$requested'; available: $(($specs | ForEach-Object { $_.Name }) -join ', ')"
        }
    }
}

# ---- encoder (Python + Pillow) ----------------------------------------------
# Written next to the frames instead of shipping a second script: this .ps1 stays the
# single reproducible entry point.
New-Item -ItemType Directory -Force -Path $captureRootFull | Out-Null
New-Item -ItemType Directory -Force -Path $outputFull | Out-Null
$encoderPath = Join-Path $captureRootFull 'encode_gifs.py'
$encoderSource = @'
"""Encode captured testbed PPM frames into an optimised GIF (Week 8 README GIFs).

Written by scripts/record_gifs.ps1 into the gitignored build/capture/ directory; this is
the only copy, so the .ps1 remains the single reproducible entry point. Pillow only (the
machine has no ffmpeg):

  * frames arrive as binary PPM (P6) from the testbed capture path,
  * each frame is downscaled with Lanczos (thin renderer lines survive) and lightly
    sharpened (the ~13 px ImGui text is what makes the panels readable after downscale),
  * one global palette is quantised from a sample of frames and reused for every frame,
    so the GIF stores one palette instead of a local table per frame,
  * no dithering: flat panel backgrounds and text stay crisp and the LZW stream stays
    small (Pillow's optimize=True then writes inter-frame deltas with transparency).
"""

import argparse
import glob
import os
import sys

from PIL import Image, ImageFilter


def parse_crop(text):
    """'top:bottom' in native framebuffer pixels; None means keep the full frame."""
    if not text:
        return None
    try:
        top_text, bottom_text = text.split(":")
        top, bottom = int(top_text), int(bottom_text)
    except ValueError:
        raise SystemExit("--crop must look like 'top:bottom' in capture pixels")
    if top < 0 or bottom <= top:
        raise SystemExit("--crop requires 0 <= top < bottom")
    return (top, bottom)


def load_frames(frame_directory, width, crop):
    paths = sorted(glob.glob(os.path.join(frame_directory, "frame_*.ppm")))
    if not paths:
        raise SystemExit("no frame_*.ppm files in %s" % frame_directory)
    frames = []
    for path in paths:
        frame = Image.open(path).convert("RGB")
        if crop is not None:
            if crop[1] > frame.height:
                raise SystemExit("--crop bottom %d exceeds capture height %d" % (crop[1], frame.height))
            frame = frame.crop((0, crop[0], frame.width, crop[1]))
        scale = width / float(frame.width)
        frame = frame.resize((width, max(1, int(round(frame.height * scale)))), Image.LANCZOS)
        # Gentle on purpose: stronger sharpening turns the ImGui text into halos that
        # survive palette quantisation worse than the softer version.
        frame = frame.filter(ImageFilter.UnsharpMask(radius=1.0, percent=70, threshold=1))
        frames.append(frame)
    return frames


def build_global_palette(frames, colors):
    """Quantise a vertical strip of sampled frames: the palette must cover the whole
    animation (sleep colour change, settling boxes), not only the first frame."""
    sample_step = max(1, len(frames) // 12)
    samples = frames[::sample_step][:12]
    strip = Image.new("RGB", (frames[0].width, frames[0].height * len(samples)))
    for index, sample in enumerate(samples):
        strip.paste(sample, (0, index * frames[0].height))
    return strip.quantize(colors=colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)


def save_gif(frames, palette, output_path, duration_ms):
    paletted = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    paletted[0].save(
        output_path,
        save_all=True,
        append_images=paletted[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", required=True, help="directory containing frame_*.ppm")
    parser.add_argument("--output", required=True, help="GIF file to write")
    parser.add_argument("--width", type=int, default=720, help="output width in pixels")
    parser.add_argument("--fps", type=float, default=12.5, help="GIF playback rate")
    parser.add_argument("--max-bytes", type=int, default=600000, help="try fewer colours above this size")
    parser.add_argument("--colors", default="128,96,72,48", help="colour ladder, largest first")
    parser.add_argument("--crop", default="", help="'top:bottom' native-pixel crop for a banner GIF")
    arguments = parser.parse_args()

    duration_ms = int(round(1000.0 / arguments.fps))
    if duration_ms <= 0:
        raise SystemExit("--fps must be positive")
    crop = parse_crop(arguments.crop)
    frames = load_frames(arguments.frames, arguments.width, crop)
    nominal_seconds = len(frames) * duration_ms / 1000.0

    ladder = [int(value) for value in arguments.colors.split(",") if value.strip()]
    if not ladder:
        raise SystemExit("--colors needs at least one value")

    output_directory = os.path.dirname(os.path.abspath(arguments.output))
    os.makedirs(output_directory, exist_ok=True)
    temporary_path = arguments.output + ".tmp.gif"

    size_bytes = 0
    used_colors = 0
    for colors in ladder:
        palette = build_global_palette(frames, colors)
        save_gif(frames, palette, temporary_path, duration_ms)
        size_bytes = os.path.getsize(temporary_path)
        used_colors = colors
        if size_bytes <= arguments.max_bytes:
            break
    os.replace(temporary_path, arguments.output)

    print(
        "GIF %s: %d frames, %dx%d, %.2f fps, %.2f s, %.1f KB (colors=%d%s)"
        % (
            os.path.basename(arguments.output),
            len(frames),
            frames[0].width,
            frames[0].height,
            arguments.fps,
            nominal_seconds,
            size_bytes / 1024.0,
            used_colors,
            "" if size_bytes <= arguments.max_bytes else ", OVER BUDGET",
        )
    )
    if size_bytes > arguments.max_bytes:
        sys.exit("GIF exceeds --max-bytes (%d > %d)" % (size_bytes, arguments.max_bytes))


if __name__ == "__main__":
    main()
'@
[System.IO.File]::WriteAllText($encoderPath, $encoderSource, (New-Object System.Text.UTF8Encoding($false)))

Write-Host "[record] testbed : $testbedPath"
Write-Host "[record] encoder : Python $encoderProbe / $encoderPath"
Write-Host "[record] output  : $outputFull"
Write-Host ""

# ---- record ----------------------------------------------------------------
$results = @()
foreach ($spec in $specs) {
    if ($onlyNames.Count -gt 0 -and ($onlyNames -notcontains $spec.Name)) {
        continue
    }
    $frameDirectory = Join-Path $captureRootFull $spec.Name
    $gifPath = Join-Path $outputFull ($spec.Name + '.gif')
    Assert-UnderCaptureRoot $frameDirectory
    if (Test-Path $frameDirectory) {
        Remove-Item -Path $frameDirectory -Recurse -Force
    }
    New-Item -ItemType Directory -Force -Path $frameDirectory | Out-Null

    $testbedArguments = @(
        '--demo', $spec.Demo,
        '--smoke', "$($spec.Smoke)",
        '--hidden',
        '--capture-dir', $frameDirectory,
        '--capture-every', "$($spec.Every)",
        '--capture-start', "$($spec.Start)"
    )
    if ($spec.HideUi) {
        $testbedArguments += '--capture-hide-ui'
    }
    if ($spec.Display -ne '') {
        $testbedArguments += @('--capture-display', $spec.Display)
    }

    Write-Host "[record] $($spec.Name): $($spec.Demo) smoke=$($spec.Smoke) every=$($spec.Every) start=$($spec.Start) - $($spec.Note)"
    $captureOutput = & $testbedPath @testbedArguments 2>&1
    $testbedExitCode = $LASTEXITCODE
    if ($testbedExitCode -ne 0) {
        $captureOutput | Select-Object -Last 15 | ForEach-Object { Write-Host "    $_" }
        throw "testbed capture for '$($spec.Name)' failed with exit code $testbedExitCode"
    }
    $captureLine = $captureOutput | Where-Object { $_ -match '^\[capture\]' } | Select-Object -Last 1
    if (-not $captureLine -or $captureLine -notmatch 'frames=(\d+)') {
        throw "testbed capture for '$($spec.Name)' printed no [capture] summary line"
    }
    $frameCount = [int]$Matches[1]
    if ($frameCount -le 0) {
        throw "testbed capture for '$($spec.Name)' wrote 0 frames"
    }

    $encodeArguments = @(
        $encoderPath,
        '--frames', $frameDirectory,
        '--output', $gifPath,
        '--width', "$Width",
        '--fps', "$Fps",
        '--max-bytes', "$MaxBytesPerGif",
        '--colors', $spec.Colors
    )
    if ($spec.Crop -ne '') {
        $encodeArguments += @('--crop', $spec.Crop)
    }
    $encodeOutput = & $PythonExe @encodeArguments 2>&1
    if ($LASTEXITCODE -ne 0) {
        $encodeOutput | ForEach-Object { Write-Host "    $_" }
        throw "GIF encoding for '$($spec.Name)' failed"
    }
    $encodeOutput | ForEach-Object { Write-Host "    $_" }

    $results += [pscustomobject]@{
        Name = $spec.Name
        Frames = $frameCount
        Bytes = (Get-Item $gifPath).Length
        Note = $spec.Note
    }

    # The PPM intermediates are hundreds of MB per GIF; only the GIF is kept.
    Assert-UnderCaptureRoot $frameDirectory
    Remove-Item -Path $frameDirectory -Recurse -Force
    Write-Host ""
}

# ---- budget check + summary -------------------------------------------------
if ($results.Count -eq 0) {
    throw "nothing was recorded (check -Only)"
}
$totalBytes = 0
Write-Host "[record] GIF summary:"
foreach ($result in $results) {
    $totalBytes += $result.Bytes
    Write-Host ("  {0,-14} {1,4} frames  {2,10}" -f $result.Name, $result.Frames, (Format-Kilobytes $result.Bytes))
}
Write-Host ("[record] total: {0} (budget {1} for all GIFs, {2} per GIF)" -f
           (Format-Kilobytes $totalBytes), (Format-Kilobytes $MaxTotalBytes), (Format-Kilobytes $MaxBytesPerGif))

if ($results | Where-Object { $_.Bytes -gt $MaxBytesPerGif }) {
    throw "at least one GIF exceeds the per-GIF budget ($MaxBytesPerGif bytes)"
}
if ($totalBytes -gt $MaxTotalBytes) {
    throw "the GIF set exceeds the total budget ($MaxTotalBytes bytes)"
}
Write-Host "[record] done: $(($results | ForEach-Object { $_.Name }) -join ', ') are in $outputFull"
