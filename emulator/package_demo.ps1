<#
 =============================================================================
  package_demo.ps1 — bundle the web emulator into a self-contained demo site.
 -----------------------------------------------------------------------------
  Collects everything the in-browser product demo needs into one flat folder
  (and a zip of it) that can be dropped onto any static web host:

      emulator/package/
        index.html            the demo page, unchanged
        sim.js                AUDIO_BASE rewritten to the bundled ./audio dir
        dist/lidar_sim.js     the compiled firmware (Emscripten glue)
        dist/lidar_sim.wasm   the compiled firmware (WebAssembly binary)
        audio/*.wav           the callout / config-prompt voice masters

  The repo layout serves audio from ../assets/original_audio (one level above
  the emulator), which doesn't exist on a standalone host — so the ONE edit
  this script makes is rewriting sim.js's AUDIO_BASE literal to './audio' and
  copying the WAV masters alongside. Every other path in the demo is already
  relative, so the folder works from any URL depth (site.com/, site.com/demo/,
  wherever).

  Usage (from anywhere):
      ./package_demo.ps1            # build folder + lidaragl-demo.zip
      ./package_demo.ps1 -NoZip     # folder only

  Requirements: a built emulator (dist/). If dist/ is missing the script runs
  build_wasm.ps1 for you (which needs the Emscripten SDK).
 =============================================================================
#>
param(
    # Skip the zip step and just produce the folder.
    [switch]$NoZip
)

$ErrorActionPreference = 'Stop'

# ---- Paths -------------------------------------------------------------------
# This script lives in emulator/; the repo root is one level up. Everything is
# derived from $PSScriptRoot so the script works no matter where it's called from.
$EmuDir   = $PSScriptRoot
$RepoRoot = Split-Path $EmuDir -Parent
$DistDir  = Join-Path $EmuDir 'dist'
$AudioSrc = Join-Path $RepoRoot 'assets\original_audio'
$OutDir   = Join-Path $EmuDir 'package'
$ZipPath  = Join-Path $EmuDir 'lidaragl-demo.zip'

# ---- 1. Make sure the WASM build exists (build it if not) ---------------------
$WasmFile = Join-Path $DistDir 'lidar_sim.wasm'
if (-not (Test-Path $WasmFile)) {
    Write-Host '[package] dist/ missing - running build_wasm.ps1 first...'
    & (Join-Path $EmuDir 'build_wasm.ps1')
    if (-not (Test-Path $WasmFile)) {
        throw 'build_wasm.ps1 did not produce dist/lidar_sim.wasm - aborting.'
    }
}

# ---- 2. Clean output folder ----------------------------------------------------
if (Test-Path $OutDir) { Remove-Item $OutDir -Recurse -Force -Confirm:$false }
New-Item -ItemType Directory -Force $OutDir | Out-Null
New-Item -ItemType Directory -Force (Join-Path $OutDir 'dist')  | Out-Null
New-Item -ItemType Directory -Force (Join-Path $OutDir 'audio') | Out-Null

# ---- 3. The demo page, verbatim ------------------------------------------------
Copy-Item (Join-Path $EmuDir 'index.html') $OutDir

# ---- 4. sim.js with the audio path retargeted ----------------------------------
# The dev tree fetches WAVs from ../assets/original_audio (served from the repo
# root); the package bundles them in ./audio instead. The literal below is kept
# on one line in sim.js specifically so this replace stays deterministic.
$SimJs   = Get-Content (Join-Path $EmuDir 'sim.js') -Raw
$DevPath = "'../assets/original_audio'"
if ($SimJs.IndexOf($DevPath) -lt 0) {
    throw "sim.js no longer contains the AUDIO_BASE literal $DevPath - update package_demo.ps1 to match."
}
$SimJs = $SimJs.Replace($DevPath, "'./audio'")
Set-Content -Path (Join-Path $OutDir 'sim.js') -Value $SimJs -Encoding utf8NoBOM

# ---- 5. The compiled firmware --------------------------------------------------
# Emscripten resolves lidar_sim.wasm relative to lidar_sim.js, so keeping the
# dist/ subfolder means zero import-path changes anywhere.
Copy-Item (Join-Path $DistDir 'lidar_sim.js')   (Join-Path $OutDir 'dist')
Copy-Item (Join-Path $DistDir 'lidar_sim.wasm') (Join-Path $OutDir 'dist')

# ---- 6. The voice masters -------------------------------------------------------
# Only the .wav masters: the .pcm clips are firmware-embed format the browser
# can't decode, and editor sidecar files (.asd) / notes have no business shipping.
$WavFiles = Get-ChildItem $AudioSrc -Filter '*.wav'
$WavFiles | Copy-Item -Destination (Join-Path $OutDir 'audio')

# ---- 7. Zip it ------------------------------------------------------------------
if (-not $NoZip) {
    if (Test-Path $ZipPath) { Remove-Item $ZipPath -Force -Confirm:$false }
    Compress-Archive -Path (Join-Path $OutDir '*') -DestinationPath $ZipPath
}

# ---- 8. Report -------------------------------------------------------------------
$SizeMB = '{0:N1}' -f ((Get-ChildItem $OutDir -Recurse | Measure-Object Length -Sum).Sum / 1MB)
Write-Host ''
Write-Host "[package] Done. $($WavFiles.Count) audio clips bundled, $SizeMB MB total."
Write-Host "[package] Folder: $OutDir"
if (-not $NoZip) { Write-Host "[package] Zip:    $ZipPath" }
Write-Host ''
Write-Host '[package] Deploy notes:'
Write-Host '  - Upload the folder contents anywhere on a static host; all paths are'
Write-Host '    relative, so site.com/demo/ or site.com/ both work.'
Write-Host '  - Must be served over http(s) - file:// cannot load WASM or fetch WAVs.'
Write-Host '  - The host should serve .wasm as application/wasm (every mainstream'
Write-Host '    static host - GitHub Pages, Netlify, Cloudflare, S3 - already does).'
