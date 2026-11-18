<#
Builds the BL616 Haute firmware and merges it into ONE flashable image.

  powershell -ExecutionPolicy Bypass -File ports\bl616_haute\build.ps1 `
      -SdkPath <bl_mcu_sdk> -ToolchainBin <toolchain_gcc_t-head_windows\bin> `
      -TinyUsbPath <tinyusb> [-DebugLog] [-Version <string>]

Output: ports\bl616_haute\build\ds5-bridge-haute[-debug]-merged.bin
  boot2 @0x0, partition table @0xE000 and @0xF000, firmware @0x10000, the gaps
  0xFF (erased flash), so one write at 0x0 programs all four regions.

-DebugLog builds the stage-1 image: the Bluetooth host on BTstack plus a USB serial
log port (VID FFFF PID D012). It is the only build until the BL616 USB device
driver lands, since without it the firmware has no USB device of its own.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$SdkPath,
    [Parameter(Mandatory = $true)][string]$ToolchainBin,
    [Parameter(Mandatory = $true)][string]$TinyUsbPath,
    [switch]$DebugLog,
    [string]$Version = 'dev',
    [switch]$Clean
)
$ErrorActionPreference = 'Stop'

# GitHub's Windows runner exports SHELL=/usr/bin/bash; the SDK's MinGW make
# would then run native Windows paths through Git Bash, which strips the
# backslashes. Force make's native command shell.
Remove-Item Env:SHELL -ErrorAction SilentlyContinue

$ProjectDir = Split-Path -Parent $PSCommandPath
$SdkPath = (Resolve-Path -LiteralPath $SdkPath).Path
$ToolchainBin = (Resolve-Path -LiteralPath $ToolchainBin).Path
$TinyUsbPath = (Resolve-Path -LiteralPath $TinyUsbPath).Path
$Make = Join-Path $SdkPath 'tools\make\mingw32-make.exe'
$CMakeBin = Join-Path $SdkPath 'tools\cmake\bin'
foreach ($p in @($Make, (Join-Path $CMakeBin 'cmake.exe'), (Join-Path $ToolchainBin 'riscv64-unknown-elf-gcc.exe'))) {
    if (-not (Test-Path -LiteralPath $p)) { throw "missing: $p" }
}

$BuildDirName = 'build'
$BuildDir = Join-Path $ProjectDir $BuildDirName
if ($Clean -and (Test-Path -LiteralPath $BuildDir)) { Remove-Item -LiteralPath $BuildDir -Recurse -Force }

$env:BL_SDK_BASE = $SdkPath
$env:TINYUSB_PATH = $TinyUsbPath
$env:DS5_VERSION = $Version
$env:DS5_BL616_DIAG_CDC = $(if ($DebugLog) { '1' } else { '0' })
$env:PATH = "$ToolchainBin;$CMakeBin;$(Join-Path $SdkPath 'tools\make');$env:PATH"
$CrossCompile = (Join-Path $ToolchainBin 'riscv64-unknown-elf-').Replace('\', '/')

Push-Location $ProjectDir
try {
    & $Make "SHELL=cmd.exe" "CHIP=bl616" "BOARD=bl616dk" "BUILD_DIR=$BuildDirName" "CROSS_COMPILE=$CrossCompile"
    if ($LASTEXITCODE -ne 0) { throw "firmware build failed with exit code $LASTEXITCODE" }
} finally {
    Pop-Location
}

$Out = Join-Path $BuildDir 'build_out'
$Boot2 = Get-ChildItem -LiteralPath $Out -Filter 'boot2_bl616_*.bin' | Select-Object -First 1
$Partition = Join-Path $Out 'partition.bin'
$Firmware = Join-Path $Out 'ds5_bridge_haute_bl616.bin'
foreach ($p in @($Partition, $Firmware)) { if (-not (Test-Path -LiteralPath $p)) { throw "build output missing: $p" } }
if (-not $Boot2) { throw "build output missing: boot2_bl616_*.bin in $Out" }

$Name = if ($DebugLog) { 'ds5-bridge-haute-debug-merged.bin' } else { 'ds5-bridge-haute-merged.bin' }
$Merged = Join-Path $BuildDir $Name
& (Join-Path $ProjectDir 'merge_image.ps1') -Boot2 $Boot2.FullName -Partition $Partition -Firmware $Firmware -Output $Merged
if ($LASTEXITCODE -ne 0) { throw "merge failed" }
