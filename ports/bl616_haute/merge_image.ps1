<#
Merges the BL616 boot2, partition table and firmware into one image for a
single flash write at 0x0:

  boot2     @0x00000
  partition @0x0E000 and @0x0F000 (the table and its backup copy)
  firmware  @0x10000

Gaps are 0xFF, i.e. what erased flash reads as, so writing the merged image is
equivalent to writing the four pieces separately. Fails rather than truncate
if a piece would overlap the next region.
#>
param(
    [Parameter(Mandatory = $true)][string]$Boot2,
    [Parameter(Mandatory = $true)][string]$Partition,
    [Parameter(Mandatory = $true)][string]$Firmware,
    [Parameter(Mandatory = $true)][string]$Output
)
$ErrorActionPreference = 'Stop'

$regions = @(
    @{ Addr = 0x0;     File = $Boot2 },
    @{ Addr = 0xE000;  File = $Partition },
    @{ Addr = 0xF000;  File = $Partition },
    @{ Addr = 0x10000; File = $Firmware }
)
$fwLen = (Get-Item -LiteralPath $Firmware).Length
$merged = New-Object byte[] (0x10000 + $fwLen)
for ($i = 0; $i -lt $merged.Length; $i++) { $merged[$i] = 0xFF }
for ($r = 0; $r -lt $regions.Count; $r++) {
    $w = $regions[$r]
    $bytes = [IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $w.File).Path)
    $limit = if ($r + 1 -lt $regions.Count) { $regions[$r + 1].Addr } else { $merged.Length }
    if ($w.Addr + $bytes.Length -gt $limit) {
        throw ("{0} ({1} bytes) at 0x{2:X} overlaps the region at 0x{3:X}" -f $w.File, $bytes.Length, $w.Addr, $limit)
    }
    [Array]::Copy($bytes, 0, $merged, $w.Addr, $bytes.Length)
}
[IO.File]::WriteAllBytes([IO.Path]::GetFullPath($Output), $merged)
$hash = (Get-FileHash -LiteralPath $Output -Algorithm SHA256).Hash
"merged: $Output"
"  $($merged.Length) bytes, SHA-256 $hash"
