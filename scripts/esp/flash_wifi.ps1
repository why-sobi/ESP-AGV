# Write the WiFi network and ntfy topic to the ESP's "wificfg" partition,
# without rebuilding or reflashing the firmware. Edit firmware\wifi.csv first
# (copy firmware\wifi.example.csv). COM3 must be free (close Webots).
#
#   .\scripts\esp\flash_wifi.ps1 [-Port COM3] [-Config firmware\wifi.csv]

param(
    [string]$Port = "COM3",
    [string]$Config = "firmware\wifi.csv"
)

$ErrorActionPreference = "Stop"
# Must match the wificfg row of firmware\partitions.csv.
$Offset = "0x310000"
$Size = "0x3000"

if (-not (Test-Path $Config)) {
    throw "$Config not found. Copy firmware\wifi.example.csv to $Config and edit it."
}

$env:IDF_PYTHON_CHECK_CONSTRAINTS = "0"
. C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1 *> $null

$image = Join-Path $env:TEMP "agv-wificfg.bin"
python -m esp_idf_nvs_partition_gen generate $Config $image $Size
if ($LASTEXITCODE -ne 0) { throw "Could not build the WiFi partition from $Config" }
python -m esptool --chip esp32s3 --port $Port write-flash $Offset $image
if ($LASTEXITCODE -ne 0) { throw "Could not write the WiFi partition to $Port" }
Remove-Item $image
Write-Host "WiFi settings written; the ESP has restarted and will connect."
