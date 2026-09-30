# ==============================================================================
# SNYPTR ONE-CLICK AUTOMATED BOARD FLASHER
# Automatically detects COM ports and flashes:
#   1. ESP32-P4 Camera Board (p4_camera/main/main.c via ESP-IDF)
#   2. Pop-Up Servo & Wi-Fi Hotspot ESP32 (pop_servo_esp32/pop_servo_esp32.ino)
# ==============================================================================

param(
    [string]$P4Port = "",
    [string]$PopPort = ""
)

Write-Host "==========================================================" -ForegroundColor Cyan
Write-Host " SNYPTR STANDALONE SYSTEM - AUTOMATED FIRMWARE FLASHER" -ForegroundColor Cyan
Write-Host "==========================================================" -ForegroundColor Cyan

# List available COM ports
$ports = Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Description, PNPDeviceID
if ($ports) {
    Write-Host "`nDetected Serial Ports:" -ForegroundColor Green
    $ports | Format-Table -AutoSize
} else {
    Write-Host "`nChecking registry SERIALCOMM for COM ports..." -ForegroundColor Yellow
    Get-ItemProperty -Path "HKLM:\HARDWARE\DEVICEMAP\SERIALCOMM" -ErrorAction SilentlyContinue
}

if ($P4Port -ne "") {
    Write-Host "`n[1/2] Building & Flashing ESP32-P4 Camera Firmware on $P4Port..." -ForegroundColor Green
    Push-Location "$PSScriptRoot\p4_camera"
    idf.py -p $P4Port build flash
    Pop-Location
}

if ($PopPort -ne "") {
    Write-Host "`n[2/2] Compiling & Flashing Pop-Up Servo + Wi-Fi ESP32 on $PopPort..." -ForegroundColor Green
    arduino-cli compile --fqbn esp32:esp32:esp32 "$PSScriptRoot\pop_servo_esp32\pop_servo_esp32.ino"
    arduino-cli upload -p $PopPort --fqbn esp32:esp32:esp32 "$PSScriptRoot\pop_servo_esp32\pop_servo_esp32.ino"
}
