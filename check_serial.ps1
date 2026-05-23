# Close any existing serial ports
$proc = Get-Process | Where-Object { $_.MainWindowTitle -like "*COM4*" }
if ($proc) {
    Write-Host "Closing existing COM4 connection..."
    Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    Start-Sleep -Seconds 2
}

$p = New-Object System.IO.Ports.SerialPort 'COM4',115200
$p.Open()
Write-Host "Port opened, waiting 8 seconds..."
Start-Sleep -Seconds 8
$bytes = $p.BytesToRead
Write-Host "Bytes available: $bytes"
if ($bytes -gt 0) {
    $data = $p.ReadExisting()
    Write-Host "=== ROVER OUTPUT ==="
    Write-Host $data
    if ($data -match "Guru Meditation") {
        Write-Host "`n!!! CRASH DETECTED !!!"
    }
} else {
    Write-Host "No data received"
}
$p.Close()
