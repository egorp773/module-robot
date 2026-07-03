# reset_and_read.ps1 - reset ESP32 (DTR toggle) + read 30s boot+steady
$port = New-Object System.IO.Ports.SerialPort COM3,115200,None,8,One
$port.Open()
$port.DtrEnable = $true
Start-Sleep -Milliseconds 100
$port.DtrEnable = $false
Start-Sleep -Milliseconds 100
$port.ReadTimeout = 500
$start = Get-Date
$end = $start.AddSeconds(30)
while ((Get-Date) -lt $end) {
  try {
    $line = $port.ReadLine()
    if ($line) { Write-Output $line }
  } catch [System.TimeoutException] {
  }
}
$port.Close()
