$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.Open()
# RTS controls EN (reset) on most ESP32 USB boards
$port.RtsEnable = $true
Start-Sleep -Milliseconds 150
$port.RtsEnable = $false
Start-Sleep -Milliseconds 50
$port.DtrEnable = $false
$port.ReadTimeout = 500
$end = (Get-Date).AddSeconds(25)
while ((Get-Date) -lt $end) {
  try { $line = $port.ReadLine(); if ($line) { Write-Output $line } }
  catch [System.TimeoutException] {}
}
$port.Close()
