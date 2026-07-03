$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.Open()
$port.DtrEnable = $true
Start-Sleep -Milliseconds 100
$port.DtrEnable = $false
Start-Sleep -Milliseconds 100
$port.ReadTimeout = 500
$end = (Get-Date).AddSeconds(30)
while ((Get-Date) -lt $end) {
  try { $line = $port.ReadLine(); if ($line) { Write-Output $line } }
  catch [System.TimeoutException] {}
}
$port.Close()
