$port = New-Object System.IO.Ports.SerialPort COM4,115200,None,8,One
$port.Open()
$port.ReadTimeout = 500
$end = (Get-Date).AddSeconds(20)
while ((Get-Date) -lt $end) {
  try { $line = $port.ReadLine(); if ($line) { Write-Output $line } }
  catch [System.TimeoutException] {}
}
$port.Close()
