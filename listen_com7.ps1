$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.Open()
$port.ReadTimeout = 500
$end = (Get-Date).AddSeconds(20)
$got = $false
while ((Get-Date) -lt $end) {
  try { $line = $port.ReadLine(); if ($line) { Write-Output $line; $got = $true } }
  catch [System.TimeoutException] {}
}
if (-not $got) { Write-Output "<<NO DATA in 20s>>" }
$port.Close()
