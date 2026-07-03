$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.Open()
$port.ReadTimeout = 1000
$end = (Get-Date).AddSeconds(5)
$count = 0
while ((Get-Date) -lt $end) {
  try {
    $line = $port.ReadLine()
    $count++
    if ($line -match '^TEL|^MOTOR|^NAV|^GPS|^ERR') { Write-Host $line }
  } catch {}
}
Write-Host "[*] total lines: $count"
$port.Close()
