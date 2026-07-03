$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.Open()
$port.NewLine = "`n"
$port.ReadTimeout = 1500
$end=(Get-Date).AddSeconds(3)
while ((Get-Date) -lt $end) {
  try { $line = $port.ReadLine(); Write-Host ('[' + (Get-Date -Format 'HH:mm:ss.fff') + '] ' + $line) } catch {}
}
$port.Close()
