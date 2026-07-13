$port = New-Object System.IO.Ports.SerialPort COM4,115200,None,8,One
$port.Open()
$port.ReadTimeout = 500
$end = (Get-Date).AddSeconds(60)
$out = "C:\robot\module\rover_com4_after_align_hang.log"
"=== capture start $(Get-Date) ===" | Out-File $out
while ((Get-Date) -lt $end) {
  try { $line = $port.ReadLine(); if ($line) { $line | Out-File -Append $out; Write-Output $line } }
  catch [System.TimeoutException] {}
}
"=== capture end $(Get-Date) ===" | Out-File -Append $out
$port.Close()
