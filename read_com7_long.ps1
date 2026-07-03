$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.ReadTimeout = 500
$port.Open()
$end = (Get-Date).AddSeconds(15)
$telCount = 0
$otherCount = 0
while ((Get-Date) -lt $end) {
  try {
    $n = $port.BytesToRead
    if ($n -gt 0) {
      $buf = New-Object byte[] $n
      $got = $port.Read($buf, 0, $n)
      $txt = [System.Text.Encoding]::UTF8.GetString($buf, 0, $got)
      foreach ($l in ($txt -split "`n")) {
        $l = $l.Trim()
        if ($l -match '^TEL,') { $telCount++ }
        elseif ($l -match '^\[') { Write-Host $l }
        elseif ($l -match '^(OK|ERR|MOTOR|NAV|DIAG|sol=)' ) { Write-Host $l; $otherCount++ }
      }
    } else { Start-Sleep -Milliseconds 20 }
  } catch { Start-Sleep -Milliseconds 50 }
}
Write-Host ("[*] TEL lines: {0}, other: {1}" -f $telCount,$otherCount)
$port.Close()
