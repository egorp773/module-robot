# Capture a longer snapshot from a single COM port, no DTR/RTS touch.
param(
  [Parameter(Mandatory=$true)][string]$PortName,
  [Parameter(Mandatory=$true)][int]$Baud,
  [Parameter(Mandatory=$true)][int]$DurationSec,
  [Parameter(Mandatory=$true)][string]$OutFile
)
$port = New-Object System.IO.Ports.SerialPort($PortName, $Baud)
$port.ReadTimeout = 500
try {
  $port.Open()
} catch {
  Add-Content -Path $OutFile -Value "ERR: cannot open $PortName at $Baud - $($_.Exception.Message)"
  exit 1
}

Add-Content -Path $OutFile -Value ("=" * 50)
Add-Content -Path $OutFile -Value ("[SNAP] {0} @ {1} baud, {2}s window, started {3:o}" -f $PortName, $Baud, $DurationSec, (Get-Date).ToUniversalTime())
$end = (Get-Date).AddSeconds($DurationSec)
$buf = New-Object System.Text.StringBuilder
$lastStatus = Get-Date
while ((Get-Date) -lt $end) {
  try {
    $n = $port.BytesToRead
    if ($n -gt 0) {
      $chunk = $port.ReadExisting()
      [void]$buf.Append($chunk)
    }
  } catch {
    Add-Content -Path $OutFile -Value "ERR during read: $($_.Exception.Message)"
    break
  }
  if (((Get-Date) - $lastStatus).TotalSeconds -ge 2) {
    $lastStatus = Get-Date
    Add-Content -Path $OutFile -Value ("[tick] buffered={0} bytes pending={1}" -f $buf.Length, $port.BytesToRead)
  }
  Start-Sleep -Milliseconds 100
}
$port.Close()
Add-Content -Path $OutFile -Value ("[SNAP] done, captured {0} chars" -f $buf.Length)
Add-Content -Path $OutFile -Value $buf.ToString()
Add-Content -Path $OutFile -Value ("=" * 50)
