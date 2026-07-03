# ws_imu_90.ps1 - manual IMU rotation test, no motor/nav start
[System.Threading.Thread]::CurrentThread.CurrentCulture = [System.Globalization.CultureInfo]::InvariantCulture
[System.Threading.Thread]::CurrentThread.CurrentUICulture = [System.Globalization.CultureInfo]::InvariantCulture

$uri = "ws://192.168.31.222:81/ws"
$log = "C:\robot\module\imu_90.log"
"=== imu90 $(Get-Date -Format o) ===" | Out-File $log -Encoding utf8

$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(5000) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] connected"

$recv = New-Object byte[] 8192
function RecvLine([int]$ms) {
  if ($ws.State -ne 'Open') { return $null }
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  try {
    $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait($ms)) { return [System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count).Trim() }
  } catch { return $null }
  return $null
}
function SendCmd([string]$cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(1000) | Out-Null } catch {}
}
function SendRead([string]$cmd,[int]$ms=1000) {
  SendCmd $cmd
  Write-Host "[>] $cmd"
  $dl=(Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $dl) {
    $blk=RecvLine 250
    if ($blk) {
      foreach($l in ($blk -split "`n")) {
        $l=$l.Trim()
        if ($l -and ($l -match '^(IMU_ZERO|IMU_DIAG|OK|ERR|STATE|\[HELP\]|\[LOG\])')) {
          Write-Host "    $l"
          $l | Out-File $log -Append -Encoding utf8
        }
      }
    }
  }
}

SendRead "STOP" 700
SendRead "LOG,0" 700
Start-Sleep -Milliseconds 300

Write-Host "[*] Keep rover still now."
SendRead "IMU_ZERO" 1200
Write-Host "[*] Rotate rover by about 90 degrees CLOCKWISE now, then hold it still."
for ($i=12; $i -ge 1; --$i) {
  Write-Host ("    {0}s" -f $i)
  Start-Sleep -Seconds 1
}

for ($i=0; $i -lt 5; ++$i) {
  SendRead "IMU_DIAG" 900
  Start-Sleep -Milliseconds 400
}

try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(1000) | Out-Null } catch {}
Write-Host "[*] log: $log"
