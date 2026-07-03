# ws_forward_05.ps1 - upload 0.5m route relative to current EKF heading, start, watchdog-stop
[System.Threading.Thread]::CurrentThread.CurrentCulture = [System.Globalization.CultureInfo]::InvariantCulture
[System.Threading.Thread]::CurrentThread.CurrentUICulture = [System.Globalization.CultureInfo]::InvariantCulture

$uri = "ws://192.168.31.222:81/ws"
$log = "C:\robot\module\nav_forward_05.log"
"=== forward 0.5m $(Get-Date -Format o) ===" | Out-File $log -Encoding utf8

$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
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
function SendWait([string]$cmd,[string]$expect,[int]$ms=2000) {
  SendCmd $cmd
  Write-Host "[>] $cmd"
  $dl=(Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $dl) {
    $blk = RecvLine 400
    if ($blk) {
      foreach($l in ($blk -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^(OK|ERR)') { Write-Host "    [<] $l" }
        if ($expect -and $l.StartsWith($expect)) { return $true }
      }
    }
  }
  Write-Host "    [!] no '$expect' in ${ms}ms"
  return $false
}
function STOP([string]$why) {
  $script:done = $true
  Write-Host "[!!!] AUTO-STOP: $why"
  "AUTO-STOP: $why" | Out-File $log -Append -Encoding utf8
  SendCmd "NAV_STOP"; Start-Sleep -Milliseconds 80
  SendCmd "STOP"; Start-Sleep -Milliseconds 80
  SendCmd "NAV_STOP"
}

SendCmd "STOP"
Start-Sleep -Milliseconds 150
SendCmd "PING"
$lat=$null; $lon=$null; $head=$null; $imu=$null
$dl=(Get-Date).AddSeconds(4)
while ((Get-Date) -lt $dl -and (-not $lat)) {
  $blk = RecvLine 400
  if ($blk) {
    foreach($l in ($blk -split "`n")) {
      $l=$l.Trim()
      if($l -match '^TEL,') {
        $p=$l -split ','
        $lat=[double]$p[1]
        $lon=[double]$p[2]
        $head=[double]$p[4]
        $imu=[double]$p[16]
      }
    }
  }
}
if (-not $lat) { Write-Host "[!] no origin/head"; STOP "no telemetry"; exit 1 }
Write-Host ("[+] origin {0} {1} head={2:N1} imu={3:N1}" -f $lat,$lon,$head,$imu)

$dist = 1.5
$rad = $head * [Math]::PI / 180.0
$x1 = [Math]::Sin($rad) * $dist
$y1 = [Math]::Cos($rad) * $dist
Write-Host ("[+] forward wp=({0:N3},{1:N3})" -f $x1,$y1)

$minX = [Math]::Min(0.0, $x1) - 1.0
$maxX = [Math]::Max(0.0, $x1) + 1.0
$minY = [Math]::Min(0.0, $y1) - 1.0
$maxY = [Math]::Max(0.0, $y1) + 1.0

SendWait "ROUTE_BEGIN,2,$lat,$lon" "OK,ROUTE_BEGIN" | Out-Null
SendWait "ROUTE_WP,0,0.000,0.000" "OK,ROUTE_WP,0" | Out-Null
SendWait ("ROUTE_WP,1,{0:F3},{1:F3}" -f $x1,$y1) "OK,ROUTE_WP,1" | Out-Null
SendWait "ROUTE_BOUNDARY_BEGIN,4" "OK,ROUTE_BOUNDARY_BEGIN" | Out-Null
SendWait ("ROUTE_BOUNDARY_PT,0,{0:F3},{1:F3}" -f $minX,$minY) "OK,ROUTE_BOUNDARY_PT,0" | Out-Null
SendWait ("ROUTE_BOUNDARY_PT,1,{0:F3},{1:F3}" -f $maxX,$minY) "OK,ROUTE_BOUNDARY_PT,1" | Out-Null
SendWait ("ROUTE_BOUNDARY_PT,2,{0:F3},{1:F3}" -f $maxX,$maxY) "OK,ROUTE_BOUNDARY_PT,2" | Out-Null
SendWait ("ROUTE_BOUNDARY_PT,3,{0:F3},{1:F3}" -f $minX,$maxY) "OK,ROUTE_BOUNDARY_PT,3" | Out-Null
SendWait "ROUTE_BOUNDARY_END" "OK,ROUTE_BOUNDARY_END" | Out-Null
SendWait "FORBID_BEGIN,0" "OK,FORBID_BEGIN" | Out-Null
SendWait "FORBID_END" "OK,FORBID_END" | Out-Null
if (-not (SendWait "ROUTE_END" "OK,ROUTE" 2500)) { STOP "route not ready"; exit 1 }

SendCmd "LOG,1"
Start-Sleep -Milliseconds 150
SendCmd "NAV_START"
Write-Host "[>] NAV_START"
$started=$false
$dl=(Get-Date).AddSeconds(2)
while ((Get-Date) -lt $dl) {
  $blk=RecvLine 300
  if ($blk) {
    foreach($l in ($blk -split "`n")) {
      $l=$l.Trim()
      if ($l -like 'OK,NAV_START*') { Write-Host "    [<] $l"; $started=$true }
      elseif ($l -like 'ERR*') { Write-Host "    [<] $l" }
    }
  }
  if ($started) { break }
}
if (-not $started) { STOP "no NAV_START ack"; exit 1 }

Write-Host "[+] NAV running. Watchdog active (max 40s)."
$runStart=Get-Date
$lastTel=Get-Date
$lastKeepAlive=Get-Date
$done=$false
while (-not $done) {
  if (((Get-Date)-$runStart).TotalSeconds -gt 40) { STOP "max runtime 40s"; break }
  if (((Get-Date)-$lastKeepAlive).TotalSeconds -gt 2.0) {
    SendCmd "PING"
    $lastKeepAlive=Get-Date
  }
  $blk=RecvLine 300
  if (-not $blk) {
    if (((Get-Date)-$lastTel).TotalSeconds -gt 4.0) { STOP "telemetry timeout"; break }
    continue
  }
  foreach($l in ($blk -split "`n")) {
    $l=$l.Trim()
    if(-not $l){continue}
    if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
    if ($l -match '^NAV,') {
      $lastTel=Get-Date
      $p=$l -split ','
      $st=$p[1]; $hErr=[double]$p[5]; $ct2=[double]$p[6]
      Write-Host ("    NAV {0} wp={1}/{2} d={3} hErr={4:N1} ct={5:N2} L={6} R={7}" -f $st,$p[2],$p[3],$p[4],$hErr,$ct2,$p[7],$p[8])
      if ([Math]::Abs($ct2) -gt 0.8) { STOP ("crossTrack {0:N2} m" -f $ct2); break }
      if ([Math]::Abs($hErr) -gt 90) { STOP ("headingErr {0:N1}" -f $hErr); break }
      if ($st -eq 'ARRIVED') { Write-Host "[+] ARRIVED"; "ARRIVED"|Out-File $log -Append -Encoding utf8; $done=$true }
      if ($st -eq 'ERROR') { STOP ("NAV ERROR " + $p[9]); break }
    } elseif ($l -match '^TEL,') {
      $lastTel=Get-Date
      $p=$l -split ','
      if ($p[6] -ne 'fixed') { STOP ("lost RTK: sol=" + $p[6]); break }
    }
  }
}

SendCmd "NAV_STOP"; Start-Sleep -Milliseconds 150; SendCmd "STOP"; Start-Sleep -Milliseconds 100; SendCmd "NAV_STOP"
Write-Host "[*] run ended. log: $log"
try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(1000) | Out-Null } catch {}
