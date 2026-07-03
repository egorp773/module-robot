# Calibrated 1.5m forward test:
# 1) short manual straight bump to measure real forward course by RTK
# 2) SET_HEADING to that course
# 3) upload/start a 1.5m route in that measured direction
[System.Threading.Thread]::CurrentThread.CurrentCulture = [System.Globalization.CultureInfo]::InvariantCulture
[System.Threading.Thread]::CurrentThread.CurrentUICulture = [System.Globalization.CultureInfo]::InvariantCulture

$uri = "ws://192.168.31.222:81/ws"
$log = "C:\robot\module\nav_forward_cal_15.log"
"=== calibrated forward 1.5m $(Get-Date -Format o) ===" | Out-File $log -Encoding utf8

$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] connected"

$recv = New-Object byte[] 8192
$telRows = New-Object System.Collections.ArrayList

function SendCmd([string]$cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(1000) | Out-Null } catch {}
}
function RecvLine([int]$ms) {
  if ($ws.State -ne 'Open') { return $null }
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  try {
    $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait($ms)) { return [System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count).Trim() }
  } catch { return $null }
  return $null
}
function AddTel([string]$l) {
  if ($l -notmatch '^TEL,') { return }
  $p = $l -split ','
  try {
    [void]$telRows.Add([pscustomobject]@{
      lat=[double]$p[1]; lon=[double]$p[2]; head=[double]$p[4];
      speed=[double]$p[11]; gpsHead=[double]$p[13]; imu=[double]$p[16];
      motL=[int]$p[18]; motR=[int]$p[19]; raw=$l
    })
  } catch {}
}
function Pump([int]$ms) {
  $dl=(Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $dl) {
    $blk = RecvLine 120
    if ($blk) {
      foreach($l in ($blk -split "`n")) {
        $l=$l.Trim()
        if(-not $l){continue}
        if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
        AddTel $l
      }
    }
  }
}
function SendWait([string]$cmd,[string]$expect,[int]$ms=2000) {
  SendCmd $cmd
  Write-Host "[>] $cmd"
  $dl=(Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $dl) {
    $blk = RecvLine 300
    if ($blk) {
      foreach($l in ($blk -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^(OK|ERR)') { Write-Host "    [<] $l" }
        if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
        AddTel $l
        if ($expect -and $l.StartsWith($expect)) { return $true }
      }
    }
  }
  Write-Host "    [!] no '$expect' in ${ms}ms"
  return $false
}
function StopAll([string]$why) {
  Write-Host "[!!!] AUTO-STOP: $why"
  "AUTO-STOP: $why" | Out-File $log -Append -Encoding utf8
  SendCmd "NAV_STOP"; Start-Sleep -Milliseconds 80
  SendCmd "STOP"; Start-Sleep -Milliseconds 80
  SendCmd "NAV_STOP"
}
function CourseDeg($a, $b) {
  $r = 6378137.0
  $lat0 = $a.lat * [Math]::PI / 180.0
  $dx = ($b.lon - $a.lon) * [Math]::PI / 180.0 * [Math]::Cos($lat0) * $r
  $dy = ($b.lat - $a.lat) * [Math]::PI / 180.0 * $r
  $dist = [Math]::Sqrt($dx*$dx + $dy*$dy)
  $course = ([Math]::Atan2($dx,$dy) * 180.0 / [Math]::PI + 360.0) % 360.0
  return [pscustomobject]@{ dx=$dx; dy=$dy; dist=$dist; course=$course }
}

SendCmd "STOP"
Start-Sleep -Milliseconds 150
SendCmd "LOG,1"
SendCmd "PING"
Pump 1200
if ($telRows.Count -lt 1) { StopAll "no telemetry"; exit 1 }

$calStart = $telRows.Count
$startFix = $telRows[$telRows.Count-1]
Write-Host "[>] calibration bump M,60,60 for 2.2s"
$bumpEnd=(Get-Date).AddMilliseconds(2200)
while ((Get-Date) -lt $bumpEnd) {
  SendCmd "M,60,60"
  if (((Get-Date).Millisecond % 3) -eq 0) { SendCmd "PING" }
  Pump 80
  Start-Sleep -Milliseconds 60
}
SendCmd "STOP"; Start-Sleep -Milliseconds 100; SendCmd "STOP"
for ($i=0; $i -lt 6; $i++) {
  SendCmd "PING"
  Pump 250
}

$calRows = @()
for ($i=$calStart; $i -lt $telRows.Count; $i++) { $calRows += $telRows[$i] }
$moving = @($calRows | Where-Object { $_.speed -gt 0.07 })
if ($moving.Count -ge 2) { $a=$moving[0]; $b=$moving[$moving.Count-1] }
elseif ($calRows.Count -ge 1) { $a=$startFix; $b=$calRows[$calRows.Count-1] }
else { StopAll "not enough calibration telemetry"; exit 1 }

$c = CourseDeg $a $b
if ($c.dist -lt 0.18) { StopAll ("calibration too short {0:N2}m" -f $c.dist); exit 1 }
$course = $c.course
Write-Host ("[+] measured forward: course={0:N1} dist={1:N2}m dx={2:N2} dy={3:N2}" -f $course,$c.dist,$c.dx,$c.dy)

$cur = $telRows[$telRows.Count-1]
Write-Host ("[+] route origin {0} {1} imu={2:N1} ekfHead={3:N1}" -f $cur.lat,$cur.lon,$cur.imu,$cur.head)

SendWait ("SET_HEADING,{0:F1}" -f $course) "OK,HEADING" 1500 | Out-Null

$dist = 1.5
$rad = $course * [Math]::PI / 180.0
$x1 = [Math]::Sin($rad) * $dist
$y1 = [Math]::Cos($rad) * $dist
Write-Host ("[+] forward wp=({0:N3},{1:N3}) by measured course" -f $x1,$y1)

$minX = [Math]::Min(0.0, $x1) - 1.0
$maxX = [Math]::Max(0.0, $x1) + 1.0
$minY = [Math]::Min(0.0, $y1) - 1.0
$maxY = [Math]::Max(0.0, $y1) + 1.0

SendWait "ROUTE_BEGIN,2,$($cur.lat),$($cur.lon)" "OK,ROUTE_BEGIN" | Out-Null
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
if (-not (SendWait "ROUTE_END" "OK,ROUTE" 2500)) { StopAll "route not ready"; exit 1 }

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
      if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
      AddTel $l
    }
  }
  if ($started) { break }
}
if (-not $started) { StopAll "no NAV_START ack"; exit 1 }

Write-Host "[+] NAV running. No telemetry timeout; max runtime 45s."
$runStart=Get-Date
$lastKeepAlive=Get-Date
$done=$false
while (-not $done) {
  if (((Get-Date)-$runStart).TotalSeconds -gt 45) { StopAll "max runtime 45s"; break }
  if (((Get-Date)-$lastKeepAlive).TotalSeconds -gt 2.0) {
    SendCmd "PING"
    $lastKeepAlive=Get-Date
  }
  $blk=RecvLine 300
  if (-not $blk) { continue }
  foreach($l in ($blk -split "`n")) {
    $l=$l.Trim()
    if(-not $l){continue}
    if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
    AddTel $l
    if ($l -match '^NAV,') {
      $p=$l -split ','
      $st=$p[1]; $hErr=[double]$p[5]; $ct2=[double]$p[6]
      Write-Host ("    NAV {0} wp={1}/{2} d={3} hErr={4:N1} ct={5:N2} L={6} R={7}" -f $st,$p[2],$p[3],$p[4],$hErr,$ct2,$p[7],$p[8])
      if ([Math]::Abs($ct2) -gt 1.0) { StopAll ("crossTrack {0:N2} m" -f $ct2); $done=$true; break }
      if ([Math]::Abs($hErr) -gt 120) { StopAll ("headingErr {0:N1}" -f $hErr); $done=$true; break }
      if ($st -eq 'ARRIVED') { Write-Host "[+] ARRIVED"; "ARRIVED"|Out-File $log -Append -Encoding utf8; $done=$true; break }
      if ($st -eq 'ERROR') { StopAll ("NAV ERROR " + $p[9]); $done=$true; break }
    }
  }
}

SendCmd "NAV_STOP"; Start-Sleep -Milliseconds 150; SendCmd "STOP"; Start-Sleep -Milliseconds 100; SendCmd "NAV_STOP"
Write-Host "[*] run ended. log: $log"
try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(1000) | Out-Null } catch {}
