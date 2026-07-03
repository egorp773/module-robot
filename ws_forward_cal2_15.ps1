[System.Threading.Thread]::CurrentThread.CurrentCulture = [System.Globalization.CultureInfo]::InvariantCulture
[System.Threading.Thread]::CurrentThread.CurrentUICulture = [System.Globalization.CultureInfo]::InvariantCulture

$uri = "ws://192.168.31.222:81/ws"
$log = "C:\robot\module\nav_forward_cal2_15.log"
"=== calibrated2 forward 1.5m $(Get-Date -Format o) ===" | Out-File $log -Encoding utf8

function OpenWs {
  $o = [pscustomobject]@{
    ws = New-Object System.Net.WebSockets.ClientWebSocket
    ct = New-Object System.Threading.CancellationTokenSource
    recv = New-Object byte[] 8192
  }
  try { $o.ws.ConnectAsync([Uri]$uri, $o.ct.Token).Wait(3500) | Out-Null } catch { return $null }
  if ($o.ws.State -ne 'Open') { return $null }
  return $o
}
function CloseWs($o) {
  if ($null -eq $o) { return }
  try { $o.ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$o.ct.Token).Wait(500) | Out-Null } catch {}
}
function SendCmd($o, [string]$cmd) {
  if ($null -eq $o -or $o.ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $o.ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$o.ct.Token).Wait(700) | Out-Null } catch {}
}
function RecvBlock($o, [int]$ms) {
  if ($null -eq $o -or $o.ws.State -ne 'Open') { return $null }
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$o.recv)
  try {
    $t=$o.ws.ReceiveAsync($seg,$o.ct.Token)
    if ($t.Wait($ms)) { return [System.Text.Encoding]::UTF8.GetString($o.recv,0,$t.Result.Count).Trim() }
  } catch {}
  return $null
}
function ReadTel($o, [int]$seconds) {
  $rows = New-Object System.Collections.ArrayList
  $end=(Get-Date).AddSeconds($seconds)
  while ((Get-Date) -lt $end) {
    SendCmd $o "PING"
    $until=(Get-Date).AddMilliseconds(600)
    while ((Get-Date) -lt $until) {
      $blk=RecvBlock $o 120
      if (-not $blk) { continue }
      foreach($l in ($blk -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
        if ($l -match '^TEL,') {
          $p=$l -split ','
          try {
            [void]$rows.Add([pscustomobject]@{
              lat=[double]$p[1]; lon=[double]$p[2]; head=[double]$p[4];
              speed=[double]$p[11]; gpsHead=[double]$p[13]; imu=[double]$p[16];
              raw=$l
            })
          } catch {}
        }
      }
    }
  }
  return @($rows)
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
function SendWait($o, [string]$cmd, [string]$expect, [int]$ms=2000) {
  SendCmd $o $cmd
  Write-Host "[>] $cmd"
  $end=(Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $end) {
    $blk=RecvBlock $o 250
    if (-not $blk) { continue }
    foreach($l in ($blk -split "`n")) {
      $l=$l.Trim()
      if ($l -match '^(OK|ERR)') { Write-Host "    [<] $l" }
      if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
      if ($l.StartsWith($expect)) { return $true }
    }
  }
  return $false
}
function StopAll($o, [string]$why) {
  Write-Host "[!!!] AUTO-STOP: $why"
  "AUTO-STOP: $why" | Out-File $log -Append -Encoding utf8
  SendCmd $o "NAV_STOP"; Start-Sleep -Milliseconds 80
  SendCmd $o "STOP"; Start-Sleep -Milliseconds 80
  SendCmd $o "NAV_STOP"
}

$o=OpenWs
if ($null -eq $o) { Write-Host "[!] connect failed"; exit 1 }
Write-Host "[+] connected for calibration"
SendCmd $o "STOP"
SendCmd $o "LOG,1"
$startRows=ReadTel $o 2
if ($startRows.Count -lt 1) { StopAll $o "no start telemetry"; CloseWs $o; exit 1 }
$start=$startRows[$startRows.Count-1]
Write-Host ("[+] start {0} {1} imu={2:N1}" -f $start.lat,$start.lon,$start.imu)

Write-Host "[>] calibration bump M,60,60 for 2.4s"
$end=(Get-Date).AddMilliseconds(2400)
while ((Get-Date) -lt $end) {
  SendCmd $o "M,60,60"
  Start-Sleep -Milliseconds 90
}
SendCmd $o "STOP"; Start-Sleep -Milliseconds 120; SendCmd $o "STOP"
CloseWs $o
Start-Sleep -Milliseconds 900

$o=OpenWs
if ($null -eq $o) { Write-Host "[!] reconnect failed"; exit 1 }
Write-Host "[+] reconnected for route"
SendCmd $o "STOP"
SendCmd $o "LOG,1"
$endRows=ReadTel $o 3
if ($endRows.Count -lt 1) { StopAll $o "no end telemetry"; CloseWs $o; exit 1 }
$endFix=$endRows[$endRows.Count-1]
$c=CourseDeg $start $endFix
if ($c.dist -lt 0.18) { StopAll $o ("calibration too short {0:N2}m" -f $c.dist); CloseWs $o; exit 1 }
$course=$c.course
Write-Host ("[+] measured forward: course={0:N1} dist={1:N2}m dx={2:N2} dy={3:N2}" -f $course,$c.dist,$c.dx,$c.dy)

if (-not (SendWait $o ("SET_HEADING,{0:F1}" -f $course) "OK,HEADING" 1500)) { StopAll $o "SET_HEADING failed"; CloseWs $o; exit 1 }

$dist=1.5
$rad=$course * [Math]::PI / 180.0
$x1=[Math]::Sin($rad) * $dist
$y1=[Math]::Cos($rad) * $dist
Write-Host ("[+] route origin {0} {1}; wp=({2:N3},{3:N3})" -f $endFix.lat,$endFix.lon,$x1,$y1)
$minX=[Math]::Min(0.0,$x1)-1.0; $maxX=[Math]::Max(0.0,$x1)+1.0
$minY=[Math]::Min(0.0,$y1)-1.0; $maxY=[Math]::Max(0.0,$y1)+1.0

SendWait $o "ROUTE_BEGIN,2,$($endFix.lat),$($endFix.lon)" "OK,ROUTE_BEGIN" | Out-Null
SendWait $o "ROUTE_WP,0,0.000,0.000" "OK,ROUTE_WP,0" | Out-Null
SendWait $o ("ROUTE_WP,1,{0:F3},{1:F3}" -f $x1,$y1) "OK,ROUTE_WP,1" | Out-Null
SendWait $o "ROUTE_BOUNDARY_BEGIN,4" "OK,ROUTE_BOUNDARY_BEGIN" | Out-Null
SendWait $o ("ROUTE_BOUNDARY_PT,0,{0:F3},{1:F3}" -f $minX,$minY) "OK,ROUTE_BOUNDARY_PT,0" | Out-Null
SendWait $o ("ROUTE_BOUNDARY_PT,1,{0:F3},{1:F3}" -f $maxX,$minY) "OK,ROUTE_BOUNDARY_PT,1" | Out-Null
SendWait $o ("ROUTE_BOUNDARY_PT,2,{0:F3},{1:F3}" -f $maxX,$maxY) "OK,ROUTE_BOUNDARY_PT,2" | Out-Null
SendWait $o ("ROUTE_BOUNDARY_PT,3,{0:F3},{1:F3}" -f $minX,$maxY) "OK,ROUTE_BOUNDARY_PT,3" | Out-Null
SendWait $o "ROUTE_BOUNDARY_END" "OK,ROUTE_BOUNDARY_END" | Out-Null
SendWait $o "FORBID_BEGIN,0" "OK,FORBID_BEGIN" | Out-Null
SendWait $o "FORBID_END" "OK,FORBID_END" | Out-Null
if (-not (SendWait $o "ROUTE_END" "OK,ROUTE" 2500)) { StopAll $o "route not ready"; CloseWs $o; exit 1 }
if (-not (SendWait $o "NAV_START" "OK,NAV_START" 2500)) { StopAll $o "no NAV_START ack"; CloseWs $o; exit 1 }

Write-Host "[+] NAV running. No telemetry timeout; max runtime 45s."
$runStart=Get-Date
$lastPing=Get-Date
$done=$false
while (-not $done) {
  if (((Get-Date)-$runStart).TotalSeconds -gt 45) { StopAll $o "max runtime 45s"; break }
  if (((Get-Date)-$lastPing).TotalSeconds -gt 2.0) { SendCmd $o "PING"; $lastPing=Get-Date }
  $blk=RecvBlock $o 300
  if (-not $blk) { continue }
  foreach($l in ($blk -split "`n")) {
    $l=$l.Trim()
    if (-not $l) { continue }
    if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
    if ($l -match '^NAV,') {
      $p=$l -split ','
      $st=$p[1]; $hErr=[double]$p[5]; $ct=[double]$p[6]
      Write-Host ("    NAV {0} wp={1}/{2} d={3} hErr={4:N1} ct={5:N2} L={6} R={7}" -f $st,$p[2],$p[3],$p[4],$hErr,$ct,$p[7],$p[8])
      if ([Math]::Abs($ct) -gt 1.0) { StopAll $o ("crossTrack {0:N2} m" -f $ct); $done=$true; break }
      if ([Math]::Abs($hErr) -gt 120) { StopAll $o ("headingErr {0:N1}" -f $hErr); $done=$true; break }
      if ($st -eq 'ARRIVED') { Write-Host "[+] ARRIVED"; $done=$true; break }
      if ($st -eq 'ERROR') { StopAll $o ("NAV ERROR " + $p[9]); $done=$true; break }
    }
  }
}
SendCmd $o "NAV_STOP"; Start-Sleep -Milliseconds 150; SendCmd $o "STOP"; Start-Sleep -Milliseconds 100; SendCmd $o "NAV_STOP"
Write-Host "[*] run ended. log: $log"
CloseWs $o
