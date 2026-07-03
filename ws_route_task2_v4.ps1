# v4: пересоздаём origin в начале КАЖДОЙ фазы, чтобы IMU reset'нулся
$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
$ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] connected"

$r = New-Object byte[] 8192
function RecvLine($ms) {
  $sr=New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
  try { $t=$ws.ReceiveAsync($sr,$ct.Token); if($t.Wait($ms)){ return [System.Text.Encoding]::UTF8.GetString($r,0,$t.Result.Count).Trim() } } catch {}
  return $null
}
function SendCmd($cmd) {
  $b=[System.Text.Encoding]::UTF8.GetBytes($cmd); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  try { $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(500)|Out-Null } catch {}
}
function StopAll($why) { Write-Host "[!!!] STOP: $why"; SendCmd "NAV_STOP"; Start-Sleep -Milliseconds 80; SendCmd "STOP"; Start-Sleep -Milliseconds 80; SendCmd "STOP" }
function CourseM($lat1,$lon1,$lat2,$lon2) {
  $r=6378137.0; $phi=$lat1*[Math]::PI/180.0
  $dx=($lon2-$lon1)*[Math]::PI/180.0*[Math]::Cos($phi)*$r
  $dy=($lat2-$lat1)*[Math]::PI/180.0*$r
  return [pscustomobject]@{dx=$dx;dy=$dy;dist=[Math]::Sqrt($dx*$dx+$dy*$dy);course=([Math]::Atan2($dx,$dy)*180.0/[Math]::PI+360.0)%360.0}
}
function GetPosFix($timeoutSec) {
  $end=(Get-Date).AddSeconds($timeoutSec); $lat=0;$lon=0;$head=0;$hacc=999
  while ((Get-Date) -lt $end) {
    $blk = RecvLine 500
    if ($blk) { foreach ($l in ($blk -split "`n")) { $l=$l.Trim()
      if ($l -match '^TEL,') { $p=$l -split ','; $lat=[double]$p[1]; $lon=[double]$p[2]; $head=[double]$p[4]; $hacc=[double]$p[8]
        if ($hacc -lt 30) { return @{lat=$lat;lon=$lon;head=$head;hacc=$hacc} } } } }
  } return $null
}

Write-Host "[*] Phase 0: capture start (RTK_FIXED)"
SendCmd "PING"
$start = GetPosFix 8
if (-not $start) { Write-Host "[!] no FIXED"; exit 1 }
Write-Host ("[+] start lat={0:F7} lon={1:F7} head={2:F1}" -f $start.lat,$start.lon,$start.head)

# === Phase 1: 1.5m прямо ===
Write-Host ""
Write-Host "[*] Phase 1: 1.5m straight (M,70,70) - reset origin first"
# Reset origin: шлём ROUTE_BEGIN с count=1 и текущей позицией - просто чтобы setOrigin вызвался
SendCmd ("ROUTE_BEGIN,1,{0},{1}" -f $start.lat,$start.lon)
Start-Sleep -Milliseconds 300
$blk = RecvLine 500
if ($blk) { Write-Host ("    [<] " + ($blk -split "`n")[0]) }

$startTime = Get-Date; $lastDist = 0; $doneP1 = $false
while (-not $doneP1) {
  if (((Get-Date)-$startTime).TotalSeconds -gt 60) { StopAll "P1 timeout"; $doneP1=$true; break }
  $blk = RecvLine 200
  if ($blk) { foreach ($l in ($blk -split "`n")) { $l=$l.Trim()
    if ($l -match '^TEL,') { $p=$l -split ','
      $curLat=[double]$p[1]; $curLon=[double]$p[2]; $curHead=[double]$p[4]; $curSpd=[double]$p[11]
      $c = CourseM $start.lat $start.lon $curLat $curLon; $d = $c.dist
      if (($d - $lastDist) -gt 0.05) { Write-Host ("    P1: dist={0:N3} dx={1:N3} dy={2:N3} head={3:F1} spd={4:N3}" -f $d,$c.dx,$c.dy,$curHead,$curSpd); $lastDist = $d }
      if ($d -ge 1.5) { Write-Host "[+] P1 ARRIVED 1.5m"; StopAll "P1 done"; $doneP1=$true; break }
      if ([Math]::Abs($c.dx) -gt 0.5) { StopAll ("P1 cross-track dx={0:N2}" -f $c.dx); $doneP1=$true; break }
      SendCmd "M,70,70" } } }
}
SendCmd "PING"; Start-Sleep -Milliseconds 800
$p1end = GetPosFix 4
if ($p1end) { $c1 = CourseM $start.lat $start.lon $p1end.lat $p1end.lon
  Write-Host ("[=] P1: planned 1.500m, real {0:N3}m  dx={1:N3} dy={2:N3}  course={3:F1}" -f $c1.dist,$c1.dx,$c1.dy,$c1.course)
  $P1_head = $p1end.head; $P1_lat = $p1end.lat; $P1_lon = $p1end.lon
} else { Write-Host "[!] no P1 end"; $P1_head = $start.head; $P1_lat = $start.lat; $P1_lon = $start.lon }

# === Phase 2: 90° вправо ===
Write-Host ""
Write-Host "[*] Phase 2: 90 deg right turn (M,-40,40) - reset origin first"
SendCmd ("ROUTE_BEGIN,1,{0},{1}" -f $P1_lat,$P1_lon)
Start-Sleep -Milliseconds 300
$blk = RecvLine 500
if ($blk) { Write-Host ("    [<] " + ($blk -split "`n")[0]) }

$startTime = Get-Date; $doneP2 = $false
while (-not $doneP2) {
  if (((Get-Date)-$startTime).TotalSeconds -gt 30) { StopAll "P2 timeout"; $doneP2=$true; break }
  $blk = RecvLine 200
  if ($blk) { foreach ($l in ($blk -split "`n")) { $l=$l.Trim()
    if ($l -match '^TEL,') { $p=$l -split ','; $curHead=[double]$p[4]
      $dH = (($curHead - $P1_head) + 540) % 360 - 180
      if ([Math]::Abs($dH) -gt 5) { Write-Host ("    P2: head={0:F1} dH={1:F1}" -f $curHead,$dH) }
      if ($dH -ge 85) { Write-Host "[+] P2 TURNED +90"; StopAll "P2 done"; $doneP2=$true; break }
      if ($dH -gt 110) { StopAll ("P2 overshoot {0:F1}" -f $dH); $doneP2=$true; break }
      SendCmd "M,-40,40" } } }
}
SendCmd "PING"; Start-Sleep -Milliseconds 800
$p2end = GetPosFix 4
if ($p2end) { $c2 = CourseM $P1_lat $P1_lon $p2end.lat $p2end.lon
  Write-Host ("[=] P2: planned 0m turn, real shift={0:F3}m  dHead={1:F1}" -f $c2.dist,(($p2end.head - $P1_head + 540) % 360 - 180))
  $P2_head = $p2end.head; $P2_lat = $p2end.lat; $P2_lon = $p2end.lon
} else { Write-Host "[!] no P2 end"; $P2_head = $P1_head; $P2_lat = $P1_lat; $P2_lon = $P1_lon }

# === Phase 3: 1м прямо ===
Write-Host ""
Write-Host "[*] Phase 3: 1.0m straight (M,70,70) - reset origin first"
SendCmd ("ROUTE_BEGIN,1,{0},{1}" -f $P2_lat,$P2_lon)
Start-Sleep -Milliseconds 300
$blk = RecvLine 500
if ($blk) { Write-Host ("    [<] " + ($blk -split "`n")[0]) }

$startTime = Get-Date; $lastDist = 0; $doneP3 = $false
while (-not $doneP3) {
  if (((Get-Date)-$startTime).TotalSeconds -gt 30) { StopAll "P3 timeout"; $doneP3=$true; break }
  $blk = RecvLine 200
  if ($blk) { foreach ($l in ($blk -split "`n")) { $l=$l.Trim()
    if ($l -match '^TEL,') { $p=$l -split ','
      $curLat=[double]$p[1]; $curLon=[double]$p[2]; $curHead=[double]$p[4]
      $c = CourseM $P2_lat $P2_lon $curLat $curLon; $d = $c.dist
      if (($d - $lastDist) -gt 0.05) { Write-Host ("    P3: dist={0:N3} dx={1:N3} dy={2:N3} head={4:F1}" -f $d,$c.dx,$c.dy,$curHead); $lastDist = $d }
      if ($d -ge 1.0) { Write-Host "[+] P3 ARRIVED 1m"; StopAll "P3 done"; $doneP3=$true; break }
      if ([Math]::Abs($c.dx) -gt 0.5) { StopAll ("P3 cross-track dx={0:N2}" -f $c.dx); $doneP3=$true; break }
      SendCmd "M,70,70" } } }
}
SendCmd "PING"; Start-Sleep -Milliseconds 800
$p3end = GetPosFix 4
if ($p3end) { $c3 = CourseM $P2_lat $P2_lon $p3end.lat $p3end.lon
  Write-Host ("[=] P3: planned 1.000m, real {0:N3}m  dx={1:N3} dy={2:N3}" -f $c3.dist,$c3.dx,$c3.dy)
  $P3_lat = $p3end.lat; $P3_lon = $p3end.lon
} else { Write-Host "[!] no P3 end"; $P3_lat = $P2_lat; $P3_lon = $P2_lon }

Write-Host ""
Write-Host "================ ROUTE SUMMARY ================"
$c12 = CourseM $start.lat $start.lon $P1_lat $P1_lon
$c23 = CourseM $P1_lat $P1_lon $P2_lat $P2_lon
$c34 = CourseM $P2_lat $P2_lon $P3_lat $P3_lon
Write-Host ("P0->P1: planned 1.500m, real {0:N3}m, course {1:F1}deg" -f $c12.dist,$c12.course)
Write-Host ("P1->P2: planned turn 90, real shift {0:N3}m  dHead={1:F1}deg" -f $c23.dist,(($P2_head - $P1_head + 540) % 360 - 180))
Write-Host ("P2->P3: planned 1.000m, real {0:N3}m, course {1:F1}deg" -f $c34.dist,$c34.course)
$ctot = CourseM $start.lat $start.lon $P3_lat $P3_lon
Write-Host ("P0->P3 total: {0:N3}m" -f $ctot.dist)
Write-Host "=============================================="

$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
