# Ехать по точкам через M-команды с постоянным WS
# Phase 1: 1.5м прямо (M,50,50)
# Phase 2: 90° право (M,-30,30)
# Phase 3: 1м прямо (M,50,50)
param([int]$PctGo=50,[int]$PctTurn=-30)

$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.ReadTimeout = 100
$port.Open()

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { $port.Close(); exit 1 }
if ($ws.State -ne 'Open') { $port.Close(); Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] WS open"

$r = New-Object byte[] 16384
function SendCmd($cmd) {
  $b=[System.Text.Encoding]::UTF8.GetBytes($cmd); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  try { $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(200)|Out-Null } catch { Write-Host "[!] send: $_" }
}
function Drain($ms) {
  $end = (Get-Date).AddMilliseconds($ms)
  $got = @()
  while ((Get-Date) -lt $end) {
    $sr = New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
    try {
      $t = $ws.ReceiveAsync($sr, $ct.Token)
      if ($t.Wait(50)) {
        $txt = [System.Text.Encoding]::UTF8.GetString($r, 0, $t.Result.Count)
        foreach ($l in ($txt -split "`n")) { $got += $l.Trim() }
      }
    } catch {}
  }
  return $got
}
function GetFixPos([int]$tries=20) {
  for ($i = 0; $i -lt $tries; $i++) {
    $lines = Drain 400
    foreach ($l in $lines) {
      if ($l -match '^TEL,') {
        $p = $l -split ','
        $hacc = [double]$p[8]
        if ($hacc -lt 0.05) { return @{lat=[double]$p[1]; lon=[double]$p[2]; head=[double]$p[4]; spd=[double]$p[11]} }
      }
    }
  }
  return $null
}
function CourseM($lat1,$lon1,$lat2,$lon2) {
  $r=6378137.0; $phi=$lat1*[Math]::PI/180.0
  $dx=($lon2-$lon1)*[Math]::PI/180.0*[Math]::Cos($phi)*$r
  $dy=($lat2-$lat1)*[Math]::PI/180.0*$r
  return [pscustomobject]@{dx=$dx;dy=$dy;dist=[Math]::Sqrt($dx*$dx+$dy*$dy);course=([Math]::Atan2($dx,$dy)*180.0/[Math]::PI+360.0)%360.0}
}

# Setup
SendCmd "PING"
Drain 1500 | Out-Null
$start = GetFixPos 30
if (-not $start) { Write-Host "[!] no FIX"; $port.Close(); $ws.Close(); exit 1 }
Write-Host ("[+] P0 lat={0:F7} lon={1:F7} hAcc=fix head={2:F1}" -f $start.lat,$start.lon,$start.head)
$startLat = $start.lat; $startLon = $start.lon; $startHead = $start.head

# === Phase 1: 1.5м прямо ===
Write-Host ""
Write-Host "[*] Phase 1: 1.5m straight (M,$PctGo,$PctGo)"
$phaseStart = Get-Date
$sent = 0
while (((Get-Date)-$phaseStart).TotalSeconds -lt 25) {
  SendCmd ("M,{0},{0}" -f $PctGo)
  $sent++
  Start-Sleep -Milliseconds 30
  $lines = Drain 25
  $pos = $null
  foreach ($l in $lines) {
    if ($l -match '^TEL,') {
      $p = $l -split ','
      $hacc = [double]$p[8]
      if ($hacc -lt 0.05) {
        $pos = @{lat=[double]$p[1]; lon=[double]$p[2]; head=[double]$p[4]; spd=[double]$p[11]}
      }
    }
  }
  if ($pos) {
    $c = CourseM $startLat $startLon $pos.lat $pos.lon
    if ($c.dist -ge 1.5) {
      Write-Host ("[+] P1 ARRIVED dist={0:N3} dx={1:N3} dy={2:N3} spd={3:N3}" -f $c.dist,$c.dx,$c.dy,$pos.spd)
      SendCmd "STOP"
      break
    }
  }
}
Write-Host ("[*] P1: $sent M-cmd sent")
$P1 = GetFixPos 15
if ($P1) {
  $c1 = CourseM $startLat $startLon $P1.lat $P1.lon
  Write-Host ("[=] P1 final: real={0:N3}m dx={1:N3} dy={2:N3} course={3:F1}" -f $c1.dist,$c1.dx,$c1.dy,$c1.course)
  $P1Lat = $P1.lat; $P1Lon = $P1.lon; $P1Head = $P1.head
} else { Write-Host "[!] P1 no fix"; $P1Lat=$startLat; $P1Lon=$startLon; $P1Head=$startHead }
Start-Sleep -Milliseconds 500

# === Phase 2: 90° право ===
Write-Host ""
Write-Host ("[*] Phase 2: 90 deg right (M,{0},{1})" -f $PctTurn,[Math]::Abs($PctTurn))
$phaseStart = Get-Date
$sent = 0
while (((Get-Date)-$phaseStart).TotalSeconds -lt 15) {
  $turnCmd = [Math]::Abs($PctTurn)
  SendCmd ("M,{0},{1}" -f (-$turnCmd), $turnCmd)
  $sent++
  Start-Sleep -Milliseconds 30
  $lines = Drain 25
  $pos = $null
  foreach ($l in $lines) {
    if ($l -match '^TEL,') {
      $p = $l -split ','
      $hacc = [double]$p[8]
      if ($hacc -lt 0.05) { $pos = @{lat=[double]$p[1]; lon=[double]$p[2]; head=[double]$p[4]; spd=[double]$p[11]} }
    }
  }
  if ($pos) {
    $dH = (($pos.head - $P1Head) + 540) % 360 - 180
    if ($dH -ge 85 -and $dH -le 110) {
      Write-Host ("[+] P2 TURNED dH={0:F1}° spd={1:N3}" -f $dH,$pos.spd)
      SendCmd "STOP"
      break
    }
  }
}
Write-Host ("[*] P2: $sent turn-cmd sent")
$P2 = GetFixPos 15
if ($P2) {
  $c2 = CourseM $P1Lat $P1Lon $P2.lat $P2.lon
  $dH2 = (($P2.head - $P1Head) + 540) % 360 - 180
  Write-Host ("[=] P2: shift={0:N3} dHead={1:F1}" -f $c2.dist,$dH2)
  $P2Lat = $P2.lat; $P2Lon = $P2.lon; $P2Head = $P2.head
} else { Write-Host "[!] P2 no fix"; $P2Lat=$P1Lat; $P2Lon=$P1Lon; $P2Head=$P1Head }
Start-Sleep -Milliseconds 500

# === Phase 3: 1м прямо ===
Write-Host ""
Write-Host "[*] Phase 3: 1m straight (M,$PctGo,$PctGo)"
$phaseStart = Get-Date
$sent = 0
while (((Get-Date)-$phaseStart).TotalSeconds -lt 15) {
  SendCmd ("M,{0},{0}" -f $PctGo)
  $sent++
  Start-Sleep -Milliseconds 30
  $lines = Drain 25
  $pos = $null
  foreach ($l in $lines) {
    if ($l -match '^TEL,') {
      $p = $l -split ','
      $hacc = [double]$p[8]
      if ($hacc -lt 0.05) { $pos = @{lat=[double]$p[1]; lon=[double]$p[2]; head=[double]$p[4]; spd=[double]$p[11]} }
    }
  }
  if ($pos) {
    $c = CourseM $P2Lat $P2Lon $pos.lat $pos.lon
    if ($c.dist -ge 1.0) {
      Write-Host ("[+] P3 ARRIVED dist={0:N3} spd={1:N3}" -f $c.dist,$pos.spd)
      SendCmd "STOP"
      break
    }
  }
}
Write-Host ("[*] P3: $sent M-cmd sent")
$P3 = GetFixPos 15
if ($P3) {
  $c3 = CourseM $P2Lat $P2Lon $P3.lat $P3.lon
  Write-Host ("[=] P3: real={0:N3} dx={1:N3} dy={2:N3}" -f $c3.dist,$c3.dx,$c3.dy)
  $P3Lat=$P3.lat; $P3Lon=$P3.lon
} else { Write-Host "[!] P3 no fix"; $P3Lat=$P2Lat; $P3Lon=$P2Lon }

Write-Host ""
Write-Host "============== ROUTE SUMMARY =============="
$cT = CourseM $startLat $startLon $P3Lat $P3Lon
Write-Host ("P0->P3 total: {0:N3}m" -f $cT.dist)
Write-Host "==========================================="

$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
$port.Close()
