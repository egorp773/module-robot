# Калибровка скорости: M,pct,pct на RunMs -> замер GPS-дистанции -> реальная м/с
param([int]$Pct=25, [int]$RunMs=300)

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { exit 1 }
if ($ws.State -ne 'Open') { exit 1 }

$recv = New-Object byte[] 8192
function RecvLine($ms) {
  if ($ws.State -ne 'Open') { return $null }
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  try { $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait($ms)) { return [System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count).Trim() }
  } catch { return $null }
  return $null
}
function SendCmd($cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(1000) | Out-Null } catch {}
}
function GetPosFix($timeoutSec) {
  $end=(Get-Date).AddSeconds($timeoutSec)
  $lat=0;$lon=0;$head=0;$hacc=999
  while ((Get-Date) -lt $end) {
    $blk = RecvLine 500
    if ($blk) {
      foreach ($l in ($blk -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^TEL,') {
          $p=$l -split ','
          $lat=[double]$p[1]; $lon=[double]$p[2]; $head=[double]$p[4]; $hacc=[double]$p[8]
          if ($hacc -lt 30) { return @{lat=$lat;lon=$lon;head=$head;hacc=$hacc} }
        }
      }
    }
  }
  return $null
}
function CourseM($lat1,$lon1,$lat2,$lon2) {
  $r=6378137.0
  $phi=$lat1*[Math]::PI/180.0
  $dx=($lon2-$lon1)*[Math]::PI/180.0*[Math]::Cos($phi)*$r
  $dy=($lat2-$lat1)*[Math]::PI/180.0*$r
  $dist=[Math]::Sqrt($dx*$dx+$dy*$dy)
  return [pscustomobject]@{dx=$dx;dy=$dy;dist=$dist}
}

SendCmd "PING"
$start = GetPosFix 8
if (-not $start) { Write-Host "[!] no FIXED"; exit 1 }
Write-Host ("[+] start lat={0:F7} lon={1:F7} hAcc={2}mm" -f $start.lat,$start.lon,$start.hacc)

Write-Host ("[*] M,{0},{0} for {1}ms" -f $Pct,$RunMs)
$t0 = Get-Date
$end = (Get-Date).AddMilliseconds($RunMs)
while ((Get-Date) -lt $end) { SendCmd ("M,{0},{0}" -f $Pct); Start-Sleep -Milliseconds 70 }
$dt = ((Get-Date) - $t0).TotalSeconds
SendCmd "STOP"; Start-Sleep -Milliseconds 100; SendCmd "STOP"
Start-Sleep -Milliseconds 1500

SendCmd "PING"
$endFix = GetPosFix 4
if (-not $endFix) { Write-Host "[!] no end fix"; exit 1 }

$c = CourseM $start.lat $start.lon $endFix.lat $endFix.lon
$v = $c.dist / $dt
Write-Host ""
Write-Host ("[=] cmd={0}%, run={1}ms, real_dist={2:N3}m, real_speed={3:N3} m/s" -f $Pct,$RunMs,$c.dist,$v)
Write-Host ("    dx={0:N3} dy={1:N3}" -f $c.dx,$c.dy)

try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(500) | Out-Null } catch {}
