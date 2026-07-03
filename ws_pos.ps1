param([int]$RunMs=4000,[int]$Pct=55)
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]'ws://192.168.31.222:81/ws', $ct.Token).Wait(3000) | Out-Null } catch { Write-Host 'ERR conn'; exit 1 }
if ($ws.State -ne 'Open') { Write-Host ('state='+$ws.State); exit 1 }
Write-Host 'OK'

function SendCmd($cmd){ $b=[System.Text.Encoding]::UTF8.GetBytes($cmd); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b); try{$ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(500)|Out-Null}catch{} }
function DrainMs([int]$ms){ $end=(Get-Date).AddMilliseconds($ms); $r=New-Object byte[] 8192; while((Get-Date) -lt $end){ $sr=New-Object System.ArraySegment[byte] -ArgumentList @(,$r); try{ $t=$ws.ReceiveAsync($sr,$ct.Token); if($t.Wait(50)){ $s=[System.Text.Encoding]::UTF8.GetString($r,0,$t.Result.Count); foreach($l in ($s -split "`n")){ $l=$l.Trim(); if($l -match '^TEL,' -or $l -match '^MOTOR,' -or $l -match '^sol=' -or $l -match '^STATE'){ return $l } } } }catch{} } return $null }

SendCmd "PING"; Start-Sleep -Milliseconds 200
$t1 = DrainMs 2500
Write-Host ("BEFORE: {0}" -f $t1)
if (-not $t1) { Write-Host 'no TEL yet, retry'; $t1 = DrainMs 2500; Write-Host ("BEFORE2: {0}" -f $t1) }

$end=(Get-Date).AddMilliseconds($RunMs)
$sent=0
while ((Get-Date) -lt $end) {
  SendCmd ("M,{0},{0}" -f $Pct); $sent++
  Start-Sleep -Milliseconds 80
  $line = DrainMs 30
  if ($line) { Write-Host ("  sent={0}: {1}" -f $sent,$line) }
}
SendCmd "STOP"; Start-Sleep -Milliseconds 200; SendCmd "STOP"
Start-Sleep -Milliseconds 800
$t2 = DrainMs 2500
Write-Host ("AFTER : {0}" -f $t2)

if ($t1 -and $t2) {
  $p1=$t1 -split ','; $p2=$t2 -split ','
  $lat1=[double]$p1[1]; $lon1=[double]$p1[2]
  $lat2=[double]$p2[1]; $lon2=[double]$p2[2]
  $dx = ($lat2-$lat1)*111320
  $dy = ($lon2-$lon1)*111320*([math]::Cos($lat1*0.0174533))
  Write-Host ("DIST dx={0:N3} dy={1:N3} |d|={2:N3} m" -f $dx,$dy,[math]::Sqrt($dx*$dx+$dy*$dy))
}
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
