# wsrun.ps1 - upload SHORT route, NAV_START, live telemetry with AUTO-STOP
$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3000) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] connected"
$recv = New-Object byte[] 4096
function RecvLine([int]$ms) {
  if ($ws.State -ne 'Open') { return $null }
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  try { $t = $ws.ReceiveAsync($segR, $ct.Token); if ($t.Wait($ms)) { return [System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count).Trim() } } catch { return $null }
  return $null
}
function SendCmd([string]$cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd); $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(1000) | Out-Null } catch {}
}
function SendWait([string]$cmd,[string]$expect,[int]$ms=2000) {
  SendCmd $cmd; Write-Host "[>] $cmd"; $dl=(Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $dl) { $blk = RecvLine 400
    if ($blk) { foreach($l in ($blk -split "`n")) { $l=$l.Trim()
      if ($l -match '^(OK|ERR|sol=)') { Write-Host "    [<] $l" }
      if ($expect -and $l.StartsWith($expect)) { return $true } } } }
  Write-Host "    [!] no '$expect' in ${ms}ms"; return $false
}
# origin
SendCmd "PING"; $lat=$null;$lon=$null;$dl=(Get-Date).AddSeconds(3)
while ((Get-Date) -lt $dl -and (-not $lat)) { $blk = RecvLine 400
  if ($blk) { foreach($l in ($blk -split "`n")){ if($l -match '^TEL,'){$p=$l -split ',';$lat=[double]$p[1];$lon=[double]$p[2]} } } }
if (-not $lat) { Write-Host "[!] no origin"; exit 1 }
Write-Host "[+] origin $lat $lon"
# SHORT route: single waypoint 0.5m east
SendWait "ROUTE_BEGIN,2,$lat,$lon" "OK,ROUTE_BEGIN" | Out-Null
SendWait "ROUTE_WP,0,0.000,0.000" "OK,ROUTE_WP,0" | Out-Null
SendWait "ROUTE_WP,1,0.500,0.000" "OK,ROUTE_WP,1" | Out-Null
SendWait "ROUTE_BOUNDARY_BEGIN,4" "OK,ROUTE_BOUNDARY_BEGIN" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,0,-1.5,-1.5" "OK,ROUTE_BOUNDARY_PT,0" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,1,2.0,-1.5"  "OK,ROUTE_BOUNDARY_PT,1" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,2,2.0,1.5"   "OK,ROUTE_BOUNDARY_PT,2" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,3,-1.5,1.5"  "OK,ROUTE_BOUNDARY_PT,3" | Out-Null
SendWait "ROUTE_BOUNDARY_END" "OK,ROUTE_BOUNDARY_END" | Out-Null
SendWait "FORBID_BEGIN,0" "OK,FORBID_BEGIN" | Out-Null
SendWait "FORBID_END" "OK,FORBID_END" | Out-Null
$ok = SendWait "ROUTE_END" "OK,ROUTE" 2500
if (-not $ok) { Write-Host "[!] route not ready, abort"; exit 1 }
SendWait "DIAG" "sol=" 2000 | Out-Null
Write-Host "[*] SHORT route (0.5m east) uploaded. NAV_START NOT sent yet."
try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(1000) | Out-Null } catch {}
