$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait() } catch { Write-Host "[!] connect err"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] not open"; exit 1 }
Write-Host "[+] connected"

function SendCmd([string]$cmd) {
  $buf = [System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg = New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  $ws.SendAsync($seg, [System.Net.WebSockets.WebSocketMessageType]::Text, $true, $ct.Token).Wait()
}
# send + wait for a line matching $expect (or timeout), printing only ACK/ERR/DIAG lines
function SendWait([string]$cmd, [string]$expect, [int]$ms=1500) {
  SendCmd $cmd
  Write-Host "[>] $cmd"
  $recv = New-Object byte[] 4096
  $deadline = (Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $deadline) {
    $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
    $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait(400)) {
      $msg = [System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count).Trim()
      foreach($l in ($msg -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^(OK|ERR|sol=)') { Write-Host "    [<] $l" }
        if ($expect -and $l.StartsWith($expect)) { return $true }
      }
    }
  }
  return $false
}

# read origin
SendCmd "PING"
$recv = New-Object byte[] 4096
$lat=$null;$lon=$null
$dl=(Get-Date).AddSeconds(3)
while ((Get-Date) -lt $dl -and (-not $lat)) {
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  $t=$ws.ReceiveAsync($segR,$ct.Token)
  if ($t.Wait(400)) {
    $msg=[System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count).Trim()
    foreach($l in ($msg -split "`n")){ if($l -match '^TEL,'){$p=$l -split ',';$lat=[double]$p[1];$lon=[double]$p[2]} }
  }
}
if (-not $lat){ Write-Host "[!] no origin"; exit 1 }
Write-Host "[+] origin $lat $lon"

SendWait "ROUTE_BEGIN,3,$lat,$lon" "OK,ROUTE_BEGIN" | Out-Null
SendWait "ROUTE_WP,0,0.000,0.000" "OK,ROUTE_WP,0" | Out-Null
SendWait "ROUTE_WP,1,1.000,0.000" "OK,ROUTE_WP,1" | Out-Null
SendWait "ROUTE_WP,2,2.000,0.000" "OK,ROUTE_WP,2" | Out-Null
# boundary box containing the line (0,0)->(2,0)
SendWait "ROUTE_BOUNDARY_BEGIN,4" "OK,ROUTE_BOUNDARY_BEGIN" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,0,-2.0,-2.0" "OK,ROUTE_BOUNDARY_PT,0" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,1,4.0,-2.0"  "OK,ROUTE_BOUNDARY_PT,1" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,2,4.0,2.0"   "OK,ROUTE_BOUNDARY_PT,2" | Out-Null
SendWait "ROUTE_BOUNDARY_PT,3,-2.0,2.0"  "OK,ROUTE_BOUNDARY_PT,3" | Out-Null
SendWait "ROUTE_BOUNDARY_END" "OK,ROUTE_BOUNDARY_END" | Out-Null
# empty forbid set
SendWait "FORBID_BEGIN,0" "OK,FORBID_BEGIN" | Out-Null
SendWait "FORBID_END" "OK,FORBID_END" | Out-Null
# finalize
SendWait "ROUTE_END" "OK,ROUTE" 2000 | Out-Null
Start-Sleep -Milliseconds 300
SendWait "DIAG" "sol=" 2000 | Out-Null
Write-Host "[*] route uploaded. NAV_START NOT sent. robot stays put."
