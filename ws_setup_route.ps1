Add-Type -AssemblyName System.Net.WebSockets
$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait() } catch { Write-Host "[!] connect err: $_"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] not open: $($ws.State)"; exit 1 }
Write-Host "[+] connected"

function SendCmd([string]$cmd) {
  $buf = [System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg = New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  $ws.SendAsync($seg, [System.Net.WebSockets.WebSocketMessageType]::Text, $true, $ct.Token).Wait()
  Write-Host "[>] $cmd"
}
function ReadFor([int]$ms) {
  $recv = New-Object byte[] 4096
  $deadline = (Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $deadline) {
    $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
    $task = $ws.ReceiveAsync($segR, $ct.Token)
    if ($task.Wait(500)) {
      $msg = [System.Text.Encoding]::UTF8.GetString($recv, 0, $task.Result.Count).Trim()
      if ($msg) { foreach($l in ($msg -split "`n")) { Write-Host "[<] $($l.Trim())" } }
    }
  }
}

# read origin from telemetry
SendCmd "PING"
$recv = New-Object byte[] 4096
$lat=$null; $lon=$null
$deadline=(Get-Date).AddSeconds(3)
while ((Get-Date) -lt $deadline -and (-not $lat)) {
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  $task = $ws.ReceiveAsync($segR, $ct.Token)
  if ($task.Wait(500)) {
    $msg = [System.Text.Encoding]::UTF8.GetString($recv, 0, $task.Result.Count).Trim()
    foreach($l in ($msg -split "`n")) {
      if ($l -match '^TEL,') { $p = $l -split ','; $lat=[double]$p[1]; $lon=[double]$p[2] }
    }
  }
}
if (-not $lat) { Write-Host "[!] no TEL/origin in 3s"; } else { Write-Host "[+] origin lat=$lat lon=$lon" }

if ($lat) {
  SendCmd "ROUTE_BEGIN,3,$lat,$lon"; ReadFor 400
  SendCmd "ROUTE_WP,0,0.000,0.000"; ReadFor 300
  SendCmd "ROUTE_WP,1,1.000,0.000"; ReadFor 300
  SendCmd "ROUTE_WP,2,2.000,0.000"; ReadFor 300
  SendCmd "ROUTE_END"; ReadFor 500
}
SendCmd "DIAG"; ReadFor 1500
Write-Host "[*] NOT sending NAV_START. Robot will not move."
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure, "bye", $ct.Token).Wait()
