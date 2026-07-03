$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3000) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] connected"

$recv = New-Object byte[] 4096
function RecvLine($ms) {
  if ($ws.State -ne 'Open') { return $null }
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  try {
    $t = $ws.ReceiveAsync($segR, $ct.Token)
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

SendCmd "PING"
$end = (Get-Date).AddSeconds(5)
while ((Get-Date) -lt $end) {
  $blk = RecvLine 600
  if ($blk) {
    foreach ($l in ($blk -split "`n")) {
      $l = $l.Trim()
      if ($l) { Write-Host $l }
    }
  }
}
try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(500) | Out-Null } catch {}
