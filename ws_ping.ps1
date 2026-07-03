$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]'ws://192.168.31.222:81/ws', $ct.Token).Wait(3000) | Out-Null } catch { Write-Host 'ERR conn'; exit 1 }
if ($ws.State -ne 'Open') { Write-Host ('state='+$ws.State); exit 1 }
Write-Host 'OK'
$buf=[System.Text.Encoding]::UTF8.GetBytes('PING')
$seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
$ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(1000) | Out-Null
Start-Sleep -Milliseconds 500
$recv=New-Object byte[] 1024
$segR=New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
$t = $ws.ReceiveAsync($segR, $ct.Token)
if ($t.Wait(1500)) { Write-Host ('[<] ' + [System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count)) }
$ws.Close()
