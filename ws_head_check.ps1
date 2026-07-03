$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
$ws.ConnectAsync([Uri]'ws://192.168.31.222:81/ws', $ct.Token).Wait(3000) | Out-Null
$bytes = [System.Text.Encoding]::UTF8.GetBytes('PING')
$sendSegment = [System.ArraySegment[byte]]::new($bytes)
$ws.SendAsync($sendSegment,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(500) | Out-Null
$r=New-Object byte[] 4096
$end=(Get-Date).AddSeconds(8)
while ((Get-Date) -lt $end) {
  $sr=[System.ArraySegment[byte]]::new($r)
  $t=$ws.ReceiveAsync($sr,$ct.Token)
  if ($t.Wait(500)) {
    $s=[System.Text.Encoding]::UTF8.GetString($r,0,$t.Result.Count)
    foreach($l in ($s -split "`n")){
      if ($l -match '^TEL,') {
        $p=$l -split ','
        Write-Host ("head={0,6:F1}  imuYaw={1,7:F1}  speed={2,5:F3}  sol={3}" -f [double]$p[4],[double]$p[16],[double]$p[11],[int]$p[5])
      }
    }
  }
}
try {
  $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
} catch {}
