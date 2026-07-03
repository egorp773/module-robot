# Подключаюсь, просто сижу 5с, шлю PING каждые 500мс. Смотрю safety каждую секунду через COM7 (в фоне)
param([int]$Sec=10)

$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.ReadTimeout = 200
$port.Open()

$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
$ws.ConnectAsync([Uri]'ws://192.168.31.222:81/ws', $ct.Token).Wait(3000) | Out-Null
Write-Host ("[+] WS state=" + $ws.State)
if ($ws.State -ne 'Open') { $port.Close(); exit 1 }

$r = New-Object byte[] 8192
function SendCmd($cmd) {
  $b=[System.Text.Encoding]::UTF8.GetBytes($cmd); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  try { $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(200)|Out-Null } catch { Write-Host "[!] send fail: $_" }
}

function ReadCom7() {
  try {
    if ($port.BytesToRead -gt 0) {
      $n = $port.BytesToRead
      $buf = New-Object byte[] $n
      $port.Read($buf, 0, $n)
      $txt = [System.Text.Encoding]::UTF8.GetString($buf, 0, $n)
      foreach ($l in ($txt -split "`n")) {
        $l = $l.Trim()
        if ($l -match 'safety=') { Write-Host ("COM7: " + $l) }
      }
    }
  } catch {}
}

for ($i = 0; $i -lt $Sec; $i++) {
  Start-Sleep -Milliseconds 200
  SendCmd "PING"
  Start-Sleep -Milliseconds 300
  ReadCom7
  Write-Host ("WS state=" + $ws.State)
}

$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
$port.Close()
