# Минимальный тест: открыть WS, держать открытым, лить M,100,100 ~150мс
param([int]$Pct=100, [int]$RunMs=1500)

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { exit 1 }
Write-Host "[+] connected"

# Receive thread: drain WS so connection stays alive
$recvBuf = New-Object byte[] 8192
function Drain($ms) {
  if ($ws.State -ne 'Open') { return }
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recvBuf)
  try {
    $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait($ms)) { $null = $t.Result } else { $null = $t }  # just consume
  } catch {}
}
function SendCmd($cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(500) | Out-Null } catch {}
}

# Print safety first
SendCmd "PING"
Start-Sleep -Milliseconds 200
$end=(Get-Date).AddSeconds(2)
while ((Get-Date) -lt $end) {
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recvBuf)
  try {
    $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait(200)) {
      $s=[System.Text.Encoding]::UTF8.GetString($recvBuf,0,$t.Result.Count)
      foreach ($l in ($s -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^TEL,') {
          $p=$l -split ','
          Write-Host ("    pre  TEL sol={0} hAcc={1}mm head={2} pvtAge={3}" -f $p[5],$p[9],$p[4],$p[13])
        }
      }
    }
  } catch {}
}

Write-Host ("[*] send M,{0},{0} for {1}ms" -f $Pct,$RunMs)
$end=(Get-Date).AddMilliseconds($RunMs)
$i=0
$t0=Get-Date
while ((Get-Date) -lt $end) {
  SendCmd ("M,{0},{0}" -f $Pct)
  $i++
  Start-Sleep -Milliseconds 100
}
SendCmd "STOP"; Start-Sleep -Milliseconds 100; SendCmd "STOP"
$dt = ((Get-Date) - $t0).TotalSeconds
Write-Host ("[*] sent {0} M-commands in {1:N2}s" -f $i,$dt)

# Print safety after
SendCmd "PING"
Start-Sleep -Milliseconds 200
$end=(Get-Date).AddSeconds(2)
while ((Get-Date) -lt $end) {
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recvBuf)
  try {
    $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait(200)) {
      $s=[System.Text.Encoding]::UTF8.GetString($recvBuf,0,$t.Result.Count)
      foreach ($l in ($s -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^TEL,') {
          $p=$l -split ','
          Write-Host ("    post TEL sol={0} hAcc={1}mm head={2} pvtAge={3}" -f $p[5],$p[9],$p[4],$p[13])
        }
      }
    }
  } catch {}
}

try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(500) | Out-Null } catch {}
