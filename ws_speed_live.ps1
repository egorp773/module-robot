# M на RunMs, читаем TEL в реальном времени, считаем gSpeed
param([int]$Pct=100, [int]$RunMs=2000)

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { exit 1 }
if ($ws.State -ne 'Open') { exit 1 }

$recvBuf = New-Object byte[] 8192
$global:speeds = New-Object System.Collections.ArrayList
$global:lastSpeed = 0.0
$global:lastPct = 0

function SendCmd($cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(500) | Out-Null } catch {}
}

# Pre: drain
SendCmd "PING"
$end=(Get-Date).AddSeconds(2)
while ((Get-Date) -lt $end) {
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recvBuf)
  try { $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait(200)) { $null = $t.Result } else { $null = $t }
  } catch {}
}

# M phase: шлём M и параллельно слушаем speed
Write-Host ("[*] M,{0},{0} for {1}ms (watching gSpeed)" -f $Pct,$RunMs)
$end=(Get-Date).AddMilliseconds($RunMs)
$global:phase="M"
$global:sentN=0
while ((Get-Date) -lt $end) {
  SendCmd ("M,{0},{0}" -f $Pct)
  $global:sentN++
  Start-Sleep -Milliseconds 80
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recvBuf)
  try {
    $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait(20)) {
      $s=[System.Text.Encoding]::UTF8.GetString($recvBuf,0,$t.Result.Count)
      foreach ($l in ($s -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^TEL,') {
          $p=$l -split ','
          $spd = [double]$p[11]
          $head = [double]$p[4]
          if ($spd -ne $global:lastSpeed -or ($global:sentN % 5) -eq 0) {
            Write-Host ("    sent={0} gSpeed={1:N3} m/s head={2:F1}" -f $global:sentN,$spd,$head)
            $global:lastSpeed = $spd
          }
        }
        if ($l -match '^MOTOR,') {
          $p=$l -split ','
          $cmdL = [int]$p[1]; $cmdR = [int]$p[2]
          $fbL = [int]$p[4]; $fbR = [int]$p[5]
          if (($global:sentN % 5) -eq 0) {
            Write-Host ("    sent={0} cmdL={1} cmdR={2} fbL={3} fbR={4}" -f $global:sentN,$cmdL,$cmdR,$fbL,$fbR)
          }
        }
      }
    }
  } catch {}
}
SendCmd "STOP"; Start-Sleep -Milliseconds 100; SendCmd "STOP"
Write-Host ("[*] sent {0} M-commands, real M-time = {1}ms" -f $global:sentN,$RunMs)

# Post: drain ещё 2с
$end=(Get-Date).AddSeconds(2)
while ((Get-Date) -lt $end) {
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recvBuf)
  try { $t = $ws.ReceiveAsync($segR, $ct.Token)
    if ($t.Wait(200)) {
      $s=[System.Text.Encoding]::UTF8.GetString($recvBuf,0,$t.Result.Count)
      foreach ($l in ($s -split "`n")) {
        $l=$l.Trim()
        if ($l -match '^TEL,') {
          $p=$l -split ','
          $spd = [double]$p[11]
          if ($spd -ne $global:lastSpeed) {
            Write-Host ("    POST gSpeed={0:N3} m/s" -f $spd)
            $global:lastSpeed = $spd
          }
        }
      }
    } else { $null = $t }
  } catch {}
}

try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(500) | Out-Null } catch {}
