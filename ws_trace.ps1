# Trace: 5 сек чисто M,50,50, каждую секунду печатаем cmdL/cmdR/fbL/fbR/head/spd
param([int]$Pct=50,[int]$RunMs=5000)

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }

$r = New-Object byte[] 16384
$lastT = 0
$lines = New-Object System.Collections.Generic.List[string]

function PumpLines([int]$msBudget) {
  $deadline = (Get-Date).AddMilliseconds($msBudget)
  while ((Get-Date) -lt $deadline) {
    $sr = New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
    try {
      $t = $ws.ReceiveAsync($sr, $ct.Token)
      if ($t.Wait(80)) {
        if ($t.Result.Count -gt 0) {
          $txt = [System.Text.Encoding]::UTF8.GetString($r, 0, $t.Result.Count)
          $lines.Add($txt)
        }
      }
    } catch { break }
  }
}

function SendCmd($cmd) {
  $b=[System.Text.Encoding]::UTF8.GetBytes($cmd); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  try { $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(500)|Out-Null } catch {}
}

# Drain startup
SendCmd "PING"
PumpLines 1500 | Out-Null
$lines.Clear()

Write-Host ("[*] T+0.0  START  M,{0},{0}" -f $Pct)
$start = Get-Date
$sent = 0
while (((Get-Date) - $start).TotalMilliseconds -lt $RunMs) {
  SendCmd ("M,{0},{0}" -f $Pct)
  $sent++
  Start-Sleep -Milliseconds 60
  PumpLines 60 | Out-Null
}
# NO STOP — just let it coast, see if hold works
Write-Host ("[*] T+{0:N1}  SENT {1} M-commands, NO STOP, coasting 2s" -f ((Get-Date)-$start).TotalSeconds,$sent)
PumpLines 2000 | Out-Null

Write-Host "================ TRACE ================"
foreach ($chunk in $lines) {
  foreach ($l in ($chunk -split "`n")) {
    $l = $l.Trim()
    if ($l -match '^(TEL|MOTOR|NAV|ERR),') { Write-Host $l }
  }
}
Write-Host "======================================="

$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
