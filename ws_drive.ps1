# Постоянное соединение, шлём M,50,50 каждые 20мс, читаем TEL.
# Без закрытия WS во время теста!
param([int]$Pct=50, [int]$RunMs=4000)

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3500) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] connected"

$r = New-Object byte[] 16384
$lines = New-Object System.Collections.Generic.List[string]
$lastTel = $null

function SendCmd($cmd) {
  $b=[System.Text.Encoding]::UTF8.GetBytes($cmd); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  try { $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(200)|Out-Null } catch {}
}

# PING + drain 1s
SendCmd "PING"
$end = (Get-Date).AddSeconds(1)
while ((Get-Date) -lt $end) {
  $sr = New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
  try { $t=$ws.ReceiveAsync($sr,$ct.Token); if($t.Wait(100)){ $txt=[System.Text.Encoding]::UTF8.GetString($r,0,$t.Result.Count); foreach($l in ($txt -split "`n")){ $l=$l.Trim(); if($l -match '^TEL,'){ $lastTel = $l } } } } catch {}
}

if (-not $lastTel) { Write-Host "[!] no TEL after 1s"; exit 1 }
Write-Host ("[+] lastTel: " + $lastTel.Substring(0, [Math]::Min(100,$lastTel.Length)))

# Phase: M loop + listen
$start = Get-Date
$sent = 0
while (((Get-Date) - $start).TotalMilliseconds -lt $RunMs) {
  SendCmd ("M,{0},{0}" -f $Pct)
  $sent++
  Start-Sleep -Milliseconds 20
  $sr = New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
  try {
    $t = $ws.ReceiveAsync($sr, $ct.Token)
    if ($t.Wait(15)) {
      $txt = [System.Text.Encoding]::UTF8.GetString($r, 0, $t.Result.Count)
      foreach ($l in ($txt -split "`n")) {
        $l = $l.Trim()
        if ($l -match '^TEL,') { $lastTel = $l }
        elseif ($l -match '^MOTOR,') { Write-Host ("MOTOR " + $l.Substring(0,[Math]::Min(60,$l.Length))) }
      }
    }
  } catch {}
}
$elapsed = ((Get-Date)-$start).TotalSeconds
Write-Host ("[*] sent={0} elapsed={1:N2}s  lastTel head=" -f $sent,$elapsed)
if ($lastTel) {
  $p = $lastTel -split ','
  Write-Host ("    head={0:F1} spd={1:F3} hAcc={2:F0}mm" -f [double]$p[4],[double]$p[11],[double]$p[8])
}

# Soft hold: держим WS открытым, шлём STOP один раз
Write-Host "[*] sending STOP and HOLDING WS for 3s to read safety"
SendCmd "STOP"
$end = (Get-Date).AddSeconds(3)
while ((Get-Date) -lt $end) {
  $sr = New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
  try { $t=$ws.ReceiveAsync($sr,$ct.Token); if($t.Wait(200)){
    $txt=[System.Text.Encoding]::UTF8.GetString($r,0,$t.Result.Count)
    foreach($l in ($txt -split "`n")){ $l=$l.Trim(); if($l -match '^TEL,' -or $l -match '^MOTOR,' -or $l -match '^NAV,'){ Write-Host ("post " + $l.Substring(0,[Math]::Min(60,$l.Length))) } }
  } } catch {}
}

# CLOSE WS (gracefully with close frame)
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
