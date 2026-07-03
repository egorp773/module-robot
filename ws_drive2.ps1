param([int]$Pct=50, [int]$RunMs=3000)

$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.ReadTimeout = 100
$port.Open()

$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
$ws.ConnectAsync([Uri]'ws://192.168.31.222:81/ws', $ct.Token).Wait(3500) | Out-Null
if ($ws.State -ne 'Open') { $port.Close(); Write-Host "[!] ws state=$($ws.State)"; exit 1 }
Write-Host "[+] WS open"

$r = New-Object byte[] 8192
function SendCmd($cmd) {
  $b=[System.Text.Encoding]::UTF8.GetBytes($cmd); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  try { $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(300)|Out-Null } catch {}
}
function ReadCom7() {
  $out = @()
  try {
    while ($port.BytesToRead -gt 0) {
      $n = $port.BytesToRead
      $buf = New-Object byte[] $n
      $port.Read($buf, 0, $n)
      $txt = [System.Text.Encoding]::UTF8.GetString($buf, 0, $n)
      foreach ($l in ($txt -split "`n")) {
        $l = $l.Trim()
        if ($l -match 'motL=0 motR=0') { $out += $l }
      }
    }
  } catch {}
  return $out
}

# Setup: drain 1s
SendCmd "PING"; Start-Sleep -Milliseconds 1000
ReadCom7 | Select-Object -Last 1 | ForEach-Object { Write-Host ("BEFORE: " + $_) }

$start = Get-Date
$sent = 0
while (((Get-Date) - $start).TotalMilliseconds -lt $RunMs) {
  SendCmd ("M,{0},{0}" -f $Pct)
  $sent++
  Start-Sleep -Milliseconds 30
  $t = $ws.ReceiveAsync((New-Object System.ArraySegment[byte] -ArgumentList @(,$r)), $ct.Token)
  if ($t.Wait(15)) {
    $txt = [System.Text.Encoding]::UTF8.GetString($r, 0, $t.Result.Count)
    foreach ($l in ($txt -split "`n")) {
      $l = $l.Trim()
      if ($l -match '^MOTOR,') { Write-Host ("MOTOR " + $l) }
    }
  }
}
# After: read COM7 for 2s while STOP
SendCmd "STOP"
$end = (Get-Date).AddSeconds(2)
while ((Get-Date) -lt $end) {
  $t = $ws.ReceiveAsync((New-Object System.ArraySegment[byte] -ArgumentList @(,$r)), $ct.Token)
  if ($t.Wait(100)) {
    $txt = [System.Text.Encoding]::UTF8.GetString($r, 0, $t.Result.Count)
    foreach ($l in ($txt -split "`n")) {
      $l = $l.Trim()
      if ($l -match '^MOTOR,' -or $l -match '^TEL,') { Write-Host ("post " + $l) }
    }
  }
  ReadCom7 | Select-Object -Last 1 | ForEach-Object { Write-Host ("AFTER: " + $_) }
}

Write-Host ("[*] sent $sent M-commands in $RunMs ms")
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
$port.Close()
