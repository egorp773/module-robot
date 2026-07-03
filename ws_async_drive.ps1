# Async model: шлём в одном task, читаем в другом
param([int]$Pct=50,[int]$RunMs=5000)

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
$ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(4000) | Out-Null
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] open"

$runUntil = (Get-Date).AddMilliseconds($RunMs)
$stopFlag = $false

# Reader (background-ish)
$r = New-Object byte[] 16384
$readerLog = New-Object System.Collections.Generic.List[string]
$reader = {
  while (-not $stopFlag -and $ws.State -eq 'Open') {
    $sr = New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
    try {
      $t = $ws.ReceiveAsync($sr, [System.Threading.CancellationToken]::None)
      if ($t.Wait(200)) {
        $txt = [System.Text.Encoding]::UTF8.GetString($r, 0, $t.Result.Count)
        foreach ($l in ($txt -split "`n")) {
          $l = $l.Trim()
          if ($l -match '^MOTOR,') { Write-Host ("MOTOR " + $l) }
          elseif ($l -match '^TEL,') { $readerLog.Add($l) }
        }
      }
    } catch {}
  }
}

# Start reader as a job
$job = Start-Job -ScriptBlock $reader

# Wait a bit for WS state to settle (safety tick)
Start-Sleep -Milliseconds 500
$b=[System.Text.Encoding]::UTF8.GetBytes('PING'); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
$ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,[System.Threading.CancellationToken]::None).Wait(500)|Out-Null
Start-Sleep -Milliseconds 500

Write-Host "[*] sending M,$Pct,$Pct for $RunMs ms (no per-tick wait)"
while ((Get-Date) -lt $runUntil -and $ws.State -eq 'Open') {
  $b=[System.Text.Encoding]::UTF8.GetBytes("M,$Pct,$Pct"); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  try {
    $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,[System.Threading.CancellationToken]::None).Wait(500)|Out-Null
  } catch { Write-Host "[!] send failed"; break }
  Start-Sleep -Milliseconds 50
}

# STOP once
$b=[System.Text.Encoding]::UTF8.GetBytes('STOP'); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
$ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,[System.Threading.CancellationToken]::None).Wait(500)|Out-Null

$stopFlag = $true
Start-Sleep -Milliseconds 500
Remove-Job $job -Force

Write-Host "[*] WS final state=$($ws.State), reader lines=$($readerLog.Count)"
if ($readerLog.Count -gt 0) {
  $first = $readerLog[0] -split ','
  $last = $readerLog[-1] -split ','
  Write-Host ("first lat={0:F7} lon={1:F7} spd={2:F3}" -f [double]$first[1],[double]$first[2],[double]$first[11])
  Write-Host ("last lat={0:F7} lon={1:F7} spd={2:F3}" -f [double]$last[1],[double]$last[2],[double]$last[11])
  $dx=([double]$last[1]-[double]$first[1])*111320
  $dy=([double]$last[2]-[double]$first[2])*111320*[math]::Cos([double]$first[1]*0.0174533)
  Write-Host ("shift: dx={0:N3} dy={1:N3} |d|={2:N3}m" -f $dx,$dy,[math]::Sqrt($dx*$dx+$dy*$dy))
}
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
