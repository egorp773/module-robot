# Параллельно: WS шлёт M + читает телеметрию, отдельный фоновый ридер COM7
param([int]$Pct=100, [int]$RunMs=2000)

# Старт фонового ридера COM7 в файл
$logPath = "C:\robot\module\com7_$(Get-Date -Format HHmmss).log"
"=== COM7 start $(Get-Date -Format o) ===" | Out-File $logPath -Encoding utf8
$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.ReadTimeout=300
$port.Open()

$stop=$false
$bg = {
  $port=$args[0]; $lp=$args[1]; $sc=$args[2]
  while (-not $sc.Value) {
    try {
      $l = $port.ReadLine()
      if ($l) { $l | Out-File $lp -Append -Encoding utf8 }
    } catch [System.TimeoutException] {}
  }
  $port.Close()
}
$sync = [pscustomobject]@{Value=$false}
Start-Job -ScriptBlock $bg -ArgumentList $port,$logPath,$sync | Out-Null

Start-Sleep -Milliseconds 200

# WS
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]'ws://192.168.31.222:81/ws', $ct.Token).Wait(3000) | Out-Null } catch { $sync.Value=$true; exit 1 }
if ($ws.State -ne 'Open') { $sync.Value=$true; exit 1 }
Write-Host "[+] connected, COM7 log: $logPath"

$recvBuf = New-Object byte[] 8192
function SendCmd($cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd)
  $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(500) | Out-Null } catch {}
}
function Drain($ms) {
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recvBuf)
  try { $t = $ws.ReceiveAsync($segR, $ct.Token); if ($t.Wait($ms)) { $null=$t.Result } } catch {}
}

# Drain pre
Drain 1500
Drain 1500

Write-Host ("[*] send M,{0},{0} for {1}ms" -f $Pct,$RunMs)
$end=(Get-Date).AddMilliseconds($RunMs)
$i=0
while ((Get-Date) -lt $end) { SendCmd ("M,{0},{0}" -f $Pct); $i++; Start-Sleep -Milliseconds 80; Drain 20 }
SendCmd "STOP"; Drain 100; SendCmd "STOP"
Write-Host ("[*] sent {0} M-commands" -f $i)

# Drain post
$end=(Get-Date).AddSeconds(3)
$lastSafety=''
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
          $msg = ("    post TEL sol={0} hAcc={1}mm head={2}" -f $p[5],$p[9],$p[4])
          if ($msg -ne $lastSafety) { Write-Host $msg; $lastSafety=$msg }
        }
      }
    }
  } catch {}
}

$sync.Value = $true
try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(500) | Out-Null } catch {}
Start-Sleep -Milliseconds 500
Write-Host "[*] COM7 log: $logPath"
