param([int]$MaxSec=60)
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
$ws.ConnectAsync([Uri]'ws://192.168.31.222:81/ws', $ct.Token).Wait(3000) | Out-Null
if ($ws.State -ne 'Open') { Write-Host "[!] ws not open"; exit 1 }
$r = New-Object byte[] 4096
$start = Get-Date
$lastHacc = 999
while (((Get-Date)-$start).TotalSeconds -lt $MaxSec) {
  $b=[System.Text.Encoding]::UTF8.GetBytes('PING'); $s=New-Object System.ArraySegment[byte] -ArgumentList @(,$b)
  $ws.SendAsync($s,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(200)|Out-Null
  Start-Sleep -Milliseconds 200
  $sr=New-Object System.ArraySegment[byte] -ArgumentList @(,$r)
  $t=$ws.ReceiveAsync($sr,$ct.Token)
  if ($t.Wait(500)) {
    $txt=[System.Text.Encoding]::UTF8.GetString($r,0,$t.Result.Count)
    foreach ($l in ($txt -split "`n")) {
      $l = $l.Trim()
      if ($l -match '^TEL,') {
        $p = $l -split ','
        $hacc = [double]$p[8]
        $sol = [int]$p[5]
        $head = [double]$p[4]
        $spd = [double]$p[11]
        $lat = [double]$p[1]
        $lon = [double]$p[2]
        if ($hacc -lt $lastHacc -or ($sol -ge 2 -and $hacc -lt 0.05)) {
          Write-Host ("[{0:F1}s] sol={1} hAcc={2:F4}m spd={3:N3} head={4:F1}" -f ((Get-Date)-$start).TotalSeconds,$sol,$hacc,$spd,$head)
          $lastHacc = $hacc
          if ($sol -ge 2 -and $hacc -lt 0.05) {
            Write-Host ("[+] FIXED at {0:F7}, {1:F7}" -f $lat,$lon)
            $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
            exit 0
          }
        }
      }
    }
  }
}
Write-Host "[!] no FIXED in ${MaxSec}s, last hacc=$lastHacc"
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,'bye',$ct.Token).Wait(500)|Out-Null
exit 1
