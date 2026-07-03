# wsgo.ps1 - NAV_START + live monitor with AUTO-STOP watchdog
$uri = "ws://192.168.31.222:81/ws"
$log = "C:\robot\module\nav_run.log"
"=== run $(Get-Date -Format o) ===" | Out-File $log -Encoding utf8
$ws = New-Object System.Net.WebSockets.ClientWebSocket
$ct = New-Object System.Threading.CancellationTokenSource
try { $ws.ConnectAsync([Uri]$uri, $ct.Token).Wait(3000) | Out-Null } catch { Write-Host "[!] connect exc"; exit 1 }
if ($ws.State -ne 'Open') { Write-Host "[!] state=$($ws.State)"; exit 1 }
Write-Host "[+] connected"
$recv = New-Object byte[] 8192
function RecvLine([int]$ms) {
  if ($ws.State -ne 'Open') { return $null }
  $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
  try { $t = $ws.ReceiveAsync($segR, $ct.Token); if ($t.Wait($ms)) { return [System.Text.Encoding]::UTF8.GetString($recv,0,$t.Result.Count).Trim() } } catch { return $null }
  return $null
}
function SendCmd([string]$cmd) {
  if ($ws.State -ne 'Open') { return }
  $buf=[System.Text.Encoding]::UTF8.GetBytes($cmd); $seg=New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
  try { $ws.SendAsync($seg,[System.Net.WebSockets.WebSocketMessageType]::Text,$true,$ct.Token).Wait(1000) | Out-Null } catch {}
}
function STOP([string]$why) {
  Write-Host "[!!!] AUTO-STOP: $why"; "AUTO-STOP: $why" | Out-File $log -Append -Encoding utf8
  SendCmd "NAV_STOP"; Start-Sleep -Milliseconds 100; SendCmd "STOP"; Start-Sleep -Milliseconds 100; SendCmd "NAV_STOP"
}

SendCmd "LOG,1"   # ensure TEL/NAV streaming
Start-Sleep -Milliseconds 200
SendCmd "NAV_START"; Write-Host "[>] NAV_START"
# confirm OK or ERR
$started=$false; $dl=(Get-Date).AddSeconds(2)
while ((Get-Date) -lt $dl) { $blk=RecvLine 300
  if ($blk) { foreach($l in ($blk -split "`n")){ $l=$l.Trim()
    if ($l -like 'OK,NAV_START*'){ Write-Host "    [<] $l"; $started=$true }
    elseif ($l -like 'ERR*'){ Write-Host "    [<] $l" } } }
  if ($started){ break } }
if (-not $started) { Write-Host "[!] NAV_START not confirmed -> stop"; STOP "no NAV_START ack"; exit 1 }
Write-Host "[+] NAV running. Watchdog active. Max 25s."

# watchdog params
$maxRun = 25            # seconds hard cap
$hdgBadSince = $null    # headingErr > 60 deg
$lastTel = Get-Date
$runStart = Get-Date
$done = $false
while (-not $done) {
  if (((Get-Date)-$runStart).TotalSeconds -gt $maxRun) { STOP "max runtime ${maxRun}s"; break }
  $blk = RecvLine 300
  if (-not $blk) {
    if (((Get-Date)-$lastTel).TotalSeconds -gt 1.5) { STOP "telemetry timeout 1.5s" ; break }
    continue
  }
  foreach($l in ($blk -split "`n")) { $l=$l.Trim(); if(-not $l){continue}
    if ($l -match '^(TEL|NAV|IMU|MOTOR|RTCM),') { $l | Out-File $log -Append -Encoding utf8 }
    if ($l -match '^NAV,') {
      $lastTel = Get-Date
      $p = $l -split ','
      # NAV,state,wpIdx,wpTotal,distToWp,headingErr,crossTrack,L,R,reason
      $st=$p[1]; $hErr=[double]$p[5]; $ct2=[double]$p[6]
      Write-Host ("    NAV {0} hErr={1:N1} ct={2:N2} L={3} R={4}" -f $st,$hErr,$ct2,$p[7],$p[8])
      if ([math]::Abs($hErr) -gt 60) { if(-not $hdgBadSince){$hdgBadSince=Get-Date} elseif(((Get-Date)-$hdgBadSince).TotalSeconds -gt 1.5){ STOP ("headingErr {0:N0} deg >1.5s" -f $hErr); break } } else { $hdgBadSince=$null }
      if ([math]::Abs($ct2) -gt 1.2) { STOP ("crossTrack {0:N2} m" -f $ct2); break }
      if ($st -eq 'ARRIVED') { Write-Host "[+] ARRIVED"; "ARRIVED" | Out-File $log -Append -Encoding utf8; $done=$true }
      if ($st -eq 'ERROR')   { STOP ("NAV ERROR " + $p[9]); break }
    }
    elseif ($l -match '^TEL,') {
      $lastTel = Get-Date
      $p = $l -split ','
      # TEL,lat,lon,alt,head,fix,sol,...   sol field index 6 = 'fixed'/...
      $solStr=$p[6]
      if ($solStr -ne 'fixed') { STOP ("lost RTK fix: sol=$solStr") ; break }
    }
  }
}
SendCmd "NAV_STOP"; Start-Sleep -Milliseconds 150; SendCmd "NAV_STOP"
Write-Host "[*] run ended, NAV_STOP sent. log: $log"
try { $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure,"bye",$ct.Token).Wait(1000) | Out-Null } catch {}
