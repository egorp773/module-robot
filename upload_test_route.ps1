# upload_test_route.ps1 - шлёт 3 точки по прямой на ровер через WebSocket
# Использование: powershell -NoProfile -ExecutionPolicy Bypass -File C:\robot\module\upload_test_route.ps1

Add-Type -AssemblyName System.Net.WebSockets
Add-Type -AssemblyName System.Threading.Tasks

$uri = "ws://192.168.31.222:81/ws"
$ws = New-Object System.Net.WebSockets.ClientWebSocket

Write-Host "[*] Connecting to $uri ..."
$ct = New-Object System.Threading.CancellationTokenSource
$ws.ConnectAsync([Uri]$uri, $ct.Token).Wait()

if ($ws.State -ne 'Open') {
    Write-Host "[!] Connect failed: $($ws.State)"
    exit 1
}
Write-Host "[+] Connected"

# helper: send text and wait for ack
function Send-Cmd([string]$cmd, [string]$expectAck) {
    $buf = [System.Text.Encoding]::UTF8.GetBytes($cmd)
    $seg = New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
    $ws.SendAsync($seg, [System.Net.WebSockets.WebSocketMessageType]::Text, $true, $ct.Token).Wait()
    Write-Host "[>] $cmd"

    # read until we see expectAck or 2s timeout
    $recv = New-Object byte[] 1024
    $deadline = (Get-Date).AddSeconds(2)
    while ((Get-Date) -lt $deadline) {
        $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
        try {
            $ws.ReceiveAsync($segR, $ct.Token).Wait(500)
        } catch { continue }
        $msg = [System.Text.Encoding]::UTF8.GetString($recv, 0, $segR.Count).Trim()
        if ($msg -match 'STATE,CONNECTED') { continue }  # skip connect banner
        if ($msg) { Write-Host "[<] $msg" }
        if ($expectAck -and $msg.StartsWith($expectAck)) {
            return $true
        }
    }
    return $false
}

# Step 1: get current GPS as origin (small trick — read one line of telemetry)
Write-Host "[*] Sending PING to read origin (lat/lon)..."
$buf = [System.Text.Encoding]::UTF8.GetBytes("PING")
$seg = New-Object System.ArraySegment[byte] -ArgumentList @(,$buf)
$ws.SendAsync($seg, [System.Net.WebSockets.WebSocketMessageType]::Text, $true, $ct.Token).Wait()

# read until we see TEL,... (which has lat,lon)
$recv = New-Object byte[] 2048
$deadline = (Get-Date).AddSeconds(3)
$lat = $null; $lon = $null
while ((Get-Date) -lt $deadline -and (-not $lat)) {
    $segR = New-Object System.ArraySegment[byte] -ArgumentList @(,$recv)
    try { $ws.ReceiveAsync($segR, $ct.Token).Wait(500) } catch { continue }
    $msg = [System.Text.Encoding]::UTF8.GetString($recv, 0, $segR.Count).Trim()
    if ($msg -match '^TEL,') {
        # TEL,lat,lon,alt,head,fixType,sol,diff,sv,hAcc,vAcc,speed,pDop,pvtAge,...
        $parts = $msg -split ','
        $lat = [double]$parts[1]
        $lon = [double]$parts[2]
    }
}
if (-not $lat) {
    Write-Host "[!] Could not read GPS in 3s — start GNSS first"
    $ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure, "bye", $ct.Token).Wait()
    exit 1
}
Write-Host "[+] Origin: lat=$lat, lon=$lon"

# Step 2: send route — 3 points on a 1m line going East
# Local meters: (0,0) = origin, (1,0) = 1m east, (2,0) = 2m east
$count = 3
Send-Cmd "ROUTE_BEGIN,$count,$lat,$lon" "OK,ROUTE_BEGIN" | Out-Null
Start-Sleep -Milliseconds 100
Send-Cmd "ROUTE_WP,0,0.000,0.000" "OK,ROUTE_WP,0" | Out-Null
Start-Sleep -Milliseconds 100
Send-Cmd "ROUTE_WP,1,1.000,0.000" "OK,ROUTE_WP,1" | Out-Null
Start-Sleep -Milliseconds 100
Send-Cmd "ROUTE_WP,2,2.000,0.000" "OK,ROUTE_WP,2" | Out-Null
Start-Sleep -Milliseconds 100
Send-Cmd "ROUTE_END" "OK,ROUTE"

Write-Host ""
Write-Host "[+] Route sent. Now press NAV_START in the app."
Write-Host "    Robot should drive ~2m East and stop."

# don't close — let user see in app
Read-Host "Press Enter to close"
$ws.CloseAsync([System.Net.WebSockets.WebSocketCloseStatus]::NormalClosure, "bye", $ct.Token).Wait()
