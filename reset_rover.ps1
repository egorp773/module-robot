# Reset ESP32 через DTR toggle (CH340 обычно поддерживает)
$port = New-Object System.IO.Ports.SerialPort COM7,1200,None,8,One
try { $port.Open(); Start-Sleep -Milliseconds 200; $port.Close(); Write-Host "[+] toggled 1200 baud -> reset" } catch { Write-Host "[!] err: $_" }
