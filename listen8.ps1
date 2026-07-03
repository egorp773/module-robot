$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.Open(); $port.ReadTimeout=500
$end=(Get-Date).AddSeconds(8); $got=$false
while((Get-Date) -lt $end){ try{ $l=$port.ReadLine(); if($l){Write-Output $l;$got=$true} }catch{} }
if(-not $got){Write-Output "<<NO DATA>>"}
$port.Close()
