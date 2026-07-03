# serial_imu_90.ps1 - manual IMU rotation test over COM7, no motor/nav start
$log = "C:\robot\module\imu_90_serial.log"
"=== imu90 serial $(Get-Date -Format o) ===" | Out-File $log -Encoding utf8

$port = New-Object System.IO.Ports.SerialPort COM7,115200,None,8,One
$port.NewLine = "`n"
$port.ReadTimeout = 150
$port.WriteTimeout = 1000
$port.DtrEnable = $false
$port.RtsEnable = $false
$port.Open()

function Drain([int]$ms, [string]$pattern = "") {
  $end = (Get-Date).AddMilliseconds($ms)
  while ((Get-Date) -lt $end) {
    try {
      $line = $port.ReadLine().Trim()
      if ($line) {
        if (-not $pattern -or $line -match $pattern) {
          Write-Host "    $line"
          $line | Out-File $log -Append -Encoding utf8
        }
      }
    } catch [System.TimeoutException] {}
  }
}
function Send([string]$cmd, [int]$ms, [string]$pattern = "") {
  Write-Host "[>] $cmd"
  $port.WriteLine($cmd)
  Drain $ms $pattern
}

Drain 800
Send "STOP" 500 "^\[STOP\]|^IMU_|^\[LOG\]"
Send "LOG,0" 500 "^\[STOP\]|^IMU_|^\[LOG\]"
Write-Host "[*] Keep rover still now."
Send "IMU_ZERO" 1000 "^IMU_|^\["
Write-Host "[*] Rotate rover by about 90 degrees CLOCKWISE now, then hold it still."
for ($i=10; $i -ge 1; --$i) {
  Write-Host ("    {0}s" -f $i)
  Start-Sleep -Seconds 1
}
Send "IMU_DIAG" 2500 "^IMU_|^\["
Send "IMU_DIAG" 1500 "^IMU_|^\["

$port.Close()
Write-Host "[*] log: $log"
