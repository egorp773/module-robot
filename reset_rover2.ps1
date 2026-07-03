Add-Type -Name 'SerialHelper' -Namespace 'Helper' -MemberDefinition @'
[DllImport("kernel32.dll", SetLastError = true)]
public static extern int CreateFile(string lpFileName, uint dwDesiredAccess, uint dwShareMode, IntPtr lpSecurityAttributes, uint dwCreationDisposition, uint dwFlagsAndAttributes, IntPtr hTemplateFile);
[DllImport("kernel32.dll")]
public static extern bool EscapeCommFunction(IntPtr hFile, uint dwFunc);
[DllImport("kernel32.dll")]
public static extern bool CloseHandle(IntPtr hObject);
'@
# Open COM port via handle
Add-Type -Name 'SerialReset' -Namespace 'Helper' -MemberDefinition @'
[DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Auto)]
public static extern IntPtr CreateFile(string lpFileName, uint dwDesiredAccess, uint dwShareMode, IntPtr lpSecurityAttributes, uint dwCreationDisposition, uint dwFlagsAndAttributes, IntPtr hTemplateFile);
'@
$h = [Helper.SerialReset]::CreateFile("\.\COM7", 0xC0000000, 0, [IntPtr]::Zero, 3, 0, [IntPtr]::Zero)
if ($h -eq [IntPtr]::Zero -or $h -eq -1) { Write-Host "[!] cannot open COM7"; exit 1 }
# DTR low (reset)
Add-Type -Name 'S2' -Namespace 'H' -MemberDefinition '[DllImport("kernel32.dll")] public static extern bool EscapeCommFunction(IntPtr hFile, uint dwFunc);'
[H.S2]::EscapeCommFunction($h, 5) | Out-Null  # 5 = CLRDTR
Write-Host "[+] DTR low (reset)"
Start-Sleep -Milliseconds 200
[H.S2]::EscapeCommFunction($h, 4) | Out-Null  # 4 = SETDTR
Write-Host "[+] DTR high"
[Helper.SerialReset].GetMethods() | Where-Object { $_.Name -eq 'CloseHandle' } | ForEach-Object { $_.Invoke($null, @($h)) | Out-Null }
