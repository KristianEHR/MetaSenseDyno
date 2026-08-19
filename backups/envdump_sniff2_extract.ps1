$ErrorActionPreference = 'Stop'
$pio = 'C:\Users\krist\.platformio\penv\Scripts\platformio.exe'
$out = 'backups\envdump_sniff2_full.txt'
& $pio run -e esp32s3-USB-sniff2 -t envdump | Out-File -FilePath $out -Encoding utf8
Select-String -Path $out -Pattern 'CPPDEFINES|METASENSE_CAN_|METASENSE_LEAF_CAN_' | ForEach-Object { $_.Line } | Out-File -FilePath 'backups\envdump_sniff2_extract.txt' -Encoding utf8
Write-Host 'WROTE:' $out
Write-Host 'WROTE: backups\envdump_sniff2_extract.txt'
