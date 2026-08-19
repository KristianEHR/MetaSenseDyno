$ErrorActionPreference = 'Stop'
$py = 'C:\Users\krist\.platformio\penv\Scripts\python.exe'
& $py -c "from pathlib import Path
p=Path(r'.pio/build/esp32s3-USB-sniff2/firmware.elf')
b=p.read_bytes()
tags=[b'[CAN-ID-REPORT]', b'[CAN-ID]', b'[CAN-RX-RAW]', b'[CAN-FRAME-RX]', b'[CAN-SNIFF-DIAG]']
for t in tags:
 print(f'{t.decode()}: {t in b}')"
