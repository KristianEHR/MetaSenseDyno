$ErrorActionPreference = 'Stop'
$py = 'C:\Users\krist\.platformio\penv\Scripts\python.exe'
& $py -c "import time, serial; s=serial.Serial('COM6',115200,timeout=0.2); s.dtr=False; s.rts=False; t=time.time()+8; out=[]
while time.time()<t:
 d=s.read(1024)
 if d: out.append(d.decode('utf-8','ignore'))
print(''.join(out))"
