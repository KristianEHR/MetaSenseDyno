import time
import pathlib
import serial
s = serial.Serial('COM6', 115200, timeout=0.2)
s.dtr = False
s.rts = False
end = time.time() + 120
chunks = []
while time.time() < end:
    d = s.read(4096)
    if d:
        chunks.append(d.decode('utf-8', 'ignore'))
text = ''.join(chunks)
out = pathlib.Path('backups/1da_capture_120s.log')
out.write_text(text, encoding='utf-8')
print(f'WROTE {len(text)} chars to {out}')
