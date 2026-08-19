import re
import collections
import pathlib

text = pathlib.Path('backups/1da_capture_30s.log').read_text(encoding='utf-8', errors='ignore')
pat = re.compile(r'\[1DA-CRC-BAD\].*?clk=(\d).*?res_rx=0x([0-9A-Fa-f]{2}).*?data=([0-9A-Fa-f]{2}) ')
rows = pat.findall(text)
print('BAD lines:', len(rows))
by_b0_clk = collections.defaultdict(lambda: collections.defaultdict(collections.Counter))
for clk_s, res_s, b0_s in rows:
    clk = int(clk_s)
    res = int(res_s, 16)
    b0 = int(b0_s, 16)
    by_b0_clk[b0][clk][res] += 1

for b0 in sorted(by_b0_clk):
    print(f'b0=0x{b0:02X}')
    for clk in range(4):
        top = by_b0_clk[b0][clk].most_common(5)
        print(f'  clk{clk}: {top}')
