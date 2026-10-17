"""fusion_extract.py 의 batch 결과(raw/bodies_<tag>.json, raw/<link>_asm_mm_<tag>.bin)를
build.py 가 읽는 raw/bodies.json + raw/<link>_asm_mm.stl 로 합친다."""
import glob, json, os, re, struct

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, 'raw')

recs, transforms, tire = [], {}, {}
for p in sorted(glob.glob(os.path.join(RAW, 'bodies_*.json'))):
    d = json.load(open(p))
    recs += d['recs']; transforms.update(d['transforms']); tire.update(d['tire_bb_mm'])
json.dump(dict(recs=recs, transforms=transforms, tire_bb_mm=tire), open(os.path.join(RAW, 'bodies.json'), 'w'))

parts = {}
for p in glob.glob(os.path.join(RAW, '*_asm_mm_*.bin')):
    link = re.match(r'(.+)_asm_mm_[^_]+\.bin$', os.path.basename(p)).group(1)
    parts.setdefault(link, []).append(p)
for link, ps in parts.items():
    data = b''.join(open(p, 'rb').read() for p in sorted(ps))
    n = len(data) // 36
    with open(os.path.join(RAW, link + '_asm_mm.stl'), 'wb') as f:
        f.write(b'asm-mm'.ljust(80, b' ')); f.write(struct.pack('<I', n))
        for k in range(n):
            f.write(struct.pack('<3f', 0, 0, 0)); f.write(data[36 * k:36 * k + 36]); f.write(b'\0\0')
    print(link, n, 'tris from', len(ps), 'batches')
print('bodies', len(recs), 'transforms', sorted(transforms), 'tires', sorted(tire))
