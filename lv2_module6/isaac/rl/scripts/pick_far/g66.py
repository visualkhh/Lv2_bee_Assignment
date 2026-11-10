import numpy as np
from coll import run
for gap in (60.0,66.0):
  for yaw in (0,45):
    lim=0
    for e in np.arange(2,31,1):
        ok=True
        for s in (1,-1):
            r=run(0.26,-0.074,"box",yaw,dy=s*e/1000,gap=gap)
            m=min(r["min_puck_entry"],r["min_puck_sweep"],r["min_puck_roll"])
            if not r["ok"] or m<0.0005: ok=False
        if not ok: break
        lim=e
    print("gap %.0f box yaw %d: lateral tolerance +-%d mm"%(gap,yaw,lim),flush=True)
