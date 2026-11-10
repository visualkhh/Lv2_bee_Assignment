import math, time, numpy as np
from coll import run
t=time.time()
spots=[(0.26,-0.074),(0.27,0.0),(0.25,0.091),(0.24,-0.10)]
print("max |lateral error| (hand aims at (x,y), truth off by dy [mm]) with no collision; entry/sweep/roll; min=clearance at 0 error")
for x,y in spots:
    for kind,yaw in (("box",0),("box",45),("cyl",0)):
        base=run(x,y,kind,yaw)
        if not base["ok"]:
            print("(%.3f,%+.3f) %s yaw%d: IK fail %s"%(x,y,kind,yaw,base.get("fail"))); continue
        out=[]
        for axis in ("dy","dx"):
            lim=0
            for e in np.arange(2,31,2):
                okk=True
                for s in (+1,-1):
                    r=run(x,y,kind,yaw,**{axis:s*e/1000})
                    m=min(r["min_puck_entry"],r["min_puck_sweep"],r["min_puck_roll"])
                    if not r["ok"] or m<0.0005: okk=False
                if not okk: break
                lim=e
            out.append("%s +-%d mm"%(axis,lim))
        m=min(base["min_puck_entry"],base["min_puck_sweep"],base["min_puck_roll"])
        print("(%.3f,%+.3f) %s yaw%d: clearance %.1f mm | %s | %.0fs"%(x,y,kind,yaw,m*1000," , ".join(out),time.time()-t),flush=True)
