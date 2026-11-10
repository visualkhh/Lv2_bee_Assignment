import math, numpy as np
from tilt_reach import *
def standoff_ok(x,y,a_deg,d):
    hd=math.atan2(y+0.00023,x-0.047985); a=math.radians(a_deg)
    r=np.array([math.cos(hd),math.sin(hd),0]); axis=math.cos(a)*r+np.array([0,0,-math.sin(a)])
    P=np.array([x,y,BOX_Z+PAD_H]); pos=P-PAD*axis-d*axis
    seeds=[np.array([hd,-0.2,0.7,1.0,0.0]),np.array([hd,0.3,0.3,0.6,0.0]),np.array([hd,0.5,1.0,-1.0,0.0])]
    q,ep,ea=ik.solve(pos,rot_at(hd,a),seeds)
    return ep<=OK_POS and ea<=OK_ANG
if __name__ == "__main__":
  for y,xm in ((-0.10,.268),(-0.074,.278),(0.0,.288),(0.091,.270)):
      row=[]
      for a in (20,30,45):
          row.append("a%d:%s"%(a,",".join("%d"%(1000*max([x for x in np.arange(0.2,xm+0.001,0.0025) if standoff_ok(x,y,a,d)] or [0])) for d in (0.03,0.05,0.08))))
      print("y %+.3f max x with standoff 30/50/80 mm back along tool | "%y+" | ".join(row))
