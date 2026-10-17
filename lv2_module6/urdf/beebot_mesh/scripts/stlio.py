import numpy as np, struct
def read_stl(p):
    with open(p,'rb') as f:
        f.read(80); n=struct.unpack('<I',f.read(4))[0]
        dt=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
        a=np.frombuffer(f.read(n*50),dtype=dt,count=n)
    return a['v'].astype(np.float64)
def write_stl(p,tri):
    tri=np.asarray(tri,dtype=np.float64)
    n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); l=np.linalg.norm(n,axis=1,keepdims=True); l[l==0]=1; n/=l
    dt=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
    a=np.zeros(len(tri),dtype=dt); a['n']=n; a['v']=tri
    with open(p,'wb') as f:
        f.write(b'bee'.ljust(80,b' ')); f.write(struct.pack('<I',len(tri))); f.write(a.tobytes())
def cluster(tri,cell):
    v=tri.reshape(-1,3); key=np.floor(v/cell).astype(np.int64)
    uk,inv=np.unique(key,axis=0,return_inverse=True); inv=inv.ravel()
    cnt=np.bincount(inv); rep=np.zeros((len(uk),3))
    for i in range(3): rep[:,i]=np.bincount(inv,weights=v[:,i])/cnt
    f=inv.reshape(-1,3)
    ok=(f[:,0]!=f[:,1])&(f[:,1]!=f[:,2])&(f[:,0]!=f[:,2]); f=f[ok]
    s=np.sort(f,axis=1); _,ui=np.unique(s,axis=0,return_index=True); f=f[np.sort(ui)]
    return rep[f]
