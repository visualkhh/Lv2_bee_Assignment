"""미션 방문 순서 오프라인 계산 — plan.yaml survey_order · build_location_order를 정할 때 (경기장 고정이라 실행 중엔 안 돌림)

  도면 벽(plan.yaml arena_walls · documents/mission/dice-map.png) 위 5 cm 격자 A*, 벽·박스를 로봇 반경 0.16 + 여유 0.05로 넓힘.
  다가설 자리는 벽에서 0.22 m(wall_clearance) 밖 면만. 출발 → 박스 6(집기) → 장소 3(놓기) → 출발 총 길이로 순서 비교
  (VERIFY는 놓을 때 같이 본다고 가정 · 적재함 12칸에 12개 다 실음). 실행: python3 route_order.py (numpy만)
"""
import heapq, itertools, math
import numpy as np
W=[0.000,0.000,4.404,0.000,0.000,6.304,4.404,6.304,0.000,0.000,0.000,6.304,4.404,0.000,4.404,6.304,1.838,4.985,2.571,4.985,0.367,4.251,1.101,4.251,3.304,4.251,4.037,4.251,3.304,3.517,4.037,3.517,1.101,2.049,1.838,2.049,2.571,2.049,3.304,2.049,1.101,1.315,1.838,1.315,2.571,1.315,4.037,1.315,1.101,4.985,1.101,5.719,2.571,4.985,2.571,5.719,3.304,4.985,3.304,5.719,1.101,3.517,1.101,4.251,3.304,2.049,3.304,3.517,1.101,2.049,1.101,2.783,1.838,1.315,1.838,2.049,1.101,0.650,1.101,1.315]
SEG=[W[i:i+4] for i in range(0,len(W),4)]
BOX=[(1.468,0.950),(3.670,3.152),(2.936,5.354),(0.734,5.354),(2.202,5.354),(3.670,3.886)]
LOC=[(3.670,0.950),(2.202,3.495),(3.670,5.354)]  # L1 L2(받침대 중심) L3
START=(0.734,0.950); HALF=0.1025; STAND=0.25+HALF; R=0.16+0.05
res=0.05; nx,ny=int(4.404/res)+1,int(6.304/res)+1
xs=np.arange(nx)*res; ys=np.arange(ny)*res; X,Y=np.meshgrid(xs,ys,indexing='ij')
def segd(px,py,s):
    ax,ay,bx,by=s; dx,dy=bx-ax,by-ay; L=dx*dx+dy*dy
    t=np.clip(((px-ax)*dx+(py-ay)*dy)/L,0,1); return np.hypot(px-ax-t*dx,py-ay-t*dy)
D=np.min([segd(X,Y,s) for s in SEG],axis=0)
for cx,cy in BOX+[(3.670,0.950),(3.670,5.354)]: D=np.minimum(D,np.hypot(np.maximum(abs(X-cx)-HALF,0),np.maximum(abs(Y-cy)-HALF,0)))
D=np.minimum(D,np.hypot(np.maximum(abs(X-2.202)-0.41,0),np.maximum(abs(Y-3.495)-0.41,0)))
free=D>R
def wall_dist(x,y): return min(float(segd(np.array(x),np.array(y),s)) for s in SEG)
def sides(cx,cy,h=HALF):
    c=[(cx-h-STAND+HALF,cy),(cx+h+STAND-HALF,cy),(cx,cy-h-STAND+HALF),(cx,cy+h+STAND-HALF)]
    return [p for p in c if wall_dist(*p)>=0.22 and free[int(round(p[0]/res)),int(round(p[1]/res))]]
def astar(a,b):
    s=(int(round(a[0]/res)),int(round(a[1]/res))); g=(int(round(b[0]/res)),int(round(b[1]/res)))
    if not free[s] or not free[g]: return math.inf
    pq=[(0,0,s)]; best={s:0}
    nb=[(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,1.414),(1,-1,1.414),(-1,1,1.414),(-1,-1,1.414)]
    while pq:
        f,c,u=heapq.heappop(pq)
        if u==g: return c*res
        if c>best.get(u,1e9): continue
        for dx,dy,w in nb:
            v=(u[0]+dx,u[1]+dy)
            if 0<=v[0]<nx and 0<=v[1]<ny and free[v]:
                nc=c+w
                if nc<best.get(v,1e9): best[v]=nc; heapq.heappush(pq,(nc+math.hypot(v[0]-g[0],v[1]-g[1]),nc,v))
    return math.inf
P=[START]+[min(sides(*b),key=lambda p:0) for b in BOX]  # 각 박스: 첫 번째 갈 수 있는 면
# 박스마다 갈 수 있는 면 중 시작점에서 가장 가까운 면
P=[START]+[min(sides(*b),key=lambda p:astar(START,p)) for b in BOX]
n=len(P); Dm=[[0 if i==j else astar(P[i],P[j]) for j in range(n)] for i in range(n)]
def tour(order): # 0=start → boxes(1..6) → 마지막 박스에서 L1(BUILD 시작)까지
    t=Dm[0][order[0]]+sum(Dm[order[k]][order[k+1]] for k in range(len(order)-1)); return t
for name,o in [("지금 1→2→3→4→5→6",[1,2,3,4,5,6]),("사용자 그림 1→4→5→3→6→2",[1,4,5,3,6,2])]: print(f"{name}: {tour(o):.2f} m")
best=min(itertools.permutations(range(1,7)),key=tour); print(f"최단 {'→'.join(map(str,best))}: {tour(best):.2f} m")
for i,b in enumerate(BOX,1): print(f"박스 {i} 다가설 면: {[(round(x,3),round(y,3)) for x,y in sides(*b)]}")

# 미션 전체: 출발 → 박스 6개(집기) → 장소 3개(놓기) → 출발 복귀 (VERIFY는 놓을 때 같이 본다고 가정)
L=[min(sides(*l,h=(0.1025 if i!=1 else 0.41)),key=lambda p:astar(START,p)) for i,l in enumerate(LOC)]
Q=P+L; m=len(Q); E=[[0 if i==j else astar(Q[i],Q[j]) for j in range(m)] for i in range(m)]
def full(bo,lo): # 인덱스: 박스 1..6, 장소 7..9
    seq=[0]+list(bo)+[6+k for k in lo]+[0]; return sum(E[seq[k]][seq[k+1]] for k in range(len(seq)-1))
cur=full([1,2,3,4,5,6],[1,2,3]); usr=full([1,4,5,3,6,2],[1,2,3])
bb,bl=min(((b,l) for b in itertools.permutations(range(1,7)) for l in itertools.permutations([1,2,3])),key=lambda x:full(*x))
print(f"\n[전체] 지금(박스 1→6, 장소 L1→L2→L3): {cur:.2f} m")
print(f"[전체] 사용자 그림(박스 1·4·5·3·6·2, 장소 L1·L2·L3): {usr:.2f} m")
print(f"[전체] 최단(박스 {'·'.join(map(str,bb))}, 장소 {'·'.join('L%d'%k for k in bl)}): {full(bb,bl):.2f} m")
print(f"0.15 m/s 평균이면 주행 시간: 지금 {cur/0.15:.0f} s · 사용자 {usr/0.15:.0f} s · 최단 {full(bb,bl)/0.15:.0f} s")
