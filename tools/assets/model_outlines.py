import json,sys,collections
m=json.load(open(sys.argv[1]))
out={}
for name,mod in m.items():
    if name=='Valve.001': continue
    V=mod['v'];I=mod['i']
    key={};pts=[]
    def vid(i):
        x,y=round(V[i][0],5),round(V[i][1],5)
        if (x,y) not in key: key[(x,y)]=len(pts);pts.append((V[i][0],V[i][1]))
        return key[(x,y)]
    net=collections.Counter()
    for k in range(0,len(I),3):
        a,b,c=vid(I[k]),vid(I[k+1]),vid(I[k+2])
        if len({a,b,c})<3: continue
        for e in ((a,b),(b,c),(c,a)): net[e]+=1; net[(e[1],e[0])]-=1
    outg=collections.defaultdict(list)
    for (a,b),c in net.items():
        for _ in range(max(c,0)): outg[a].append(b)
    loops=[]
    while True:
        start=next((v for v in outg if outg[v]),None)
        if start is None: break
        L=[start];cur=outg[start].pop()
        while cur!=start:
            L.append(cur);cur=outg[cur].pop()
        loops.append(L)
    areas=[]
    for L in loops:
        s=0
        for i in range(len(L)):
            x0,y0=pts[L[i]];x1,y1=pts[L[(i+1)%len(L)]];s+=x0*y1-x1*y0
        areas.append(round(s/2,4))
    print(name,len(loops),[len(l) for l in loops],areas)
    out[name]=[[c for i in L for c in (round(pts[i][0],4),round(pts[i][1],4))] for L in loops]
json.dump(out,open(sys.argv[2],'w'),separators=(',',':'))
