import sys, math
J=[];C=[];R=[]
for l in open(sys.argv[1]):
    f=l.split(); t=float(f[0])
    if f[1]=='J': J.append((t,float(f[2])))
    elif f[1]=='C': C.append((t,float(f[2])))
for l in open(sys.argv[2]):
    f=l.split()
    if f[1]=='RUN': R.append((float(f[0]),float(f[2])))
# unwrap
U=[]; off=0; prev=None
for t,p in J:
    if prev is not None:
        d=p-prev
        if d>math.pi: off-=2*math.pi
        elif d<-math.pi: off+=2*math.pi
    prev=p; U.append((t,p+off))
print('| commanded | controller cmd | achieved (last 1 s) | t90 | dead time | coast rad | coast deg |')
for i,(t0,v) in enumerate(R):
    t1=R[i+1][0] if i+1<len(R) else t0+7.5
    cs=[(t,c) for t,c in C if t0<=t<t1]; nz=[(t,c) for t,c in cs if c!=0]
    if not nz: print(v,'no command'); continue
    tstart=nz[0][0]; tzero=min(t for t,c in cs if c==0 and t>nz[-1][0]); cc=nz[len(nz)//2][1]
    seg=[(t,p) for t,p in U if tzero-1.0<=t<=tzero]
    n=len(seg); mt=sum(t for t,_ in seg)/n; mp=sum(p for _,p in seg)/n
    ach=sum((t-mt)*(p-mp) for t,p in seg)/sum((t-mt)**2 for t,_ in seg)
    run=[(t,p) for t,p in U if tstart-0.2<=t<=t1]
    p0=[p for t,p in U if t<=tstart][-1]
    moved=[t for t,p in run if abs(p-p0)>0.02]; tm=moved[0] if moved else float('nan')
    t90=float('nan')
    for (ta,pa),(tb,pb) in zip(run,run[1:]):
        if tb>tm and abs((pb-pa)/(tb-ta))>=0.9*abs(ach): t90=tb-tm; break
    pz=[p for t,p in U if t<=tzero][-1]; pr=[p for t,p in U if t<=t1-0.3][-1]
    print('| %+.2f | %+.3f | %+.2f | %.2f s | %.2f s | %+.3f | %.1f |'%(v,cc,ach,t90,tm-tstart,pr-pz,math.degrees(abs(pr-pz))))
