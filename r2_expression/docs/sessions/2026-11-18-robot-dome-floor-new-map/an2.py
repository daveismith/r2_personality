import sys, math
J=[];P=[];L=[]
for l in open(sys.argv[1]):
    f=l.split(); t=float(f[0])
    if f[1]=='J': J.append((t,float(f[2])))
    elif f[1]=='P': P.append((t,float(f[2])))
    elif f[1]=='LOG': L.append((t,' '.join(f[2:])))
def pos(t):
    b=[p for (tt,p) in J if tt<=t]; return b[-1]
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi
cmds=[l.split(None,1) for l in open(sys.argv[2])]
for i,(t,c) in enumerate(cmds):
    t=float(t); te=float(cmds[i+1][0]) if i+1<len(cmds) else t+12
    ps=[(a,v) for a,v in P if t<=a<te]; nz=[(a,v) for a,v in ps if v!=0]
    if not nz: print(c.strip(),'NO MOTION'); continue
    segs=0; prev=0
    for a,v in ps:
        if v!=0 and prev==0: segs+=1
        prev=v
    end=nz[-1][0]; pf=pos(te-0.5); p0=pos(t)
    a=c.split(); tgt=None
    if a[0]=='look': tgt=-float(a[-1])
    if a[0]=='glance': tgt=p0
    err=math.degrees(wrap(pf-tgt)) if tgt is not None else float('nan')
    mn=min(abs(v) for _,v in nz); mx=max(abs(v) for _,v in nz)
    print('%-34s from %6.1f to %6.1f deg  err %+5.1f deg  motion %.1fs  bursts %d  |cmd| %.2f..%.2f'%(c.strip()[:34],math.degrees(wrap(p0)),math.degrees(wrap(pf)),err,end-nz[0][0],segs,mn,mx))
