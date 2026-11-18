import sys, math
J=[]; P=[]; ev=[]
for l in open(sys.argv[1]):
    f=l.split()
    t=float(f[0])
    if f[1]=='J': J.append((t,float(f[2])))
    elif f[1]=='P': P.append((t,float(f[2])))
    elif f[1] in('CH','LOG'): ev.append((t,' '.join(f[1:])))
def pos(t): 
    b=[p for (tt,p) in J if tt<=t]; return b[-1] if b else float('nan')
# segments of nonzero P
segs=[]; cur=None
for t,v in P:
    if v!=0 and cur is None: cur=[t,t,v]
    elif v!=0: cur[1]=t; 
    elif v==0 and cur is not None: segs.append((cur[0],t,cur[2])); cur=None
print('start      dur   cmd    p0      p_stop  p_final  travel  cruise  coast')
for a,b,v in segs:
    p0=pos(a); ps=pos(b); pf=pos(b+2.5)
    pts=[(t,p) for t,p in J if a+min(1.0,(b-a)*0.5)<=t<=b]
    cr=float('nan')
    if len(pts)>=3:
        n=len(pts); mt=sum(t for t,_ in pts)/n; mp=sum(p for _,p in pts)/n
        cr=sum((t-mt)*(p-mp) for t,p in pts)/sum((t-mt)**2 for t,_ in pts)
    print('%.2f %5.2f %6.2f %7.3f %7.3f %7.3f %7.3f %7.3f %6.3f'%(a%1000,b-a,v,p0,ps,pf,pf-p0,cr,pf-ps))
for t,e in ev:
    if 'retry' in e or 'pulse' in e or 'band' in e or 'gave up' in e or 'settled' in e or 'reached' in e: print('%.2f'%(t%1000),e[:150])
