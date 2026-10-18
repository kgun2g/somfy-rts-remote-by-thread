# Offline report calculation; does not connect to any device.
import os,sys,json,math,re,io,contextlib
os.environ['PYTHONIOENCODING']='utf-8'
os.environ['PYTHONDONTWRITEBYTECODE']='1'
sys.path.insert(0,'sim/tools')
import lsfloor_decode as l, pmdump_decode as pm
from nvs_parse import parse
files=['nvs_COM7_1015_0800_lsfloor.bin','nvs_COM7_1016_0722_topnight.bin','nvs_COM7_1016_1621_lpnight.bin']
results=[]
for fn in files:
 meta,samples=l.load('logs/'+fn); rates=l.lp_rates(meta,samples); a=l.analyse(samples,int(meta.get('nph',4)),int(meta.get('fac',0)),rates)
 sx=[]; sl=[]
 for p in range(4):
  mine=[s for s in samples if s['phase']==p and s['t_s']>0]; prev=[s for s in samples if s['t_s']<mine[0]['t_s']]; pts=([(prev[-1]['t_s'],prev[-1]['mv'])] if prev else [])+[(s['t_s'],s['mv']) for s in mine]; tail=pts[len(pts)//3:]; xs=[t/3600 for t,v in tail]; xm=sum(xs)/len(xs); sx.append(sum((x-xm)**2 for x in xs)); sl.append(l.lsq_slope(tail))
 se=a['sigma']*math.sqrt(sum(1/sx[p] for p in [0,2])/4+sum(1/sx[p] for p in [1,3])/4)
 corrected=max(0,(a['dslope']+2*se)/l.K_LO)
 results.append(dict(file=fn,fac=meta.get('fac',0),dry=meta.get('dry',0),cpu_flags=sorted(set(s['flags'] for s in samples if s['flags']!=65535)),phase_rates=rates,S=a.get('S'),sigma=a.get('sigma'),slopes=sl,reported_se=a.get('dslope_se'),actual_timestamp_se=se,reported_upper=a.get('dI_95'),timestamp_upper=corrected,delta_slope=a.get('dslope'),drift=a.get('Ms'),verdict=a.get('verdict')))
print(json.dumps(results,ensure_ascii=False))
