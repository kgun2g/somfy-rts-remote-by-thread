"""Read saved NVS only. No serial/device access. Output audit data as JSON."""
import sys,json,struct,re,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'sim/tools'))
from nvs_parse import parse
import pmdump_decode as pm
import lsfloor_decode as l

def ns(path,name):
    _,items=parse(path.read_bytes()); d={}
    for x in items:
        if x['ns_name']==name and (isinstance(x['value'],(bytes,bytearray)) or x['key'] not in d):d[x['key']]=x['value']
    return d

def snap(path):
    b=ns(path,'batlog'); p=ns(path,'pmdump'); out={'file':path.name,'batlog':{k:v for k,v in b.items() if isinstance(v,int)}}
    if b.get('dis_el',0):
        out['bat_rate']=b.get('ls_cnt',0)/b['dis_el'];out['bat_sleep_pct']=b.get('ls_us',0)/b['dis_el']/1e4
        if isinstance(b.get('wi_cnt'),bytes):out['task_rates']=[x/b['dis_el'] for x in struct.unpack('<'+'I'*(len(b['wi_cnt'])//4),b['wi_cnt'])]
    if p.get('locks0') and p.get('locks'):
        def text(v):return v.split(b'\x00')[0].decode('utf8') if isinstance(v,bytes) else v
        a,z=pm._snap(text(p['locks0'])),pm._snap(text(p['locks'])); wall=(z['boot_us']-a['boot_us'])/1e6
        out['pm']={'wall_s':wall,'boot0_s':a['boot_us']/1e6,'boot1_s':z['boot_us']/1e6,'count':z['ls_cnt']-a['ls_cnt'],'reject':z['ls_rej']-a['ls_rej'],'sleep_pct':(z['sleep_us']-a['sleep_us'])/(wall*1e4) if wall>0 else None,'rate':(z['ls_cnt']-a['ls_cnt'])/wall if wall>0 else None}
    out['pmdump']={k:v for k,v in p.items() if isinstance(v,int)}
    return out
files=sorted((ROOT/'logs').glob('nvs_COM[78]_1018_*.bin'))
results=[snap(p) for p in files]
meta,samples=l.load(str(ROOT/'logs/nvs_COM7_1018_0751_wakenight.bin'))
cal=l.analyse_wake(samples,meta,int(meta.get('nph',5)))
q=cal['per_wake_uc'];rate_old=32.26;rate_new=4.18
budget={'nominal_mah':700,'usable_mah_assumption':560,'target_days':15,'target_average_ma':560/360,'wake_charge_model_uc':q,'wake_saving_model_ma':(rate_old-rate_new)*q/1000,'wake_saving_model_mah_day':(rate_old-rate_new)*q/1000*24,'remaining_all_wake_model_ma':rate_new*q/1000,'cases':[]}
for idle in [1.0,1.2,1.3,1.5,2.0,3.0,4.56]:
    for n in [20,50]:
        active=n*10*50/3600;daily=24*idle+active
        budget['cases'].append({'idle_ma':idle,'uses_day':n,'active_extra_ma_assumption':50,'active_mah_day':active,'total_mah_day':daily,'days':560/daily})
# One deterministic HP/LP interleaving: no cross-core lock protects press_latch.
latch=1;hp_lat=latch;hp_load_for_clear=latch;latch|=2;latch=hp_load_for_clear & ~hp_lat
assert latch==0 # LP bit B lost by HP stale store
# Deterministic counterexample for HIGH-level IRQ: no physical edge after t=0.
# Re-enabling a level IRQ while HIGH immediately asserts it again.
events=[]
for t in range(0,1001,31): events.append({'ms':t,'physical_edges_after_initial':0})
assert len(events)>30
out={'level_irq_counterexample':{'one_second_isr_count':len(events),'gpio_wakeup_disabled_at_ms':2000,'irq_type_after_disable':'HIGH_LEVEL unchanged','note':'model of API semantics, not an on-device timing measurement'},'snapshots':results,'wake_calibration':{'meta':meta,'analysis':cal},'budget':budget,'counterexample':{'hp_observed_mask':hp_lat,'lp_added_mask':2,'final_latch':latch}}
print(json.dumps(out,ensure_ascii=False,indent=2))
