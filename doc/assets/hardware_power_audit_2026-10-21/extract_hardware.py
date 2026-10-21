"""Read KiCad originals/exported netlist. Never save or modify original designs."""
import json,re,xml.etree.ElementTree as ET,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def sexpr(s):
    tokens=re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+',s)
    stack=[];out=None
    for t in tokens:
        if t=='(':
            n=[]
            if stack:stack[-1].append(n)
            stack.append(n)
        elif t==')':out=stack.pop()
        else:
            if t.startswith('"'):t=t[1:-1].replace('\\"','"').replace('\\\\','\\')
            stack[-1].append(t)
    return out
def children(n,k):return [x for x in n if isinstance(x,list) and x and x[0]==k]
def child(n,k):return next(iter(children(n,k)),[])
tree=ET.parse(HERE/'schematic.net.xml').getroot()
comps={c.attrib['ref']:{'value':c.findtext('value'),'footprint':c.findtext('footprint'),'datasheet':c.findtext('datasheet'),'pins':{}} for c in tree.findall('components/comp')}
nets={}
for net in tree.findall('nets/net'):
    name=net.attrib['name'];nodes=[dict(x.attrib) for x in net.findall('node')];nets[name]=nodes
    for x in nodes:comps[x['ref']]['pins'][x['pin']]={'net':name,'name':x.get('pinfunction'),'type':x.get('pintype')}
pcb=sexpr((ROOT/'kicad/somfy_blinds_v6.kicad_pcb').read_text(encoding='utf8'))
footprints={}
for fp in children(pcb,'footprint'):
    props={p[1]:p[2] for p in children(fp,'property')}
    pads=[]
    for p in children(fp,'pad'):
        nn=child(p,'net');pads.append({'number':p[1],'type':p[2],'net':nn[-1] if nn else '', 'pinfunction':child(p,'pinfunction')[1:]})
    footprints[props.get('Reference','?')]={'value':props.get('Value'),'footprint':fp[1],'at':child(fp,'at')[1:],'layer':child(fp,'layer')[1:],'pads':pads}
out={'components':comps,'nets':nets,'footprints':footprints,'sha256':{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['kicad/somfy_blinds_v6.kicad_sch','kicad/somfy_blinds_v6.kicad_pcb']}}
(HERE/'hardware_nets.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf8')
for ref,c in comps.items():
    print(ref,c['value'],', '.join(f"{p}={v['net']}({v['name']})" for p,v in c['pins'].items()))
