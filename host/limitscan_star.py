#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 模拟限位扫描：arm3d.html 场景内对 6 轴采样，检测与立柱/地面的最小间隙。不动真机。
import os, json, socket, base64, struct, time, subprocess, urllib.request, math, sys
URL="http://127.0.0.1:8790/arm3d.html"; PORT=9501
BASE=[0.309,2.583,2.226,-0.044,-0.118,-0.346]
JLIM=[[-178.8,178.8],[-178.8,178.8],[-169.0,189.1],[-178.8,178.8],[-178.8,178.8],[-178.8,178.8]]
AMP=[0.16,0.19,0.22,0.16,0.26,0.30]; LAG=0.30
OUTDIR=os.path.expanduser("/opt/openhwlab/robot-viz/arm_limit_shots")
os.makedirs(OUTDIR, exist_ok=True)

class WS:
    def __init__(s,url):
        r=url[5:]; hp,path=r.split("/",1); h,p=hp.split(":")
        s.s=socket.create_connection((h,int(p)),5); k=base64.b64encode(os.urandom(16)).decode()
        s.s.sendall((f"GET /{path} HTTP/1.1\r\nHost: {hp}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {k}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        b=b""
        while b"\r\n\r\n" not in b:
            c=s.s.recv(4096)
            if not c: raise IOError("closed")
            b+=c
        s.s.settimeout(120)
    def send(s,o):
        d=json.dumps(o).encode(); m=os.urandom(4); n=len(d); h=bytearray([0x81])
        if n<126: h.append(0x80|n)
        elif n<65536: h.append(0x80|126); h+=struct.pack(">H",n)
        else: h.append(0x80|127); h+=struct.pack(">Q",n)
        h+=m+bytes(d[i]^m[i%4] for i in range(n)); s.s.sendall(bytes(h))
    def ex(s,n):
        b=b""
        while len(b)<n:
            c=s.s.recv(n-len(b))
            if not c: raise IOError
            b+=c
        return b
    def recv(s):
        b=s.ex(2)[1]; ln=b&0x7f
        if ln==126: ln=struct.unpack(">H",s.ex(2))[0]
        elif ln==127: ln=struct.unpack(">Q",s.ex(8))[0]
        return json.loads(s.ex(ln).decode("utf-8","replace"))

proc=subprocess.Popen(["/snap/bin/chromium","--headless=new","--no-sandbox","--disable-gpu","--enable-unsafe-swiftshader","--disable-dev-shm-usage","--no-first-run","--remote-allow-origins=*","--disable-application-cache","--disk-cache-dir=/tmp/chrcache_ls1","--user-data-dir=/tmp/chrprof_ls1",f"--remote-debugging-port={PORT}","--window-size=1500,1100","about:blank"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
wsurl=None
for _ in range(80):
    time.sleep(0.5)
    try:
        for t in json.load(urllib.request.urlopen(f"http://127.0.0.1:{PORT}/json")):
            if t.get("type")=="page": wsurl=t["webSocketDebuggerUrl"]; break
        if wsurl: break
    except Exception: pass
if not wsurl: print("no page"); proc.kill(); sys.exit(1)
ws=WS(wsurl); mid=[0]; EVENTS=[]
def call(m,p=None):
    mid[0]+=1; ws.send({"id":mid[0],"method":m,"params":p or {}})
    while True:
        r=ws.recv()
        if "method" in r: EVENTS.append(r)
        if r.get("id")==mid[0]: return r
call("Page.enable"); call("Runtime.enable")
call("Page.navigate",{"url":URL})
def ev(expr, awaitP=False, timeout=None):
    r=call("Runtime.evaluate",{"expression":expr,"returnByValue":True,"awaitPromise":awaitP})
    res=r.get("result",{})
    if res.get("exceptionDetails"): return {"__exc__":json.dumps(res["exceptionDetails"],ensure_ascii=False)[:400]}
    return res.get("result",{}).get("value")

# wait for meshes
ok=False
for i in range(120):
    v=ev("(typeof nodes!=='undefined'&&nodes.length>=11&&typeof loaded!=='undefined'&&loaded>=total&&nodes[10].jn.children.length>0)")
    if v is True: ok=True; break
    time.sleep(1)
print("meshes ready:", ok, flush=True)
if not ok: proc.kill(); sys.exit(1)

SETUP = r'''(function(){
  if(window.__CT) return 'exists';
  function meshOf(jn){ var r=null; jn.children.forEach(function(ch){ if(ch.isMesh && !r) r=ch; }); return r; }
  function samp(mesh, maxN){ var pos=mesh.geometry.attributes.position; var n=pos.count; var stride=Math.max(1,Math.ceil(n/maxN)); var a=[]; for(var i=0;i<n;i+=stride){ a.push(pos.getX(i),pos.getY(i),pos.getZ(i)); } return new Float32Array(a); }
  var ground=[], colLinks=[];
  for(var i=1;i<=10;i++){ var m=meshOf(nodes[i].jn); if(!m) continue; var pts=samp(m, i<=7?2500:(i==8?2000:1200)); var L={i:i,pts:pts}; ground.push(L); if(i>=2) colLinks.push(L); }
  var colM=meshOf(nodes[0].jn); var colPts=samp(colM,6000);
  window.__CT={ground:ground, colLinks:colLinks, colLocal:colPts, col:null, v:new THREE.Vector3()};
  window.__CT.pose=function(rads){ for(var k=0;k<6;k++){ Q[k]=rads[k]; } apply(); scene.updateMatrixWorld(true); return true; };
  window.__CT.setGrip=function(g){ GRIP=g; apply(); scene.updateMatrixWorld(true); };
  window.__CT.colStats=function(){
    var v=window.__CT.v; var mtx=nodes[0].jn.matrixWorld;
    var mn={x:1e9,y:1e9,z:1e9}, mx={x:-1e9,y:-1e9,z:-1e9}; var pts=[];
    for(var k=0;k<window.__CT.colLocal.length;k+=3){ v.set(window.__CT.colLocal[k],window.__CT.colLocal[k+1],window.__CT.colLocal[k+2]).applyMatrix4(mtx); pts.push(v.x,v.y,v.z); if(v.x<mn.x)mn.x=v.x; if(v.y<mn.y)mn.y=v.y; if(v.z<mn.z)mn.z=v.z; if(v.x>mx.x)mx.x=v.x; if(v.y>mx.y)mx.y=v.y; if(v.z>mx.z)mx.z=v.z; }
    var cx=(mn.x+mx.x)/2, cy=(mn.y+mx.y)/2;
    var rs=[]; for(var k=0;k<pts.length;k+=3){ var z=pts[k+2]; if(z>mn.z+0.05 && z<mx.z-0.02){ rs.push(Math.hypot(pts[k]-cx, pts[k+1]-cy)); } }
    rs.sort(function(a,b){return a-b;});
    function pc(p){ return rs[Math.min(rs.length-1, Math.floor(rs.length*p))]; }
    window.__CT.col={cx:cx, cy:cy, z0:mn.z, z1:mx.z, rc:pc(0.9), r50:pc(0.5), r75:pc(0.75), r85:pc(0.85), r90:pc(0.9), r95:pc(0.95), rmax:rs[rs.length-1], size:[mx.x-mn.x, mx.y-mn.y, mx.z-mn.z]};
    return window.__CT.col;
  };
  window.__CT.measure=function(){
    var v=window.__CT.v; var minZ=1e9, mzL=-1, colClear=1e9, ccL=-1;
    var C=window.__CT.col; var zT=C? (C.z1+0.03):0;
    function scan(list, doCol){
      for(var li=0;li<list.length;li++){ var L=list[li]; var mtx=nodes[L.i].jn.matrixWorld; var p=L.pts;
        for(var k=0;k<p.length;k+=3){ v.set(p[k],p[k+1],p[k+2]).applyMatrix4(mtx);
          if(v.z<minZ){ minZ=v.z; mzL=L.i; }
          if(doCol && C && v.z>C.z0 && v.z<zT){ var d=Math.hypot(v.x-C.cx, v.y-C.cy)-C.rc; if(d<colClear){ colClear=d; ccL=L.i; } }
        }
      }
    }
    scan(window.__CT.ground, false);
    if(C) scan(window.__CT.colLinks, true);
    return {minZ:+minZ.toFixed(4), colClear:(colClear===1e9?null:+colClear.toFixed(4)), mzL:mzL, ccL:ccL};
  };
  window.__CT.scan=function(poses){ var out=[]; for(var i=0;i<poses.length;i++){ window.__CT.pose(poses[i]); out.push(window.__CT.measure()); } return out; };
  return 'CT ready; groundLinks='+ground.length+' colLocal='+window.__CT.colLocal.length;
})()'''
print("setup:", ev(SETUP), flush=True)
v=ev("(function(){ window.__CT.setGrip(0.10); window.__CT.pose(%s); return window.__CT.colStats(); })()" % json.dumps(BASE))
print("colStats:", json.dumps(v,ensure_ascii=False), flush=True)
base_m=ev("window.__CT.measure()")
print("baseline:", json.dumps(base_m,ensure_ascii=False), flush=True)

# star scans
stars={}
for j in range(6):
    lo=max(-0.9, JLIM[j][0]*math.pi/180-BASE[j]); hi=min(0.9, JLIM[j][1]*math.pi/180-BASE[j])
    grid=[round(i*0.02,4) for i in range(-45,46)]
    offs=[o for o in grid if (lo-1e-9)<=o<=(hi+1e-9)]
    if 0.0 not in offs:
        offs.append(0.0); offs.sort()
    poses=[]
    for o in offs:
        p=list(BASE); p[j]=BASE[j]+o; poses.append(p)
    res=ev("window.__CT.scan(%s)" % json.dumps(poses))
    if isinstance(res,dict) and res.get("__exc__"): print("star j%d EXC:"%(j+1), res["__exc__"]); continue
    stars["j%d"%(j+1)]=[[round(offs[i],4), res[i]["minZ"], res[i]["colClear"], res[i]["mzL"], res[i]["ccL"]] for i in range(len(offs))]
    print("star j%d done (%d poses)"%(j+1,len(offs)), flush=True)

# wave trajectory samples
def wave_pose(th, amp, base=BASE):
    out=[]
    for i in range(6):
        vv=amp*AMP[i]*(0.75*math.sin(th-i*LAG)+0.25*math.sin(2*th-2*i*LAG))
        if i==5: vv += amp*0.17*math.sin(2.6*th+0.4)
        out.append(base[i]+vv)
    return out
poses=[]
meta=[]
for k in range(60):
    th=2*math.pi*k/60
    poses.append(wave_pose(th,1.0)); meta.append(("traj",k,1.0))
for k in range(60):
    th=2*math.pi*k/60
    poses.append(wave_pose(th,1.2)); meta.append(("traj",k,1.2))
import random
random.seed(42)
mx=[AMP[0]*1.2,AMP[1]*1.2,AMP[2]*1.2,AMP[3]*1.2,AMP[4]*1.2,(AMP[5]+0.17)*1.2]
for k in range(220):
    p=[BASE[i]+random.uniform(-mx[i],mx[i]) for i in range(6)]
    poses.append(p); meta.append(("rand",k,1.2))
# corners
for msk in range(64):
    p=[BASE[i]+(mx[i] if (msk>>i)&1 else -mx[i]) for i in range(6)]
    poses.append(p); meta.append(("corner",msk,1.2))
res=ev("window.__CT.scan(%s)" % json.dumps(poses))
if isinstance(res,dict) and res.get("__exc__"):
    print("wave EXC:",res["__exc__"]); res=[]
print("wave scan done:", len(res), flush=True)

# analyze + save
report={"base":BASE,"col":v,"baseline":base_m,"stars":{}, "wave":[], "meta":[]}
rep={}
def viol(m, gm=0.015, cm=0.025):
    return (m["minZ"]<gm) or (m["colClear"] is not None and m["colClear"]<cm)
for jn,arr in stars.items():
    d={round(o,4):(mz,cc) for o,mz,cc,_,_ in arr}
    def okv(o):
        k=round(o,4)
        if k not in d: return None
        mz,cc=d[k]
        return not viol({"minZ":mz,"colClear":cc})
    lo=0.0; o=-0.02
    while True:
        r=okv(o)
        if r is None or not r: break
        lo=o; o=round(o-0.02,4)
    hi=0.0; o=0.02
    while True:
        r=okv(o)
        if r is None or not r: break
        hi=o; o=round(o+0.02,4)
    fvneg=None; o=-0.02
    while True:
        r=okv(o)
        if r is None: break
        if not r: fvneg=o; break
        o=round(o-0.02,4)
    fvpos=None; o=0.02
    while True:
        r=okv(o)
        if r is None: break
        if not r: fvpos=o; break
        o=round(o+0.02,4)
    rep[jn]={"safe_lo":round(lo,3),"safe_hi":round(hi,3),"fvneg":fvneg,"fvpos":fvpos,
             "worst":min(arr,key=lambda r:(r[1], r[2] if r[2] is not None else 9))}
wave_worst={"minZ":9,"colClear":9,"idx":-1}
nviol=0
for i,m in enumerate(res):
    report["wave"].append([meta[i][0],m["minZ"],m["colClear"],m["mzL"],m["ccL"]])
    report["meta"].append([meta[i][0],meta[i][1],meta[i][2]])
    if m["minZ"]<wave_worst["minZ"] or (m["colClear"] is not None and m["colClear"]<wave_worst["colClear"]): pass
    if (m["minZ"]<wave_worst["minZ"]): wave_worst["minZ"]=m["minZ"]
    if m["colClear"] is not None and m["colClear"]<wave_worst["colClear"]: wave_worst["colClear"]=m["colClear"]
    if viol(m): nviol+=1
report["stars"]=rep
report["wave_worst"]=wave_worst
report["wave_violations_margin"]=nviol
save={"base_pose":"宠物位","base_rad":BASE,"col":v,"baseline":base_m,"star_safe":{k:{"safe_lo":rep[k]["safe_lo"],"safe_hi":rep[k]["safe_hi"]} for k in rep},"star_detail":rep,"wave":{"violations":nviol,"worst":wave_worst},"margins":{"ground":0.015,"column":0.025}}
open("/tmp/limscan_result.json","w").write(json.dumps(save,ensure_ascii=False,indent=1))
print("SAVED /tmp/limscan_result.json", flush=True)
print(json.dumps({k:{"lo":rep[k]["safe_lo"],"hi":rep[k]["safe_hi"]} for k in rep},ensure_ascii=False), flush=True)
print("wave violations(margin):",nviol,"| worst minZ:",wave_worst["minZ"],"| worst colClear:",wave_worst["colClear"], flush=True)
# screenshots: baseline + worst-wave
SHOT=r'''(function(){ return true; })()'''
def shot(name, cam=None):
    if cam:
        ev("(function(){ camera.position.set(%f,%f,%f); controls.target.set(%f,%f,%f); controls.update(); renderer.render(scene,camera); return 1; })()" % tuple(cam[0])+tuple(cam[1]) if False else "")
    r=call("Page.captureScreenshot",{"format":"png"})
    open(os.path.join(OUTDIR,name),"wb").write(base64.b64decode(r["result"]["data"]))
    print("shot:",name, flush=True)
ev("(function(){ window.__CT.pose(%s); camera.position.set(1.6,-1.6,1.15); controls.target.set(0,0,0.45); controls.update(); renderer.render(scene,camera); return 1; })()" % json.dumps(BASE))
time.sleep(1.0); shot("00_baseline_宠物位.png")
# worst wave pose index = find idx with min of (minZ, colClear)
best=None
for i,m in enumerate(res):
    score=min(m["minZ"], m["colClear"] if m["colClear"] is not None else 9)
    if best is None or score<best[0]: best=(score,i,m)
print("worst wave pose idx",best[1],"score",best[0],"m",best[2],flush=True)
# regenerate that pose and shoot
mi=best[1]; tag,idx,scale=meta[mi]
if tag=="traj": p=wave_pose(2*math.pi*idx/60, scale)
elif tag=="rand":
    random.seed(42)
    # replay to idx
    # simpler: store poses list (we have it)
    p=poses[mi]
else: p=poses[mi]
ev("(function(){ window.__CT.pose(%s); camera.position.set(1.7,-1.5,1.0); controls.target.set(0,0,0.45); controls.update(); renderer.render(scene,camera); return 1; })()" % json.dumps(p))
time.sleep(1.0); shot("01_worst_wave.png")
# star-boundary shots (interesting ones)
nshots=0
for j in range(6):
    for side in ("fvneg","fvpos"):
        fv=rep["j%d"%(j+1)].get(side)
        if fv is not None and abs(fv)<=0.7 and nshots<6:
            p=list(BASE); p[j]=BASE[j]+fv
            ev("(function(){ window.__CT.pose(%s); camera.position.set(1.8,-1.2,1.0); controls.target.set(0,0,0.45); controls.update(); renderer.render(scene,camera); return 1; })()" % json.dumps(p))
            time.sleep(0.8)
            shot("1%d_%s_j%d_%+.2frad.png" % (nshots+1, "pos" if fv>0 else "neg", j+1, fv))
            nshots+=1
print("DONE_ALL", flush=True)
proc.kill()
