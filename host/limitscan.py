#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 第二轮：波浪真实轨迹密集扫描 + 立柱模型校验 + 输出限位 JSON
import os, json, socket, base64, struct, time, subprocess, urllib.request, math, sys
URL="http://127.0.0.1:8790/arm3d.html"; PORT=9503
BASE=[0.309,2.583,2.226,-0.044,-0.118,-0.346]
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
        s.s.settimeout(180)
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
proc=subprocess.Popen(["/snap/bin/chromium","--headless=new","--no-sandbox","--disable-gpu","--enable-unsafe-swiftshader","--disable-dev-shm-usage","--no-first-run","--remote-allow-origins=*","--disable-application-cache","--disk-cache-dir=/tmp/chrcache_ls2","--user-data-dir=/tmp/chrprof_ls2",f"--remote-debugging-port={PORT}","--window-size=1500,1100","about:blank"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
wsurl=None
for _ in range(80):
    time.sleep(0.5)
    try:
        for t in json.load(urllib.request.urlopen(f"http://127.0.0.1:{PORT}/json")):
            if t.get("type")=="page": wsurl=t["webSocketDebuggerUrl"]; break
        if wsurl: break
    except Exception: pass
if not wsurl: print("no page"); proc.kill(); sys.exit(1)
ws=WS(wsurl); mid=[0]
def call(m,p=None):
    mid[0]+=1; ws.send({"id":mid[0],"method":m,"params":p or {}})
    while True:
        r=ws.recv()
        if r.get("id")==mid[0]: return r
call("Page.enable"); call("Runtime.enable")
call("Page.navigate",{"url":URL})
def ev(expr):
    r=call("Runtime.evaluate",{"expression":expr,"returnByValue":True})
    res=r.get("result",{})
    if res.get("exceptionDetails"): return {"__exc__":json.dumps(res["exceptionDetails"],ensure_ascii=False)[:400]}
    return res.get("result",{}).get("value")
ok=False
for i in range(120):
    if ev("(typeof nodes!=='undefined'&&nodes.length>=11&&typeof loaded!=='undefined'&&loaded>=total&&nodes[10].jn.children.length>0)") is True: ok=True; break
    time.sleep(1)
print("meshes ready:", ok, flush=True)
SETUP = open("/tmp/ctsetup.js").read() if os.path.exists("/tmp/ctsetup.js") else None
# inline setup (with cp point capture + RC-param measure + brute force)
SETUP = r'''(function(){
  if(window.__CT) return 'exists';
  function meshOf(jn){ var r=null; jn.children.forEach(function(ch){ if(ch.isMesh && !r) r=ch; }); return r; }
  function samp(mesh, maxN){ var pos=mesh.geometry.attributes.position; var n=pos.count; var stride=Math.max(1,Math.ceil(n/maxN)); var a=[]; for(var i=0;i<n;i+=stride){ a.push(pos.getX(i),pos.getY(i),pos.getZ(i)); } return new Float32Array(a); }
  var ground=[], colLinks=[];
  for(var i=1;i<=10;i++){ var m=meshOf(nodes[i].jn); if(!m) continue; var pts=samp(m, i<=7?2500:(i==8?2000:1200)); var L={i:i,pts:pts}; ground.push(L); if(i>=2) colLinks.push(L); }
  var colM=meshOf(nodes[0].jn); var colPts=samp(colM,6000);
  window.__CT={ground:ground, colLinks:colLinks, colLocal:colPts, col:null, v:new THREE.Vector3(),
               colWorld:null};
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
    window.__CT.colWorld=pts;
    return window.__CT.col;
  };
  window.__CT.measure=function(){
    var v=window.__CT.v; var minZ=1e9, mzL=-1, colClear=1e9, ccL=-1, cpx=0,cpy=0,cpz=0, gx=0,gy=0;
    var C=window.__CT.col; var zT=C? (C.z1+0.03):0;
    function scan(list, doCol){
      for(var li=0;li<list.length;li++){ var L=list[li]; var mtx=nodes[L.i].jn.matrixWorld; var p=L.pts;
        for(var k=0;k<p.length;k+=3){ v.set(p[k],p[k+1],p[k+2]).applyMatrix4(mtx);
          if(v.z<minZ){ minZ=v.z; mzL=L.i; gx=v.x; gy=v.y; }
          if(doCol && C && v.z>C.z0 && v.z<zT){ var d=Math.hypot(v.x-C.cx, v.y-C.cy)-C.rc; if(d<colClear){ colClear=d; ccL=L.i; cpx=v.x; cpy=v.y; cpz=v.z; } }
        }
      }
    }
    scan(window.__CT.ground, false);
    if(C) scan(window.__CT.colLinks, true);
    return {minZ:+minZ.toFixed(4), colClear:(colClear===1e9?null:+colClear.toFixed(4)), mzL:mzL, ccL:ccL, cp:[+cpx.toFixed(3),+cpy.toFixed(3),+cpz.toFixed(3)]};
  };
  window.__CT.measureRC=function(rc){ // 用指定 rc 重算当前姿态 colClear
    var v=window.__CT.v; var C=window.__CT.col; var zT=C.z1+0.03; var best=1e9;
    for(var li=0;li<window.__CT.colLinks.length;li++){ var L=window.__CT.colLinks[li]; var mtx=nodes[L.i].jn.matrixWorld; var p=L.pts;
      for(var k=0;k<p.length;k+=3){ v.set(p[k],p[k+1],p[k+2]).applyMatrix4(mtx);
        if(v.z>C.z0 && v.z<zT){ var d=Math.hypot(v.x-C.cx, v.y-C.cy)-rc; if(d<best) best=d; } } }
    return +best.toFixed(4);
  };
  window.__CT.bruteCol=function(){ // 真距：抽样点 cloud 对 colWorld 全量
    var v=window.__CT.v; var best=1e9; var CW=window.__CT.colWorld;
    for(var li=0;li<window.__CT.colLinks.length;li++){ var L=window.__CT.colLinks[li]; var mtx=nodes[L.i].jn.matrixWorld; var p=L.pts; var stp=Math.max(1, Math.floor((p.length/3)/1200))*3;
      for(var k=0;k<p.length;k+=stp){ v.set(p[k],p[k+1],p[k+2]).applyMatrix4(mtx);
        for(var j=0;j<CW.length;j+=3){ var dx=v.x-CW[j], dy=v.y-CW[j+1], dz=v.z-CW[j+2]; var d2=dx*dx+dy*dy+dz*dz; if(d2<best) best=d2; } } }
    return +Math.sqrt(best).toFixed(4);
  };
  window.__CT.scan=function(poses){ var out=[]; for(var i=0;i<poses.length;i++){ window.__CT.pose(poses[i]); out.push(window.__CT.measure()); } return out; };
  return 'CT ready; groundLinks='+ground.length;
})()'''
print("setup:", ev(SETUP), flush=True)
v=ev("(function(){ window.__CT.setGrip(0.506); window.__CT.pose(%s); return window.__CT.colStats(); })()" % json.dumps(BASE))
print("colStats:", json.dumps(v,ensure_ascii=False), flush=True)
base_m=ev("window.__CT.measure()")
print("baseline:", json.dumps(base_m,ensure_ascii=False), flush=True)
br=ev("window.__CT.bruteCol()"); print("baseline brute col-dist:", br, flush=True)

def wave_pose(th, amp, base=BASE):
    out=[]
    for i in range(6):
        vv=amp*AMP[i]*(0.75*math.sin(th-i*LAG)+0.25*math.sin(2*th-2*i*LAG))
        if i==5: vv += amp*0.17*math.sin(2.6*th+0.4)
        out.append(base[i]+vv)
    return out
poses=[]; meta=[]
for scale in (1.0, 1.15):
    for k in range(240):
        th=2*math.pi*k/240
        poses.append(wave_pose(th,scale)); meta.append(["traj",k,scale,round(th,4)])
import random
random.seed(7)
mx=[AMP[0]*1.15,AMP[1]*1.15,AMP[2]*1.15,AMP[3]*1.15,AMP[4]*1.15,(AMP[5]+0.17)*1.15]
for k in range(150):
    poses.append([BASE[i]+random.uniform(-mx[i],mx[i]) for i in range(6)]); meta.append(["rand",k,1.15,0])
for msk in range(64):
    poses.append([BASE[i]+(mx[i] if (msk>>i)&1 else -mx[i]) for i in range(6)]); meta.append(["corner",msk,1.15,0])
res=ev("window.__CT.scan(%s)" % json.dumps(poses))
print("scan done:", len(res), flush=True)
save={"base_rad":BASE,"col":v,"baseline":base_m,"brute_baseline":br,"rows":res,"meta":meta}
open("/tmp/limscan2_result.json","w").write(json.dumps(save,ensure_ascii=False))
# analysis
def viol(m): return (m["minZ"]<0.015) or (m["colClear"] is not None and m["colClear"]<0.025)
for tag in ("traj10","traj115","rand","corner"):
    subset=[(r,m) for r,m in zip(res,meta) if (m[0]=="traj" and ((tag=="traj10" and m[2]==1.0) or (tag=="traj115" and m[2]==1.15))) or (tag=="rand" and m[0]=="rand") or (tag=="corner" and m[0]=="corner")]
    if not subset: continue
    mc=min(r["colClear"] for r,m in subset); mnz=min(r["minZ"] for r,m in subset)
    nv=sum(1 for r,m in subset if viol(r))
    print("%s: n=%d viol=%d worst colClear=%.4f worst minZ=%.4f"%(tag,len(subset),nv,mc,mnz), flush=True)
# worst traj@1.0
tr=[(r,m) for r,m in zip(res,meta) if m[0]=="traj" and m[2]==1.0]
w=min(tr,key=lambda x:x[0]["colClear"])
print("worst traj@1.0:", json.dumps(w[0],ensure_ascii=False), "meta:",w[1], flush=True)
i=res.index(w[0])
ev("(function(){ window.__CT.pose(%s); return 1; })()" % json.dumps(poses[i]))
print("  rc sensitivity at worst traj pose: r90=%.4f r95=%.4f rmax=%.4f  brute=%.4f" % (
    ev("window.__CT.measureRC(%f)"%v["r90"]) if False else w[0]["colClear"],
    ev("window.__CT.measureRC(%f)"%v["r95"]), ev("window.__CT.measureRC(%f)"%v["rmax"]), ev("window.__CT.bruteCol()")), flush=True)
# corner worst
cw=min([(r,m) for r,m in zip(res,meta) if m[0]=="corner"],key=lambda x:x[0]["colClear"])
print("worst corner:", json.dumps(cw[0],ensure_ascii=False),"meta:",cw[1], flush=True)
open("/tmp/limscan2_summary.json","w").write(json.dumps({"worst_traj":w[0],"worst_traj_meta":w[1],"worst_corner":cw[0]},ensure_ascii=False))
print("DONE", flush=True)
proc.kill()
