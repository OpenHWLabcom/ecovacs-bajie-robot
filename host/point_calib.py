#!/usr/bin/env python3
# 八界 对点校准探针：连接 robot-ws，取头部/手臂相机图 + 下发 hand_observe，记录位姿。
# 用法: python3 point_calib.py [--bearings 0,-40,40] [--d 0.6] [--z 0.45] [--shots DIR] [--dry]
# 安全: 漂移护栏(默认 0.8m 超限即 stop_all 中止) + 每步超时。
import os, sys, json, socket, base64, struct, time, argparse, math, hashlib

HOST="192.168.1.100"; PORT=17890; PATH="/robot-ws/"
def log(*a): print(time.strftime("%H:%M:%S"), *a, flush=True)

class WS:
    def __init__(s, host, port, path):
        s.s=socket.create_connection((host,port),5); s.lock=__import__("threading").Lock()
        key=base64.b64encode(os.urandom(16)).decode()
        s.s.sendall((f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        b=b""
        while b"\r\n\r\n" not in b: b+=s.s.recv(4096)
        s.s.settimeout(60)
    def _tx(s, obj):
        data=json.dumps(obj,ensure_ascii=False).encode(); mask=os.urandom(4); n=len(data); h=bytearray([0x81])
        if n<126: h.append(0x80|n)
        elif n<65536: h.append(0x80|126); h+=struct.pack(">H",n)
        else: h.append(0x80|127); h+=struct.pack(">Q",n)
        h+=mask+bytes(data[i]^mask[i%4] for i in range(n)); s.s.sendall(bytes(h))
    def send(s,obj):
        with s.lock: s._tx(obj)
    def _rx(s,n):
        b=b""
        while len(b)<n:
            c=s.s.recv(n-len(b))
            if not c: raise IOError("eof")
            b+=c
        return b
    def recv(s):
        b=s._rx(2); ln=b[1]&0x7f
        if ln==126: ln=struct.unpack(">H",s._rx(2))[0]
        elif ln==127: ln=struct.unpack(">Q",s._rx(8))[0]
        pay=s._rx(ln)
        try: return json.loads(pay.decode("utf-8","replace"))
        except Exception: return None

def hdr(mode,cmd,uuid): return {"mode":mode,"type":"request","cmd":cmd,"ts":int(time.time()),"uuid":uuid,"host":""}
def mission(ws,name,data,uuid): ws.send({"header":hdr("mission","start",uuid),"body":{"name":name,"task_id":uuid,"data":data or {}}})

state={"pos":None,"work":None,"imgs":{},"fin":{}}
def feed(o):
    if not o or not o.get("body"): return
    b=o["body"]; n=b.get("name"); d=b.get("data") or {}; h=o.get("header") or {}
    if n=="robot_info":
        if d.get("pos"): state["pos"]=d["pos"]
        if d.get("workState"): state["work"]=d["workState"]
    elif n=="image_query":
        rd=d.get("result_data") or {}
        img=(rd.get("rgb_image") or {}).get("data")
        if img: state["imgs"][b.get("task_id")]=img; log("img", b.get("task_id"), len(img),"b64")
    if h.get("type")=="notify" and h.get("cmd")=="finish":
        state["fin"][b.get("task_id")]=time.time()

def wrap(a):
    while a>math.pi: a-=2*math.pi
    while a<-math.pi: a+=2*math.pi
    return a

def wait_idle(ws, timeout=30, settle=1.5):
    # 先给任务一点时间起来，再等 work 回落
    t=time.time()
    while time.time()-t<settle:
        try: feed(ws.recv())
        except socket.timeout: pass
    while time.time()-t<timeout:
        try: feed(ws.recv())
        except socket.timeout: pass
        w=state["work"] or {}
        if not w.get("name") or w.get("cmd")=="idle": return True
    return False

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--bearings",default="0")
    ap.add_argument("--d",type=float,default=0.6)
    ap.add_argument("--z",type=float,default=0.45)
    ap.add_argument("--ox",type=float,default=0.16)
    ap.add_argument("--oy",type=float,default=0.0)
    ap.add_argument("--shots",default=os.path.expanduser("~/calib_shots"))
    ap.add_argument("--maxdrift",type=float,default=0.8)
    ap.add_argument("--dry",action="store_true")
    a=ap.parse_args()
    os.makedirs(a.shots,exist_ok=True)
    bearings=[float(x) for x in a.bearings.split(",") if x.strip()!=""]

    ws=WS(HOST,PORT,PATH)
    ws.s.settimeout(12)
    ws.send({"header":hdr("event","oneshot","one1"),"body":{"name":"robot_info","task_id":"one1","data":{"topics":["pos","workState"]}}})
    ws.send({"header":hdr("event","subscribe","sub1"),"body":{"name":"robot_info","task_id":"sub1","data":{"topics":["pos","workState"]}}})
    # pump until we have a pose
    t0=time.time()
    while state["pos"] is None and time.time()-t0<10:
        try: feed(ws.recv())
        except socket.timeout: break
    if state["pos"] is None:
        log("!! 没拿到位姿，检查 robot-ws / 网络"); return 2
    log("起始位姿", json.dumps(state["pos"],ensure_ascii=False))
    start=(state["pos"]["x"],state["pos"]["y"])

    def capture(cam, tag):
        tid="calib-%s-%d"%(cam,int(time.time()*1000))
        mission(ws,"image_query",{"camera_type":cam},tid)
        t=time.time()
        while tid not in state["imgs"] and time.time()-t<12:
            try: feed(ws.recv())
            except socket.timeout: break
        b64=state["imgs"].get(tid)
        if b64:
            open(os.path.join(a.shots,"%s_%s.png"%(tag,cam)),"wb").write(base64.b64decode(b64))
        return b64 is not None

    results=[]
    for deg in bearings:
        if not wait_idle(ws,40): log("!! 机器人忙，跳过"); continue
        p=state["pos"]; C=(p["x"],p["y"]); psi=p["yaw"] or 0.0
        rad=math.radians(deg)
        lx=a.ox+a.d*math.cos(rad); ly=a.oy+a.d*math.sin(rad); lz=a.z
        c=math.cos(psi); s=math.sin(psi)
        P=(C[0]+lx*c-ly*s, C[1]+lx*s+ly*c, lz)
        log("bearing %+d: C=(%.2f,%.2f) psi=%.3f  -> hand_observe(map %.2f,%.2f,%.2f)"%(deg,C[0],C[1],psi,P[0],P[1],P[2]))
        if a.dry: continue
        # 先拍手臂相机(看当前朝向)
        capture(1, "b%+d_before"%int(deg))
        tid="calib-ho-%d"%int(time.time()*1000)
        mission(ws,"hand_observe",{"frame_id":"map","position":{"x":round(P[0],3),"y":round(P[1],3),"z":round(P[2],3)},"orientation":{"x":0,"y":0,"z":0,"w":1},"box_length":{"x":0.02,"y":0.02,"z":0.005}},tid)
        log("  已下发 hand_observe: "+tid)
        if not wait_idle(ws,60): log("  !! 等待超时")
        p1=state["pos"]; C1=(p1["x"],p1["y"]); psi1=p1["yaw"] or 0.0
        err=math.degrees(wrap(psi1-psi-math.radians(deg)))
        drift=math.hypot(C1[0]-start[0],C1[1]-start[1])
        log("  完成: C1=(%.2f,%.2f) psi1=%.3f  yaw响应err=%+.1f°  drift=%.2fm"%(C1[0],C1[1],psi1,err,drift))
        capture(1,"b%+d_after"%int(deg)); capture(2,"b%+d_headafter"%int(deg))
        results.append({"bearing":deg,"err":round(err,1),"drift":round(drift,3),"C0":C,"psi0":psi,"C1":C1,"psi1":psi1,"P":P})
        if drift>a.maxdrift:
            log("!! 漂移超限 %.2f>%.2f → stop_all 中止"%(drift,a.maxdrift))
            mission(ws,"stop_all",{},"calib-abort-%d"%int(time.time())); break
    json.dump(results,open(os.path.join(a.shots,"result.json"),"w"),ensure_ascii=False,indent=1)
    log("结果写入", os.path.join(a.shots,"result.json"))
    return 0

if __name__=="__main__":
    sys.exit(main())
