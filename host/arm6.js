/* arm6.js — 6 自由度机械臂：3D 科幻风 · 调试 + 模拟 + 取物 Skill 视觉/算法模拟
 * 关节：joint1..joint6。运动学 = 八界**实机机械臂 mvp03**（对齐物理定位）
 *   参数取自本机 /linux/app/e2e_inference/utils/rm_ik/python3/ik_rbtdef.py
 *   （ee_controller.py 实际用 QPIK("mvp03")），改进DH(MDH)，限位 ±178°。
 * 底盘/臂基座可跟随真机定位(state.pos) → 模拟与物理定位对齐。
 * 自足：不依赖外部库。挂到控制台 8790 页面。 */
(function () {
  "use strict";
  var D2R = Math.PI / 180, R2D = 180 / Math.PI;
  var $ = function (id) { return document.getElementById(id); };

  /* ---------------- 6 关节模型（可调） ---------------- */
  var A  = [0.000, 0.310, 0.270, 0.075, 0.060, 0.050]; // 沿 X 的连杆长
  var DZ = [0.150, 0.000, 0.000, 0.000, 0.000, 0.000]; // 沿 Z 的偏移
  var AX = ["z", "y", "y", "x", "y", "x"];             // 关节轴
  var LIM = [178, 178, 178, 178, 178, 178];            // 量程(度) 供量程弧/显示
  /* 真机 mvp03 关节实际限位(度) —— 取自下板 arm_escape/urdf/mvp0_3_zj.urdf：
     J1/J2/J4/J5/J6 = ±3.12rad(±178.8°)；J3 不对称 = -2.95..+3.30rad(-169.0°..189.1°) */
  var JLO = [-178.8, -178.8, -169.0, -178.8, -178.8, -178.8];
  var JHI = [ 178.8,  178.8,  189.1,  178.8,  178.8,  178.8];
  var N = 6;
  var HOME = [0, 0, 0, 0, 0, 0];                   // 默认/归位：垂直站立(臂竖直向上)
  var Q = HOME.map(function (v) { return v * D2R; });

  /* ---------------- 4x4 线性代数 ---------------- */
  function mm(A, B) { var C = new Array(16); for (var i = 0; i < 4; i++) for (var j = 0; j < 4; j++) { var s = 0; for (var k = 0; k < 4; k++) s += A[i * 4 + k] * B[k * 4 + j]; C[i * 4 + j] = s; } return C; }
  function rotm(ax, t) { var c = Math.cos(t), s = Math.sin(t);
    if (ax === "z") return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    if (ax === "y") return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
    return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1]; }
  function tr(x, y, z) { return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1]; }
  function pos(T) { return { x: T[3], y: T[7], z: T[11] }; }
  function sub(a, b) { return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }; }
  function dot(a, b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
  function cross(a, b) { return { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }; }
  function clamp(v, a, b) { return v < a ? a : v > b ? b : v; }
  function len(a) { return Math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
  function unit(a) { var l = len(a) || 1; return { x: a.x / l, y: a.y / l, z: a.z / l }; }

  /* ---------------- 真实运动学：真机 URDF 链 (mvp0_3_zj.urdf，八界 mvp03) ----------------
   * 每个关节链: T = T * origin(rpy,xyz) * rot(axis, q)；/joint_states 的关节角即此约定。
   * 原 MDH(ik_rbtdef QPIK "mvp03") 零位/坐标约定与 URDF 不同 → 镜像时姿态会偏，改用 URDF。 */
  var URDF = [
    { xyz: [0, 0, 0.1421],           rpy: [0, -Math.PI / 2, 0], axis: [1, 0, 0] },
    { xyz: [0.0424, -0.060, 0],      rpy: [0, 0, 0],          axis: [0, -1, 0] },
    { xyz: [0.3398, 0.0102, 0],      rpy: [0, 0, 0],          axis: [0, 1, 0] },   // 长杆 J2→J3 = 34cm(实测)
    { xyz: [0.0752, 0.120, 0],      rpy: [0, 0, 0],          axis: [1, 0, 0] },  // J3→J4 弯头段：大臂/小臂错开(外侧对外侧~18cm)
    { xyz: [0.2578, 0.0337, 0],      rpy: [0, 0, 0],          axis: [0, -1, 0] },  // 小连杆 J4→J5 = 26cm(实测)
    { xyz: [0.056, -0.0425, 0.0765], rpy: [0, Math.PI / 2, 0], axis: [0, 0, 1] }
  ];
  var DH = { d: [0.1845, 0, 0, 0.4005, 0, 0.075], a: [0, 0, 0.3245, 0, 0, -0.0765],
             alpha: [0, Math.PI / 2, Math.PI, Math.PI / 2, Math.PI / 2, -Math.PI / 2],
             offset: [0, Math.PI / 2, Math.PI / 2, 0, 0, 0] };   // 保留供 arm6DH() 参考
  function rpyT(r, p, y) {
    var cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    return [ cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0,
             sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0,
             -sp,     cp * sr,                cp * cr,                0,
             0, 0, 0, 1 ];
  }
  function axisT(ax, q) {
    var x = ax[0], y = ax[1], z = ax[2], c = Math.cos(q), s = Math.sin(q), C = 1 - c;
    return [ x * x * C + c, x * y * C - z * s, x * z * C + y * s, 0,
             y * x * C + z * s, y * y * C + c, y * z * C - x * s, 0,
             z * x * C - y * s, z * y * C + x * s, z * z * C + c, 0,
             0, 0, 0, 1 ];
  }
  function oriT(j) { var T = rpyT(j.rpy[0], j.rpy[1], j.rpy[2]); T[3] = j.xyz[0]; T[7] = j.xyz[1]; T[11] = j.xyz[2]; return T; }
  /* FK：URDF 关节链 → 关节/末端点 + 各关节世界轴 + 末端变换 */
  function fk(q) {
    var T = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], pts = [], axes = [];
    pts.push(pos(T));
    for (var i = 0; i < N; i++) {
      var j = URDF[i], ax = j.axis;
      var To = mm(T, oriT(j));   // 关节坐标系(世界)
      axes.push({ x: To[0] * ax[0] + To[1] * ax[1] + To[2] * ax[2],
                  y: To[4] * ax[0] + To[5] * ax[1] + To[6] * ax[2],
                  z: To[8] * ax[0] + To[9] * ax[1] + To[10] * ax[2] });
      T = mm(To, axisT(ax, q[i]));
      pts.push(pos(T));
    }
    var tip = { x: T[3] + 0.12 * T[2], y: T[7] + 0.12 * T[6], z: T[11] + 0.12 * T[10] }; // 夹爪尖(沿末帧 Z = J6 轴向)
    pts.push(tip);
    return { pts: pts, axes: axes, T: T };
  }

  /* ---------------- CCD IK ---------------- */
  function ik(target, iter) {
    iter = iter || 120;
    for (var it = 0; it < iter; it++) {
      var f = fk(Q);
      for (var i = N - 1; i >= 0; i--) {
        var base = f.pts[i], ee = f.pts[N + 1], ax = f.axes[i];
        var v1 = sub(ee, base), v2 = sub(target, base);
        var a = Math.atan2(dot(cross(v1, v2), ax), dot(v1, v2));
        Q[i] = clamp(Q[i] + a, JLO[i] * D2R, JHI[i] * D2R);
        f = fk(Q);
      }
    }
  }

  /* ---------------- 3D 相机 + 透视投影（科幻风） ---------------- */
  var CAM = { az: -0.95, el: 0.42, dist: 4.6, tgt: { x: -0.05, y: -0.06, z: 0.50 } };
  var VB = null, FOCAL = 900;
  var cv = $("arm6"), cx = cv ? cv.getContext("2d") : null;

  function camBasis() {
    var ca = Math.cos(CAM.az), sa = Math.sin(CAM.az), ce = Math.cos(CAM.el), se = Math.sin(CAM.el);
    var dir = { x: ce * ca, y: ce * sa, z: se };           // tgt -> eye
    var eye = { x: CAM.tgt.x + CAM.dist * dir.x, y: CAM.tgt.y + CAM.dist * dir.y, z: CAM.tgt.z + CAM.dist * dir.z };
    var f = unit({ x: CAM.tgt.x - eye.x, y: CAM.tgt.y - eye.y, z: CAM.tgt.z - eye.z });
    var r = unit(cross(f, { x: 0, y: 0, z: 1 }));
    var u = cross(r, f);
    return { eye: eye, f: f, r: r, u: u };
  }
  function proj3(p) {
    if (!VB) return null;
    var d = { x: p.x - VB.eye.x, y: p.y - VB.eye.y, z: p.z - VB.eye.z };
    var zv = d.x * VB.f.x + d.y * VB.f.y + d.z * VB.f.z;
    if (zv < 0.06) return null;
    var xv = d.x * VB.r.x + d.y * VB.r.y + d.z * VB.r.z;
    var yv = d.x * VB.u.x + d.y * VB.u.y + d.z * VB.u.z;
    return { x: cv.width / 2 + xv * FOCAL / zv, y: cv.height / 2 - yv * FOCAL / zv, s: FOCAL / zv, d: zv };
  }
  function line3(a, b, w, col, glow) {
    var A = proj3(a), B = proj3(b); if (!A || !B) return;
    cx.save(); if (glow) { cx.shadowColor = col; cx.shadowBlur = glow; }
    cx.strokeStyle = col; cx.lineWidth = w; cx.lineCap = "round";
    cx.beginPath(); cx.moveTo(A.x, A.y); cx.lineTo(B.x, B.y); cx.stroke(); cx.restore();
  }

  /* ---------------- 机器人底盘（行走）位姿 in world；机械臂/目标在其局部系 ---------------- */
  var BASE = { x: 0, y: 0, yaw: 0 };
  /* 机械臂基座在底盘系(前x/左y/上z)的【安装偏移】——实机臂不是装在底盘正中，
     是前移+架在升降柱上。⇒ 现用真机 URDF 链真值：
     base_footprint→base_link(0,0,.07)→lift_base(.095)→lift_joint(.0145,.0995,.1)→arm_base(-.0435,-.1625,.29)
     = (-0.058, -0.063, 0.555) + (0,0,lift)。 */
  var ARM_BASE = { x: -0.058, y: -0.063, z: 0.555 };   // URDF 真值（后5.8 / 右6.3 / 上55.5 cm）
  var LIFT = 0;                                     // 升降立柱高度(m, 0..0.44) = 第7自由度
  var ARM_YAW = 0;                                  // 臂基座朝向校正(rad)，对齐真机方位用
  var GRIP = 0.55;                                  // 夹爪开合 0(闭)..1(开)
  var PIVOT_Z = 0.50;                               // 相机旋转固定点高度（fitView 自动调）
  try { var _b0 = window.__a6baseInit; if (_b0) { ARM_BASE.x = +_b0.x || 0; ARM_BASE.y = +_b0.y || 0; ARM_BASE.z = +_b0.z || 0; } } catch (e) {}
  /* 对齐物理定位：底盘/臂基座跟随真机(state.pos map 位姿)。默认开；关=自由拖动模拟。 */
  window.arm6Follow = true;
  window.arm6Locked = false;   // 执行中锁定：禁用关节拖动/滑杆/高度尺
  var _a6lastRobotPos = null;
  function syncBaseFromRobot(){
    if(!window.arm6Follow) return;
    var R = window.arm6Robot;
    if(R && R.pos && R.pos.x != null){
      BASE.x = R.pos.x; BASE.y = R.pos.y; BASE.yaw = R.pos.yaw || 0; _a6lastRobotPos = R.pos;
    }
  }
  function W(p) { var c = Math.cos(BASE.yaw), s = Math.sin(BASE.yaw);
    var cy = Math.cos(ARM_YAW), sy = Math.sin(ARM_YAW);
    var ax = p.x * cy - p.y * sy, ay = p.x * sy + p.y * cy;
    var lx = ARM_BASE.x + ax, ly = ARM_BASE.y + ay, lz = ARM_BASE.z + LIFT + p.z;
    return { x: BASE.x + lx * c - ly * s, y: BASE.y + lx * s + ly * c, z: lz }; }
  function invW(p) { var c = Math.cos(BASE.yaw), s = Math.sin(BASE.yaw), dx = p.x - BASE.x, dy = p.y - BASE.y;
    var lx = dx * c + dy * s - ARM_BASE.x, ly = -dx * s + dy * c - ARM_BASE.y;
    var cy = Math.cos(-ARM_YAW), sy = Math.sin(-ARM_YAW);
    return { x: lx * cy - ly * sy, y: lx * sy + ly * cy, z: p.z - ARM_BASE.z - LIFT }; }
  /* 底盘系(不含臂偏移)投影——机身用 */
  function Wc(p) { var c = Math.cos(BASE.yaw), s = Math.sin(BASE.yaw); return { x: BASE.x + p.x * c - p.y * s, y: BASE.y + p.x * s + p.y * c, z: p.z }; }
  function ptC(p) { return proj3(Wc(p)); }
  /* 旋转固定点 = 臂基座正上方（相机绕它转，模型原地转，不漂） */
  function basePivot() { var p = Wc({ x: 0, y: 0, z: 0 }); return { x: p.x, y: p.y, z: PIVOT_Z + LIFT }; }
  /* 自动适应画面：把整条臂(含夹爪)缩放到画面内并居中 */
  function fitView() {
    FOCAL = cv.height * 3.6;
    function bbox() {
      CAM.tgt = basePivot(); VB = camBasis();
      var ps = fk(Q).pts.map(function (p) { return ptW(p); });
      var g = ptW({ x: 0, y: 0, z: 0 }); if (g) ps.push(g);
      ps = ps.filter(Boolean);
      var xs = ps.map(function (p) { return p.x; }), ys = ps.map(function (p) { return p.y; });
      return { w: Math.max.apply(null, xs) - Math.min.apply(null, xs), h: Math.max.apply(null, ys) - Math.min.apply(null, ys), cy: (Math.max.apply(null, ys) + Math.min.apply(null, ys)) / 2 };
    }
    var b = bbox();
    if (b.h > 1) CAM.dist = clamp(CAM.dist * Math.max(b.h / (cv.height * 0.85), b.w / (cv.width * 0.85)), 0.5, 18);
    for (var it = 0; it < 3; it++) { b = bbox(); if (Math.abs(b.cy - cv.height / 2) < 1) break; PIVOT_Z += (cv.height / 2 - b.cy) / FOCAL * CAM.dist; }
    draw();
  }
  window.arm6Fit = fitView;
  function ptW(p) { return proj3(W(p)); }
  function lineW(a, b, w, col, glow) { line3(W(a), W(b), w, col, glow); }

  /* 星点背景（一次性，分数坐标） */
  var STARS = []; for (var si = 0; si < 80; si++) STARS.push([Math.random(), Math.random(), 0.4 + Math.random() * 1.4, 0.15 + Math.random() * 0.5]);

  /* ---------------- 画布绘制 ---------------- */
  function draw() {
    if (!cx) return;
    syncBaseFromRobot();
    CAM.tgt = basePivot();   /* 旋转/缩放始终绕这个固定点(臂基座) */
    var W = cv.width, H = cv.height;
    VB = camBasis(); FOCAL = H * 3.6;
    // 背景（深空渐变 + 星点 + 地平辉光）
    var g = cx.createLinearGradient(0, 0, 0, H);
    g.addColorStop(0, "#f3f8fe"); g.addColorStop(0.55, "#e7f0fa"); g.addColorStop(1, "#dbe8f6");
    cx.fillStyle = g; cx.fillRect(0, 0, W, H);
    for (var k = 0; k < STARS.length; k++) { var s = STARS[k]; cx.fillStyle = "rgba(14,116,144," + s[3] + ")"; cx.fillRect(s[0] * W, s[1] * H * 0.62, s[2], s[2]); }
    var rg = cx.createRadialGradient(W * 0.5, H * 0.62, 10, W * 0.5, H * 0.62, W * 0.55);
    rg.addColorStop(0, "rgba(56,189,248,.10)"); rg.addColorStop(1, "rgba(56,189,248,0)");
    cx.fillStyle = rg; cx.fillRect(0, 0, W, H);

    drawFloor();
    drawBase();
    drawBody();
    drawKeepout();
    drawAxes();
    drawCalPoints();
    drawTrail();
    drawArm();
    drawJointDials();
    drawMotorLabels();
    drawTarget();
    drawCalibDial();
    updateHud();
  }

  function drawFloor() {
    // 透视网格
    var i;
    cx.save(); cx.lineWidth = 1;
    for (i = -6; i <= 6; i++) {
      var a1 = ptW({ x: i * 0.1, y: -0.6, z: 0 }), b1 = ptW({ x: i * 0.1, y: 0.6, z: 0 });
      if (a1 && b1) { cx.strokeStyle = "rgba(37,99,165,.14)"; cx.beginPath(); cx.moveTo(a1.x, a1.y); cx.lineTo(b1.x, b1.y); cx.stroke(); }
      var a2 = ptW({ x: -0.6, y: i * 0.1, z: 0 }), b2 = ptW({ x: 0.6, y: i * 0.1, z: 0 });
      if (a2 && b2) { cx.strokeStyle = "rgba(37,99,165,.14)"; cx.beginPath(); cx.moveTo(a2.x, a2.y); cx.lineTo(b2.x, b2.y); cx.stroke(); }
    }
    cx.restore();
    // 距离环（0.2/0.4/0.6/0.8）
    [0.2, 0.4, 0.6, 0.8].forEach(function (r) {
      cx.save(); cx.setLineDash([3, 4]); cx.strokeStyle = "rgba(8,145,178,.32)"; cx.lineWidth = 1;
      cx.beginPath(); var first = true;
      for (var a = 0; a <= 360; a += 12) { var p = ptW({ x: r * Math.cos(a * D2R), y: r * Math.sin(a * D2R), z: 0 }); if (!p) { first = true; continue; } if (first) { cx.moveTo(p.x, p.y); first = false; } else cx.lineTo(p.x, p.y); }
      cx.stroke(); cx.restore();
      var lp = ptW({ x: r, y: 0, z: 0 }); if (lp) { cx.fillStyle = "#0369a1"; cx.font = "10px ui-monospace,monospace"; cx.textAlign = "center"; cx.fillText(r.toFixed(1) + "m", lp.x, lp.y + 12); }
    });
    cx.textAlign = "left";
  }

  function drawBase() {
    var H = 0.07;   // 底盘厚度
    var c0 = proj3(Wc({ x: 0, y: 0, z: 0 })); if (!c0) return;
    var c1 = proj3(Wc({ x: 0, y: 0, z: H })) || c0;
    var rs = 0.30 * (c0.s || 300), ry = rs * 0.42;
    cx.save();
    // 侧壁（厚壁，深黑）
    cx.beginPath();
    cx.ellipse(c0.x, c0.y, rs, ry, 0, 0, Math.PI);
    cx.lineTo(c1.x - rs, c1.y);
    cx.ellipse(c1.x, c1.y, rs, ry, 0, Math.PI, 0, true);
    cx.closePath(); cx.fillStyle = "#191919"; cx.fill(); cx.strokeStyle = "#0d0d0d"; cx.lineWidth = 1; cx.stroke();
    // 顶面（米灰）
    cx.beginPath(); cx.ellipse(c1.x, c1.y, rs, ry, 0, 0, 7); cx.fillStyle = "#d4d0c8"; cx.fill(); cx.strokeStyle = "#9a958c"; cx.lineWidth = 1.5; cx.stroke();
    cx.beginPath(); cx.ellipse(c1.x, c1.y, rs * 0.6, ry * 0.6, 0, 0, 7); cx.strokeStyle = "rgba(154,149,140,.7)"; cx.lineWidth = 1; cx.stroke();
    // 前方激光雷达
    var lp = ptC({ x: 0.24, y: 0, z: H + 0.02 });
    if (lp) { cx.beginPath(); cx.arc(lp.x, lp.y, Math.max(3, 0.028 * (lp.s || 300)), 0, 7); cx.fillStyle = "#111"; cx.fill(); cx.strokeStyle = C_BLUE; cx.lineWidth = 1.2; cx.stroke(); }
    // 前向箭头
    var fwd = proj3(Wc({ x: 0.55, y: 0, z: H + 0.02 }));
    if (fwd) { cx.strokeStyle = "#0e7490"; cx.lineWidth = 2.5; cx.beginPath(); cx.moveTo(c1.x, c1.y); cx.lineTo(fwd.x, fwd.y); cx.stroke();
      var ang = Math.atan2(fwd.y - c1.y, fwd.x - c1.x); cx.beginPath(); cx.moveTo(fwd.x, fwd.y); cx.lineTo(fwd.x - 11 * Math.cos(ang - 0.4), fwd.y - 11 * Math.sin(ang - 0.4)); cx.lineTo(fwd.x - 11 * Math.cos(ang + 0.4), fwd.y - 11 * Math.sin(ang + 0.4)); cx.closePath(); cx.fillStyle = "#0e7490"; cx.fill(); }
    cx.fillStyle = "rgba(34,34,34,.92)"; cx.font = "11px ui-monospace,monospace"; cx.textAlign = "center";
    cx.fillText("移动底盘 · 拖动移动", c1.x, c1.y + ry + 16); cx.textAlign = "left";
    cx.restore();
  }

  function boxEdges(x0, x1, yA, z0, z1) {
    // 8 角点 → 12 边
    var C = [];
    for (var xi = 0; xi < 2; xi++) for (var yi = -1; yi <= 1; yi += 2) for (var zi = 0; zi < 2; zi++)
      C.push({ x: xi ? x1 : x0, y: yi * yA, z: zi ? z1 : z0 });
    var E = [[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]];
    return E.map(function (e) { return [C[e[0]], C[e[1]]]; });
  }
  function circle3(r, z, col, n) {
    cx.save(); cx.strokeStyle = col; cx.lineWidth = 1.2; cx.setLineDash([4, 4]); cx.beginPath(); var first = true;
    for (var a = 0; a <= 360; a += (360 / (n || 40))) { var p = ptW({ x: r * Math.cos(a * D2R), y: r * Math.sin(a * D2R), z: z }); if (!p) { first = true; continue; } if (first) { cx.moveTo(p.x, p.y); first = false; } else cx.lineTo(p.x, p.y); }
    cx.stroke(); cx.restore();
  }

  function drawKeepout() {
    // 基座禁入圆柱
    circle3(SAFE.rMin, 0, "rgba(220,38,38,.7)"); circle3(SAFE.rMin, SAFE.zMax, "rgba(220,38,38,.3)");
    for (var a = 0; a < 360; a += 45) { var x = SAFE.rMin * Math.cos(a * D2R), y = SAFE.rMin * Math.sin(a * D2R); lineW({ x: x, y: y, z: 0 }, { x: x, y: y, z: SAFE.zMax }, 1, "rgba(220,38,38,.3)"); }
    // 胸口屏幕禁入盒
    var c = SAFE.chest;
    boxEdges(0, c.xFront, c.yAbs, c.zLo, c.zHi).forEach(function (e) { lineW(e[0], e[1], 1, "rgba(220,38,38,.5)"); });
    var cp = ptW({ x: c.xFront, y: c.yAbs, z: c.zHi }); if (cp) { cx.fillStyle = "#dc2626"; cx.font = "11px ui-monospace,monospace"; cx.fillText("胸口屏幕禁入区", cp.x - 50, cp.y - 4); }
  }

  function drawAxes() {
    var O = { x: 0, y: 0, z: 0 };
    var AXd = [[{ x: 0.5, y: 0, z: 0 }, "#0e7490", "前 x"], [{ x: 0, y: 0.5, z: 0 }, "#15803d", "左 y"], [{ x: 0, y: 0, z: 0.6 }, "#4338ca", "上 z"]];
    AXd.forEach(function (ax) { lineW(O, ax[0], 2, ax[1], 8); var p = ptW(ax[0]); if (p) { cx.fillStyle = ax[1]; cx.font = "bold 11px ui-monospace,monospace"; cx.fillText(ax[2], p.x + 4, p.y); } });
    // 罗盘方位
    cx.font = "10px ui-monospace,monospace"; cx.textAlign = "center";
    [["前", 0], ["左", 90], ["右", -90], ["后", 180]].forEach(function (d) { var a = d[1] * D2R, p = ptW({ x: 0.9 * Math.cos(a), y: 0.9 * Math.sin(a), z: 0.02 }); if (p) { cx.fillStyle = "#334155"; cx.fillText(d[0], p.x, p.y); } });
    cx.textAlign = "left";
    // 高度尺（x=0.9 竖线）
    lineW({ x: 0.9, y: 0, z: 0 }, { x: 0.9, y: 0, z: 1.0 }, 1, "rgba(71,85,105,.5)");
    cx.font = "10px ui-monospace,monospace";
    for (var z = 0.2; z <= 1.001; z += 0.2) { var p = ptW({ x: 0.9, y: 0, z: z }); if (p) { cx.fillStyle = "#475569"; cx.fillText("z" + z.toFixed(1), p.x + 4, p.y); } }
  }

  var CPS = [[0.45, 0, 0.40, "正前"], [0.45, 0, 0.70, "前上"], [0.30, 0, 0.40, "前近"], [0.70, 0, 0.40, "前远"], [0.10, 0.45, 0.40, "左"], [0.10, -0.45, 0.40, "右"]];
  function drawCalPoints() {
    cx.textAlign = "center"; cx.font = "10px ui-monospace,monospace";
    CPS.forEach(function (q) { var p = ptW({ x: q[0], y: q[1], z: q[2] }); if (!p) return; cx.save(); cx.shadowColor = "#0e7490"; cx.shadowBlur = 8; cx.fillStyle = "#0369a1"; cx.beginPath(); cx.arc(p.x, p.y, 2.5, 0, 7); cx.fill(); cx.restore(); cx.fillStyle = "#0e7490"; cx.fillText(q[3], p.x, p.y - 6); });
    cx.textAlign = "left";
  }

  var trailW = [];
  function drawTrail() {
    var f = fk(Q); var wp = W(f.pts[N + 1]);
    var last = trailW[trailW.length - 1];
    if (!last || Math.hypot(last.x - wp.x, last.y - wp.y, last.z - wp.z) > 0.006) { trailW.push(wp); if (trailW.length > 160) trailW.shift(); }
    if (trailW.length < 2) return;
    cx.save(); cx.strokeStyle = "rgba(14,116,144,.45)"; cx.lineWidth = 1.5; cx.beginPath(); var st = false;
    for (var i = 0; i < trailW.length; i++) { var sc = proj3(trailW[i]); if (!sc) { st = false; continue; } if (!st) { cx.moveTo(sc.x, sc.y); st = true; } else cx.lineTo(sc.x, sc.y); }
    cx.stroke(); cx.restore();
  }
  /* —— 实机配色（据管理员实测描述）—— */
  var C_LINK = "#d4d0c8", C_LINK_ED = "#9a958c";   // 连杆：哑光米灰
  var C_JOINT = "#222222", C_JOINT_ED = "#0c0c0c"; // 关节壳：哑光炭黑
  var C_BLUE = "#0099ff";                            // 品牌亮蓝点缀
  var JCOL = ["#dc2626","#ea580c","#b45309","#16a34a","#0e7490","#9333ea"];   // 🎨 J1..J6 区分色
  var JRTXT = ["±179°","±179°","-169~189°","±179°","±179°","±179°"];            // 可动范围
  var C_COL = "#d4d0c8", C_COL_ED = "#9a958c";     // 升降柱米灰
  /* 凸包 + 填充多边形（画机身剪影用） */
  function hull(pts) { if (pts.length < 3) return pts; pts = pts.slice().sort(function (a, b) { return a.x - b.x || a.y - b.y; });
    var cr = function (o, a, b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }, lo = [], up = [];
    for (var i = 0; i < pts.length; i++) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], pts[i]) <= 0) lo.pop(); lo.push(pts[i]); }
    for (var i = pts.length - 1; i >= 0; i--) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], pts[i]) <= 0) up.pop(); up.push(pts[i]); }
    lo.pop(); up.pop(); return lo.concat(up); }
  function poly(p, fill, stroke, lw, alpha) { if (!p || !p.length) return; cx.save(); if (alpha != null) cx.globalAlpha = alpha;
    cx.beginPath(); cx.moveTo(p[0].x, p[0].y); for (var i = 1; i < p.length; i++) cx.lineTo(p[i].x, p[i].y); cx.closePath();
    if (fill) { cx.fillStyle = fill; cx.fill(); } if (stroke) { cx.strokeStyle = stroke; cx.lineWidth = lw || 1; cx.stroke(); } cx.restore(); }
  function boxSilC(cx, cy, hx, hy, z0, z1, fill, edge) { var cs = [];
    for (var a = 0; a < 2; a++) for (var b = 0; b < 2; b++) for (var c = 0; c < 2; c++) { var p = ptC({ x: cx + (a ? hx : -hx), y: cy + (b ? hy : -hy), z: (c ? z1 : z0) }); if (p) cs.push(p); }
    poly(hull(cs), fill, edge, 1.5); }
  function boxSil(hx, hy, z0, z1, fill, edge) { boxSilC(0, 0, hx, hy, z0, z1, fill, edge); }
  /* 水平半圆盘（底盘系，a0..a1 角度） */
  function semidisk(r, z, a0, a1, fill, edge) { cx.beginPath(); var first = true;
    for (var a = a0; a <= a1 + 0.01; a += 6) { var p = ptC({ x: r * Math.cos(a * Math.PI / 180), y: r * Math.sin(a * Math.PI / 180), z: z }); if (!p) { first = true; continue; } if (first) { cx.moveTo(p.x, p.y); first = false; } else cx.lineTo(p.x, p.y); }
    cx.closePath(); cx.fillStyle = fill; cx.fill(); if (edge) { cx.strokeStyle = edge; cx.lineWidth = 1.2; cx.stroke(); } }
  /* 机身（底盘系）：升降立柱(随LIFT伸缩) + 正面竖屏/黑饰条 + 前置托盘 + 顶部头部模组 + 左壁J1安装座 */
  function drawBody() {
    var hx = 0.045, hy = 0.057, z0 = 0.07, z1 = ARM_BASE.z + 0.02 + LIFT;   // 立柱：顶到臂基座高度(URDF)
    var cy = -(hy + ARM_BASE.y);   // 立柱中心 y：使『立柱→臂』连接杆中点 y=0（正对底盘中心）
    // 立柱（偏置在底盘中心一侧）
    boxSilC(0, cy, hx, hy, z0, z1, C_COL, C_COL_ED);
    // 正面：黑屏 + 竖向黑饰条
    var fv = [{ x: hx, y: cy - hy, z: z0 }, { x: hx, y: cy + hy, z: z0 }, { x: hx, y: cy + hy, z: z1 }, { x: hx, y: cy - hy, z: z1 }].map(ptC);
    if (fv.every(function (p) { return p; })) {
      var cen = { x: (fv[0].x + fv[1].x + fv[2].x + fv[3].x) / 4, y: (fv[0].y + fv[1].y + fv[2].y + fv[3].y) / 4 };
      poly(fv, "#191919", null, 0);
      poly(fv.map(function (p) { return { x: cen.x + (p.x - cen.x) * 0.6, y: cen.y + (p.y - cen.y) * 0.82 }; }), "#0e2230", "#15455c", 1);
      var dk = [{ x: hx, y: cy - hy * 0.99, z: z0 }, { x: hx, y: cy - hy * 0.6, z: z0 }, { x: hx, y: cy - hy * 0.6, z: z1 }, { x: hx, y: cy - hy * 0.99, z: z1 }].map(ptC);
      if (dk.every(function (p) { return p; })) poly(dk, "#222222", null, 0, 0.92);
    }
    // 半圆形置物托盘：紧贴行走圆盘上方（下移）
    semidisk(0.11, 0.09 + LIFT, 180, 360, "#c9c6bf", "#9a958c");
    // 顶部头部模组 + 两头相机（随升降、随立柱偏置）
    boxSilC(0, cy, hx * 1.15, hy * 1.7, z1, z1 + 0.12, C_COL, C_COL_ED);
    [cy + hy * 0.95, cy - hy * 0.95].forEach(function (yy) { var p = ptC({ x: hx * 1.15 + 0.002, y: yy, z: z1 + 0.055 });
      if (p) { cx.save(); cx.beginPath(); cx.arc(p.x, p.y, Math.max(2, 0.02 * (p.s || 300)), 0, 7); cx.fillStyle = "#111"; cx.fill(); cx.strokeStyle = C_BLUE; cx.lineWidth = 1.5; cx.stroke(); cx.restore(); } });
    // 水平弯头：立柱壁 →(水平) J1（无竖直段）
    var j1e = fk(Q).pts[1];
    var wallY = cy + (ARM_BASE.y >= 0 ? hy : -hy);
    var A = ptC({ x: ARM_BASE.x, y: wallY, z: ARM_BASE.z + LIFT + j1e.z });
    var B = ptW(j1e);
    if (A && B) {
      var _cd = Math.hypot(A.x - B.x, A.y - B.y);
      if (_cd < 22) { var _ck = (22 - _cd) / (_cd || 1); A = { x: A.x + (A.x - B.x) * _ck, y: A.y + (A.y - B.y) * _ck, s: A.s }; }
      beam(A, B, Math.max(11, 0.06 * ((A && A.s) || 300)), Math.max(11, 0.06 * ((B && B.s) || 300)), C_JOINT, C_JOINT_ED, true);
    }
  }
  /* 实体“梁”：屏幕空间里画一根有宽度的实心连杆（带高光） */
  function beam(A, B, wA, wB, body, edge, hi) {
    if (!A || !B) return;
    var dx = B.x - A.x, dy = B.y - A.y, L = Math.hypot(dx, dy) || 1, nx = -dy / L, ny = dx / L;
    var w = (wA + wB) / 2;
    cx.save(); cx.lineCap = "round"; cx.lineJoin = "round";
    if (edge) { cx.strokeStyle = edge; cx.lineWidth = w + 1.6; cx.beginPath(); cx.moveTo(A.x, A.y); cx.lineTo(B.x, B.y); cx.stroke(); }
    cx.strokeStyle = body; cx.lineWidth = w; cx.beginPath(); cx.moveTo(A.x, A.y); cx.lineTo(B.x, B.y); cx.stroke();
    if (hi) { var o = w * 0.22; cx.strokeStyle = "rgba(255,255,255,.18)"; cx.lineWidth = Math.max(1, w * 0.16); cx.beginPath(); cx.moveTo(A.x + nx * o, A.y + ny * o); cx.lineTo(B.x + nx * o, B.y + ny * o); cx.stroke(); }
    cx.restore();
  }
  /* 关节：沿【旋转轴】的圆柱（能看出电机朝向，如 J1 竖直）。ax=轴在屏幕上的方向 */
  function housing(p, r, ax) {
    if (!p) return; cx.save();
    var dx = 0, dy = 0;
    if (ax && (ax.x || ax.y)) { var L = Math.hypot(ax.x, ax.y) || 1; dx = ax.x / L; dy = ax.y / L; }
    var hl = Math.max(r * 0.95, 6);
    cx.lineCap = "round"; cx.lineJoin = "round";
    cx.strokeStyle = C_JOINT_ED; cx.lineWidth = r * 2 + 3;
    cx.beginPath(); cx.moveTo(p.x - dx * hl, p.y - dy * hl); cx.lineTo(p.x + dx * hl, p.y + dy * hl); cx.stroke();
    cx.strokeStyle = C_JOINT; cx.lineWidth = r * 2;
    cx.beginPath(); cx.moveTo(p.x - dx * hl, p.y - dy * hl); cx.lineTo(p.x + dx * hl, p.y + dy * hl); cx.stroke();
    var ex = p.x + dx * hl, ey = p.y + dy * hl;   // 朝相机端面
    cx.beginPath(); cx.arc(ex, ey, r, 0, 7); cx.fillStyle = C_JOINT; cx.fill();
    cx.strokeStyle = "rgba(255,255,255,.14)"; cx.lineWidth = 1.5; cx.stroke();
    cx.beginPath(); cx.arc(ex, ey, Math.max(2, r * 0.28), 0, 7); cx.fillStyle = C_BLUE; cx.fill();
    cx.restore();
  }
  /* 关节旋转轴在屏幕上的方向(用于画出电机朝向) */
  function axisScr(f, j) {
    var base = f.pts[j], ax = f.axes[j - 1]; if (!base || !ax) return null;
    var w0 = ptW(base), w1 = ptW({ x: base.x + ax.x * 0.06, y: base.y + ax.y * 0.06, z: base.z + ax.z * 0.06 });
    if (!w0 || !w1) return null; return { x: w1.x - w0.x, y: w1.y - w0.y };
  }
  /* 90° 折角件：从 A 沿垂直方向折出，再到 B */
  function elbowLink(A, B, ax, w) {
    if (!A || !B) return;
    var dx = B.x - A.x, dy = B.y - A.y, L = Math.hypot(dx, dy) || 1;
    var ux = dx / L, uy = dy / L;
    var px = -uy, py = ux;
    if (ax && (ax.x || ax.y)) { px = ax.x - (ax.x * ux + ax.y * uy) * ux; py = ax.y - (ax.x * ux + ax.y * uy) * uy; var pl = Math.hypot(px, py) || 1; px /= pl; py /= pl; }
    var d = Math.min(L * 0.55, 26);
    var C = { x: A.x + px * d, y: A.y + py * d };
    cx.save(); cx.lineCap = "round"; cx.lineJoin = "round";
    [[A, C], [C, B]].forEach(function (seg) {
      var p0 = seg[0], p1 = seg[1];
      if (C_LINK_ED) { cx.strokeStyle = C_LINK_ED; cx.lineWidth = w + 2; cx.beginPath(); cx.moveTo(p0.x, p0.y); cx.lineTo(p1.x, p1.y); cx.stroke(); }
      cx.strokeStyle = C_LINK; cx.lineWidth = w; cx.beginPath(); cx.moveTo(p0.x, p0.y); cx.lineTo(p1.x, p1.y); cx.stroke();
    });
    cx.restore();
  }
  /* 双折角件(Z)：A→折出→沿→折回→B（两个 90°） */
  function doubleElbow(A, B, ax, w) {
    if (!A || !B) return;
    var dx = B.x - A.x, dy = B.y - A.y, L = Math.hypot(dx, dy) || 1;
    var ux = dx / L, uy = dy / L, px = -uy, py = ux;
    if (ax && (ax.x || ax.y)) { px = ax.x - (ax.x * ux + ax.y * uy) * ux; py = ax.y - (ax.x * ux + ax.y * uy) * uy; var pl = Math.hypot(px, py) || 1; px /= pl; py /= pl; }
    var d = Math.min(L * 0.5, 24);
    var C1 = { x: A.x + px * d, y: A.y + py * d }, C2 = { x: B.x + px * d, y: B.y + py * d };
    cx.save(); cx.lineCap = "round"; cx.lineJoin = "round";
    [[A, C1], [C1, C2], [C2, B]].forEach(function (seg) {
      var p0 = seg[0], p1 = seg[1];
      if (C_LINK_ED) { cx.strokeStyle = C_LINK_ED; cx.lineWidth = w + 2; cx.beginPath(); cx.moveTo(p0.x, p0.y); cx.lineTo(p1.x, p1.y); cx.stroke(); }
      cx.strokeStyle = C_LINK; cx.lineWidth = w; cx.beginPath(); cx.moveTo(p0.x, p0.y); cx.lineTo(p1.x, p1.y); cx.stroke();
    });
    cx.restore();
  }
  function drawArm() {
    var f = fk(Q), P = f.pts.map(ptW);
    // (臂基座→J1 的竖直过渡不再画；由 drawBody 的水平弯头直接接到 J1)
    // 连杆（米灰盒式梁）：i=1 J1→J2 弯头 / i=2 J2→J3 【长杆·粗】 / i=3 J3→J4 弯头 / i=4 J4→J5 【小连杆·细】 / i=5 J5→J6 弯头
    var WID = [0, 0.052, 0.088, 0.052, 0.046, 0.052];
    for (var i = 1; i < N; i++) {
      var s1 = P[i], s2 = P[i + 1]; if (!s1 || !s2) continue;
      var mid = ((s1.s || 300) + (s2.s || 300)) / 2;
      var w = clamp((WID[i] || 0.052) * mid, 5, 74);
      if (i === 2 || i === 4) beam(s1, s2, w, w * 0.88, C_LINK, C_LINK_ED, true);   // 长杆/小连杆 = 直杆
      else if (i === 3) doubleElbow(s1, s2, axisScr(f, i), w);                        // J3-J4 = 双 90° 折角
      else elbowLink(s1, s2, axisScr(f, i), w);                                       // J1-J2 / J5-J6 = 单 90° 折角
    }
    // 关节（沿旋转轴的圆柱，φ~80mm，蓝色轴心）
    for (var j = 1; j <= N; j++) {
      var pj = P[j]; if (!pj) continue;
      housing(pj, clamp(0.040 * (pj.s || 300), 6, 40), axisScr(f, j));
    }
    // 末端：腕法兰 + 两指夹爪（随 J6 旋转 + 开合）
    var wr = P[N], tip = P[N + 1];
    if (wr && tip) {
      var sc = (tip.s || 300);
      var wrE = f.pts[N], tipE = f.pts[N + 1], T6 = f.T;
      var axU = unit(sub(tipE, wrE));                    // 腕/工具轴(世界)
      var sp3 = unit({ x: T6[4], y: T6[5], z: T6[6] });  // 手指张开方向 = 末帧 Y（随 J6 转）
      var pA = ptW(wrE), pU = ptW({ x: wrE.x + axU.x * 0.05, y: wrE.y + axU.y * 0.05, z: wrE.z + axU.z * 0.05 }), pN = ptW({ x: wrE.x + sp3.x * 0.05, y: wrE.y + sp3.y * 0.05, z: wrE.z + sp3.z * 0.05 });
      var ux = tip.x - wr.x, uy = tip.y - wr.y, uL = Math.hypot(ux, uy) || 1; ux /= uL; uy /= uL;
      var nx = -uy, ny = ux;
      if (pA && pU && pN) {
        ux = pU.x - pA.x; uy = pU.y - pA.y; var uL2 = Math.hypot(ux, uy) || 1; ux /= uL2; uy /= uL2;
        nx = pN.x - pA.x; ny = pN.y - pA.y;
        var pr = nx * ux + ny * uy; nx -= pr * ux; ny -= pr * uy;
        var nL = Math.hypot(nx, ny) || 1; nx /= nL; ny /= nL;
      }
      beam(wr, tip, Math.max(10, 0.055 * sc), Math.max(9, 0.05 * sc), C_JOINT, C_JOINT_ED, false);   // 腕/掌块
      var gapIn = Math.max(3, 0.006 * sc);
      var gapOut = Math.max(5, (0.008 + GRIP * 0.032) * sc);   // 开合 → 指尖外张距离
      var flen = Math.max(18, 0.11 * sc), fwf = Math.max(4, 0.016 * sc);
      for (var sgn = -1; sgn <= 1; sgn += 2) {
        var b0 = { x: tip.x + nx * sgn * gapIn, y: tip.y + ny * sgn * gapIn };
        var b1 = { x: tip.x + ux * flen + nx * sgn * gapOut, y: tip.y + uy * flen + ny * sgn * gapOut };
        beam(b0, b1, fwf, Math.max(3, fwf * 0.82), C_JOINT, C_JOINT_ED, false);   // 指体
        beam({ x: b0.x - nx * sgn * 2, y: b0.y - ny * sgn * 2 }, { x: b1.x - nx * sgn * 2, y: b1.y - ny * sgn * 2 }, Math.max(2, 0.008 * sc), Math.max(2, 0.007 * sc), "#b9b9b9", null, false);  // 防滑垫
        cx.save(); cx.beginPath(); cx.arc(b1.x, b1.y, Math.max(2, 0.007 * sc), 0, 7); cx.fillStyle = C_BLUE; cx.fill(); cx.restore();  // 指尖蓝点
      }
    }
  }

  /* 电机标注：每个关节旁标 J1..J6（可开关） */
  var showMotorLabels = true;
  window.arm6MotorLabels = function (v) { if (v !== undefined) { showMotorLabels = !!v; draw(); } return showMotorLabels; };
  function drawMotorLabels() {
    if (!showMotorLabels) return;
    var f = fk(Q);
    for (var i = 1; i <= N; i++) {
      var p = proj3(W(f.pts[i])); if (!p) continue;
      var r = clamp(0.040 * (p.s || 300), 6, 40);
      var jc = JCOL[i - 1];
      var label = "J" + i + " " + JRTXT[i - 1] + (i === 6 ? "·爪转" : "");
      cx.save(); cx.font = "bold 11px ui-monospace,monospace";
      var w = cx.measureText(label).width + 10, h = 15;
      var tx = p.x + r + 6 + i * 9, ty = p.y - r - 6 - i * 5;
      cx.strokeStyle = jc; cx.lineWidth = 1.5; cx.beginPath(); cx.moveTo(p.x + r * 0.7, p.y - r * 0.7); cx.lineTo(tx, ty + h / 2); cx.stroke();
      cx.fillStyle = "rgba(255,255,255,.92)"; rrect(tx, ty, w, h, 4); cx.fill();
      cx.strokeStyle = jc; cx.lineWidth = 1.5; cx.stroke();
      cx.fillStyle = jc; cx.textAlign = "left"; cx.textBaseline = "middle";
      cx.fillText(label, tx + 5, ty + h / 2 + 0.5);
      cx.restore();
    }
    cx.textAlign = "left"; cx.textBaseline = "alphabetic";
  }
  function drawCalibDial() {
    var s = (LEARN.calib && LEARN.calib.samples) || null;
    var W = cv.width, H = cv.height;
    var cxd = W - 86, cyd = 128, R = 58;
    cx.save();
    cx.fillStyle = "rgba(255,255,255,.45)"; cx.strokeStyle = "rgba(14,116,144,.35)"; cx.lineWidth = 1;
    cx.beginPath(); cx.arc(cxd, cyd, R + 22, 0, 7); cx.fill(); cx.stroke();
    cx.strokeStyle = "rgba(14,116,144,.5)"; cx.beginPath(); cx.arc(cxd, cyd, R, 0, 7); cx.stroke();
    cx.textAlign = "center";
    cx.fillStyle = "#0e5f78"; cx.font = "9px ui-monospace,monospace";
    cx.fillText("标定偏差° (前/左/右)", cxd, cyd - R - 8);
    cx.font = "8px ui-monospace,monospace"; cx.fillStyle = "#334155";
    cx.fillText("前", cxd, cyd - R + 10); cx.fillText("左", cxd - R + 9, cyd + 3); cx.fillText("右", cxd + R - 9, cyd + 3);
    if (s) {
      s.forEach(function (o) {
        var phi = o.bearing * D2R, dx = -Math.sin(phi), dy = -Math.cos(phi);
        var x0 = cxd + dx * R * 0.7, y0 = cyd + dy * R * 0.7, x1 = cxd + dx * R, y1 = cyd + dy * R;
        var e = o.err, col = Math.abs(e) < 4 ? "#16a34a" : Math.abs(e) < 10 ? "#d97706" : "#dc2626";
        cx.strokeStyle = col; cx.lineWidth = 2; cx.beginPath(); cx.moveTo(x0, y0); cx.lineTo(x1, y1); cx.stroke();
        cx.fillStyle = col; cx.font = "8px ui-monospace,monospace";
        cx.fillText((e > 0 ? "+" : "") + e.toFixed(0), cxd + dx * (R + 12), cyd + dy * (R + 12) + 3);
      });
      cx.fillStyle = "#0e5f78"; cx.font = "8px ui-monospace,monospace";
      cx.fillText("n=" + (s[0].n || "?") + "轮", cxd, cyd + 3);
    }
    cx.fillStyle = "#0e7490"; cx.beginPath(); cx.arc(cxd, cyd, 2, 0, 7); cx.fill();
    cx.restore(); cx.textAlign = "left";
    // 数字表(方位:偏差°)
    cx.save();
    cx.fillStyle = "rgba(255,255,255,.5)"; cx.strokeStyle = "rgba(14,116,144,.35)"; cx.lineWidth = 1;
    var nx = W - 204;
    cx.beginPath(); cx.rect(nx, 214, 202, 54); cx.fill(); cx.stroke();
    cx.fillStyle = "#0e5f78"; cx.font = "8px ui-monospace,monospace"; cx.textAlign = "left";
    cx.fillText("方位:偏差(°)  5列 右→左", nx + 6, 225);
    if (s) {
      s.forEach(function (o, i) {
        var col = i % 5, row = (i / 5) | 0, x = nx + 6 + col * 39, y = 236 + row * 11;
        var e = o.err, c = Math.abs(e) < 4 ? "#16a34a" : Math.abs(e) < 10 ? "#d97706" : "#dc2626";
        cx.fillStyle = c; cx.fillText(o.bearing + ":" + (e > 0 ? "+" : "") + e.toFixed(0), x, y);
      });
    }
    cx.restore(); cx.textAlign = "left";
  }

  var selJoint = 0;
  function rrect(x, y, w, h, r) { cx.beginPath(); if (cx.roundRect) { cx.roundRect(x, y, w, h, r); } else { cx.rect(x, y, w, h); } }
  function drawJointDials() {
    var f = fk(Q);
    for (var i = 1; i <= 6; i++) {
      var p = proj3(W(f.pts[i])); if (!p) continue;
      var rd = clamp(0.03 * p.s, 9, 30);
      var a = Q[i - 1] * R2D, lim = LIM[i - 1], pct = a / lim, sel = (selJoint === i);
      var lbf = Math.max(11, Math.round(rd * 0.6)), amf = Math.max(12, Math.round(rd * 0.62));
      var bw = Math.max(50, Math.round(rd * 2 + 24)), bh = Math.round(rd * 2 + 40);
      cx.save(); cx.translate(p.x, p.y);
      // 半透明底
      rrect(-bw / 2, -bh / 2, bw, bh, Math.min(10, Math.round(rd * 0.5)));
      cx.fillStyle = sel ? "rgba(255,255,255,.74)" : "rgba(255,255,255,.5)";
      cx.strokeStyle = sel ? "#dc2626" : "rgba(14,116,144,.35)"; cx.lineWidth = sel ? 2 : 1; cx.fill(); cx.stroke();
      // 量程弧 + 指针
      cx.beginPath(); cx.arc(0, 0, rd * 0.74, -1.25 * Math.PI, 0.25 * Math.PI); cx.strokeStyle = "rgba(14,116,144,.5)"; cx.lineWidth = 1.5; cx.stroke();
      var ang = -0.5 * Math.PI + pct * (0.75 * Math.PI);
      var nc = Math.abs(pct) > 0.85 ? "#dc2626" : Math.abs(pct) > 0.6 ? "#d97706" : "#0891b2";
      cx.beginPath(); cx.moveTo(0, 0); cx.lineTo(Math.cos(ang) * rd * 0.7, Math.sin(ang) * rd * 0.7); cx.strokeStyle = nc; cx.lineWidth = 2.5; cx.stroke();
      cx.beginPath(); cx.arc(0, 0, 2, 0, 7); cx.fillStyle = nc; cx.fill();
      // 文字(在底板内)
      cx.textAlign = "center";
      cx.fillStyle = JCOL[i - 1]; cx.font = "bold " + lbf + "px ui-monospace,monospace";
      cx.fillText("j" + i, 0, -bh / 2 + lbf + 2);
      cx.fillStyle = nc; cx.font = "bold " + amf + "px ui-monospace,monospace";
      cx.fillText(a.toFixed(0) + "\u00b0", 0, bh / 2 - 5);
      if (sel) { cx.fillStyle = "#334155"; cx.font = (amf - 3) + "px ui-monospace,monospace"; cx.fillText("限" + Math.round(pct * 100) + "%", 0, bh / 2 - 5 - amf); }
      cx.restore();
    }
  }

  var tPhase = 0;
  function drawTarget() {
    var t = ptW(TGT); if (!t) return;
    tPhase += 0.09;
    var r1 = 12 + Math.sin(tPhase) * 2, r2 = 20 + Math.cos(tPhase) * 3;
    cx.save();
    cx.shadowColor = "#059669"; cx.shadowBlur = 14;
    cx.strokeStyle = "#059669"; cx.lineWidth = 2; cx.beginPath(); cx.arc(t.x, t.y, r1, 0, 7); cx.stroke();
    cx.strokeStyle = "rgba(5,150,105,.5)"; cx.lineWidth = 1; cx.beginPath(); cx.arc(t.x, t.y, r2, 0, 7); cx.stroke();
    cx.beginPath(); cx.moveTo(t.x - r2, t.y); cx.lineTo(t.x - r1, t.y); cx.moveTo(t.x + r1, t.y); cx.lineTo(t.x + r2, t.y);
    cx.moveTo(t.x, t.y - r2); cx.lineTo(t.x, t.y - r1); cx.moveTo(t.x, t.y + r1); cx.lineTo(t.x, t.y + r2); cx.stroke();
    cx.restore();
    // 末端→目标 连线（虚）
    var f = fk(Q), e = ptW(f.pts[N + 1]);
    if (e) { cx.save(); cx.setLineDash([4, 4]); cx.strokeStyle = "rgba(5,150,105,.35)"; cx.lineWidth = 1; cx.beginPath(); cx.moveTo(e.x, e.y); cx.lineTo(t.x, t.y); cx.stroke(); cx.restore(); }
  }

  /* ---------------- 多维数据 HUD ---------------- */
  function quatOf(T) {
    var m00 = T[0], m01 = T[1], m02 = T[2], m10 = T[4], m11 = T[5], m12 = T[6], m20 = T[8], m21 = T[9], m22 = T[10];
    var tr = m00 + m11 + m22, qw, qx, qy, qz, s;
    if (tr > 0) { s = Math.sqrt(tr + 1) * 2; qw = 0.25 * s; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s; }
    else if (m00 > m11 && m00 > m22) { s = Math.sqrt(1 + m00 - m11 - m22) * 2; qw = (m21 - m12) / s; qx = 0.25 * s; qy = (m01 + m10) / s; qz = (m02 + m20) / s; }
    else if (m11 > m22) { s = Math.sqrt(1 + m11 - m00 - m22) * 2; qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = 0.25 * s; qz = (m12 + m21) / s; }
    else { s = Math.sqrt(1 + m22 - m00 - m11) * 2; qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = 0.25 * s; }
    return { w: qw, x: qx, y: qy, z: qz };
  }
  function rpyOf(T) {
    var r00 = T[0], r10 = T[4], r20 = T[8], r21 = T[9], r22 = T[10];
    var pitch = Math.atan2(-r20, Math.sqrt(r00 * r00 + r10 * r10));
    var roll = Math.atan2(r21, r22), yaw = Math.atan2(r10, r00);
    return { r: roll * R2D, p: pitch * R2D, y: yaw * R2D };
  }
  function updateHud() {
    var hud = $("arm6Hud"); if (!hud) return;
    var f = fk(Q), e = f.pts[N + 1];
    var deg = Q.map(function (v) { return (v * R2D).toFixed(1); });
    var rpy = rpyOf(f.T); var q = quatOf(f.T);
    var rxy = Math.sqrt(e.x * e.x + e.y * e.y);
    var az = Math.atan2(e.y, e.x) * R2D, el = Math.atan2(e.z, rxy) * R2D;
    var tool = unit({ x: f.T[2], y: f.T[6], z: f.T[10] });
    var errv = len(sub(e, TGT));
    var bear = Math.atan2(TGT.y, TGT.x) * R2D;
    var mp = (typeof window.arm6MapOf === "function") ? window.arm6MapOf(TGT.x, TGT.y, TGT.z) : null;
    var cal = (ARM_BASE.x || ARM_BASE.y || ARM_BASE.z) ? ("物理模型·臂基座偏移 前" + ARM_BASE.x.toFixed(2) + " 左" + ARM_BASE.y.toFixed(2) + " 上" + ARM_BASE.z.toFixed(2) + " m（方位表停用）") : ((LEARN.calib && LEARN.calib.samples && LEARN.calib.samples.length) ? ("按方位表 " + LEARN.calib.samples.length + " 点 · 均值偏差 " + LEARN.calib.meanErrDeg + "°") : "未校准");
    var L = [
      "◤ 6-DOF ARM · TELEMETRY ◢",
      "底盘 x " + BASE.x.toFixed(2) + "  y " + BASE.y.toFixed(2) + "  yaw " + (BASE.yaw * R2D).toFixed(0) + "°  升降 " + (LIFT * 100).toFixed(0) + "cm"  + (window.arm6Follow ? ("  [📍跟随真机" + (_a6lastRobotPos ? "" : "·待位姿") + "]") : "  [自由模拟]"),
      "视野 az " + (CAM.az * R2D).toFixed(0) + "°  el " + (CAM.el * R2D).toFixed(0) + "°  d " + CAM.dist.toFixed(2) + "  " + (window.__a6fps || 0) + "fps",
      "关节(°)  j1 " + deg[0] + "  j2 " + deg[1] + "  j3 " + deg[2],
      "         j4 " + deg[3] + "  j5 " + deg[4] + "  j6 " + deg[5],
      "末端 局部  x " + e.x.toFixed(3) + "  y " + e.y.toFixed(3) + "  z " + e.z.toFixed(3) + " m",
      "姿态 RPY  " + rpy.r.toFixed(1) + "° / " + rpy.p.toFixed(1) + "° / " + rpy.y.toFixed(1) + "°",
      "极坐标  水平半径 r " + rxy.toFixed(3) + " m · 仰角 " + el.toFixed(1) + "°",
      "方位角  相对机身 " + az.toFixed(1) + "°",
      "工具轴  (" + tool.x.toFixed(2) + ", " + tool.y.toFixed(2) + ", " + tool.z.toFixed(2) + ")",
      "姿态四元数  w " + q.w.toFixed(3) + " x " + q.x.toFixed(3) + " y " + q.y.toFixed(3) + " z " + q.z.toFixed(3),
      "工作空间  距基座 " + len(e).toFixed(3) + " m   最大臂展 ~0.99 m (mvp03)",
      "—— 模型 · 真机 URDF 链 mvp0_3_zj ——",
      "关节轴  " + URDF.map(function (j) { return "(" + j.axis.map(function (v) { return (v > 0 ? "+" : "") + v; }).join("") + ")"; }).join(""),
      "限位(°)   " + JLO.map(function (v, i) { return v + "~" + JHI[i]; }).join("  "),
      "—— 目标 ——",
      "目标 局部  x " + TGT.x.toFixed(2) + "  y " + TGT.y.toFixed(2) + "  z " + TGT.z.toFixed(2) + " m   方位 " + bear.toFixed(0) + "°",
      mp ? ("目标 map   x " + mp.x.toFixed(2) + "  y " + mp.y.toFixed(2) + "  z " + mp.z.toFixed(2) + " m") : "目标 map   待机器人位姿…",
      "目标偏差 Δ  " + (TGT.x - e.x).toFixed(3) + "  " + (TGT.y - e.y).toFixed(3) + "  " + (TGT.z - e.z).toFixed(3) + " m",
      "IK 误差  " + (errv * 1000).toFixed(0) + " mm   安全区 " + (safeClamp(TGT).clamped ? "已限制" : "OK"),
      "标定  " + cal
    ];
    var padL = function (s, w) { s = String(s); while (s.length < w) s = " " + s; return s; };
    var padR = function (s, w) { s = String(s); while (s.length < w) s = s + " "; return s; };
    L.splice(L.length - 1, 0, "—— 关节定位 ——");
    L.splice(L.length - 1, 0, padR("jnt", 4) + padL("ang°", 7) + padL("lim%", 6) + padL("X", 6) + padL("Y", 6) + padL("Z", 6));
    for (var ji = 5; ji >= 0; ji--) {
      var jp = f.pts[ji + 1], a = Q[ji] * R2D, pc = Math.round(Q[ji] / (LIM[ji] * D2R) * 100);
      L.splice(L.length - 1, 0, padR((ji + 1 === 6 ? "j6爪" : "j" + (ji + 1)), 4) + padL(((a >= 0 ? "+" : "") + a.toFixed(1)), 7) + padL(((pc >= 0 ? "+" : "") + pc) + "%", 6) + padL(jp.x.toFixed(2), 6) + padL(jp.y.toFixed(2), 6) + padL(jp.z.toFixed(2), 6) + ((ji + 1 === selJoint) ? "  <" : ""));
    }
    var R = window.arm6Robot;
    if (R) {
      L.push("—— 真机 (八界) ——");
      var bt = R.battery || {};
      L.push("电量 " + (bt.value != null ? Math.round(bt.value) : "?") + "%" + (bt.isCharge ? " 充电中" : "") + "  mode " + (bt.mode != null ? bt.mode : "?"));
      var ps = R.pos || {};
      L.push("位置 " + (ps.room || "?") + "  x" + (ps.x != null ? ps.x.toFixed(2) : "?") + " y" + (ps.y != null ? ps.y.toFixed(2) : "?") + " yaw" + (ps.yaw != null ? (ps.yaw * R2D).toFixed(0) + "°" : "?"));
      var w = R.work || {};
      L.push("任务 " + (w.cmd || "idle") + "  " + (w.name || "-") + "  " + (w.task_id || ""));
      L.push("告警 " + ((R.alarm && R.alarm.length) ? R.alarm.join(",") : "无") + "   OTA " + (R.ota ? 1 : 0) + "  维护 " + (R.maint ? 1 : 0));
      if (R.leds && R.leds.states) {
        var stt = R.leds.states, jm = (stt.joint_motor && stt.joint_motor.joints) ? stt.joint_motor.joints.join("") : "?";
        L.push("驱动 关节" + jm + " 撞板" + (stt.crash ? (stt.crash.enabled ? 1 : 0) : "?") + " 头部" + (stt.head_motor ? (stt.head_motor.enabled ? 1 : 0) : "?") + " 驱动轮" + (stt.driver_motor ? (stt.driver_motor.enabled ? 1 : 0) : "?"));
      }
      if (R.map) L.push("地图 " + (R.map.mname || "") + " " + R.map.totalWidth + "x" + R.map.totalHeight + " px=" + R.map.pixel + " org(" + R.map.originX + "," + R.map.originY + ")");
      if (R.fur && R.fur.info) L.push("家具 " + R.fur.info.length + " 项");
      if (R.dis) L.push("消毒灯 " + R.dis.status);
      L.push("消息 " + R.msgs + " 条  " + (R.cam || ""));
    }
    hud.textContent = L.join("\n");
    if (window.arm6ParamsUpdate) window.arm6ParamsUpdate();
    if (_a6dirty) { _a6dirty = false; push3D(true); }
    var ml = document.getElementById("a6MapLive");
    if (ml) ml.textContent = mp ? ("map(" + mp.x.toFixed(2) + "," + mp.y.toFixed(2) + "," + mp.z.toFixed(2) + ")") : "—";
  }

  var TGT = { x: 0.35, y: 0.0, z: 0.55 };
  var LEARN = { trimDeg: 0, samples: [[20, "正前"], [5, "左"], [-8, "右"]] };
  window.arm6Learn = LEARN;
  window.arm6AutoRot = true;
  window.arm6Limit = false;    // 安全限制(安全区/可达/步长) 默认【关】= 不限
  window.arm6Redraw = function () { draw(); };
  window.arm6Snapshot = function () {
    var f = fk(Q), e = f.pts[N + 1], rpy = rpyOf(f.T);
    return { time: new Date().toISOString(), joints_deg: Q.map(function (v) { return +(v * R2D).toFixed(2); }),
      ee_local: { x: +e.x.toFixed(4), y: +e.y.toFixed(4), z: +e.z.toFixed(4) }, rpy_deg: { r: +rpy.r.toFixed(1), p: +rpy.p.toFixed(1), y: +rpy.y.toFixed(1) },
      target_local: { x: +TGT.x.toFixed(3), y: +TGT.y.toFixed(3), z: +TGT.z.toFixed(3) },
      target_map: (window.arm6MapOf ? window.arm6MapOf(TGT.x, TGT.y, TGT.z) : null), base: BASE, base_offset: { x: ARM_BASE.x, y: ARM_BASE.y, z: ARM_BASE.z }, calib: (LEARN.calib || null) };
  };
  /* 调试 */
  window.arm6Cam = function () { return { az: CAM.az, el: CAM.el, dist: CAM.dist }; };
  window.arm6Proj = function (p) { VB = camBasis(); return proj3(p); };
  window.arm6TGT = function () { return { x: TGT.x, y: TGT.y, z: TGT.z }; };
  window.arm6Base = function () { return { x: BASE.x, y: BASE.y, yaw: BASE.yaw }; };
  window.arm6Sel = function () { return selJoint; };
  window.arm6BaseOffset = ARM_BASE;
  window.arm6GetBaseOffset = function () { return { x: ARM_BASE.x, y: ARM_BASE.y, z: ARM_BASE.z }; };
  window.arm6Lift = function () { return LIFT; };
  /* 同步到嵌入的真机 3D 页(iframe #arm3dF) */
  var _p3t = 0, _a6skip = false, _a6dirty = false;
  function push3D(force) {
    if (_a6skip) { _a6skip = false; return; }
    var t = Date.now(); if (!force && t - _p3t < 90) return; _p3t = t;
    var f = document.getElementById("arm3dF"); if (!f) return;
    try { var w = f.contentWindow; if (w && w.arm3dSetJoints) { w.arm3dSetJoints(Q.slice()); if (w.arm3dSetLift) w.arm3dSetLift(LIFT); if (w.arm3dSetGrip) w.arm3dSetGrip(GRIP * 0.69); } } catch (e) {}
  }
  window.arm6Push3D = function () { push3D(true); };
  window.arm6On3DJoints = function (rad) { if (!rad || rad.length < 6) return; for (var i = 0; i < 6; i++) Q[i] = +rad[i]; syncSliders(); _a6skip = true; draw(); };
  window.arm6On3DLift = function (m) { LIFT = clamp(+m || 0, 0, 0.44); if (window.arm6RulerSync) window.arm6RulerSync(); var lf = $("arm6Lift"); if (lf) { lf.value = Math.round(LIFT * 100); var v = $("arm6LiftV"); if (v) v.textContent = Math.round(LIFT * 100) + " cm"; } };
  window.arm6On3DGrip = function (g) { GRIP = clamp(+g || 0, 0, 1); var s = $("arm6Grip"); if (s) { s.value = Math.round(GRIP * 100); var v = $("arm6GripV"); if (v) v.textContent = s.value + "%"; } };
  window.arm6SetLift = function (m) { LIFT = clamp(+m || 0, 0, 0.44); if (window.arm6RulerSync) window.arm6RulerSync(); draw(); push3D(true); return LIFT; };
  window.arm6GetLift = function () { return LIFT; };
  window.arm6SetJoints = function (rad) { if (!rad || rad.length < 6) return false; Q = rad.slice(0, 6).map(function (v, i) { return clamp(+v, JLO[i] * D2R, JHI[i] * D2R); }); syncSliders(); _a6dirty = true; draw(); return true; };
  window.arm6Limits = function () { return { lo: JLO.slice(), hi: JHI.slice() }; };
  /* 真机 mvp03 电机/关节数据(URDF mvp0_3_zj.urdf)：朝向=旋转轴；正转=绕轴右手正向；安装位=相对父连杆(m)；par→ch=父/子连杆 */
  var JAX = ["X", "-Y", "Y", "X", "-Y", "Z"];
  var JMETA = [
    { org: "0, 0, 0.1421",           par: "arm_base_link", ch: "arm1_link" },
    { org: "0.0424, -0.060, 0",      par: "arm1_link",     ch: "arm2_link" },
    { org: "0.3245, 0.0097, 0",      par: "arm2_link",     ch: "arm3_link" },
    { org: "0.0752, 0.0503, 0",      par: "arm3_link",     ch: "arm4_link" },
    { org: "0.3253, 0.0425, 0",      par: "arm4_link",     ch: "arm5_link" },
    { org: "0.056, -0.0425, 0.0765", par: "arm5_link",     ch: "arm6_link" }
  ];
  window.arm6ParamsHTML = function () {
    var T = 'padding:2px 6px;', TR = 'text-align:right;', TC = 'text-align:center;';
    var h = '<table style="border-collapse:collapse;width:100%">' +
      '<tr style="color:#0e7490">' +
      '<th style="text-align:left;padding:2px 8px 2px 2px">电机</th>' +
      '<th style="' + TC + '">朝向(轴)</th>' +
      '<th style="' + TC + '">正转(+)</th>' +
      '<th style="' + TC + '">安装位(父系 m)</th>' +
      '<th style="' + TC + '">父→子连杆</th>' +
      '<th style="' + TC + '">下限</th>' +
      '<th style="' + TC + '">上限</th>' +
      '<th style="' + TC + '">当前</th>' +
      '<th style="' + TC + '">速度</th></tr>';
    for (var i = 0; i < 6; i++) {
      var m = JMETA[i];
      var loS = (JLO[i] > -178.9) ? 'color:#b45309;font-weight:600' : '';
      var hiS = (JHI[i] < 178.9) ? 'color:#b45309;font-weight:600' : '';
      h += '<tr>' +
        '<td style="padding:2px 8px 2px 2px;color:#0e5f78;font-weight:600">J' + (i + 1) + (i === 5 ? ' 夹爪旋转' : '') + '</td>' +
        '<td style="' + T + TC + '">' + JAX[i] + '</td>' +
        '<td style="' + T + TC + '">↺ 绕' + JAX[i] + '</td>' +
        '<td style="' + T + TC + ';color:#475569">' + m.org + '</td>' +
        '<td style="' + T + TC + ';color:#94a3b8;font-size:11px">' + m.par + '→' + m.ch + '</td>' +
        '<td style="' + T + TR + loS + '">' + JLO[i] + '°</td>' +
        '<td style="' + T + TR + hiS + '">' + JHI[i] + '°</td>' +
        '<td id="a6pc' + i + '" style="' + T + TR + 'color:#0891b2;font-weight:600">' + (Q[i] * R2D).toFixed(1) + '°</td>' +
        '<td style="' + T + TR + 'color:#64748b">1.0</td></tr>';
    }
    h += '<tr style="border-top:1.5px solid #cfe0f0">' +
      '<td style="padding:2px 8px 2px 2px;color:#0e5f78;font-weight:600">夹爪指×2</td>' +
      '<td style="' + T + TC + '">±Y</td><td style="' + T + TC + '">↺ 开/合</td>' +
      '<td style="' + T + TC + ';color:#475569">0, 0, 0.077</td>' +
      '<td style="' + T + TC + ';color:#94a3b8;font-size:11px">arm6_link→gripper</td>' +
      '<td style="' + T + TR + '">0°</td><td style="' + T + TR + '">29°</td>' +
      '<td style="' + T + TR + ';color:#94a3b8">—</td><td style="' + T + TR + ';color:#64748b">0.2</td></tr>';
    h += '<tr>' +
      '<td style="padding:2px 8px 2px 2px;color:#0e5f78;font-weight:600">立柱升降</td>' +
      '<td style="' + T + TC + '">Z</td><td style="' + T + TC + '">↑ 上升</td>' +
      '<td style="' + T + TC + ';color:#475569">-0.0145, 0.0995, 0.1</td>' +
      '<td style="' + T + TC + ';color:#94a3b8;font-size:11px">lift_base→lift</td>' +
      '<td style="' + T + TR + '">0</td><td style="' + T + TR + '">44cm</td>' +
      '<td id="a6pcLift" style="' + T + TR + 'color:#0891b2;font-weight:600">' + (LIFT * 100).toFixed(0) + ' cm</td>' +
      '<td style="' + T + TR + ';color:#64748b">1.0 m/s</td></tr>';
    h += '<tr>' +
      '<td style="padding:2px 8px 2px 2px;color:#0e5f78;font-weight:600">头部俯仰</td>' +
      '<td style="' + T + TC + '">-Y</td><td style="' + T + TC + '">↺ 绕-Y</td>' +
      '<td style="' + T + TC + ';color:#475569">0.083, -0.046, 0.546</td>' +
      '<td style="' + T + TC + ';color:#94a3b8;font-size:11px">lift→head_pitch</td>' +
      '<td style="' + T + TR + '">-180°</td><td style="' + T + TR + '">180°</td>' +
      '<td style="' + T + TR + ';color:#94a3b8">—</td><td style="' + T + TR + ';color:#64748b">—</td></tr>';
    return h + '</table>';
  };
  window.arm6ParamsUpdate = function () {
    for (var i = 0; i < 6; i++) { var el = document.getElementById('a6pc' + i); if (el) el.textContent = (Q[i] * R2D).toFixed(1) + '°'; }
    var lf = document.getElementById('a6pcLift'); if (lf) lf.textContent = (LIFT * 100).toFixed(0) + ' cm';
  };
  function notifyJoints() { /* 已改为：仅点「✅ 确定执行」才下发真机（拖动不再自动下发） */ }
  window.arm6SetBaseOffset = function (o) { if (o) { if (o.x != null) ARM_BASE.x = +o.x; if (o.y != null) ARM_BASE.y = +o.y; if (o.z != null) ARM_BASE.z = +o.z; } draw(); return { x: ARM_BASE.x, y: ARM_BASE.y, z: ARM_BASE.z }; };
  window.arm6SetArmYaw = function (deg) { ARM_YAW = (+deg || 0) * D2R; try { localStorage.setItem("a6armyaw", String(+deg || 0)); } catch (e) {} draw(); return +deg || 0; };
  window.arm6GetArmYaw = function () { return +(ARM_YAW * R2D).toFixed(2); };
  window.arm6SetGrip = function (v) { GRIP = clamp(+v || 0, 0, 1); try { localStorage.setItem("a6grip", String(GRIP)); } catch (e) {} draw(); push3D(true); return GRIP; };
  window.arm6GetGrip = function () { return GRIP; };
  window.arm6DH = function () { return { d: DH.d.slice(), a: DH.a.slice(), alpha: DH.alpha.slice(), offset: DH.offset.slice(), lim: LIM.slice() }; };
  window.arm6JointPts = function () { var f = fk(Q); return f.pts.map(function (p) { var s = proj3(W(p)); return s ? { x: s.x, y: s.y } : null; }); };
  window.arm6FK = function (q) { return fk(q || Q).pts.map(function (p) { return [p.x, p.y, p.z]; }); };
  window.arm6ProjW = function (p) { VB = camBasis(); return proj3(W(p)); };

  /* ---------------- 关节滑杆 ---------------- */
  var box = $("arm6Sliders");
  function buildSliders() {
    if (!box) return;
    var html = "";
    for (var i = 0; i < N; i++) {
      html += '<div style="display:grid;grid-template-columns:34px 1fr 52px;gap:8px;align-items:center;font-size:12.5px;margin-top:5px">' +
              '<span style="color:var(--muted)">j' + (i + 1) + '</span>' +
              '<input type="range" id="arm6q' + i + '" min="' + JLO[i] + '" max="' + JHI[i] + '" step="1" value="' + HOME[i] + '" style="width:100%"/>' +
              '<span id="arm6v' + i + '" class="sub">' + HOME[i] + '°</span></div>';
    }
    box.innerHTML = html;
    for (var k = 0; k < N; k++) bindSlider(k);
  }
  function bindSlider(i) {
    var s = $("arm6q" + i), v = $("arm6v" + i);
    if (!s) return;
    s.oninput = function () {
      if (window.arm6Locked) { s.value = Math.round(Q[i] * R2D); return; }
      Q[i] = clamp(parseFloat(s.value), JLO[i], JHI[i]) * D2R;
      if (v) v.textContent = s.value + "°";
      _a6dirty = true; draw(); notifyJoints();
    };
  }
  function syncSliders() {
    for (var i = 0; i < N; i++) {
      var s = $("arm6q" + i), v = $("arm6v" + i);
      var d = Math.round(Q[i] * R2D);
      if (s) s.value = d;
      if (v) v.textContent = d + "°";
    }
  }

  /* ---------------- 安全区（防挤压自身外壳/屏幕） ---------------- */
  var SAFE = {
    xMin: 0.30, xMax: 0.82, yAbs: 0.38, zMin: 0.18, zMax: 0.82, rMin: 0.34,
    shoulder: { x: 0, y: 0, z: 0.15 }, Rmax: 0.78, Rmin: 0.24,
    chest: { yAbs: 0.26, zLo: 0.42, zHi: 1.15, xFront: 0.40 }
  };
  function safeClamp(p) {
    var x = clamp(p.x, SAFE.xMin, SAFE.xMax), y = clamp(p.y, -SAFE.yAbs, SAFE.yAbs), z = clamp(p.z, SAFE.zMin, SAFE.zMax);
    var r = Math.sqrt(x * x + y * y);
    if (r < SAFE.rMin) { if (r < 1e-4) { x = SAFE.rMin; y = 0; } else { x *= SAFE.rMin / r; y *= SAFE.rMin / r; } }
    /* 可达半径：末端够得着的球壳；超出 → IK 永远到不了（拖不动）→ 拉回球壳 */
    var sh = SAFE.shoulder, dx = x - sh.x, dy = y - sh.y, dz = z - sh.z, d = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1e-4;
    if (d > SAFE.Rmax) { var k = SAFE.Rmax / d; x = sh.x + dx * k; y = sh.y + dy * k; z = sh.z + dz * k; }
    else if (d < SAFE.Rmin) { var k2 = SAFE.Rmin / d; x = sh.x + dx * k2; y = sh.y + dy * k2; z = sh.z + dz * k2; }
    x = clamp(x, SAFE.xMin, SAFE.xMax); y = clamp(y, -SAFE.yAbs, SAFE.yAbs); z = clamp(z, SAFE.zMin, SAFE.zMax);
    var c = SAFE.chest;
    if (Math.abs(y) <= c.yAbs && z >= c.zLo && z <= c.zHi && x < c.xFront) { x = c.xFront; }
    return { x: x, y: y, z: z, clamped: (Math.abs(x - p.x) > 1e-4 || Math.abs(y - p.y) > 1e-4 || Math.abs(z - p.z) > 1e-4) };
  }
  window.arm6Safe = SAFE;
  window.arm6SafeClamp = safeClamp;

  /* ---------------- 拖动 / 视角 ---------------- */
  var drag = null, orbit = null, mode = null, jointDrag = null;
  function sxy(e) { var r = cv.getBoundingClientRect(); return { x: (e.clientX - r.left) * (cv.width / r.width), y: (e.clientY - r.top) * (cv.height / r.height) }; }
  function rayDir(s) { var xn = (s.x - cv.width / 2) / FOCAL, yn = (cv.height / 2 - s.y) / FOCAL; return unit({ x: VB.f.x + VB.r.x * xn + VB.u.x * yn, y: VB.f.y + VB.r.y * xn + VB.u.y * yn, z: VB.f.z + VB.r.z * xn + VB.u.z * yn }); }
  function rayPlane(s, planePt) {
    var dir = rayDir(s), n = { x: -VB.f.x, y: -VB.f.y, z: -VB.f.z };
    var den = dir.x * n.x + dir.y * n.y + dir.z * n.z; if (Math.abs(den) < 1e-6) return null;
    var u = ((planePt.x - VB.eye.x) * n.x + (planePt.y - VB.eye.y) * n.y + (planePt.z - VB.eye.z) * n.z) / den;
    return { x: VB.eye.x + u * dir.x, y: VB.eye.y + u * dir.y, z: VB.eye.z + u * dir.z };
  }
  function rayFloor(s) { var dir = rayDir(s); if (Math.abs(dir.z) < 1e-4) return null; var t = (0 - VB.eye.z) / dir.z; if (t < 0) return null; return { x: VB.eye.x + dir.x * t, y: VB.eye.y + dir.y * t, z: 0 }; }
  /* 底盘拖动 → 真机（步长限制：每步前进≤0.15m / 转≤0.4rad，忙时跳过） */
  var baseSent = { x: 0, y: 0 };
  function maybeSendChassis() {
    if (typeof window.arm6SendChassis !== "function") return;
    var dx = BASE.x - baseSent.x, dy = BASE.y - baseSent.y;
    if (Math.sqrt(dx * dx + dy * dy) < (window.arm6Limit ? 0.10 : 0.03)) return;   // 阈值（不限时更密）
    if (window.arm6SendChassis(dx, dy * 1.4)) { baseSent.x = BASE.x; baseSent.y = BASE.y; }
  }
  function setArmWorld(ww) { if (!ww) return; var rp = window.arm6Limit ? safeClamp(invW(ww)) : invW(ww); TGT = { x: rp.x, y: rp.y, z: rp.z };
    var prev = Q.slice(); ik(TGT);
    var slew = 0.12;   // rad/帧 关节限速，拖动更顺（防跳）
    for (var i = 0; i < N; i++) { var d = Q[i] - prev[i]; if (d > slew) Q[i] = prev[i] + slew; else if (d < -slew) Q[i] = prev[i] - slew; }
    syncSliders(); draw(); notifyJoints(); }
  if (cv) {
    cv.addEventListener("pointerdown", function (e) {
      cv.setPointerCapture && cv.setPointerCapture(e.pointerId);
      if (e.shiftKey || e.button === 2) { orbit = { x: e.clientX, y: e.clientY, az: CAM.az, el: CAM.el }; return; }
      if (window.arm6Locked) return;   // 执行中锁定拖动
      var sp = sxy(e), bc = proj3({ x: BASE.x, y: BASE.y, z: 0 });
      var f = fk(Q), near = 1e9;
      for (var i = 0; i <= N + 1; i++) { if (f.pts[i].z < 0.24) continue; var pp = ptW(f.pts[i]); if (pp) { var d = Math.hypot(sp.x - pp.x, sp.y - pp.y); if (d < near) near = d; } }
      var tp = ptW(TGT); if (tp) { var dt = Math.hypot(sp.x - tp.x, sp.y - tp.y); if (dt < near) near = dt; }
      var diskHit = bc ? (Math.hypot(sp.x - bc.x, sp.y - bc.y) < (0.30 * (bc.s || 300)) * 0.98) : false;
      if (!window.arm6Follow && diskHit && near > 30) { mode = "base"; var wf = rayFloor(sp); if (wf) { BASE.x = wf.x; BASE.y = wf.y; } baseSent = { x: BASE.x, y: BASE.y }; draw(); return; }
      // 关节轮盘命中(底盘优先, 才轮到关节)
      var hitJ = 0;
      for (var hi = 1; hi <= 6; hi++) { var hp = ptW(f.pts[hi]); if (hp) { var hrd = clamp(0.03 * hp.s, 9, 30); if (Math.hypot(sp.x - hp.x, sp.y - hp.y) < hrd + 4) { hitJ = hi; break; } } }
      if (hitJ) {
        selJoint = hitJ;
        var hpx = ptW(f.pts[hitJ]), aw = f.axes[hitJ - 1];
        var sgn = (aw.x * VB.f.x + aw.y * VB.f.y + aw.z * VB.f.z) > 0 ? 1 : -1;   // 轴朝向相机→逆时针为正
        mode = "joint"; jointDrag = { j: hitJ, cx: hpx.x, cy: hpx.y, a0: Math.atan2(sp.y - hpx.y, sp.x - hpx.x), q0: Q[hitJ - 1], sgn: sgn };
        draw(); return;
      }
      mode = "arm"; drag = { plane: W(TGT) }; setArmWorld(rayPlane(sp, drag.plane));
    });
    cv.addEventListener("pointermove", function (e) {
      if (orbit) { CAM.az = orbit.az - (e.clientX - orbit.x) * 0.008; CAM.el = clamp(orbit.el + (e.clientY - orbit.y) * 0.006, -0.15, 1.35); draw(); return; }
      if (mode === "base") { var wf = rayFloor(sxy(e)); if (wf) { BASE.x = wf.x; BASE.y = wf.y; draw(); maybeSendChassis(); } return; }
      if (mode === "joint" && jointDrag) {
        var sj = sxy(e);
        var rdx = sj.x - jointDrag.cx, rdy = sj.y - jointDrag.cy, rr = Math.hypot(rdx, rdy);
        if (rr > 16) {   // 离关节中心太近→角度不稳，忽略
          var aj = Math.atan2(rdy, rdx), da = aj - jointDrag.a0;
          if (da > Math.PI) da -= 2 * Math.PI; else if (da < -Math.PI) da += 2 * Math.PI;
          var dq = da * jointDrag.sgn;
          Q[jointDrag.j - 1] = clamp(Q[jointDrag.j - 1] + dq, JLO[jointDrag.j - 1] * D2R, JHI[jointDrag.j - 1] * D2R);
          jointDrag.a0 = aj;
        }
        syncSliders(); draw(); notifyJoints(); return;
      }
      if (mode !== "arm" || !drag) return;
      setArmWorld(rayPlane(sxy(e), drag.plane));
    });
    ["pointerup", "pointercancel"].forEach(function (ev) { cv.addEventListener(ev, function () { if (mode === "arm" && drag) emitPose(); if (mode === "joint") notifyJoints(); drag = null; mode = null; orbit = null; jointDrag = null; }); });
    cv.addEventListener("dblclick", function () { CAM.az = -0.95; CAM.el = 0.42; PIVOT_Z = 0.50; CAM.dist = 4.6; fitView(); });
    cv.addEventListener("wheel", function (e) { e.preventDefault(); CAM.dist = clamp(CAM.dist * (e.deltaY < 0 ? 0.9 : 1.11), 0.5, 16); draw(); }, { passive: false });
    cv.addEventListener("contextmenu", function (e) { e.preventDefault(); });
  }

  /* ---------------- 真机联动（拖动示教） ---------------- */
  function emitPose() { /* 已改为：仅点「✅ 确定执行」才下发真机（拖动不再自动下发） */ }
  window.arm6GetPose = function () {
    var f = fk(Q), e = f.pts[N + 1];
    return { x: e.x, y: e.y, z: e.z, joints: Q.map(function (v) { return +(v * R2D).toFixed(1); }) };
  };
  window.arm6SetTarget = function (p) { TGT = { x: p.x, y: p.y, z: p.z }; ik(TGT); syncSliders(); _a6dirty = true; draw(); };

  function goHome() { Q = HOME.map(function (v) { return v * D2R; }); syncSliders(); _a6dirty = true; draw(); notifyJoints(); }
  function goZero() { Q = [0, 0, 0, 0, 0, 0]; syncSliders(); _a6dirty = true; draw(); notifyJoints(); }
  if ($("arm6Home")) $("arm6Home").onclick = goHome;
  if ($("arm6Zero")) $("arm6Zero").onclick = goZero;

  /* ---------------- 取物 Skill：视觉 + 算法模拟 ---------------- */
  var pcv = $("pickOv"), pcx = pcv ? pcv.getContext("2d") : null;
  var pcam = $("camArm") || $("camHead") || $("pickCam");
  var plog = $("pickLog"), pnow = $("pickNow");
  var PHASES = [
    "采集 RGB-D（头部相机 + 深度）",
    "目标检测（YOLO：瓶子 0.97）",
    "6D 位姿估计（像素→相机→基座）",
    "IK 求解（ik_qp，6 关节到位）",
    "接近 + 夹爪闭合（抓取）",
    "抬起 → 放置 → 复位"
  ];
  var st = { run: false, i: -1, t: 0, bbox: null };
  var det = { items: [], label: "", n: 0, dur: "?" };

  function realDetect() {
    var lab = ($("pickLabel") && $("pickLabel").value.trim()) || "bottle";
    if (plog) plog.textContent = "🔍 /run_test label=" + lab + " …（真机识别，约 6~15s）";
    if (pnow) pnow.textContent = "识别中…";
    fetch("http://" + location.hostname + ":8794/robot", {
      method: "POST", headers: { "Content-Type": "text/plain" },
      body: JSON.stringify({ path: "run_test", params: { label: lab } })
    }).then(function (r) { return r.json(); }).then(function (j) {
      var k = null; try { k = JSON.parse(j.stdout || ""); } catch (e) {}
      if (!k) { if (plog) plog.textContent = "识别失败：" + JSON.stringify(j).slice(0, 220); if (pnow) pnow.textContent = "失败"; return; }
      var items = (k.items || []).concat(k.items_unknow || []);
      det = { items: items, label: lab, n: (k.items || []).length, dur: j.duration_sec || "?" };
      if (pnow) pnow.textContent = "检测到 " + items.length + " 个";
      if (plog) {
        plog.textContent = "🔍 /run_test label=" + lab + "  耗时 " + det.dur + "s\n" +
          "命中(items)=" + (k.items || []).length + "  未知(items_unknow)=" + (k.items_unknow || []).length + "  camera_type=" + k.camera_type;
        items.forEach(function (it, i) { plog.textContent += "\n  [" + i + "] " + (it.name || "?") + "  bbox=" + JSON.stringify(it.bbox) + "  props=" + JSON.stringify(it.ovd_property || {}); });
      }
    }).catch(function (e) { if (plog) plog.textContent = "识别错误：" + e; if (pnow) pnow.textContent = "错误"; });
  }
  if ($("pickDetect")) $("pickDetect").onclick = realDetect;

  function realDemo() {
    if (!confirm("真机取物流程：会让机器人真实开门/抓取/放置，可能几十分钟。确定下发？")) return;
    if (plog) plog.textContent = "▶ 已下发真机 /run_demo（run_laundry.py）…运行中（无进度回传，看机器人）";
    if (pnow) pnow.textContent = "真机运行中…";
    fetch("http://" + location.hostname + ":8794/robot", {
      method: "POST", headers: { "Content-Type": "text/plain" },
      body: JSON.stringify({ path: "run_demo", params: {} })
    }).then(function (r) { return r.text(); })
      .then(function (t) { if (plog) plog.textContent = "✔ /run_demo 返回：\n" + t.slice(0, 800); if (pnow) pnow.textContent = "完成"; })
      .catch(function (e) { if (plog) plog.textContent = "/run_demo 连接中断（机器人可能仍在跑）：" + e; });
  }
  if ($("pickDemo")) $("pickDemo").onclick = realDemo;
  function plogline(s) { if (!plog) return; plog.textContent += "\n" + s; plog.scrollTop = plog.scrollHeight; }

  function pickStart() {
    if (st.run) return; st.run = true; st.i = 0; st.t = performance.now();
    st.bbox = { x: 250, y: 150, w: 150, h: 190, conf: 0.97, lab: "bottle" };
    if (plog) plog.textContent = "▶ 调用 Skill：取物 (pick)";
    plogline("[" + ts() + "] skill=pickplace_0107  model.rknn 载入");
    plogline("[" + ts() + "] 相机 head@rgb 1280x720 · depth 已对齐");
    if ($("arm6Demo")) demoPick();
  }
  function pickStop() { st.run = false; st.i = -1; st.bbox = null; if (pnow) pnow.textContent = "已停止"; plogline("[" + ts() + "] ■ 停止"); }
  if ($("pickRun")) $("pickRun").onclick = pickStart;
  if ($("pickStop")) $("pickStop").onclick = pickStop;
  function ts() { var d = new Date(); return d.toTimeString().slice(0, 8); }

  function drawPick() {
    if (!pcx) return;
    var W = pcv.width, H = pcv.height;
    pcx.clearRect(0, 0, W, H);
    var ok = false;
    if (pcam && pcam.src && pcam.src.slice(0, 5) === "data:" && pcam.complete && pcam.naturalWidth) {
      try { pcx.drawImage(pcam, 0, 0, W, H); ok = true; } catch (e) {}
    }
    if (!ok) {
      for (var yy = 0; yy < H; yy += 4) { var c = 40 + (yy / H) * 60; pcx.fillStyle = "rgb(" + (c * .5 | 0) + "," + (c * .7 | 0) + "," + (c) + ")"; pcx.fillRect(0, yy, W, 4); }
      pcx.fillStyle = "#243049"; pcx.fillRect(0, H * 0.62, W, H * 0.38);
      pcx.fillStyle = "#7dd3fc"; pcx.fillRect(W * 0.52, H * 0.34, 46, 150);
      pcx.fillStyle = "#0891b2"; pcx.fillRect(W * 0.545, H * 0.27, 22, 34);
      pcx.fillStyle = "#0ea5e9"; pcx.fillRect(W * 0.55, H * 0.245, 12, 12);
    }
    if (st.run && st.bbox) {
      var b = st.bbox;
      pcx.strokeStyle = "#22c55e"; pcx.lineWidth = 3; pcx.setLineDash([]);
      pcx.strokeRect(b.x, b.y, b.w, b.h);
      pcx.fillStyle = "#22c55e"; pcx.font = "bold 16px ui-monospace,monospace";
      pcx.fillText(b.lab + " " + b.conf.toFixed(2), b.x, b.y - 8);
      var cxp = b.x + b.w / 2, cyp = b.y + b.h / 2;
      pcx.strokeStyle = "#f59e0b"; pcx.lineWidth = 2;
      pcx.beginPath(); pcx.moveTo(cxp - 10, cyp); pcx.lineTo(cxp + 10, cyp); pcx.moveTo(cxp, cyp - 10); pcx.lineTo(cxp, cyp + 10); pcx.stroke();
      var px = Math.round(cxp) - 640, py = Math.round(cyp) - 360;
      var lines = [
        "detect: " + b.lab + "  conf=" + b.conf.toFixed(2),
        "bbox(px): x=" + b.x + " y=" + b.y + " w=" + b.w + " h=" + b.h,
        "center(px): (" + Math.round(cxp) + "," + Math.round(cyp) + ")  → 光学中心偏移 (" + px + "," + py + ")",
        "camera: z≈0.42m  →  base x≈0.31 y≈-0.02 z≈0.55"
      ];
      pcx.fillStyle = "rgba(7,12,23,.78)"; pcx.fillRect(8, 8, 470, 18 * lines.length + 12);
      pcx.fillStyle = "#9fb0c8"; pcx.font = "13px ui-monospace,monospace";
      for (var i = 0; i < lines.length; i++) pcx.fillText(lines[i], 16, 28 + i * 18);
    }
    if (det.items.length) {
      var FW = 640, FH = 400, kx = W / FW, ky = H / FH;
      for (var z = 0; z < det.items.length; z++) {
        var it = det.items[z], bb = it.bbox || [0, 0, 0, 0];
        var bx = bb[0] * kx, by = bb[1] * ky, bw = (bb[2] - bb[0]) * kx, bh = (bb[3] - bb[1]) * ky;
        var good = it.name && it.name !== "未知物品";
        pcx.strokeStyle = good ? "#22c55e" : "#94a3b8"; pcx.lineWidth = 2; pcx.setLineDash([]);
        pcx.strokeRect(bx, by, bw, bh);
        pcx.fillStyle = pcx.strokeStyle; pcx.font = "bold 12px ui-monospace,monospace";
        pcx.fillText((it.name || "?") + " [" + bb.join(",") + "]", bx, by > 14 ? by - 4 : by + bh + 13);
      }
      pcx.fillStyle = "rgba(7,12,23,.8)"; pcx.fillRect(8, 8, 470, 42);
      pcx.fillStyle = "#9fb0c8"; pcx.font = "12px ui-monospace,monospace";
      pcx.fillText("真机 /run_test  label=" + det.label + "  items=" + det.n + "  用时" + det.dur + "s", 16, 26);
      pcx.fillText("检测框 frame≈" + FW + "x" + FH + "  共 " + det.items.length + " 个", 16, 42);
    }
    if (st.run) {
      var txt = (st.i >= 0 && st.i < PHASES.length) ? PHASES[st.i] : "准备…";
      pcx.fillStyle = "rgba(34,197,94,.15)"; pcx.fillRect(0, H - 30, W, 30);
      pcx.fillStyle = "#86efac"; pcx.font = "bold 15px system-ui,sans-serif";
      pcx.fillText("阶段 " + (st.i + 1) + "/" + PHASES.length + "：" + txt, 12, H - 9);
    }
  }

  function tick() {
    if (st.run) {
      var el = (performance.now() - st.t) / 1000;
      var k = Math.min(PHASES.length - 1, Math.floor(el / 1.6));
      if (k !== st.i) {
        st.i = k;
        plogline("[" + ts() + "] " + (k + 1) + "/" + PHASES.length + " " + PHASES[k]);
        if (pnow) pnow.textContent = "阶段 " + (k + 1) + "/" + PHASES.length;
        if (k === 1 && st.bbox) plogline("[" + ts() + "] YOLO: bottle 0.97  bbox=(250,150,150,190)");
        if (k === 2) plogline("[" + ts() + "] 位姿: pos(0.31,-0.02,0.55)  rotvec(2.60e-2,1.59,1.03e-2)");
        if (k === 3) plogline("[" + ts() + "] IK: q=[0,-28,52,1,18,0]  err=0.004m");
        if (k === 4) plogline("[" + ts() + "] gripper: 0.9 → 0.15 (闭合)");
        if (k === 5) plogline("[" + ts() + "] place done → 复位");
      }
      if (el > PHASES.length * 1.6) { plogline("[" + ts() + "] ✔ 取物完成"); st.run = false; if (pnow) pnow.textContent = "✔ 完成"; }
    }
    var __t = performance.now(); if (window.__a6fpsT === undefined) window.__a6fpsT = __t;
    window.__a6fc = (window.__a6fc || 0) + 1;
    if (__t - window.__a6fpsT >= 1000) { window.__a6fps = window.__a6fc; window.__a6fc = 0; window.__a6fpsT = __t; }
    if (__t - (window.__a6drawT || 0) >= 33) {
      window.__a6drawT = __t;
      if (window.arm6AutoRot && !drag && !orbit) CAM.az += 0.011;
      draw();
    }
    var _now = performance.now();
    if (!window.__pickLast || _now - window.__pickLast > 70) { window.__pickLast = _now; drawPick(); }
    requestAnimationFrame(tick);
  }

  var demoSeq = [
    [0, -28, 52, 1, 18, 0],
    [0, -12, 40, 0, 40, 0],
    [0, 8, 26, 0, 62, 0],
    [0, -20, 55, 0, 30, 0]
  ];
  function demoPick() {
    var i = 0;
    var iv = setInterval(function () {
      if (i >= demoSeq.length) { clearInterval(iv); return; }
      Q = demoSeq[i].map(function (v) { return v * D2R; });
      syncSliders(); draw(); i++;
    }, 1600);
  }
  if ($("arm6Demo")) $("arm6Demo").onclick = demoPick;

  /* ---------------- 启动 ---------------- */
  buildSliders();
  syncSliders();
  draw();
  fitView();
  requestAnimationFrame(tick);
  if (plog) plog.textContent = "待机 —— 点「▶ 调用 Skill 取物」跑一遍（识别→定位→IK→抓取→放置）。\n（画面背景用头部相机实时图；识别框/位姿为模拟算法输出）";
})();
