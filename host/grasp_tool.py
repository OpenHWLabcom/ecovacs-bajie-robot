#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
八界机器人 · 抓取工具（基于官方 bajie_sdk）
用法：
  只识别（会有一次头部取图/准备动作）：
    python3 grasp_tool.py --detect --labels 玩具,杯子,纸巾,瓶子,书本,笔
  抓取指定物品（全链路，会动）：先头部识别 → lookto 对准手臂 → 手臂重检匹配 → eco_pick
    python3 grasp_tool.py --pick 杯子 --idx 0 --labels 杯子
关键：bot 上的 SDK 连本机 ws://127.0.0.1:9900/
"""
import sys, json, argparse

DEFAULT_LABELS = "玩具,杯子,纸巾,瓶子,书本,笔,文具盒,橡皮擦,钥匙串,玩具汽车"


def jprint(o):
    print(json.dumps(o, ensure_ascii=False), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", default="ws://127.0.0.1:9900/")
    ap.add_argument("--labels", default=DEFAULT_LABELS)
    ap.add_argument("--detect", action="store_true", help="只识别，不抓取")
    ap.add_argument("--arm-pick", default="", help="跳过头部：直接用手臂视角检测并抓取该物品名")
    ap.add_argument("--place", default="", help="放置：头部视角中该物品(框中心)作为放置点")
    ap.add_argument("--place-box", default="", help="放置框 x1,y1,x2,y2（头部像素）")
    ap.add_argument("--place-point", default="", help="放置点 px,py（头部像素）")
    ap.add_argument("--place-recapture", action="store_true", help="用重投影精准放置")
    ap.add_argument("--th", type=float, default=0.15, help="检测阈值")
    ap.add_argument("--pick", default="", help="要抓取的物品名（包含匹配），空=不抓")
    ap.add_argument("--idx", type=int, default=0, help="同名候选取第几个")
    ap.add_argument("--timeout", type=float, default=150.0)
    a = ap.parse_args()

    labels = [x.strip() for x in a.labels.split(",") if x.strip()]

    try:
        from bajie_sdk import BajieRobot, CameraType, DetectObjectsRequest, OvdEndpoint
    except Exception as e:
        jprint({"ok": False, "err": "import bajie_sdk fail: %s" % e}); return 1

    r = BajieRobot(a.ws)
    if not r.Connect(timeout_sec=8.0):
        jprint({"ok": False, "err": "connect fail", "ws": a.ws}); return 1

    try:
        # 0) 放置
        if a.place or a.place_box or a.place_point:
            head = r.eco_captureImages(CameraType.HEAD, timeout_sec=30.0)
            if a.place_point:
                ref = [float(x) for x in a.place_point.split(",")]
            elif a.place_box:
                ref = [float(x) for x in a.place_box.split(",")]
            else:
                items = list(r.eco_detect_objects(
                    DetectObjectsRequest(rgb_image=head.rgb_image.img, labels=[a.place], entry=OvdEndpoint.OVD,
                                         ovd_obj_thresh=a.th, box_obj_thresh=a.th), timeout=15.0).items or [])
                jprint({"ok": True, "stage": "place-detect", "count": len(items),
                        "items": [{"name": it.name, "bbox": list(map(int, it.bbox))} for it in items]})
                if not items:
                    jprint({"ok": False, "stage": "place", "err": "place target not found", "wanted": a.place}); return 2
                ref = list(map(float, items[min(a.idx, len(items) - 1)].bbox))
            st = r.eco_place_with_view(head, ref, a.place_recapture, timeout_sec=a.timeout)
            ec = getattr(getattr(st, "error_info", None), "code", None)
            jprint({"ok": ec in (None, 0, "0"), "stage": "place", "ref": [round(x) for x in ref], "code": ec})
            return 0 if ec in (None, 0, "0") else 4

        # 0) 手臂视角直接抓（跳过头部）
        if a.arm_pick:
            arm = r.eco_captureImages(CameraType.ARM, timeout_sec=30.0)
            items = list(r.eco_detect_objects(
                DetectObjectsRequest(rgb_image=arm.rgb_image.img, labels=[a.arm_pick], entry=OvdEndpoint.OVD,
                                     ovd_obj_thresh=a.th, box_obj_thresh=a.th), timeout=15.0).items or [])
            jprint({"ok": True, "stage": "arm-detect", "count": len(items),
                    "items": [{"i": k, "name": it.name, "bbox": list(map(int, it.bbox))} for k, it in enumerate(items)]})
            if not items:
                jprint({"ok": False, "stage": "arm-pick", "err": "arm no target", "wanted": a.arm_pick}); return 2
            tgt = items[min(a.idx, len(items) - 1)]
            nb = tgt.to_named_bbox_for_grab()
            jprint({"ok": True, "stage": "picking", "name": tgt.name, "bbox": list(map(int, nb.bbox))})
            st = r.eco_pick(arm, nb, timeout_sec=a.timeout)
            ec = getattr(getattr(st, "error_info", None), "code", None)
            jprint({"ok": ec in (None, 0, "0"), "stage": "pick", "name": tgt.name, "code": ec})
            return 0 if ec in (None, 0, "0") else 4

        # 1) 头部取图 + 开放词汇检测
        head = r.eco_captureImages(CameraType.HEAD, timeout_sec=30.0)
        resp = r.eco_detect_objects(
            DetectObjectsRequest(rgb_image=head.rgb_image.img, labels=labels, entry=OvdEndpoint.OVD),
            timeout=15.0,
        )
        items = list(resp.items or [])
        out = [{"i": k, "name": it.name, "bbox": list(map(int, it.bbox))} for k, it in enumerate(items)]
        jprint({"ok": True, "stage": "detect", "labels": labels, "count": len(out), "items": out})

        if a.detect or not a.pick:
            return 0

        # 2) 选目标
        cand = [(k, it) for k, it in enumerate(items) if (a.pick in (it.name or ""))]
        if not cand:
            jprint({"ok": False, "stage": "pick", "err": "no target", "wanted": a.pick}); return 2
        k, tgt = cand[min(a.idx, len(cand) - 1)]
        nb = tgt.to_named_bbox_for_grab()
        jprint({"ok": True, "stage": "target", "idx": k, "name": tgt.name, "bbox": list(map(int, nb.bbox))})

        # 3) 对准手臂（头部 bbox → 位姿 → 观察）
        st = r.eco_lookto(nb.bbox, view=head, camera_type=CameraType.ARM, timeout_sec=40.0)
        jprint({"ok": True, "stage": "lookto", "code": getattr(getattr(st, "error_info", None), "code", None)})

        # 4) 手臂取图 + 重检（同标签）+ 跨视图匹配
        arm = r.eco_captureImages(CameraType.ARM, timeout_sec=30.0)
        arm_items = list(r.eco_detect_objects(
            DetectObjectsRequest(rgb_image=arm.rgb_image.img, labels=[tgt.name], entry=OvdEndpoint.OVD),
            timeout=15.0,
        ).items or [])
        if not arm_items:
            jprint({"ok": False, "stage": "pick", "err": "arm view no detection"}); return 3
        matched = r.eco_match_obj_views(head.rgb_image.img, nb.bbox, arm.rgb_image.img, arm_items)
        jprint({"ok": True, "stage": "matched", "name": getattr(matched, "name", tgt.name)})

        # 5) 抓取
        st2 = r.eco_pick(arm, matched, timeout_sec=a.timeout)
        ec = getattr(getattr(st2, "error_info", None), "code", None)
        jprint({"ok": (ec in (None, 0, "0")), "stage": "pick", "name": tgt.name, "code": ec,
                "task_id": getattr(st2, "task_id", None)})
        return 0 if ec in (None, 0, "0") else 4
    except Exception as e:
        jprint({"ok": False, "err": "%s: %s" % (type(e).__name__, e)}); return 5
    finally:
        try: r.Disconnect()
        except Exception: pass


if __name__ == "__main__":
    sys.exit(main())
