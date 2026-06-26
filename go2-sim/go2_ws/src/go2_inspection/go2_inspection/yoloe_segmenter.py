#!/usr/bin/env python3
"""yoloe_segmenter -- Phase 3 (object segmentation) for the autonomous inspection.

Runs the team's VERIFIED open-vocabulary segmentation (YOLOE-seg + set_classes) on EACH per-wall
panorama produced by zone_wall_follower, and crops every detected object. It does DETECTION + CROP ONLY:
the actual reading (gauge value, unit, risk) happens downstream on the crops we send out (the Claude
reader / report side), so no needle-angle or OCR here -- this stays a lean perception pass.

INPUT  (from zone_wall_follower):  ~/gauges/<zone>/panorama_00.png, panorama_01.png, ... (+ panoramas.json)
                                   (falls back to panorama.png if no per-segment panoramas exist)
OUTPUT: ~/gauges/<zone>/gauges/<class>_<seg>_<i>.png   one clean crop per detected object
        ~/gauges/<zone>/detections.json                every detection (class, conf, bbox, panorama)
        ~/gauges/<zone>/gauges.json                    gauge-type crops in the schema gauge_inspector
                                                        (Claude) + get_zone_gauges already consume
        ~/gauges/<zone>/gauges_contact_sheet.png       montage for get_zone_image

Model + prompts are the user's physically-verified script, refactored from a live webcam loop into this
batch-over-panoramas pass. It detects nothing in Gazebo (no real instruments there), so it DEGRADES
GRACEFULLY: if ultralytics/YOLOE or the weights are unavailable, it writes an empty (available:false)
result and exits 0 -- the mission keeps going; on the real Go2 (weights present, real gauges) it produces
the crops. Runs ONCE post-sweep, OFF the locomotion loop (real-time-safe).

  ros2 run go2_inspection yoloe_segmenter ~/gauges/zone_0
"""
import os, json, glob, argparse
import numpy as np
import cv2

# --- open-vocabulary prompts (user's verified set) ---
PROMPTS = ["analog gauge", "safety goggles", "construction helmet", "red danger light",
           "candle fire", "plastic water bottle", "digital display"]


def _color_for(obj_type):
    if "danger" in obj_type or "fire" in obj_type:
        return (0, 0, 255)
    if "water" in obj_type:
        return (255, 0, 0)
    if "goggles" in obj_type or "helmet" in obj_type:
        return (0, 255, 255)
    return (0, 255, 0)


def _load_model(weights):
    from ultralytics import YOLO          # raises ImportError if ultralytics absent
    model = YOLO(weights)                 # YOLOE-seg weights (auto-download by name if online)
    model.set_classes(PROMPTS)            # open-vocabulary prompts
    return model


def detect_on_panorama(model, pano_bgr, conf, device, seg_idx, zone, out_dir):
    """Run YOLOE on one panorama -> list of detection dicts (+ save each crop). Mirrors the user's
    per-frame loop, applied to a stitched wall panorama. Detect + crop only -- no reading."""
    # size the inference to the panorama's long side (rounded to 32, capped) instead of the default 640:
    # a wide wall panorama letterboxed to 640 shrinks a ~0.15m gauge to ~15px and YOLOE misses it.
    H, W = pano_bgr.shape[:2]
    imgsz = int(min(1600, max(640, ((max(W, H) + 31) // 32) * 32)))
    kw = {"conf": conf, "imgsz": imgsz, "verbose": False}
    if device:
        kw["device"] = device
    res = model(pano_bgr, **kw)
    dets = []
    if not res or res[0].boxes is None or len(res[0].boxes) == 0:
        return dets
    r = res[0]
    boxes = r.boxes.xyxy.cpu().numpy()
    cls_ids = r.boxes.cls.cpu().numpy().astype(int)
    confs = r.boxes.conf.cpu().numpy()
    for i, box in enumerate(boxes):
        x0, y0, x1, y1 = [int(v) for v in box]
        x0, y0 = max(0, x0), max(0, y0); x1, y1 = min(W, x1), min(H, y1)
        if x1 <= x0 or y1 <= y0:
            continue
        obj_type = PROMPTS[cls_ids[i]] if cls_ids[i] < len(PROMPTS) else str(cls_ids[i])
        crop = pano_bgr[y0:y1, x0:x1].copy()
        fn = f"{obj_type.replace(' ', '_')}_S{seg_idx:02d}_{i:02d}.png"
        cv2.imwrite(os.path.join(out_dir, fn), crop)
        # monotonic left->right ordering key across walls (segment-major, then x in panorama) so the
        # downstream reader/report can order detections; consumers that need it read 'lateral'.
        dets.append({"id": f"{zone}_S{seg_idx:02d}_{i:02d}", "zone": zone, "segment": seg_idx,
                     "type": obj_type, "conf": round(float(confs[i]), 3),
                     "bbox_pano": [x0, y0, x1, y1], "lateral": seg_idx * 100000 + x0,
                     "file": f"gauges/{fn}"})
    return dets


def _contact_sheet(zone_dir, dets, ch=160):
    if not dets:
        return
    sheet = np.full((ch + 20, ch * len(dets), 3), 40, np.uint8)
    for k, g in enumerate(dets):
        c = cv2.imread(os.path.join(zone_dir, g["file"]))
        if c is None:
            continue
        s = ch / max(c.shape[:2])
        c = cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))))
        sheet[:c.shape[0], k * ch:k * ch + c.shape[1]] = c
        cv2.putText(sheet, g["type"].split()[0][:8], (k * ch + 4, ch + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, _color_for(g["type"]), 1)
    cv2.imwrite(os.path.join(zone_dir, "gauges_contact_sheet.png"), sheet)


def _clean(zone_dir):
    """Drop a prior run's crops/results so reruns don't accumulate stale objects."""
    for p in glob.glob(os.path.join(zone_dir, "gauges", "*.png")):
        try:
            os.remove(p)
        except OSError:
            pass
    for n in ("detections.json", "gauges.json", "gauges_contact_sheet.png"):
        f = os.path.join(zone_dir, n)
        if os.path.exists(f):
            try:
                os.remove(f)
            except OSError:
                pass


def _write_empty(zone_dir, zone, reason):
    os.makedirs(zone_dir, exist_ok=True)
    json.dump({"zone": zone, "available": False, "reason": reason, "n_detections": 0, "detections": []},
              open(os.path.join(zone_dir, "detections.json"), "w"), indent=2)
    json.dump({"zone": zone, "n_gauges": 0, "gauges": []},
              open(os.path.join(zone_dir, "gauges.json"), "w"), indent=2)
    print(f"yoloe_segmenter: {reason} -> wrote empty result for {zone}")


def process_zone(zone_dir, weights="yoloe-26s-seg.pt", conf=0.10, device=""):
    zone_dir = os.path.expanduser(zone_dir)
    zone = os.path.basename(zone_dir.rstrip("/"))
    panos = sorted(glob.glob(os.path.join(zone_dir, "panorama_*.png")))
    if not panos:
        p = os.path.join(zone_dir, "panorama.png")
        panos = [p] if os.path.exists(p) else []
    _clean(zone_dir)                       # drop a prior run's crops/results FIRST (incl. the no-panorama case)
    if not panos:
        _write_empty(zone_dir, zone, "no panoramas to segment"); return []
    # Don't trigger a (slow, possibly-hanging) network auto-download in sim: only load if the weights file
    # exists locally OR download is explicitly allowed. Real Go2: pre-pull weights + set YOLOE_WEIGHTS=/path,
    # or set YOLOE_ALLOW_DOWNLOAD=1 for a one-time fetch.
    if not os.path.exists(os.path.expanduser(weights)) and not os.environ.get("YOLOE_ALLOW_DOWNLOAD"):
        _write_empty(zone_dir, zone,
                     f"weights '{weights}' not found locally (set YOLOE_WEIGHTS=/path or YOLOE_ALLOW_DOWNLOAD=1)")
        return []
    try:
        model = _load_model(weights)
    except Exception as e:
        _write_empty(zone_dir, zone, f"YOLOE unavailable ({type(e).__name__}: {e})"); return []
    out_dir = os.path.join(zone_dir, "gauges"); os.makedirs(out_dir, exist_ok=True)
    all_dets = []
    for seg_idx, pano_path in enumerate(panos):
        img = cv2.imread(pano_path)
        if img is None:
            continue
        try:
            all_dets.extend(detect_on_panorama(model, img, conf, device, seg_idx, zone, out_dir))
        except Exception as e:
            print(f"yoloe_segmenter: detection failed on {os.path.basename(pano_path)}: {e}")
    json.dump({"zone": zone, "available": True, "n_detections": len(all_dets), "detections": all_dets},
              open(os.path.join(zone_dir, "detections.json"), "w"), indent=2)
    # gauge-type crops in the schema the Claude reader + get_zone_gauges already use (reading happens there).
    # 'lateral' gives a stable left->right order across walls for the report/scoring side.
    gauges = [{"id": d["id"], "zone": zone, "type": "gauge", "file": d["file"],
               "segment": d["segment"], "lateral": d["lateral"]}
              for d in all_dets if "gauge" in d["type"]]
    json.dump({"zone": zone, "n_gauges": len(gauges), "gauges": gauges},
              open(os.path.join(zone_dir, "gauges.json"), "w"), indent=2)
    _contact_sheet(zone_dir, all_dets)
    print(f"yoloe_segmenter: {len(all_dets)} objects ({len(gauges)} gauges) across {len(panos)} "
          f"panoramas -> {zone_dir}/gauges/")
    return all_dets


def main():
    ap = argparse.ArgumentParser(description="YOLOE open-vocab segmentation of wall panoramas (Phase 3).")
    ap.add_argument("zone_dir", nargs="?", default="~/gauges/zone_0")
    ap.add_argument("--weights", default=os.environ.get("YOLOE_WEIGHTS", "yoloe-26s-seg.pt"))
    ap.add_argument("--conf", type=float, default=0.10)
    ap.add_argument("--device", default=os.environ.get("YOLOE_DEVICE", ""), help="'', cuda, or cpu")
    a = ap.parse_args()
    process_zone(a.zone_dir, weights=a.weights, conf=a.conf, device=a.device)


if __name__ == "__main__":
    main()
