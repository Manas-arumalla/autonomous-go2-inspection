#!/usr/bin/env python3
"""panorama_segmenter -- Phase 3 of the autonomous gauge inspection.

INPUT  (from zone_sweeper, Phase 2):  ~/gauges/<zone>/panorama.png + frames.json (+ panorama_meta.json)
OUTPUT:  ~/gauges/<zone>/gauges/gauge_NN.png  (one CLEAN crop per gauge)
         ~/gauges/<zone>/gauges.json          ({id, zone, lateral, src_frame, bbox_pano, bbox_frame})
         ~/gauges/<zone>/gauges_contact_sheet.png

Pipeline:
  1. FastSAM segments the (strip-stitched) per-zone PANORAMA -- class-agnostic, returns every blob.
     The panorama is long+short (e.g. 3300x480); letterboxing it whole to FastSAM's square input shrinks
     each gauge to ~30 px and drops some. So we SLIDE a fixed-width window across it (tiling) -> every
     gauge is detected at consistent resolution REGARDLESS of total width -> scalable to any zone width.
  2. Filter gauge-like blobs (bright face vs the dark wall, plausible size, square-ish, off the floor).
  3. MERGE over-segmented fragments + cross-tile duplicates: FastSAM often splits ONE gauge into
     rim/dial/danger-arc masks, and a gauge can appear in two overlapping tiles. Real gauges are far
     apart along the wall, so merging by horizontal-interval overlap collapses both WITHOUT merging
     distinct gauges. -> one detection per gauge (spatial dedup -- each gauge once).
  4. For each gauge, take the actual CROP from the SINGLE source frame where it sits centred (nearest
     captured lateral position) -- un-sheared, full-resolution. The strip panorama distorts the needle
     (the value!), so it is used ONLY for detection + spatial dedup, never for the crop Claude reads.

REAL-TIME / ON-DEVICE: FastSAM-s (~23 MB) runs in ONE post-sweep pass -- it is OFF the locomotion
control loop, so it never blocks the robot. A few hundred ms on the Jetson Orin Nano (sm_87, CUDA).
Camera-frame only (no extrinsic/TF) -> the external real camera drops in unchanged. Standalone CLI OR
`ros2 run go2_inspection panorama_segmenter <zone_dir>`.
"""
import os, json, argparse

import numpy as np
import cv2

# --- gauge-blob acceptance (panorama pixels) ---
SIZE_MIN_PX = 40       # a gauge face is >= ~40 px; below = noise/tick
SIZE_MAX_PX = 340      # above = wall/floor slab, not a single gauge
ASPECT_LO, ASPECT_HI = 0.40, 2.60   # round/square-ish (a thin sliver is a fragment, not a face)
BRIGHT_MIN = 110       # the gauge face is bright; the wall panel is dark
MERGE_GAP_PX = 24      # fragments of one gauge overlap/abut within this; distinct gauges are far apart


def detect_boxes(pano_bgr, conf, iou, scales, device, weights="FastSAM-s.pt"):
    """Run FastSAM on the WHOLE panorama at several input resolutions and POOL every segment's box
    [x0,y0,x1,y1] (global coords). FastSAM-s is resolution-sensitive and NON-monotonic -- one scale can
    miss a gauge another catches (empirically 1024 missed one the 2048 pass found) -- so the multi-scale
    UNION plus the downstream interval-merge make detection robust without betting on one magic imgsz.
    The model loads once; passes are cheap and run ONCE post-sweep (off the locomotion loop)."""
    from ultralytics import FastSAM
    model = FastSAM(weights)
    H, W = pano_bgr.shape[:2]
    boxes = []
    for sz in scales:
        res = model(pano_bgr, device=device, retina_masks=True, imgsz=int(sz),
                    conf=conf, iou=iou, verbose=False)[0]
        if res.masks is None:
            continue
        for m in res.masks.data.cpu().numpy():
            mm = (m > 0.5).astype(np.uint8)
            if mm.shape != (H, W):
                mm = cv2.resize(mm, (W, H), interpolation=cv2.INTER_NEAREST)
            ys, xs = np.where(mm)
            if len(xs) < 50:
                continue
            boxes.append([int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())])
    return boxes


def gauge_candidates(boxes, gray):
    """Filter detected boxes down to gauge-like ones. Brightness = mean of the box region in the
    panorama gray (the bright gauge face fills most of the box; the dark wall slab does not)."""
    H, W = gray.shape
    cands = []
    for x0, y0, x1, y1 in boxes:
        bw, bh = x1 - x0 + 1, y1 - y0 + 1
        if not (SIZE_MIN_PX <= bw <= SIZE_MAX_PX and SIZE_MIN_PX <= bh <= SIZE_MAX_PX):
            continue
        if not (ASPECT_LO <= bw / bh <= ASPECT_HI):
            continue
        if float(gray[y0:y1 + 1, x0:x1 + 1].mean()) < BRIGHT_MIN:
            continue
        if y1 >= H - 3:                     # spans to the floor edge -> not a wall gauge
            continue
        cands.append([x0, y0, x1, y1])
    return cands


def merge_fragments(cands, gap=MERGE_GAP_PX):
    """Collapse boxes whose x-intervals overlap/abut into one union box (one gauge)."""
    if not cands:
        return []
    cands = sorted(cands, key=lambda b: b[0])
    merged = [list(cands[0])]
    for x0, y0, x1, y1 in cands[1:]:
        m = merged[-1]
        if x0 <= m[2] + gap:                # within the current cluster -> same gauge
            m[0], m[1] = min(m[0], x0), min(m[1], y0)
            m[2], m[3] = max(m[2], x1), max(m[3], y1)
        else:
            merged.append([x0, y0, x1, y1])
    return merged


def crop_full_gauge(frame_bgr, fxc, approx_diam, pad, thresh=140):
    """Crop the FULL gauge from its (clean) best frame, re-centred on the actual bright panel near the
    expected column fxc. The panorama bbox under-covers the dim side of a dial, so a high-reading needle
    can sit just outside it; the frame is un-sheared, so we re-detect the bright panel and crop a square
    around it -> the whole dial + needle is always inside. Returns (crop_bgr, [x0,y0,x1,y1]) or None."""
    H, W = frame_bgr.shape[:2]
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    band = int(max(approx_diam, 80))
    bx0, bx1 = max(0, int(fxc) - band), min(W, int(fxc) + band)
    _, th = cv2.threshold(gray[:, bx0:bx1], thresh, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) > 0.15 * approx_diam * approx_diam]
    if not cnts:
        return None
    x, y, w, h = cv2.boundingRect(min(cnts, key=lambda c: abs(bx0 + cv2.boundingRect(c)[0]
                                                              + cv2.boundingRect(c)[2] / 2 - fxc)))
    ccx, ccy = bx0 + x + w // 2, y + h // 2
    half = max(w, h) // 2 + pad
    cx0, cy0 = max(0, ccx - half), max(0, ccy - half)
    cx1, cy1 = min(W, ccx + half), min(H, ccy + half)
    return frame_bgr[cy0:cy1, cx0:cx1], [int(cx0), int(cy0), int(cx1), int(cy1)]


def segment(zone_dir, conf=0.35, iou=0.9, scales=(1024, 2048), device="cuda", pad=12):
    pano = cv2.imread(os.path.join(zone_dir, "panorama.png"))
    if pano is None:
        raise FileNotFoundError(f"no panorama.png in {zone_dir}")
    fm = json.load(open(os.path.join(zone_dir, "frames.json")))
    ppm, lmin, cx, frames = fm["ppm"], fm["lmin"], fm["cx"], fm["frames"]
    H = pano.shape[0]
    gray = cv2.cvtColor(pano, cv2.COLOR_BGR2GRAY)

    boxes = detect_boxes(pano, conf, iou, scales, device)
    gauges = merge_fragments(gauge_candidates(boxes, gray))
    gauges.sort(key=lambda b: b[0])

    out_dir = os.path.join(zone_dir, "gauges")
    os.makedirs(out_dir, exist_ok=True)
    zone = os.path.basename(zone_dir.rstrip("/"))
    meta = []
    for gid, (x0, y0, x1, y1) in enumerate(gauges, 1):
        lat = lmin + (x0 + x1) / 2.0 / ppm                   # gauge's lateral position on the wall
        bf = min(frames, key=lambda f: abs(f["lat"] - lat))  # frame where the gauge sits ~centred
        fimg = cv2.imread(os.path.join(zone_dir, bf["file"]))
        if fimg is None:
            continue
        fxc = cx + (lat - bf["lat"]) * ppm                   # gauge centre column in THAT frame (~cx)
        ref = crop_full_gauge(fimg, fxc, y1 - y0, pad)       # re-centre on the bright panel (no clip)
        if ref is not None:
            crop, bbox_frame = ref
        else:                                                # fallback: geometric box from the bbox
            bw = x1 - x0
            fx0 = int(max(0, fxc - bw / 2 - pad)); fx1 = int(min(fimg.shape[1], fxc + bw / 2 + pad))
            fy0 = int(max(0, y0 - pad)); fy1 = int(min(H, y1 + pad))
            crop, bbox_frame = fimg[fy0:fy1, fx0:fx1], [fx0, fy0, fx1, fy1]
        if crop.size == 0:
            continue
        fn = f"gauge_{gid:02d}.png"
        cv2.imwrite(os.path.join(out_dir, fn), crop)
        meta.append({"id": f"{zone}_G{gid:02d}", "zone": zone, "lateral": round(lat, 3),
                     "src_frame": bf["file"], "file": f"gauges/{fn}",
                     "bbox_pano": [x0, y0, x1, y1], "bbox_frame": bbox_frame})

    json.dump({"zone": zone, "n_gauges": len(meta), "gauges": meta},
              open(os.path.join(zone_dir, "gauges.json"), "w"), indent=2)
    _contact_sheet(zone_dir, meta)
    return meta


def _contact_sheet(zone_dir, meta, ch=160):
    if not meta:
        return
    sheet = np.full((ch, ch * len(meta), 3), 40, np.uint8)
    for k, g in enumerate(meta):
        c = cv2.imread(os.path.join(zone_dir, g["file"]))
        if c is None:
            continue
        s = ch / max(c.shape[:2])
        c = cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))))
        sheet[:c.shape[0], k * ch:k * ch + c.shape[1]] = c
        cv2.putText(sheet, g["id"].split("_")[-1], (k * ch + 4, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imwrite(os.path.join(zone_dir, "gauges_contact_sheet.png"), sheet)


def main():
    ap = argparse.ArgumentParser(description="FastSAM gauge segmentation of a zone panorama (Phase 3).")
    ap.add_argument("zone_dir", nargs="?", default="~/gauges/zone_1")
    ap.add_argument("--device", default="cuda", help="cuda (Orin/desktop) or cpu")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--scales", default="1024,2048", help="comma-separated FastSAM input sizes to pool")
    a = ap.parse_args()
    zd = os.path.expanduser(a.zone_dir)
    scales = tuple(int(s) for s in a.scales.split(","))
    g = segment(zd, conf=a.conf, scales=scales, device=a.device)
    print(f"{len(g)} gauges -> {zd}/gauges/  (+ gauges.json, gauges_contact_sheet.png)")
    for x in g:
        print(f"  {x['id']}  lat={x['lateral']:+.2f}m  src={x['src_frame']}")


if __name__ == "__main__":
    main()
