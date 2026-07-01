#!/usr/bin/env python3
"""detect_utils -- open-vocabulary object detection helpers (YOLOE) for the inspection scan.

Shared by zone_inspector (live detection during the viewpoint spin). Detection + crop only; no reading.

YOLOE text-prompt mode (ultralytics 8.x API):
    from ultralytics import YOLOE
    model = YOLOE("yoloe-11s-seg.pt")
    model.set_classes(names, model.get_text_pe(names))     # encode the open-vocab text prompts
The model returned by predict() emits class ids that index into `names` (== PROMPTS order here).

`get_text_pe` needs a CLIP text backend (ultralytics' CLIP fork). If it is absent, load_model raises a
clear error and the caller degrades gracefully (scan still runs, captures nothing). Weights are NOT
auto-downloaded in the sim path: set YOLOE_WEIGHTS=/abs/path or place ~/weights/yoloe-11s-seg.pt.
"""

import os
import numpy as np
import cv2

# Open-vocabulary prompts -- EXPORTED FROM THE YOLOE TUNER (yoloe_tuner.py "Export config").
# Order defines the class ids; the prompt strings (incl. their exact wording) are what was encoded and
# tuned, so do NOT "clean up" the spelling -- changing a string changes its embedding and class id.
PROMPTS = [
    "black office chair",
    "brown wooden rectangular cupboard",
    "wooden desk",
    "brown trashcan",
    "human person",
    "blackish wooded chair",
    "wooden tote with a cadboad boxes",
    "red white stripped cone",
    "metalic rack with columns",
    "black crate",
    "concrete bariers",
    "ash fumes",
    "red fire extinguisher",
    "exit sign",
    "orange flame fire",
    # --- analog gauges (appended for the gauge-reading layer; ADR-016 M4). These add NEW class ids;
    #     ids 0-14 above are unchanged, so existing detection is unaffected. The CLIP prompt-embedding
    #     cache (keyed by the prompt list) simply rebuilds once for the new set. zone_inspector then
    #     detects + 3D-localizes + crops gauges like any object; gauge_inspector reads their values. ---
    "white round analog gauge with a needle",
    "circular pressure dial meter",
]

# predict() params exported from the tuner (conf is passed per-call as det_conf). imgsz 1280 upsamples the
# 640px camera so small/distant props are detectable; agnostic_nms MUST be passed False explicitly because
# YOLOE forces it True internally otherwise.
TUNED = {
    "iou": 0.59,
    "imgsz": 1280,
    "max_det": 21,
    "agnostic_nms": False,
    "retina_masks": False,
    "augment": False,
    "single_cls": False,
}

DEFAULT_WEIGHTS = os.environ.get("YOLOE_WEIGHTS", os.path.expanduser("~/weights/yoloe-11s-seg.pt"))

# stable BGR colour per semantic group (hazards red, people orange, safety yellow, else green)
_RED = (40, 40, 220)
_ORANGE = (0, 140, 255)
_YELLOW = (0, 220, 220)
_GREEN = (60, 200, 60)


_CYAN = (206, 194, 54)  # BGR; analog gauges


def is_gauge(name):
    """True if a detected class is an analog gauge/dial — the crops the gauge-reading layer (the model) reads."""
    n = (name or "").lower()
    return "gauge" in n or "dial" in n


def color_for(name):
    n = name.lower()
    if is_gauge(n):
        return _CYAN
    # safety equipment / markers first (so "red fire extinguisher" is yellow, not red)
    if any(k in n for k in ("extinguisher", "hydrant", "exit", "cone", "barrier", "barier")):
        return _YELLOW
    if any(k in n for k in ("fire", "flame", "fumes", "smoke", "ash", "barrel", "electrical")):
        return _RED
    if "person" in n or "human" in n:
        return _ORANGE
    return _GREEN


def _pe_cache_path(weights, prompts):
    import hashlib

    h = hashlib.md5("|".join(prompts).encode()).hexdigest()[:10]
    return os.path.join(os.path.dirname(weights) or ".", f"yoloe_pe_{h}.pt")


def _get_text_pe_bounded(model, names):
    """`model.get_text_pe()` downloads the CLIP/MobileCLIP text backend on first use. With no/slow network
    that download BLOCKS with no internal timeout, which would hang the whole zone_inspector node *inside
    __init__* before its FSM ever starts (the node then sits forever after arriving at a zone — it can't
    even degrade, because nothing raised). Bound it on a daemon thread: if the prompt embeddings aren't
    ready within YOLOE_PE_TIMEOUT seconds, RAISE so the caller degrades gracefully (navigate + spin, no
    detection) instead of hanging. The proper cure is the on-disk PE cache (built once with network) — when
    that exists this path is never taken."""
    import threading

    box = {}

    def _work():
        try:
            box["pe"] = model.get_text_pe(names)
        except Exception as e:  # noqa: BLE001 — surface the real failure to the caller
            box["err"] = e

    t = threading.Thread(target=_work, daemon=True)
    t.start()
    timeout = float(os.environ.get("YOLOE_PE_TIMEOUT", "90"))
    t.join(timeout)
    if t.is_alive():  # the download is still blocked; leave the daemon thread to die with the process
        raise TimeoutError(
            f"CLIP text backend unavailable within {timeout:.0f}s — pre-build the PE cache with network "
            f"(YOLOE_ALLOW_DOWNLOAD=1) or raise YOLOE_PE_TIMEOUT"
        )
    if "err" in box:
        raise box["err"]
    return box["pe"]


def load_model(weights=None, device="", prompts=PROMPTS):
    """Load YOLOE and set the open-vocab text prompts. The prompt embeddings are computed ONCE via the
    CLIP/MobileCLIP text backend and cached next to the weights; subsequent loads reuse the cache, so a
    runtime scan never needs the (large) text backend on disk. Raises (caller degrades gracefully) if
    ultralytics or the weights file are unavailable and no embedding cache exists."""
    weights = os.path.expanduser(weights or DEFAULT_WEIGHTS)
    if not os.path.exists(weights) and not os.environ.get("YOLOE_ALLOW_DOWNLOAD"):
        raise FileNotFoundError(
            f"YOLOE weights not found: {weights} (set YOLOE_WEIGHTS=/abs/path or YOLOE_ALLOW_DOWNLOAD=1)"
        )
    from ultralytics import YOLOE  # ImportError if ultralytics missing
    import torch

    names = list(prompts)
    model = YOLOE(weights)
    cache = _pe_cache_path(weights, names)
    pe = None
    if os.path.exists(cache):
        try:
            pe = torch.load(cache, map_location="cpu")
        except Exception:
            pe = None
    if pe is None:
        pe = _get_text_pe_bounded(model, names)  # bounded: raises (→ graceful degrade) if CLIP unfetchable
        try:
            torch.save(pe.cpu(), cache)
        except Exception:
            pass
    model.set_classes(names, pe)  # YOLOE open-vocab API (embeddings, not just names)
    if device:
        try:
            model.to(device)
        except Exception:
            pass
    return model


def infer(
    model,
    img_bgr,
    conf=0.26,
    imgsz=None,
    iou=None,
    max_det=None,
    agnostic_nms=None,
    retina_masks=None,
    augment=None,
    single_cls=None,
    device="",
    prompts=PROMPTS,
):
    """Run YOLOE on one BGR frame -> [(class_name, conf, [x0,y0,x1,y1]), ...] in pixel coords.

    Detection params default to the tuner-exported TUNED values; pass overrides to change them. imgsz is
    rounded to a multiple of 32 (YOLO requirement). agnostic_nms is forwarded explicitly because YOLOE
    forces it True internally unless overridden."""
    H, W = img_bgr.shape[:2]
    imgsz = TUNED["imgsz"] if imgsz is None else imgsz
    kw = {
        "conf": float(conf),
        "iou": float(TUNED["iou"] if iou is None else iou),
        "imgsz": int(((int(imgsz) + 31) // 32) * 32),
        "max_det": int(TUNED["max_det"] if max_det is None else max_det),
        "agnostic_nms": bool(TUNED["agnostic_nms"] if agnostic_nms is None else agnostic_nms),
        "retina_masks": bool(TUNED["retina_masks"] if retina_masks is None else retina_masks),
        "augment": bool(TUNED["augment"] if augment is None else augment),
        "single_cls": bool(TUNED["single_cls"] if single_cls is None else single_cls),
        "verbose": False,
    }
    if device:
        kw["device"] = device
    res = model.predict(img_bgr, **kw)
    out = []
    if not res or res[0].boxes is None or len(res[0].boxes) == 0:
        return out
    r = res[0]
    boxes = r.boxes.xyxy.cpu().numpy()
    cls_ids = r.boxes.cls.cpu().numpy().astype(int)
    confs = r.boxes.conf.cpu().numpy()
    for i, b in enumerate(boxes):
        x0, y0, x1, y1 = [int(v) for v in b]
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(W, x1), min(H, y1)
        if x1 <= x0 or y1 <= y0:
            continue
        name = prompts[cls_ids[i]] if 0 <= cls_ids[i] < len(prompts) else str(cls_ids[i])
        out.append((name, float(confs[i]), [x0, y0, x1, y1]))
    return out


def contact_sheet(zone_dir, objects, ch=160):
    """Montage of one crop per unique object (objects: dicts with 'class' + 'crop' relative path)."""
    rows = [o for o in objects if o.get("crop")]
    if not rows:
        return None
    sheet = np.full((ch + 22, ch * len(rows), 3), 40, np.uint8)
    for k, o in enumerate(rows):
        c = cv2.imread(os.path.join(zone_dir, o["crop"]))
        if c is None:
            continue
        s = ch / max(c.shape[:2])
        c = cv2.resize(c, (max(1, int(c.shape[1] * s)), max(1, int(c.shape[0] * s))))
        sheet[: c.shape[0], k * ch : k * ch + c.shape[1]] = c
        cv2.putText(
            sheet,
            o["class"].split()[0][:9],
            (k * ch + 4, ch + 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color_for(o["class"]),
            1,
        )
    out = os.path.join(zone_dir, "objects_contact_sheet.png")
    cv2.imwrite(out, sheet)
    return out
