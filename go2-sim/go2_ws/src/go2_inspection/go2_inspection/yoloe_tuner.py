#!/usr/bin/env python3
"""Interactive UI to tune YOLOE open-vocabulary detection live against the sim camera.

Drive the robot around (teleop) so objects come into view, watch YOLOE run live on /camera/image_raw, and
tweak the open-vocab prompts plus every per-call detection parameter (conf, iou, imgsz, max_det, agnostic_nms,
retina_masks, augment, single_cls) until detection looks good. "Export config" then writes the tuned values
so they can be pasted into the pipeline (detect_utils.PROMPTS + zone_inspector det_conf, etc.).

This is a development tool. Run it with plain python (not `ros2 run`, not streamlit):

    # Terminal 1 - sim (any inspection world; GUI so you can see and drive):
    ros2 launch go2_bringup inspection_nav.launch.py world:=inspection_arena.sdf headless:=false
    #   (or just the bare sim: ros2 launch go2_bringup sim.launch.py world:=inspection_arena.sdf)
    # Terminal 2 - teleop to drive toward objects:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # Terminal 3 - this tuner:
    export YOLOE_WEIGHTS=~/weights/yoloe-11s-seg.pt
    python3 ~/go2-inspection/go2-sim/go2_ws/src/go2_inspection/go2_inspection/yoloe_tuner.py

Threading design (Tkinter is single-threaded): three threads with strict ownership.
  * Tk main thread: owns all widgets/Vars plus a cheap ~30 fps after() loop (read latest annotated frame ->
    PhotoImage -> Label; push slider/checkbox values into a plain params dict). Never blocks on inference.
  * ROS daemon thread: rclpy spin; the camera callback only decodes the frame into shared state.
  * Inference worker daemon thread: the only thread that touches the YOLOE model -- it re-encodes prompts
    (set_classes) and runs predict(); never calls Tk. Shared state is lock-guarded; heavy work is outside
    the lock. Closing the window runs one coordinated shutdown (stop event -> cancel after -> stop rclpy ->
    join with timeout -> destroy root). No cv2.imshow, which would fight the Tk mainloop.
"""

import os
import sys
import time
import json
import threading
import argparse

import numpy as np
import cv2

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage

# Make detect_utils importable whether or not the workspace is sourced (this file lives inside the package).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from go2_inspection import detect_utils  # noqa: E402

IMGSZ_OPTIONS = [480, 640, 800, 960, 1280]  # all multiples of 32


def ensure_mobileclip_findable():
    """Ensure the MobileCLIP text encoder is found locally so changing prompts stays fast and offline.

    get_text_pe (used when prompts change to new values) pulls the MobileCLIP text encoder via
    ultralytics' weights_dir, which may point at a stale path and trigger a ~572 MB re-download. If the
    encoder exists at ~/weights, symlink it into the resolved weights_dir to avoid that."""
    try:
        from ultralytics.utils import SETTINGS

        src = os.path.expanduser("~/weights/mobileclip_blt.ts")
        wd = os.path.expanduser(str(SETTINGS.get("weights_dir", "") or ""))
        if not (src and os.path.exists(src) and wd):
            return
        dst = os.path.join(wd, "mobileclip_blt.ts")
        if not os.path.exists(dst):
            os.makedirs(wd, exist_ok=True)
            try:
                os.symlink(src, dst)
            except OSError:
                pass
    except Exception:
        pass


# ROS camera node
class CamNode(Node):
    """Subscribes to the camera and stores the latest frame as RGB uint8 (decoded exactly like
    zone_inspector._im, so tuning here matches the pipeline)."""

    def __init__(self, topic):
        super().__init__("yoloe_tuner_cam")
        self._lock = threading.Lock()
        self._frame = None
        self.n_msgs = 0
        self.create_subscription(RosImage, topic, self._im, qos_profile_sensor_data)

    def _im(self, m):
        try:
            buf = np.frombuffer(m.data, dtype=np.uint8)
            step = m.step if m.step else m.width * 3
            if step * m.height > buf.size:
                return
            rows = np.ascontiguousarray(buf.reshape(m.height, step)[:, : m.width * 3])
            img = rows.reshape(m.height, m.width, 3)
            enc = (m.encoding or "rgb8").lower()
            if enc == "bgr8":
                img = np.ascontiguousarray(img[:, :, ::-1])  # -> RGB
            elif enc != "rgb8":
                return
        except Exception:
            return
        with self._lock:
            self._frame = img
            self.n_msgs += 1

    def latest(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()


# the app
class TunerApp:
    def __init__(self, root, args):
        self.root = root
        self.device = args.device
        self.topic = args.topic
        self.export_path = os.path.expanduser(args.export)

        self.stop_event = threading.Event()
        self._after_id = None

        # shared state (lock-guarded)
        self._slock = threading.Lock()  # annotated frame + detections + status
        self._annotated = None  # BGR uint8 (drawn by worker)
        self._dets = []  # [(name, conf)]
        self._infer_ms = 0.0
        self._status = "loading model..."
        self._counts = {}

        self._plock = threading.Lock()  # params snapshot (main -> worker)
        self._params = {}
        self._prompts_pending = None  # set by Apply button; worker consumes
        self._prompts_current = list(detect_utils.PROMPTS)

        self.model = None
        self.model_err = ""

        self._build_ui()
        # Load the model on the main thread before the worker threads start, so failures surface cleanly.
        ensure_mobileclip_findable()
        try:
            self.model = detect_utils.load_model(args.weights, self.device, self._prompts_current)
            self._status = f"model ready ({len(self._prompts_current)} prompts)"
        except Exception as e:
            self.model_err = f"{type(e).__name__}: {e}"
            self._status = f"YOLOE UNAVAILABLE - {self.model_err} (camera still shown)"

        # ROS
        if not rclpy.ok():
            rclpy.init()
        self.cam = CamNode(self.topic)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.cam)
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()

        # inference worker (only toucher of self.model)
        self.worker_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self.worker_thread.start()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._tick()  # start the GUI refresh loop

    # ---------------- UI ----------------
    def _build_ui(self):
        self.root.title("YOLOE detection tuner")
        self.root.geometry("1180x760")
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill="both", expand=True)

        # left: video
        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)
        # status bar packed at the BOTTOM first so it always keeps its strip; the (expanding) video then
        # fills only the space ABOVE it -- otherwise a maximized image grows tall enough to push it off-screen.
        self.status_label = ttk.Label(left, text="", foreground="#0a6", anchor="w")
        self.status_label.pack(side="bottom", fill="x")
        self.video_label = ttk.Label(left, text="waiting for camera...", anchor="center")
        self.video_label.pack(side="top", fill="both", expand=True)

        # right: controls (scrollable not needed; keep compact)
        right = ttk.Frame(main, width=440)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)

        # prompts
        ttk.Label(right, text="Prompts (one open-vocab class per line):").pack(anchor="w")
        self.prompt_text = tk.Text(right, height=12, width=52, wrap="none")
        self.prompt_text.pack(fill="x")
        self.prompt_text.insert("1.0", "\n".join(self._prompts_current))
        prow = ttk.Frame(right)
        prow.pack(fill="x", pady=2)
        ttk.Button(prow, text="Apply prompts", command=self._apply_prompts).pack(side="left")
        ttk.Button(prow, text="Reset to pipeline", command=self._reset_prompts).pack(
            side="left", padx=4
        )

        ttk.Separator(right).pack(fill="x", pady=6)

        # numeric sliders
        self.conf = tk.DoubleVar(value=0.25)
        self.iou = tk.DoubleVar(value=0.70)
        self.max_det = tk.DoubleVar(
            value=100
        )  # DoubleVar: ttk.Scale writes floats; IntVar.get() would raise TclError
        self._slider(right, "conf (confidence threshold)", self.conf, 0.01, 0.95, 0.01)
        self._slider(right, "iou (NMS overlap)", self.iou, 0.10, 0.95, 0.01)
        self._slider(right, "max_det", self.max_det, 1, 300, 1, is_int=True)

        # imgsz
        ir = ttk.Frame(right)
        ir.pack(fill="x", pady=2)
        ttk.Label(ir, text="imgsz").pack(side="left")
        self.imgsz = tk.IntVar(value=640)
        ttk.OptionMenu(ir, self.imgsz, 640, *IMGSZ_OPTIONS).pack(side="left", padx=6)

        # checkboxes
        self.agnostic = tk.BooleanVar(value=False)
        self.retina = tk.BooleanVar(value=False)
        self.augment = tk.BooleanVar(value=False)
        self.single_cls = tk.BooleanVar(value=False)
        self.draw_masks = tk.BooleanVar(value=True)
        self.draw_labels = tk.BooleanVar(value=True)
        self.detect_on = tk.BooleanVar(value=True)
        for txt, var in [
            ("class-agnostic NMS", self.agnostic),
            ("retina (full-res) masks", self.retina),
            ("augment (TTA, ~2-3x slower)", self.augment),
            ("single_cls", self.single_cls),
            ("draw masks", self.draw_masks),
            ("draw labels", self.draw_labels),
            ("detection ON (uncheck = raw feed)", self.detect_on),
        ]:
            ttk.Checkbutton(right, text=txt, variable=var).pack(anchor="w")

        ttk.Separator(right).pack(fill="x", pady=6)
        ttk.Button(right, text="Export config", command=self._export).pack(anchor="w")
        self.export_label = ttk.Label(right, text="", foreground="#06a", wraplength=420)
        self.export_label.pack(fill="x")

        ttk.Separator(right).pack(fill="x", pady=6)
        ttk.Label(right, text="Detections (this frame):").pack(anchor="w")
        self.det_box = tk.Text(right, height=12, width=52, wrap="none", state="disabled")
        self.det_box.pack(fill="both", expand=True)

    def _slider(self, parent, label, var, lo, hi, res, is_int=False):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=1)
        val = ttk.Label(row, width=6)
        ttk.Label(row, text=label).pack(side="top", anchor="w")
        sc = ttk.Scale(row, from_=lo, to=hi, variable=var, orient="horizontal")
        sc.pack(side="left", fill="x", expand=True)
        val.pack(side="right")

        def _upd(*_):
            val.config(text=(f"{int(var.get())}" if is_int else f"{var.get():.2f}"))

        var.trace_add("write", _upd)
        _upd()

    # ---------------- prompt apply ----------------
    def _parse_prompts(self):
        raw = self.prompt_text.get("1.0", "end")
        names, seen = [], set()
        for line in raw.splitlines():
            s = line.strip()
            if s and s not in seen:  # skip blanks/dupes; ' ' gets stripped to '' -> dropped
                names.append(s)
                seen.add(s)
        return names

    def _apply_prompts(self):
        names = self._parse_prompts()
        if not names:
            self._set_status("prompts empty - keeping current")
            return
        with self._plock:
            self._prompts_pending = names
        self._set_status(f"encoding {len(names)} prompts...")

    def _reset_prompts(self):
        self.prompt_text.delete("1.0", "end")
        self.prompt_text.insert("1.0", "\n".join(detect_utils.PROMPTS))
        self._apply_prompts()

    def _set_status(self, txt):
        with self._slock:
            self._status = txt

    # ---------------- ROS spin ----------------
    def _spin_ros(self):
        try:
            self.executor.spin()
        except Exception:
            pass

    # ---------------- inference worker (sole model owner) ----------------
    def _infer_loop(self):
        while not self.stop_event.is_set():
            # 1) apply a pending prompt change (re-encode off the GUI thread)
            pending = None
            with self._plock:
                if self._prompts_pending is not None:
                    pending = self._prompts_pending
                    self._prompts_pending = None
            if pending is not None and self.model is not None:
                try:
                    pe = self._embed(pending)
                    self.model.set_classes(
                        pending, pe
                    )  # ultralytics early-outs when sorted(new)==sorted(current),
                    # so a pure reorder of an identical set is ignored and the model keeps its old order. Read back
                    # the model's authoritative class order (id == list index) so Export emits the order the model
                    # actually uses, never a desynced one.
                    nm = self.model.names
                    self._prompts_current = [nm[i] for i in range(len(nm))]
                    if (
                        sorted(self._prompts_current) == sorted(pending)
                        and self._prompts_current != pending
                    ):
                        self._set_status(
                            f"{len(pending)} prompts active (reorder of same set ignored by model)"
                        )
                    else:
                        self._set_status(f"{len(self._prompts_current)} prompts active")
                except Exception as e:
                    self._set_status(f"prompt encode failed: {type(e).__name__}: {e}")

            # 2) grab the freshest frame + a params snapshot
            frame = self.cam.latest()
            with self._plock:
                p = dict(self._params)
            if frame is None or not p:
                self.stop_event.wait(0.02)
                continue

            bgr = cv2.cvtColor(
                frame, cv2.COLOR_RGB2BGR
            )  # predict expects BGR (cv2), like the pipeline
            dets = []
            if self.model is not None and p.get("detect_on", True):
                try:
                    t0 = time.time()
                    res = self.model.predict(
                        bgr,
                        conf=float(p["conf"]),
                        iou=float(p["iou"]),
                        imgsz=int(p["imgsz"]),
                        max_det=int(p["max_det"]),
                        agnostic_nms=bool(p["agnostic"]),
                        retina_masks=bool(p["retina"]),
                        augment=bool(p["augment"]),
                        single_cls=bool(p["single_cls"]),
                        device=self.device,
                        verbose=False,
                    )
                    infer_ms = (time.time() - t0) * 1000.0
                    annotated, dets = self._annotate(bgr, res, p)
                except Exception as e:
                    annotated = bgr
                    self._set_status(f"inference error: {type(e).__name__}: {e}")
                    infer_ms = 0.0
            else:
                annotated, infer_ms = bgr, 0.0

            counts = {}
            for nm, _c in dets:
                counts[nm] = counts.get(nm, 0) + 1
            with self._slock:
                self._annotated = annotated
                self._dets = dets
                self._counts = counts
                self._infer_ms = infer_ms
            self.stop_event.wait(0.005)

    def _embed(self, names):
        """Prompt embeddings, reusing detect_utils' on-disk cache (so a set tried here is also fast when
        you put it in the pipeline). Falls back to get_text_pe (MobileCLIP) on cache miss."""
        import torch

        weights = os.path.expanduser(detect_utils.DEFAULT_WEIGHTS)
        cache = detect_utils._pe_cache_path(weights, names)
        if os.path.exists(cache):
            try:
                return torch.load(cache, map_location="cpu")
            except Exception:
                pass
        pe = self.model.get_text_pe(names)
        try:
            torch.save(pe.cpu(), cache)
        except Exception:
            pass
        return pe

    def _annotate(self, bgr, res, p):
        out = bgr.copy()
        dets = []
        if not res or res[0].boxes is None or len(res[0].boxes) == 0:
            return out, dets
        r = res[0]
        names = r.names
        boxes = r.boxes.xyxy.cpu().numpy()
        cls_ids = r.boxes.cls.cpu().numpy().astype(int)
        confs = r.boxes.conf.cpu().numpy()
        # optional mask overlay
        if p.get("draw_masks") and getattr(r, "masks", None) is not None:
            try:
                md = r.masks.data.cpu().numpy()  # (N, h, w) in [0,1]
                H, W = out.shape[:2]
                overlay = out.copy()
                for i, m in enumerate(md):
                    nm = names.get(int(cls_ids[i]), str(cls_ids[i])) if i < len(cls_ids) else "?"
                    col = detect_utils.color_for(nm)
                    mk = cv2.resize(m, (W, H)) > 0.5
                    overlay[mk] = col
                out = cv2.addWeighted(overlay, 0.35, out, 0.65, 0)
            except Exception:
                pass
        for i, b in enumerate(boxes):
            x0, y0, x1, y1 = [int(v) for v in b]
            nm = names.get(int(cls_ids[i]), str(cls_ids[i]))
            cf = float(confs[i])
            col = detect_utils.color_for(nm)
            cv2.rectangle(out, (x0, y0), (x1, y1), col, 2)
            if p.get("draw_labels"):
                cv2.putText(
                    out,
                    f"{nm} {cf:.2f}",
                    (x0, max(0, y0 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    col,
                    1,
                    cv2.LINE_AA,
                )
            dets.append((nm, cf))
        return out, dets

    # ---------------- GUI refresh loop (main thread only) ----------------
    def _tick(self):
        try:
            # push current widget values to the worker
            with self._plock:
                self._params = {
                    "conf": self.conf.get(),
                    "iou": self.iou.get(),
                    "max_det": self.max_det.get(),
                    "imgsz": self.imgsz.get(),
                    "agnostic": self.agnostic.get(),
                    "retina": self.retina.get(),
                    "augment": self.augment.get(),
                    "single_cls": self.single_cls.get(),
                    "draw_masks": self.draw_masks.get(),
                    "draw_labels": self.draw_labels.get(),
                    "detect_on": self.detect_on.get(),
                }
            with self._slock:
                bgr = self._annotated
                dets = list(self._dets)
                counts = dict(self._counts)
                infer_ms = self._infer_ms
                status = self._status
            # if no annotated frame yet, show the raw camera frame
            if bgr is None:
                raw = self.cam.latest()
                bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR) if raw is not None else None

            if bgr is not None and bgr.dtype == np.uint8 and bgr.ndim == 3:
                rgb = cv2.cvtColor(np.ascontiguousarray(bgr), cv2.COLOR_BGR2RGB)
                im = Image.fromarray(rgb)
                # fit to the label preserving aspect ratio, scaling UP or down (thumbnail() only shrinks, so
                # on a maximized window it would cap at the native frame size and leave empty space).
                lw = max(320, self.video_label.winfo_width())
                lh = max(240, self.video_label.winfo_height())
                iw, ih = im.size
                scale = min(lw / iw, lh / ih)
                im = im.resize((max(1, int(iw * scale)), max(1, int(ih * scale))))
                self._photo = ImageTk.PhotoImage(im)  # strong ref (rebind frees the old one)
                self.video_label.configure(image=self._photo, text="")
                self.video_label.image = self._photo

            fps = (1000.0 / infer_ms) if infer_ms > 0 else 0.0
            self.status_label.config(
                text=f"{status}   |   {len(dets)} det   infer {infer_ms:.0f} ms (~{fps:.1f} FPS)   "
                f"cam msgs {self.cam.n_msgs}"
            )
            # detection table
            self.det_box.config(state="normal")
            self.det_box.delete("1.0", "end")
            if counts:
                self.det_box.insert(
                    "end",
                    "counts: "
                    + ", ".join(f"{k}={v}" for k, v in sorted(counts.items(), key=lambda x: -x[1]))
                    + "\n\n",
                )
            for nm, cf in sorted(dets, key=lambda x: -x[1])[:40]:
                self.det_box.insert("end", f"{cf:.2f}  {nm}\n")
            self.det_box.config(state="disabled")
        except tk.TclError:
            return  # window is being destroyed; stop quietly
        except Exception as e:
            try:
                self.status_label.config(text=f"ui tick error: {type(e).__name__}: {e}")
            except Exception:
                pass
        finally:
            if not self.stop_event.is_set():
                self._after_id = self.root.after(33, self._tick)

    # ---------------- export ----------------
    def _export(self):
        with self._plock:
            p = dict(self._params)
        prompts = self._prompts_current
        lines = [
            "# YOLOE tuner export -- paste into the pipeline.",
            "# 1) detect_utils.PROMPTS  <- this list (order defines class ids):",
            "PROMPTS = [",
        ]
        lines += [
            "    " + ", ".join(f'"{n}"' for n in prompts[i : i + 4]) + ","
            for i in range(0, len(prompts), 4)
        ]
        lines += [
            "]",
            "",
            "# 2) zone_inspector det_conf (and detect_utils.infer conf arg):",
            f"det_conf: {float(p.get('conf', 0.25)):.2f}",
            "",
            "# 3) Other predict() params you tuned. conf + prompts transfer directly; the rest only take",
            "#    effect in the pipeline if added to detect_utils.infer()'s model.predict(...) call:",
            f"iou: {float(p.get('iou', 0.7)):.2f}",
            f"imgsz: {int(p.get('imgsz', 640))}        # note: detect_utils.infer currently auto-computes imgsz from frame size",
            f"max_det: {int(p.get('max_det', 100))}",
            f"agnostic_nms: {bool(p.get('agnostic', False))}",
            f"retina_masks: {bool(p.get('retina', False))}",
            f"augment: {bool(p.get('augment', False))}",
            f"single_cls: {bool(p.get('single_cls', False))}",
        ]
        text = "\n".join(lines)
        try:
            os.makedirs(os.path.dirname(self.export_path), exist_ok=True)
            with open(self.export_path, "w") as f:
                f.write(text + "\n")
            self.export_label.config(text=f"wrote {self.export_path}")
        except Exception as e:
            self.export_label.config(text=f"export failed: {e}")
        print(
            "\n===== YOLOE TUNER CONFIG =====\n" + text + "\n==============================\n",
            flush=True,
        )

    # ---------------- shutdown ----------------
    def _on_close(self):
        self.stop_event.set()
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.cam.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        for th in (getattr(self, "worker_thread", None), getattr(self, "ros_thread", None)):
            if th is not None:
                th.join(timeout=2.0)
        try:
            self.root.destroy()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="Live YOLOE detection tuner (Tkinter UI).")
    ap.add_argument("--topic", default="/camera/image_raw", help="ROS Image topic")
    ap.add_argument(
        "--weights", default=detect_utils.DEFAULT_WEIGHTS, help="YOLOE -seg weights path"
    )
    ap.add_argument(
        "--device", default=os.environ.get("YOLOE_DEVICE", ""), help="'', 'cuda:0', or 'cpu'"
    )
    ap.add_argument("--export", default="~/gauges/yoloe_tuner_config.txt", help="export file path")
    args = ap.parse_args()

    root = tk.Tk()
    TunerApp(root, args)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
