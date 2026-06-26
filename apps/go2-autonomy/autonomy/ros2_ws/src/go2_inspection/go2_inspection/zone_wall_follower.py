#!/usr/bin/env python3
"""zone_wall_follower -- MAP-DRIVEN inspection scan of ONE zone with LIVE object segmentation.

Replaces the old panorama-stitch sweep. We ALREADY know the wall layout: zone_segmenter gave each zone
a POLYGON (its room boundary in map frame) in zones.yaml. We walk the polygon's wall SEGMENTS
deterministically -- always FACING the wall (gauges/instruments are wall-mounted) -- and, while strafing
along each wall, run the YOLOE open-vocab segmentation LIVE on the camera stream. When an object is
detected ahead, the robot STOPS, takes a clean STATIONARY crop, records it, then continues. Output is a
per-zone report + one cropped image per detected object -- NO panorama (the stitch quality was poor and a
stitched image is not what the reader/report needs anyway; a tight stationary crop is).

Per wall segment S (endpoints A,B in map frame):
  1. SEG_NAV    : Nav2 to a standoff point (S offset `standoff` INTO the room), heading = FACE the wall.
  2. SEG_GATE   : confirm a wall is actually there (/scan front ~ standoff). Open space (doorway/opening
                  the polygon also traces) -> SKIP (no wasted scan, don't drive out of the room).
  3. SEG_SQUARE : rotate so the front faces the wall NORMAL exactly (clean, parallel strafe).
  4. SEG_SCAN   : strafe A->B along the wall. PER THE USER: only linear-y (vy strafe) + yaw (vyaw, to keep
                  FACING the wall) are commanded -- NO vx. YOLOE runs continuously in a BACKGROUND thread
                  (off the 10 Hz control loop, real-time-safe). A confident, horizontally-centred, NOT-yet-
                  -captured detection -> SEG_CAPTURE.
  5. SEG_CAPTURE: STOP, let the gait settle, take a STATIONARY YOLOE detection (preferring the object that
                  triggered the stop), crop it, save it, remember its map position (dedup), then resume.
After the last wall: BACKOFF to a nav-safe pose, write detections.json + gauges.json + a contact sheet.

Output schema is UNCHANGED from the old yoloe_segmenter post-pass, so get_zone_image / get_zone_gauges /
the Claude reader keep working:
  ~/gauges/<zone>/gauges/<type>_S<seg>_<i>.png   one clean stationary crop per detected object
  ~/gauges/<zone>/detections.json                every capture (type, conf, bbox, map_xy, file)
  ~/gauges/<zone>/gauges.json                    gauge-type crops in the reader/report schema
  ~/gauges/<zone>/gauges_contact_sheet.png       montage for get_zone_image

  ros2 run go2_inspection zone_wall_follower --ros-args -p zone_id:=zone_0 -p zones_file:=...

Camera frame + odometry/TF only (no camera extrinsic for motion). Detection reuses the team's verified
YOLOE model/prompts (go2_inspection.yoloe_segmenter). Degrades gracefully: if ultralytics/weights are
absent it still scans every wall (so the motion is demonstrable) but captures nothing -> empty report.
Runs in sim and ports unchanged to the real Go2 (set use_sim_time:=false there).
"""
import os, json, math, glob, shutil, threading, time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from nav2_msgs.action import NavigateToPose
import tf2_ros

GAUGES_ROOT = os.path.expanduser("~/gauges")     # MUST match the readers (mission_control_server / gauge_inspector)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    """Smallest signed angle a-b in (-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def point_in_poly(p, poly):
    """Ray-cast point-in-polygon (winding-agnostic) -> resolves which side of a wall is INSIDE the room
    without trusting the contour winding (OpenCV's image-frame winding flips when mapped to map y)."""
    x, y = p
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def simplify_poly(poly, collinear_deg):
    """Merge consecutive near-collinear vertices so jagged grid contours collapse to real wall edges:
    drop vertex V if the turn A->V->B bends less than collinear_deg (V sits on a straight run)."""
    pts = [list(map(float, p)) for p in poly]
    if len(pts) < 4:
        return pts
    thr = math.radians(collinear_deg)
    changed = True
    while changed and len(pts) > 3:
        changed = False
        n = len(pts)
        for i in range(n):
            a = np.array(pts[(i - 1) % n]); v = np.array(pts[i]); b = np.array(pts[(i + 1) % n])
            d1 = v - a; d2 = b - v
            if np.linalg.norm(d1) < 1e-6 or np.linalg.norm(d2) < 1e-6:
                pts.pop(i); changed = True; break
            turn = abs(ang_diff(math.atan2(d2[1], d2[0]), math.atan2(d1[1], d1[0])))
            if turn < thr:
                pts.pop(i); changed = True; break
    return pts


def wall_segments(poly, standoff, corner_margin, min_wall_len, collinear_deg, robot_xy=None, max_segments=6):
    """PURE geometry (no ROS) -> ordered scannable wall segments from a zone polygon. Each segment:
    A,B (wall endpoints), len, s/e (standoff start/end, offset INTO the room and trimmed off the corners),
    nrm (inward unit normal), face (heading that looks AT the wall). Inward normal is resolved by
    point-in-polygon, so polygon winding never matters. Kept module-level so it is unit-testable offline."""
    poly = simplify_poly(poly, collinear_deg)
    if len(poly) < 3:
        return []
    segs = []
    n = len(poly)
    for i in range(n):
        A = np.array(poly[i], float); B = np.array(poly[(i + 1) % n], float)
        L = float(np.linalg.norm(B - A))
        if L < min_wall_len:
            continue
        d = (B - A) / L
        nrm = np.array([-d[1], d[0]])                      # a unit normal to the wall
        mid = (A + B) / 2.0
        if not point_in_poly(mid + nrm * 0.25, poly):      # make nrm point INTO the room
            nrm = -nrm
        face = math.atan2(-nrm[1], -nrm[0])                # heading that looks AT the wall
        if not point_in_poly(mid + nrm * standoff, poly):  # CONCAVE room: standing at standoff would be
            continue                                       # OUTSIDE the room -> can't inspect this wall, skip
        trim = min(corner_margin, max(0.0, (L - 0.4) / 2.0))
        s = A + d * trim + nrm * standoff                  # standoff start, off the corner
        e = B - d * trim + nrm * standoff
        segs.append({"A": A.tolist(), "B": B.tolist(), "len": round(L, 2),
                     "s": s, "e": e, "nrm": nrm, "face": face})
    if not segs:
        return []
    # Safety cap (pathological polygons only): if there are more edges than max_segments, drop the SHORTEST
    # ones (doorway/contour stubs) -- never a long real wall -- and do it BEFORE rotation so coverage is
    # deterministic, NOT start-pose-dependent. Default max_segments=12 >> a real room's ~4-8 walls.
    if 0 < max_segments < len(segs):
        keep = sorted(sorted(range(len(segs)), key=lambda j: -segs[j]["len"])[:max_segments])
        segs = [segs[j] for j in keep]
    # start from the wall whose standoff start is nearest the robot, keep polygon order (adjacent walls
    # -> minimal repositioning) -> rotate the list so that nearest segment is first.
    if robot_xy is not None:
        k = min(range(len(segs)), key=lambda j: np.linalg.norm(np.array(segs[j]["s"]) - np.array(robot_xy)))
        segs = segs[k:] + segs[:k]
    return segs


class ZoneWallFollower(Node):
    def __init__(self):
        super().__init__("zone_wall_follower")
        self.zone_id = self.declare_parameter("zone_id", "zone_0").value
        self.zones_file = os.path.expanduser(self.declare_parameter("zones_file", "").value)
        # --- geometry / motion ---
        self.standoff = float(self.declare_parameter("standoff", 1.0).value)         # scan distance to wall
        self.corner_margin = float(self.declare_parameter("corner_margin", 0.5).value)   # trim each wall end
        self.min_wall_len = float(self.declare_parameter("min_wall_len", 1.2).value)     # skip short edges
        self.collinear_deg = float(self.declare_parameter("collinear_deg", 18.0).value)  # merge straight runs
        self.wall_present = float(self.declare_parameter("wall_present", 1.0).value)      # +margin over standoff
        self.strafe_speed = float(self.declare_parameter("strafe_speed", 0.15).value)    # vy along the wall
        self.scan_yaw_gain = float(self.declare_parameter("scan_yaw_gain", 1.2).value)   # vyaw face-the-wall gain
        self.max_vyaw = float(self.declare_parameter("max_vyaw", 0.4).value)             # vyaw clamp
        self.side_clear = float(self.declare_parameter("side_clear", 0.40).value)
        self.front_stop = float(self.declare_parameter("front_stop", 0.40).value)        # hard front e-stop
        self.square_arc = math.radians(float(self.declare_parameter("square_arc_deg", 28.0).value))
        self.square_ticks = int(self.declare_parameter("square_ticks", 200).value)       # max square-up ticks
        self.square_ok_deg = math.radians(float(self.declare_parameter("square_ok_deg", 6.0).value))  # squared thr
        self.backoff_dist = float(self.declare_parameter("backoff_dist", 1.6).value)
        self.max_segments = int(self.declare_parameter("max_segments", 12).value)
        self.build_timeout = float(self.declare_parameter("build_timeout", 20.0).value)
        self.out_dir = os.path.expanduser(self.declare_parameter("out_dir", "~/gauges").value)
        # --- live detection (YOLOE) ---
        self.det_conf = float(self.declare_parameter("det_conf", 0.10).value)            # YOLOE confidence
        self.det_weights = self.declare_parameter(
            "det_weights", os.environ.get("YOLOE_WEIGHTS", "yoloe-26s-seg.pt")).value
        self.det_device = self.declare_parameter("det_device", os.environ.get("YOLOE_DEVICE", "")).value
        self.center_frac = float(self.declare_parameter("center_frac", 0.6).value)       # capture only if the
        #     detection centre is within this central fraction of image width (object is ahead -> clean crop)
        self.dedup_radius = float(self.declare_parameter("dedup_radius", 0.6).value)     # m; same-gauge guard
        self.settle_ticks = int(self.declare_parameter("settle_ticks", 6).value)         # 0.1s ticks to settle
        self.capture_pad = float(self.declare_parameter("capture_pad", 0.10).value)      # crop bbox padding
        self.cooldown_ticks = int(self.declare_parameter("cooldown_ticks", 8).value)     # post-capture lockout
        # CAPTURE budget: must cover settle + >=2 fresh inferences. On the 8GB Orin one YOLOE inference can be
        # 1-2s, so 2 fresh frames alone is up to ~4s -> 150 ticks (15s) leaves real margin (was 40 = 4s, which
        # timed out EVERY capture and saved nothing).
        self.capture_timeout = int(self.declare_parameter("capture_timeout", 150).value)

        if self.out_dir != GAUGES_ROOT:
            self.get_logger().warn(f"out_dir={self.out_dir} != {GAUGES_ROOT}; the mission/services read from "
                                   f"{GAUGES_ROOT} and will NOT see these outputs")

        self.zone_center = [0.0, 0.0]; self.zone_polygon = []
        if self.zones_file and os.path.exists(self.zones_file):
            self._load_zone()
        else:
            self.get_logger().error(f"zones_file missing: {self.zones_file}"); raise SystemExit(1)

        # sensors / actuators
        self.K = None; self.W = None; self.H = None
        self.img = None; self.front = None; self.scan = None
        self._img_lock = threading.Lock()
        self.tf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.tf, self)
        sens = ReentrantCallbackGroup()                      # sensors stay live during a (blocking) timer tick
        self.create_subscription(CameraInfo, "/camera/camera_info", self._ci, 10, callback_group=sens)
        self.create_subscription(Image, "/camera/image_raw", self._im, qos_profile_sensor_data, callback_group=sens)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data, callback_group=sens)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # detection model + background inference thread
        self.model = None; self.model_reason = ""
        self._det_lock = threading.Lock()
        self.live_dets = []          # latest detections: list of (type, conf, [x0,y0,x1,y1])
        self.live_img = None         # the RGB frame those detections were computed on
        self._infer_count = 0        # increments each completed inference (capture waits for a fresh one)
        self._stop_infer = False
        self._infer_thread = None
        self._load_detector()

        # FSM state
        self.segments = []; self.seg_i = 0
        self.detections = []         # saved captures (manifest rows)
        self.captured = []           # map (x,y) of captured/attempted objects -> dedup
        self.state = "BUILD"
        self.failed = False
        self.nav_done = None; self.nav_status = None
        self.sq_ticks = 0; self.bo_ticks = 0; self.gate_ticks = 0; self.build_ticks = 0
        self.scan_ticks = 0; self.scan_best = 0.0; self.stall_ticks = 0; self.nopose_ticks = 0
        self.scan_budget = 0; self.cooldown = 0
        self.cap_ticks = 0; self._cap_mark = None; self._cap_cand = None; self._cap_cand_xy = None
        self.sweep_start = None; self.lat_axis = None
        self.sweep_target = 0.0; self.sweep_sign = 1.0
        self._clean_outputs()
        # motion FSM on its own mutually-exclusive group -> never re-enters itself mid-tick
        self.create_timer(0.1, self._tick, callback_group=MutuallyExclusiveCallbackGroup())
        self._start_infer()
        self.get_logger().info(f"zone_wall_follower ready: zone={self.zone_id} "
                               f"({len(self.zone_polygon)}-pt polygon), standoff={self.standoff}m, "
                               f"YOLOE={'on' if self.model else 'OFF (' + self.model_reason + ')'}")

    # ---------- detection model ----------
    def _load_detector(self):
        w = os.path.expanduser(self.det_weights)
        if not os.path.exists(w) and not os.environ.get("YOLOE_ALLOW_DOWNLOAD"):
            self.model_reason = (f"weights '{self.det_weights}' not found locally "
                                 f"(set YOLOE_WEIGHTS=/path or YOLOE_ALLOW_DOWNLOAD=1)")
            self.get_logger().warn(f"YOLOE disabled: {self.model_reason}; will scan walls but capture nothing")
            return
        try:
            from go2_inspection.yoloe_segmenter import _load_model
            self.model = _load_model(w)                       # load from the EXPANDED path (not '~/...')
            self.get_logger().info(f"YOLOE loaded: {w} (device='{self.det_device or 'auto'}')")
        except Exception as e:
            self.model_reason = f"YOLOE unavailable ({type(e).__name__}: {e})"
            self.get_logger().warn(f"YOLOE disabled: {self.model_reason}; will scan walls but capture nothing")

    def _start_infer(self):
        if self.model is None or self._infer_thread is not None:
            return
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

    def _infer_loop(self):
        """Continuous YOLOE inference OFF the control loop. Reads the latest frame, runs the model, stashes
        (detections, the exact frame). Variable inference latency never stalls the 10 Hz motion timer."""
        while not self._stop_infer and rclpy.ok():
            with self._img_lock:
                img = None if self.img is None else self.img.copy()
            if img is None:
                time.sleep(0.05); continue
            try:
                dets = self._infer(img)
            except Exception as e:
                if not self._stop_infer:                      # don't touch the logger during teardown
                    self.get_logger().warn(f"inference error: {e}", throttle_duration_sec=5.0)
                dets = []
            with self._det_lock:
                self.live_dets = dets
                self.live_img = img
                self._infer_count += 1
            time.sleep(0.02)                                  # yield; inference itself dominates the period

    def _infer(self, img_rgb):
        """Run YOLOE on one RGB frame -> [(type, conf, [x0,y0,x1,y1]), ...] in pixel coords."""
        from go2_inspection.yoloe_segmenter import PROMPTS
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)   # model was verified on BGR (cv2) images
        H, W = img_bgr.shape[:2]
        imgsz = int(min(1280, max(640, ((max(W, H) + 31) // 32) * 32)))
        kw = {"conf": self.det_conf, "imgsz": imgsz, "verbose": False}
        if self.det_device:
            kw["device"] = self.det_device
        res = self.model(img_bgr, **kw)
        out = []
        if not res or res[0].boxes is None or len(res[0].boxes) == 0:
            return out
        r = res[0]
        boxes = r.boxes.xyxy.cpu().numpy()
        cls_ids = r.boxes.cls.cpu().numpy().astype(int)
        confs = r.boxes.conf.cpu().numpy()
        for i, b in enumerate(boxes):
            x0, y0, x1, y1 = [int(v) for v in b]
            x0, y0 = max(0, x0), max(0, y0); x1, y1 = min(W, x1), min(H, y1)
            if x1 <= x0 or y1 <= y0:
                continue
            typ = PROMPTS[cls_ids[i]] if cls_ids[i] < len(PROMPTS) else str(cls_ids[i])
            out.append((typ, float(confs[i]), [x0, y0, x1, y1]))
        return out

    # ---------- outputs ----------
    def _clean_outputs(self):
        """Remove a prior run's crops/results so this scan's report is clean (also clears any old panoramas
        left by the previous panorama-stitch version)."""
        d = os.path.join(self.out_dir, self.zone_id)
        if not os.path.isdir(d):
            return
        for p in (glob.glob(os.path.join(d, "gauges", "*.png"))
                  + glob.glob(os.path.join(d, "panorama*.png"))
                  + glob.glob(os.path.join(d, "frames_*.json"))
                  + [os.path.join(d, n) for n in ("detections.json", "gauges.json", "panoramas.json",
                                                  "frames.json", "gauges_contact_sheet.png")]):
            if os.path.exists(p):
                try:
                    os.remove(p)
                except OSError:
                    pass
        for fr in glob.glob(os.path.join(d, "frames_*")):
            if os.path.isdir(fr):
                shutil.rmtree(fr, ignore_errors=True)

    # ---------- zone / sensors ----------
    def _load_zone(self):
        z = next((z for z in json.load(open(self.zones_file))["zones"] if z["id"] == self.zone_id), None)
        if z is None:
            self.get_logger().error(f"{self.zone_id} not in {self.zones_file}"); raise SystemExit(1)
        self.zone_center = z["center"]
        self.zone_polygon = z.get("polygon", [])

    def _ci(self, m):
        if self.K is None and m.k[0] > 0.0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5]); self.W, self.H = m.width, m.height

    def _im(self, m):
        """Decode /camera/image_raw honouring encoding (rgb8/bgr8) + row stride. Internal canonical = RGB."""
        try:
            buf = np.frombuffer(m.data, dtype=np.uint8)
            step = m.step if m.step else m.width * 3
            if step * m.height > buf.size:
                return
            rows = np.ascontiguousarray(buf.reshape(m.height, step)[:, :m.width * 3])
            img = rows.reshape(m.height, m.width, 3)
            enc = (m.encoding or "rgb8").lower()
            if enc == "bgr8":
                img = np.ascontiguousarray(img[:, :, ::-1])   # -> RGB
            elif enc != "rgb8":
                self.get_logger().warn(f"unsupported camera encoding '{m.encoding}'", throttle_duration_sec=10.0)
                return
        except Exception as e:
            self.get_logger().warn(f"image decode failed: {e}", throttle_duration_sec=10.0)
            return
        with self._img_lock:
            self.img = img
            if self.W is None:                                # fall back to image size if CameraInfo is late
                self.W, self.H = m.width, m.height

    def _scan(self, m):
        self.scan = m
        i0 = int(round((0.0 - m.angle_min) / m.angle_increment))
        win = [m.ranges[j] for j in range(i0 - 4, i0 + 5)
               if 0 <= j < len(m.ranges) and m.range_min < m.ranges[j] < m.range_max and math.isfinite(m.ranges[j])]
        self.front = float(np.median(win)) if win else None

    def _front_raw_min(self):
        """Min FINITE range in the narrow front arc, IGNORING the range_min lower bound -> catches a wall
        that is closer than range_min (which _scan's `front` filters out as None)."""
        m = self.scan
        if m is None:
            return None
        best = float("inf")
        for j, r in enumerate(m.ranges):
            a = m.angle_min + j * m.angle_increment
            if abs(a) <= self.square_arc and math.isfinite(r) and r > 0.05 and r < best:
                best = r
        return None if best == float("inf") else best

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            return np.array([t.translation.x, t.translation.y]), yaw_of(t.rotation)
        except Exception:
            return None, None

    def stop(self):
        self.cmd.publish(Twist())

    def drive(self, vx, vy, vyaw=0.0):
        t = Twist(); t.linear.x = float(vx); t.linear.y = float(vy); t.angular.z = float(vyaw)
        self.cmd.publish(t)

    def _tick(self):
        getattr(self, "_st_" + self.state, lambda: None)()

    # ---------- geometry: polygon -> wall segments ----------
    def _build_segments(self):
        p0, _ = self.pose()
        return wall_segments(self.zone_polygon, self.standoff, self.corner_margin, self.min_wall_len,
                             self.collinear_deg, robot_xy=(None if p0 is None else p0.tolist()),
                             max_segments=self.max_segments)

    # ---------- FSM ----------
    def _st_BUILD(self):
        p, _ = self.pose()
        if p is None:
            self.build_ticks += 1
            if self.build_ticks * 0.1 > self.build_timeout:
                self.get_logger().error(f"no map->base_link TF after {self.build_timeout:.0f}s "
                                        f"(localization down?); ABORT")
                self.failed = True; self._finish()
            return
        self.segments = self._build_segments()
        if not self.segments:
            self.get_logger().warn(f"{self.zone_id}: no scannable walls in polygon; DONE")
            self._finish(); return
        self.get_logger().info(f"{self.zone_id}: {len(self.segments)} wall segments to scan "
                               f"(lengths {[s['len'] for s in self.segments]})")
        self.seg_i = 0
        self._begin_segment()

    def _begin_segment(self):
        self.nav_done = None; self.nav_status = None
        self.sq_ticks = 0; self.gate_ticks = 0
        self.scan_ticks = 0; self.scan_best = 0.0; self.stall_ticks = 0; self.nopose_ticks = 0
        self.cooldown = 0; self.cap_ticks = 0
        self._cap_mark = None; self._cap_cand = None; self._cap_cand_xy = None
        self.state = "SEG_NAV"

    def _cur(self):
        return self.segments[self.seg_i]

    def _st_SEG_NAV(self):
        seg = self._cur()
        if self.nav_done is None:
            if not self.nav.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("waiting for Nav2 ...", throttle_duration_sec=5.0); return
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"; g.pose.header.stamp = self.get_clock().now().to_msg()
            g.pose.pose.position.x, g.pose.pose.position.y = float(seg["s"][0]), float(seg["s"][1])
            g.pose.pose.orientation.z = math.sin(seg["face"] / 2); g.pose.pose.orientation.w = math.cos(seg["face"] / 2)
            self.get_logger().info(f"wall {self.seg_i + 1}/{len(self.segments)} (len {seg['len']}m): "
                                   f"NAV -> standoff ({seg['s'][0]:.1f},{seg['s'][1]:.1f}) face {math.degrees(seg['face']):.0f}deg")
            self.nav_done = False
            self.nav.send_goal_async(g).add_done_callback(self._nav_acc)
        elif self.nav_done == "reject":
            self.get_logger().warn(f"wall {self.seg_i + 1}: nav rejected; SKIP"); self._advance()
        elif self.nav_done is True:
            if self.nav_status != 4:
                p, y = self.pose()
                seg = self._cur()
                far = p is None or np.linalg.norm(p - np.array(seg["s"])) > 0.8
                if far:
                    d = "no pose" if p is None else f"{np.linalg.norm(p - np.array(seg['s'])):.2f}m off"
                    self.get_logger().warn(f"wall {self.seg_i + 1}: nav not SUCCEEDED ({self.nav_status}) "
                                           f"and {d} standoff; SKIP")
                    self._advance(); return
            self.gate_ticks = 0; self.state = "SEG_GATE"

    def _nav_acc(self, fut):
        h = fut.result()
        if not h.accepted:
            self.nav_done = "reject"; return

        def _res(f):
            try:
                self.nav_status = f.result().status
            except Exception:
                self.nav_status = None
            self.nav_done = True
        h.get_result_async().add_done_callback(_res)

    def _st_SEG_GATE(self):
        """Is a wall actually in front (within standoff + margin)? If not, this polygon edge is a doorway /
        the room opening -> SKIP. A DEAD lidar (no /scan ever) is NOT a doorway -> abort loudly."""
        self.stop()
        self.gate_ticks += 1
        if self.scan is None:
            if self.gate_ticks < 30:
                return
            self.get_logger().error(f"wall {self.seg_i + 1}: no /scan -- lidar down; ABORT")
            self.failed = True; self._finish(); return
        thr = self.standoff + self.wall_present
        near = self._front_raw_min()
        if (self.front is not None and self.front <= thr) or (near is not None and near <= thr):
            d = self.front if self.front is not None else near
            self.get_logger().info(f"wall {self.seg_i + 1}: wall confirmed @ {d:.2f}m; SQUARE_UP ->")
            self.sq_ticks = 0; self.state = "SEG_SQUARE"
        else:
            fr = "none" if self.front is None else f"{self.front:.2f}m"
            self.get_logger().warn(f"wall {self.seg_i + 1}: open ahead (front {fr}) -> doorway/opening; SKIP")
            self._advance()

    def _wall_offset(self):
        """Bearing to the wall NORMAL = the min-range beam in a NARROW front arc (square_arc, ~28deg). Narrow
        on purpose: the scan starts off a corner, so a wide cone would catch the PERPENDICULAR corner wall and
        square up to the WRONG wall."""
        m = self.scan
        if m is None:
            return None
        best_a, best_r = None, float("inf")
        for j, r in enumerate(m.ranges):
            a = m.angle_min + j * m.angle_increment
            if abs(a) <= self.square_arc and m.range_min < r < m.range_max and math.isfinite(r) and r < best_r:
                best_r, best_a = r, a
        return best_a

    def _st_SEG_SQUARE(self):
        off = self._wall_offset()
        self.sq_ticks += 1
        if off is not None and abs(off) > 0.04 and self.sq_ticks < self.square_ticks:
            self.drive(0.0, 0.0, max(-0.4, min(0.4, 1.5 * off)))
            return
        self.stop()
        if off is not None and abs(off) > self.square_ok_deg and self.sq_ticks >= self.square_ticks:
            self.get_logger().warn(f"wall {self.seg_i + 1}: could NOT square up (residual "
                                   f"{math.degrees(off):.0f}deg after {self.square_ticks * 0.1:.0f}s); "
                                   f"scanning DEGRADED (vy may not be parallel)")
        p, y = self.pose()
        if p is None:
            if self.sq_ticks > self.square_ticks + 50:
                self.get_logger().warn(f"wall {self.seg_i + 1}: no pose after square-up; SKIP"); self._advance()
            return
        self.lat_axis = np.array([math.cos(y + math.pi / 2), math.sin(y + math.pi / 2)])   # robot's left
        self.sweep_start = p
        seg = self._cur()
        self.sweep_target = float(np.dot(np.array(seg["e"]) - p, self.lat_axis))
        self.sweep_sign = 1.0 if self.sweep_target >= 0 else -1.0
        self.scan_ticks = 0; self.scan_best = 0.0; self.stall_ticks = 0; self.nopose_ticks = 0
        self.cooldown = 0; self.cap_ticks = 0
        self._cap_mark = None; self._cap_cand = None; self._cap_cand_xy = None
        self.scan_budget = int(abs(self.sweep_target) / max(self.strafe_speed, 1e-3) / 0.1 * 1.8) + 30
        self.get_logger().info(f"wall {self.seg_i + 1}: squared up; SCAN {self.sweep_target:+.2f}m lateral ->")
        self.state = "SEG_SCAN"

    def _lateral(self):
        p, _ = self.pose()
        return None if p is None else float(np.dot(p - self.sweep_start, self.lat_axis))

    def _lateral_clear(self, sign):
        """Min /scan range in a +-20deg arc centred on the CURRENT strafe direction -> halts the open-loop
        strafe before a sideways (corner) collision."""
        return self._arc_min(math.copysign(math.pi / 2, sign), math.radians(20))

    def _arc_min(self, centre, arc):
        m = self.scan
        if m is None:
            return float("inf")
        best = float("inf")
        for j, r in enumerate(m.ranges):
            a = m.angle_min + j * m.angle_increment
            if abs(ang_diff(a, centre)) <= arc and m.range_min < r < m.range_max and math.isfinite(r):
                best = min(best, r)
        return best

    def _gauge_map_xy(self, bbox):
        """Approx map (x,y) of a detected object: robot pose + (front lidar range) along the detection
        bearing. Used ONLY for dedup (same gauge crossing the FOV), so coarse is fine."""
        p, y = self.pose()
        if p is None:
            return None
        f = self.front if (self.front is not None and self.front > 0.2) else self.standoff
        if self.K and self.W:
            fx, _, cx, _ = self.K
            u = 0.5 * (bbox[0] + bbox[2])
            beta = math.atan2((u - cx), fx)              # +ve = object to image-right
        else:
            beta = 0.0
        ang = y - beta                                   # image-right = robot-right = yaw - beta
        rng = f / max(0.3, math.cos(beta))
        return [float(p[0] + rng * math.cos(ang)), float(p[1] + rng * math.sin(ang))]

    def _along_wall(self):
        """Signed position of the robot along the current wall A->B (map metres). Direction-independent of
        the strafe sign -> a stable left->right ordering key for downstream scoring."""
        seg = self._cur()
        A = np.array(seg["A"], float); B = np.array(seg["B"], float)
        d = B - A; L = float(np.linalg.norm(d))
        if L < 1e-6:
            return 0.0
        p, _ = self.pose()
        if p is None:
            return 0.0
        return float(np.dot(p - A, d / L))

    def _best_centered(self, dets, W):
        """Highest-confidence detection whose horizontal centre is within the central center_frac band."""
        if not dets or not W:
            return None
        lo = W * (0.5 - self.center_frac / 2.0); hi = W * (0.5 + self.center_frac / 2.0)
        best = None
        for (typ, conf, bbox) in dets:
            u = 0.5 * (bbox[0] + bbox[2])
            if lo <= u <= hi and (best is None or conf > best[1]):
                best = (typ, conf, bbox)
        return best

    def _pick_for_capture(self, dets, W):
        """Prefer the detection that matches the object which TRIGGERED the stop (same type, nearest bbox
        centre) so on a multi-instrument wall we crop the intended object, not the global best one."""
        if not dets or not W:
            return None
        if self._cap_cand is not None:
            ct, _, cb = self._cap_cand
            ccx = 0.5 * (cb[0] + cb[2])
            pool = [d for d in dets if d[0] == ct] or dets
            best = min(pool, key=lambda d: abs(0.5 * (d[2][0] + d[2][2]) - ccx))
            if abs(0.5 * (best[2][0] + best[2][2]) - ccx) <= W * 0.25:
                return best
        return self._best_centered(dets, W)

    def _st_SEG_SCAN(self):
        if self.cooldown > 0:
            self.cooldown -= 1
        lat = self._lateral()
        if lat is None:                                  # TF/localization dropout -> HALT
            self.stop(); self.nopose_ticks += 1
            if self.nopose_ticks > 15:
                self.get_logger().warn(f"wall {self.seg_i + 1}: lost localization mid-scan; end segment")
                self._advance()
            return
        self.nopose_ticks = 0
        if self.front is not None and self.front < self.front_stop:
            self.stop()
            self.get_logger().warn(f"wall {self.seg_i + 1}: front {self.front:.2f}m < {self.front_stop:.2f}m e-stop; end segment")
            self._advance(); return
        self.scan_ticks += 1
        if abs(lat) > self.scan_best + 0.02:
            self.scan_best = abs(lat); self.stall_ticks = 0
        else:
            self.stall_ticks += 1
        if self.scan_ticks > self.scan_budget or self.stall_ticks > 30:
            self.stop()
            why = "stalled (no translation)" if self.stall_ticks > 30 else "watchdog timeout"
            self.get_logger().warn(f"wall {self.seg_i + 1}: SCAN {why}; end segment")
            self._advance(); return
        # live detection trigger: a confident, centred, not-yet-captured object ahead -> STOP & capture
        if self.cooldown == 0 and self.model is not None:
            with self._det_lock:
                dets = list(self.live_dets)
            cand = self._best_centered(dets, self.W)
            if cand is not None:
                xy = self._gauge_map_xy(cand[2])
                dup = xy is not None and any(math.dist(xy, c) < self.dedup_radius for c in self.captured)
                if not dup:
                    self.stop()
                    self._cap_cand = cand                 # carry the trigger forward into CAPTURE
                    self._cap_cand_xy = xy                # remember for dedup even if the crop fails
                    self._cap_mark = None                 # the inference snapshot is taken AFTER settle
                    self.cap_ticks = 0
                    self.get_logger().info(f"wall {self.seg_i + 1}: {cand[0]} ({cand[1]:.2f}) ahead; STOP to capture")
                    self.state = "SEG_CAPTURE"; return
        # otherwise keep strafing along the wall (vy) while facing it (vyaw); NO vx
        if abs(lat) < abs(self.sweep_target) - 0.05:
            clr = self._lateral_clear(self.sweep_sign)
            if clr < self.side_clear:
                self.stop()
                self.get_logger().warn(f"wall {self.seg_i + 1}: lateral obstacle {clr:.2f}m < {self.side_clear:.2f}m; end segment")
                self._advance(); return
            off = self._wall_offset()
            vyaw = 0.0 if off is None else max(-self.max_vyaw, min(self.max_vyaw, self.scan_yaw_gain * off))
            self.drive(0.0, self.sweep_sign * self.strafe_speed, vyaw)
        else:
            self.stop()
            self.get_logger().info(f"wall {self.seg_i + 1}: SCAN done; end segment")
            self._advance()

    def _st_SEG_CAPTURE(self):
        """Stationary capture: stop, let the gait settle, snapshot the inference counter AFTER settling, wait
        for a FRESH stationary inference, crop the object that triggered the stop, save it, remember its
        position (dedup), then resume the strafe. ALWAYS records a dedup position (even on failure) so a
        flickering detection can't loop the robot in place."""
        self.stop()
        self.cap_ticks += 1
        if self.cap_ticks < self.settle_ticks:
            return
        if self._cap_mark is None:                       # take the freshness mark only after the gait settled
            with self._det_lock:
                self._cap_mark = self._infer_count
        if self.cap_ticks > self.capture_timeout:        # inference stalled/too slow -> give up, keep scanning
            self.get_logger().warn(f"wall {self.seg_i + 1}: capture timed out; continue")
            self._end_capture(False); return
        with self._det_lock:
            fresh = self._cap_mark is not None and self._infer_count >= self._cap_mark + 2
            dets = list(self.live_dets) if fresh else None
            img = None if (not fresh or self.live_img is None) else self.live_img.copy()
        if not fresh:
            return                                       # wait for inferences taken AFTER the stop settled
        saved = False
        pick = self._pick_for_capture(dets, img.shape[1] if img is not None else self.W)
        if pick is not None and img is not None:
            saved = self._save_capture(img, pick)
        if not saved:
            self.get_logger().info(f"wall {self.seg_i + 1}: no stationary detection on capture; continue")
        self._end_capture(saved)

    def _end_capture(self, saved):
        # On a FAILED capture, _save_capture didn't record a dedup position -> record the trigger's xy here so
        # the strafe moves past the object instead of re-triggering at the same spot.
        if not saved and self._cap_cand_xy is not None:
            self.captured.append(self._cap_cand_xy)
        self.cooldown = self.cooldown_ticks
        self._cap_mark = None; self._cap_cand = None; self._cap_cand_xy = None
        self.state = "SEG_SCAN"

    def _save_capture(self, img_rgb, det):
        typ, conf, bbox = det
        H, W = img_rgb.shape[:2]
        x0, y0, x1, y1 = bbox
        pw = int((x1 - x0) * self.capture_pad); ph = int((y1 - y0) * self.capture_pad)
        x0 = max(0, x0 - pw); y0 = max(0, y0 - ph); x1 = min(W, x1 + pw); y1 = min(H, y1 + ph)
        if x1 <= x0 or y1 <= y0:
            return False
        crop_bgr = cv2.cvtColor(img_rgb[y0:y1, x0:x1].copy(), cv2.COLOR_RGB2BGR)
        i = len(self.detections)
        fn = f"{typ.replace(' ', '_')}_S{self.seg_i:02d}_{i:02d}.png"
        gdir = os.path.join(self.out_dir, self.zone_id, "gauges"); os.makedirs(gdir, exist_ok=True)
        cv2.imwrite(os.path.join(gdir, fn), crop_bgr)
        xy = self._gauge_map_xy(bbox)
        if xy is not None:
            self.captured.append(xy)
        # spatial, strafe-direction-independent ordering key (position along the wall A->B), so downstream
        # left->right scoring matches regardless of which way we strafed the wall.
        lateral = self.seg_i * 100000 + int(self._along_wall() * 1000)
        self.detections.append({
            "id": f"{self.zone_id}_S{self.seg_i:02d}_{i:02d}", "zone": self.zone_id, "segment": self.seg_i,
            "type": typ, "conf": round(float(conf), 3), "bbox": [int(x0), int(y0), int(x1), int(y1)],
            "map_xy": [round(xy[0], 3), round(xy[1], 3)] if xy else None,
            "lateral": lateral, "file": f"gauges/{fn}"})
        self.get_logger().info(f"wall {self.seg_i + 1}: CAPTURED {typ} ({conf:.2f}) -> {fn}")
        return True

    def _advance(self):
        self.seg_i += 1
        if self.seg_i < len(self.segments):
            self._begin_segment()
        else:
            self.bo_ticks = 0; self.state = "BACKOFF"

    def _st_BACKOFF(self):
        """Back off the last wall to a nav-safe distance (outside the costmap inflation) so the next mission
        action can plan from here. Backs straight out (vx<0) -- the ONLY vx motion, after all scanning is
        done -- gated on REAR clearance so it can't reverse into the opposite wall in a small room."""
        self.bo_ticks += 1
        rear = self._arc_min(math.pi, math.radians(25))
        if (self.front is not None and self.front < self.backoff_dist and self.bo_ticks < 120
                and rear > self.side_clear):
            self.drive(-self.strafe_speed, 0.0)
        else:
            self.stop()
            self._finish()

    def _finish(self):
        d = os.path.join(self.out_dir, self.zone_id); os.makedirs(d, exist_ok=True)
        avail = self.model is not None
        json.dump({"zone": self.zone_id, "available": avail,
                   "reason": "" if avail else self.model_reason, "failed": self.failed,
                   "n_detections": len(self.detections), "detections": self.detections},
                  open(os.path.join(d, "detections.json"), "w"), indent=2)
        gauges = [{"id": x["id"], "zone": self.zone_id, "type": x["type"], "file": x["file"],
                   "segment": x["segment"], "lateral": x["lateral"], "conf": x["conf"], "map_xy": x.get("map_xy")}
                  for x in self.detections if "gauge" in x["type"]]
        json.dump({"zone": self.zone_id, "n_gauges": len(gauges), "gauges": gauges},
                  open(os.path.join(d, "gauges.json"), "w"), indent=2)
        try:
            from go2_inspection.yoloe_segmenter import _contact_sheet
            _contact_sheet(d, self.detections)
        except Exception as e:
            self.get_logger().warn(f"contact sheet failed: {e}")
        self._stop_infer = True
        tag = "ABORTED (sensor/localization fault)" if self.failed else "DONE"
        self.get_logger().info(f"{self.zone_id}: wall-scan {tag} -- {len(self.detections)} objects "
                               f"({len(gauges)} gauges) -> {d}")
        self.state = "DONE"


def main(args=None):
    import sys
    rclpy.init(args=args)
    n = ZoneWallFollower()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        while rclpy.ok() and n.state != "DONE":
            ex.spin_once(timeout_sec=0.1)
        n.get_logger().info("wall-scan complete; shutting down")
    except KeyboardInterrupt:
        pass
    # an interrupted / never-DONE run is a FAILURE (the run-once contract: DONE means _finish() wrote the
    # report) -> non-zero exit so the mission orchestrator doesn't treat a partial zone as success.
    failed = n.failed or n.state != "DONE"
    n._stop_infer = True
    if n._infer_thread is not None:
        n._infer_thread.join(timeout=10.0)
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
