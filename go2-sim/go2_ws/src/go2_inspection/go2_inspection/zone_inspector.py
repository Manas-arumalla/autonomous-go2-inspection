#!/usr/bin/env python3
"""zone_inspector -- VIEWPOINT + 360-degree SPIN camera inspection of ONE zone.

Replaces the old wall-follower (which only faced walls and missed room-interior objects). For a zone we:
  BUILD    : load the zone polygon, sample a few safe observation viewpoints inside it (eroded by a
             safety margin so the robot stays clear of walls).
  VP_NAV   : Nav2 (NavigateToPose) to each viewpoint. If Nav2 rejects/aborts far from the goal, SKIP it.
  VP_SPIN  : slow in-place 360-degree yaw (publish /cmd_vel directly, NOT Nav2) while YOLOE open-vocab
             detection runs CONTINUOUSLY in a background thread. Every detection is projected to the MAP
             frame via the depth camera, de-duplicated across the whole zone (class + world position),
             and the best-confidence crop is kept.
After the last viewpoint: write detections.json (every observation), objects.json (deduped uniques with
world xyz + n_observations), an objects contact sheet, an annotated zone_map.png, and report.md/.csv.

Covers walls AND interior props (drums/pallets/crates/fire/person), not just wall-mounted instruments.
Degrades gracefully: no YOLOE weights / CLIP backend -> still navigates + spins every viewpoint and
writes an empty (available:false) result, exit 0. Run-once contract: exit 0 on DONE, 1 on abort.

  ros2 run go2_inspection zone_inspector --ros-args -p zone_id:=zone_0 -p zones_file:=...
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
from sensor_msgs.msg import Image, CameraInfo
from nav2_msgs.action import NavigateToPose, ComputePathToPose
import tf2_ros

from go2_inspection import detect_utils
from go2_inspection import report_utils

GAUGES_ROOT = os.path.expanduser("~/gauges")  # output root (kept; readers expect it)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def quat_to_R(x, y, z, w):
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class ZoneInspector(Node):
    def __init__(self):
        super().__init__("zone_inspector")
        self.zone_id = self.declare_parameter("zone_id", "zone_0").value
        self.zones_file = os.path.expanduser(self.declare_parameter("zones_file", "").value)
        self.out_dir = os.path.expanduser(self.declare_parameter("out_dir", "~/gauges").value)
        self.map_yaml = self.declare_parameter("map_yaml", report_utils.DEFAULT_MAP_YAML).value
        # --- viewpoints / motion ---
        self.safety_margin = float(
            self.declare_parameter("safety_margin", 0.5).value
        )  # erode poly (m)
        self.vp_spacing = float(
            self.declare_parameter("vp_spacing", 2.5).value
        )  # grid spacing (m). Was 3.0; densified so a far-corner gauge is also seen from a CLOSER viewpoint
        #    (closer => denser depth => more localized observations), improving recall without lowering any
        #    detection threshold.
        self.max_viewpoints = int(self.declare_parameter("max_viewpoints", 5).value)  # was 4
        self.grid_res = float(self.declare_parameter("grid_res", 0.1).value)  # poly raster res
        self.spin_speed = float(
            self.declare_parameter("spin_speed", 0.4).value
        )  # rad/s in-place yaw
        self.spin_overlap = math.radians(
            float(self.declare_parameter("spin_overlap_deg", 20.0).value)
        )
        self.nav_timeout = float(self.declare_parameter("nav_timeout", 120.0).value)
        self.build_timeout = float(self.declare_parameter("build_timeout", 20.0).value)
        self.max_nav_retries = int(
            self.declare_parameter("max_nav_retries", 2).value
        )  # retry transient aborts
        self.nav_settle = float(
            self.declare_parameter("nav_settle", 1.0).value
        )  # s; cooldown before a goal
        # --- detection (defaults are the tuner-exported values; see detect_utils.PROMPTS/TUNED) ---
        self.det_conf = float(
            self.declare_parameter("det_conf", 0.40).value
        )  # confidence floor: keep only
        #   the stronger detections to limit low-confidence false positives; the location dedup plus the
        #   persistence/position checks recover the rest.
        self.det_iou = float(self.declare_parameter("det_iou", detect_utils.TUNED["iou"]).value)
        self.det_imgsz = int(
            self.declare_parameter("det_imgsz", detect_utils.TUNED["imgsz"]).value
        )
        self.det_max_det = int(
            self.declare_parameter("det_max_det", detect_utils.TUNED["max_det"]).value
        )
        self.det_weights = self.declare_parameter(
            "det_weights", detect_utils.DEFAULT_WEIGHTS
        ).value
        self.det_device = self.declare_parameter(
            "det_device", os.environ.get("YOLOE_DEVICE", "")
        ).value
        self.center_only = bool(
            self.declare_parameter("center_only", False).value
        )  # spin sees all bearings
        # --- depth -> map projection ---
        self.max_depth = float(self.declare_parameter("max_depth", 6.0).value)
        # Depth-localization gate. The OLD rule "≥30% of the bbox depth-patch pixels are valid" is far too
        # strict for a SMALL bbox (a gauge seen across the room subtends few pixels, and a thin round dial
        # has sparse/edge-holed depth) -- so a real, correctly-seen gauge localized on only ~1 frame and got
        # filtered by min_observations. What actually makes the median depth reliable
        # is an ABSOLUTE count of valid samples, not a fraction of a tiny patch. So require
        # max(min_valid_px, min_valid_frac·patch): the absolute floor protects tiny bboxes, the (lowered)
        # fraction still guards a large mostly-invalid patch. Principled (geometry of distant objects), not
        # tuned to any target.
        self.min_valid_frac = float(self.declare_parameter("min_valid_frac", 0.15).value)
        self.min_valid_px = int(self.declare_parameter("min_valid_px", 12).value)
        self.dedup_radius = float(self.declare_parameter("dedup_radius", 0.6).value)
        # Final-pass consolidation of localization-noise duplicates. A weak (few-frame) detection can land
        # ~1 m off a strong one of the SAME object (noisy median depth), escaping dedup_radius -> two
        # markers for one gauge. Absorb a weak detection into a MUCH stronger same-class one within
        # consolidate_radius, but ONLY when the weak has <= consolidate_obs_frac of the strong's
        # observations -- so two comparably-well-seen DISTINCT objects are never merged (recall preserved).
        # Uses only observation counts + geometry; no ground truth.
        self.consolidate_radius = float(self.declare_parameter("consolidate_radius", 1.5).value)
        # weak := seen <= consolidate_obs_frac of the stronger one's frames. The looser depth gate that
        # fixed recall (min_valid_px) also lets a few sparse-depth frames localize an object ~0.7-1 m off
        # (noisy Z) -> a weak outlier of the SAME gauge. Across runs these outliers are 0.11-0.37 of the
        # strong detection's observations; 0.5 absorbs them with margin while still keeping two
        # comparably-well-seen DISTINCT gauges separate (the 1.5 m radius is the primary same-object prior;
        # gauges in a facility are metres apart). Recall-safe.
        self.consolidate_obs_frac = float(self.declare_parameter("consolidate_obs_frac", 0.5).value)
        self.capture_pad = float(self.declare_parameter("capture_pad", 0.10).value)
        # detect-then-approach (ADR-017): after the survey/spin localizes gauges, drive to a CLOSE,
        # fronto-parallel, resolution-driven standoff per gauge and grab a high-res read crop. Default OFF
        # (current spin-only behaviour unchanged). Scales reading to large rooms (the spin can't).
        self.read_approach = bool(self.declare_parameter("read_approach", False).value)
        self.read_target_px = float(self.declare_parameter("read_target_px", 120.0).value)  # px on the dial
        self.read_asset_size = float(self.declare_parameter("read_asset_size", 0.26).value)  # gauge size (m)
        self.read_dmin = float(self.declare_parameter("read_dmin", 0.5).value)
        self.read_dmax = float(self.declare_parameter("read_dmax", 1.2).value)
        self.read_arc_deg = float(self.declare_parameter("read_arc_deg", 60.0).value)
        self.read_burst = int(self.declare_parameter("read_burst", 5).value)  # frames -> pick the sharpest
        # reachability clearance for the READING pose = the robot's footprint half-width + margin (~0.3 m).
        # Deliberately NOT obstacle_check_radius (1.2 m, for phantom-rejection): a read pose is ~0.8 m off
        # the wall, so a 1.2 m clearance would reject every near-wall reading pose.
        self.read_clearance = float(self.declare_parameter("read_clearance", 0.30).value)
        # next-best-view: if a close read is poor (gauge too small / not detected / blurry), re-approach
        # from the next candidate angle. Keeps the BEST crop across attempts (never makes it worse).
        self.read_max_attempts = int(self.declare_parameter("read_max_attempts", 2).value)
        # opt-in false-positive rejection: drop a gauge the close approach REACHED + photographed but could
        # not re-detect (read_confirmed==False). Default OFF (a real gauge could read 0 if out of the close
        # FOV / occluded) -- read_confirmed is always recorded so the data stays honest either way.
        self.read_drop_unconfirmed = bool(self.declare_parameter("read_drop_unconfirmed", False).value)
        # settle time at the read pose before capturing -- lets the body level out after stopping (CHAMP
        # pitches nose-up on braking, which can tilt the low fixed camera up off a low gauge).
        self.read_settle = float(self.declare_parameter("read_settle", 1.5).value)
        self.optical_frame = self.declare_parameter("optical_frame", "camera_link_optical").value
        # --- reliability: detect only while SPINNING (walking blurs the feed + mislocalizes), and double-
        #     check every projected position against the zone polygon + the saved occupancy map ---
        self.spin_settle = float(
            self.declare_parameter("spin_settle", 0.4).value
        )  # s; skip NAV->stop blur
        self.validate_map = bool(self.declare_parameter("validate_map", True).value)
        self.obstacle_check_radius = float(
            self.declare_parameter("obstacle_check_radius", 1.2).value
        )  # m;
        #   a localized non-dynamic detection must be within this of a mapped obstacle/unknown cell. 1.2 m is
        #   generous for furniture-sized props that may project onto a cell the (possibly stale) map marks free
        #   -- it still rejects detections floating far out in open space. Lower it to tighten the check.
        self.zone_margin = float(
            self.declare_parameter("zone_margin", 1.0).value
        )  # m outside poly allowed
        self.min_observations = int(
            self.declare_parameter("min_observations", 2).value
        )  # drop 1-frame phantoms

        self.zone_center = [0.0, 0.0]
        self.zone_polygon = []
        self.zone_nav = None
        self.zone_label = ""
        if self.zones_file and os.path.exists(self.zones_file):
            self._load_zone()
        else:
            self.get_logger().error(f"zones_file missing: {self.zones_file}")
            raise SystemExit(1)

        # sensors / actuators
        self.K = None
        self.W = None
        self.H = None
        self.img = None
        self.img_stamp = None
        self.depth = None
        self._img_lock = threading.Lock()
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        sens = ReentrantCallbackGroup()
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._ci, 10, callback_group=sens
        )
        self.create_subscription(
            Image, "/camera/image_raw", self._im, qos_profile_sensor_data, callback_group=sens
        )
        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self._dep,
            qos_profile_sensor_data,
            callback_group=sens,
        )
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        # nav-reachability hardening: pre-check each survey viewpoint with Nav2's global planner and drop
        # the unreachable ones (so the robot doesn't waste time timing-out toward an unreachable viewpoint).
        self.vp_reach_check = bool(self.declare_parameter("vp_reach_check", True).value)
        self.vp_reach_timeout = float(self.declare_parameter("vp_reach_timeout", 8.0).value)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.reach = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._reach_gen = 0

        # detection model + background inference thread. _capture gates inference to the SPIN phase only:
        # the thread idles during BUILD/VP_NAV so no walking/transition frame is ever inferred (and CPU is
        # freed for the planner during travel). Plain bool is fine across threads under the GIL.
        self.model = None
        self.model_reason = ""
        self._det_lock = threading.Lock()
        self.live_dets = []
        self.live_img = None
        self.live_stamp = None
        self._infer_count = 0
        self._stop_infer = False
        self._infer_thread = None
        self._capture = False
        self._load_detector()

        # occupancy map for position double-checking (loaded lazily in BUILD once TF/params are ready)
        self._plausible = None
        self._occ_gray = None
        self._map_res = self._map_ox = self._map_oy = None
        self._read_targets = []
        self._read_i = 0
        self._map_H = self._map_W = None

        # CPU inference at imgsz 1280 is slow (~1-3 s/frame) -> only ~1-3 frames per object during a 0.4 rad/s
        # spin, so the default min_observations=2 could drop a real prop glimpsed at a single viewpoint. On
        # CPU relax to 1 (on GPU 2 correctly kills 1-frame phantoms).
        if self.model is not None and self.min_observations > 1:
            try:
                import torch

                cpu = self.det_device == "cpu" or (
                    self.det_device == "" and not torch.cuda.is_available()
                )
            except Exception:
                cpu = self.det_device == "cpu"
            if cpu:
                self.min_observations = 1
                self.get_logger().info("CPU inference -> min_observations=1 (low spin frame rate)")

        # FSM state
        self.viewpoints = []
        self.vp_i = 0
        self.objects = []  # deduped uniques (map xy keyed)
        self.detections = []  # every accepted observation
        self.state = "BUILD"
        self.failed = False
        self.nav_done = None
        self.nav_status = None
        self._nav_gh = None
        self._nav_gen = 0
        self.nav_retries = 0
        self.nav_settle_ticks = 0
        self.build_ticks = 0
        self.nav_ticks = 0
        self.spin_yaw0 = None
        self.spin_last = None
        self.spin_accum = 0.0
        self.spin_ticks = 0
        self._proc_mark = 0
        self._clean_outputs()
        self.create_timer(0.1, self._tick, callback_group=MutuallyExclusiveCallbackGroup())
        self._start_infer()
        self.get_logger().info(
            f"zone_inspector ready: zone={self.zone_id} ({self.zone_label}), "
            f"{len(self.zone_polygon)}-pt polygon, YOLOE="
            f"{'on' if self.model else 'OFF (' + self.model_reason + ')'}"
        )

    # ---------- detection ----------
    def _load_detector(self):
        try:
            self.model = detect_utils.load_model(
                self.det_weights, self.det_device, detect_utils.PROMPTS
            )
            self.get_logger().info(
                f"YOLOE loaded: {self.det_weights} ({len(detect_utils.PROMPTS)} prompts)"
            )
        except Exception as e:
            self.model_reason = f"{type(e).__name__}: {e}"
            self.get_logger().warn(
                f"YOLOE disabled: {self.model_reason}; will scan but capture nothing"
            )

    def _start_infer(self):
        if self.model is None or self._infer_thread is not None:
            return
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

    def _infer_loop(self):
        while not self._stop_infer and rclpy.ok():
            if not self._capture:  # only infer while SPINNING (see _capture note)
                time.sleep(0.05)
                continue
            with self._img_lock:
                img = None if self.img is None else self.img.copy()
                stamp = self.img_stamp
            if img is None:
                time.sleep(0.05)
                continue
            try:
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                dets = detect_utils.infer(
                    self.model,
                    img_bgr,
                    self.det_conf,
                    imgsz=self.det_imgsz,
                    iou=self.det_iou,
                    max_det=self.det_max_det,
                    device=self.det_device,
                )
            except Exception as e:
                if not self._stop_infer:
                    self.get_logger().warn(f"inference error: {e}", throttle_duration_sec=5.0)
                dets = []
            with self._det_lock:
                self.live_dets = dets
                self.live_img = img
                self.live_stamp = stamp
                self._infer_count += 1
            time.sleep(0.02)

    # ---------- outputs ----------
    def _clean_outputs(self):
        d = os.path.join(self.out_dir, self.zone_id)
        if not os.path.isdir(d):
            return
        for p in glob.glob(os.path.join(d, "crops", "*.png")) + [
            os.path.join(d, n)
            for n in (
                "detections.json",
                "objects.json",
                "objects_contact_sheet.png",
                "zone_map.png",
                "report.md",
                "report.csv",
            )
        ]:
            try:
                os.remove(p)
            except OSError:
                pass

    # ---------- zone / sensors ----------
    def _load_zone(self):
        z = next(
            (z for z in json.load(open(self.zones_file))["zones"] if z["id"] == self.zone_id), None
        )
        if z is None:
            self.get_logger().error(f"{self.zone_id} not in {self.zones_file}")
            raise SystemExit(1)
        self.zone_center = z["center"]
        self.zone_polygon = z.get("polygon", [])
        self.zone_nav = z.get("nav_point")
        self.zone_label = z.get("label", "")

    def _load_validation_map(self):
        """Load the saved occupancy grid and build a 'plausible object location' mask = (occupied OR
        unknown) cells dilated by obstacle_check_radius. A localized detection that lands OUTSIDE this mask
        is floating in known-free space (a bad projection / phantom) and gets rejected (dynamic classes
        exempt). Failure to load just disables the map check (zone-polygon check still applies)."""
        if not self.validate_map:
            return
        try:
            gray, res, ox, oy, H, W = report_utils.load_occupancy(self.map_yaml)
        except Exception as e:
            self.get_logger().warn(f"map position-check disabled (load failed: {e})")
            self.validate_map = False
            return
        self._occ_gray = gray  # raw occupancy (free=254/occ=0/unknown=205) — for wall-normal estimation
        not_free = (gray < 250).astype(np.uint8)  # occupied(0) + unknown(205); free is 254
        r = max(1, int(self.obstacle_check_radius / max(res, 1e-6)))
        self._plausible = cv2.dilate(
            not_free, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        )
        self._map_res, self._map_ox, self._map_oy, self._map_H, self._map_W = res, ox, oy, H, W
        self.get_logger().info(
            f"position double-check ON: zone polygon (+{self.zone_margin:.1f} m) + "
            f"occupancy within {self.obstacle_check_radius:.1f} m of a mapped obstacle"
        )

    def _ci(self, m):
        if self.K is None and m.k[0] > 0.0:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])
            self.W, self.H = m.width, m.height

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
                img = np.ascontiguousarray(img[:, :, ::-1])
            elif enc != "rgb8":
                self.get_logger().warn(
                    f"unsupported camera encoding '{m.encoding}'", throttle_duration_sec=10.0
                )
                return
        except Exception as e:
            self.get_logger().warn(f"image decode failed: {e}", throttle_duration_sec=10.0)
            return
        with self._img_lock:
            self.img = img
            self.img_stamp = m.header.stamp
            if self.W is None:
                self.W, self.H = m.width, m.height

    def _dep(self, m):
        """Decode the registered 32FC1 depth image (metres, aligned to RGB)."""
        if (m.encoding or "32FC1").upper() != "32FC1":
            self.get_logger().warn(
                f"unexpected depth encoding '{m.encoding}'", throttle_duration_sec=10.0
            )
            return
        try:
            d = np.frombuffer(m.data, dtype=np.float32)
            if d.size < m.width * m.height:
                return
            self.depth = d[: m.width * m.height].reshape(m.height, m.width)
        except Exception:
            return

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            return np.array([t.translation.x, t.translation.y]), yaw_of(t.rotation)
        except Exception:
            return None, None

    def stop(self):
        self.cmd.publish(Twist())

    def drive(self, vx, vy, vyaw=0.0):
        t = Twist()
        t.linear.x = float(vx)
        t.linear.y = float(vy)
        t.angular.z = float(vyaw)
        self.cmd.publish(t)

    def _tick(self):
        getattr(self, "_st_" + self.state, lambda: None)()

    # ---------- viewpoints ----------
    def _sample_viewpoints(self):
        """Erode the zone polygon by safety_margin; sample a grid of interior points (+ nav_point),
        return up to max_viewpoints ordered nearest-first from the current robot pose."""
        poly = self.zone_polygon
        if len(poly) < 3:
            return [tuple(self.zone_nav or self.zone_center)]
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        res = self.grid_res
        pad = self.safety_margin + 0.2
        minx, miny = min(xs) - pad, min(ys) - pad
        Wp = int((max(xs) + pad - minx) / res) + 1
        Hp = int((max(ys) + pad - miny) / res) + 1
        mask = np.zeros((Hp, Wp), np.uint8)
        pts = np.array([[int((x - minx) / res), int((y - miny) / res)] for x, y in poly], np.int32)
        cv2.fillPoly(mask, [pts], 1)
        r = max(1, int(self.safety_margin / res))
        er = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))

        def to_world(cx, cy):
            return (minx + cx * res, miny + cy * res)

        def in_eroded(x, y):
            cx = int((x - minx) / res)
            cy = int((y - miny) / res)
            return 0 <= cx < Wp and 0 <= cy < Hp and er[cy, cx] > 0

        cands = []
        if self.zone_nav and in_eroded(*self.zone_nav):
            cands.append(tuple(self.zone_nav))
        step = max(1, int(self.vp_spacing / res))
        for cy in range(0, Hp, step):
            for cx in range(0, Wp, step):
                if er[cy, cx] > 0:
                    cands.append(to_world(cx, cy))
        if not cands:  # eroded mask empty (thin/small zone) -> deepest free point
            if er.sum() == 0:
                cands = [tuple(self.zone_nav or self.zone_center)]
            else:
                ys2, xs2 = np.where(er > 0)
                cands = [to_world(int(xs2.mean()), int(ys2.mean()))]
        # dedup within ~1.5 m, keep order (nav_point first)
        uniq = []
        for c in cands:
            if all(math.dist(c, u) > 1.5 for u in uniq):
                uniq.append(c)
        # order nearest-first from robot
        p, _ = self.pose()
        if p is not None:
            uniq.sort(key=lambda c: math.dist(c, (float(p[0]), float(p[1]))))
        return uniq[: self.max_viewpoints]

    # ---------- FSM ----------
    def _st_BUILD(self):
        p, _ = self.pose()
        if p is None:
            self.build_ticks += 1
            if self.build_ticks * 0.1 > self.build_timeout:
                self.get_logger().error("no map->base_link TF (localization down?); ABORT")
                self.failed = True
                self._finish()
            return
        self._load_validation_map()
        self.viewpoints = self._sample_viewpoints()
        if not self.viewpoints:
            self.get_logger().warn(f"{self.zone_id}: no viewpoints; DONE")
            self._finish()
            return
        self.get_logger().info(
            f"{self.zone_id}: {len(self.viewpoints)} viewpoints {[(round(x,1),round(y,1)) for x,y in self.viewpoints]}"
        )
        self.vp_i = 0
        if self.vp_reach_check and self.reach.server_is_ready():
            self._begin_reach_check()  # drop unreachable viewpoints + order by Nav2 path cost
        else:
            self._begin_viewpoint()

    # ---------- nav-reachability pre-check (Nav2 global planner) ----------
    def _begin_reach_check(self):
        """Ask Nav2's global planner (ComputePathToPose) for a path to each sampled viewpoint from the
        robot's current pose; DROP viewpoints with no valid path, ORDER the rest by path length (shortest
        first). Stops the robot wasting time toward unreachable viewpoints (the cross-maze timeouts) and
        visits the cheapest-to-reach first. Falls back to all viewpoints if the planner rejects everything
        (never strand with zero)."""
        self._reach_i = 0
        self._reach_scored = []
        self._reach_done = None
        self._reach_ticks = 0
        self._reach_gen += 1
        self.state = "REACH_CHECK"
        self.get_logger().info(
            f"{self.zone_id}: reachability pre-check of {len(self.viewpoints)} viewpoint(s)"
        )

    def _st_REACH_CHECK(self):
        self.stop()
        if self._reach_i >= len(self.viewpoints):
            self._finish_reach_check()
            return
        vx, vy = self.viewpoints[self._reach_i]
        if self._reach_done is None:
            g = ComputePathToPose.Goal()
            g.goal.header.frame_id = "map"
            g.goal.header.stamp = rclpy.time.Time().to_msg()
            g.goal.pose.position.x = float(vx)
            g.goal.pose.position.y = float(vy)
            g.goal.pose.orientation.w = 1.0
            g.use_start = False  # plan from the robot's current pose
            self._reach_done = False
            self._reach_ticks = 0
            gen, vp = self._reach_gen, (vx, vy)
            self.reach.send_goal_async(g).add_done_callback(lambda fut: self._reach_acc(fut, gen, vp))
            return
        self._reach_ticks += 1
        if self._reach_done in (True, "reject"):
            self._reach_i += 1
            self._reach_done = None
            return
        if self._reach_ticks * 0.1 > self.vp_reach_timeout:  # planner slow -> keep it but rank last
            self._reach_scored.append(((vx, vy), 1e6))
            self.get_logger().warn(f"  viewpoint {self._reach_i + 1}: reach-check timeout; keep (rank last)")
            self._reach_i += 1
            self._reach_done = None

    def _reach_acc(self, fut, gen, vp):
        if gen != self._reach_gen:
            return
        try:
            h = fut.result()
        except Exception:
            self._reach_done = "reject"
            return
        if not h.accepted:
            self._reach_done = "reject"
            return

        def _res(f):
            if gen != self._reach_gen:
                return
            try:
                r = f.result()
                path = r.result.path
                if r.status == 4 and path.poses and len(path.poses) >= 2:
                    length = sum(
                        math.hypot(
                            b.pose.position.x - a.pose.position.x,
                            b.pose.position.y - a.pose.position.y,
                        )
                        for a, b in zip(path.poses[:-1], path.poses[1:])
                    )
                    self._reach_scored.append((vp, length))
                    self._reach_done = True
                else:
                    self._reach_done = "reject"
            except Exception:
                self._reach_done = "reject"

        h.get_result_async().add_done_callback(_res)

    def _finish_reach_check(self):
        if self._reach_scored:
            self._reach_scored.sort(key=lambda s: s[1])  # shortest path first
            dropped = len(self.viewpoints) - len(self._reach_scored)
            self.viewpoints = [vp for vp, _ in self._reach_scored]
            self.get_logger().info(
                f"{self.zone_id}: {len(self.viewpoints)} reachable viewpoint(s) (dropped {dropped} "
                f"unreachable), ordered by path cost"
            )
        else:
            self.get_logger().warn(
                f"{self.zone_id}: reach-check found none reachable; proceeding with all (planner may be cold)"
            )
        self.vp_i = 0
        self._begin_viewpoint()

    def _begin_viewpoint(self):
        self.nav_done = None
        self.nav_status = None
        self.nav_ticks = 0
        self.nav_retries = 0
        self._nav_gen += 1  # invalidate any in-flight callback from before
        self.nav_settle_ticks = int(self.nav_settle / 0.1)  # settle BEFORE the first goal too
        self.state = "VP_NAV"

    def _st_VP_NAV(self):
        if self.nav_settle_ticks > 0:  # cooldown so we don't fire a goal at a
            self.nav_settle_ticks -= 1
            self.stop()
            return  #   controller_server still recovering (anti-cascade)
        vx, vy = self.viewpoints[self.vp_i]
        if self.nav_done is None:
            if not self.nav.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("waiting for Nav2 ...", throttle_duration_sec=5.0)
                return
            yaw = math.atan2(
                self.zone_center[1] - vy, self.zone_center[0] - vx
            )  # face zone centre
            g = NavigateToPose.Goal()
            # stamp 0 = "use latest available TF": rtabmap's map->odom correction lags sim-now by up to a
            # processing period, so a now()-stamped goal needs a future map->odom and the controller fails
            # to transform it ("Unable to transform goal pose into costmap frame"). Latest-TF is correct here.
            g.pose.header.frame_id = "map"
            g.pose.header.stamp = rclpy.time.Time().to_msg()
            g.pose.pose.position.x = float(vx)
            g.pose.pose.position.y = float(vy)
            g.pose.pose.orientation.z = math.sin(yaw / 2)
            g.pose.pose.orientation.w = math.cos(yaw / 2)
            self.get_logger().info(
                f"viewpoint {self.vp_i + 1}/{len(self.viewpoints)}: NAV -> ({vx:.1f},{vy:.1f})"
            )
            self.nav_done = False
            gen = self._nav_gen
            self.nav.send_goal_async(g).add_done_callback(lambda fut: self._nav_acc(fut, gen))
            return
        self.nav_ticks += 1
        if self.nav_done == "reject":
            self._retry_or_skip("nav rejected")
            return
        if self.nav_done is True:
            if self.nav_status != 4:  # 4 = SUCCEEDED
                p, _ = self.pose()
                far = p is None or np.linalg.norm(p - np.array([vx, vy])) > 0.8
                if far:
                    self._retry_or_skip(f"nav not SUCCEEDED ({self.nav_status})")
                    return
            self._begin_spin()
            return
        if self.nav_ticks * 0.1 > self.nav_timeout:
            self._retry_or_skip("nav timeout")

    def _retry_or_skip(self, reason):
        """A transient Nav2 abort (status 6 from a goal-ack timeout / momentary controller stall) should NOT
        permanently skip a reachable viewpoint. Cancel any still-active goal, then retry up to max_nav_retries
        after a settle (which also breaks the rapid-fire goal CASCADE that was instant-aborting later
        viewpoints); only skip once retries are exhausted."""
        if self._nav_gh is not None:  # stop a still-running goal before re-sending
            try:
                self._nav_gh.cancel_goal_async()
            except Exception:
                pass
            self._nav_gh = None
        self.stop()
        self._nav_gen += 1  # any callback from the abandoned goal is now stale
        if self.nav_retries < self.max_nav_retries:
            self.nav_retries += 1
            self.get_logger().warn(
                f"viewpoint {self.vp_i + 1}: {reason}; RETRY "
                f"{self.nav_retries}/{self.max_nav_retries} after {self.nav_settle:.1f}s"
            )
            self.nav_done = None
            self.nav_status = None
            self.nav_ticks = 0
            self.nav_settle_ticks = int(self.nav_settle / 0.1)
        else:
            self.get_logger().warn(
                f"viewpoint {self.vp_i + 1}: {reason}; SKIP (retries exhausted)"
            )
            self._advance()

    def _nav_acc(self, fut, gen):
        if gen != self._nav_gen:  # superseded by a retry/new viewpoint; ignore
            return
        h = fut.result()
        if not h.accepted:
            self.nav_done = "reject"
            return
        self._nav_gh = h  # keep the handle so a retry can cancel it

        def _res(f):
            if gen != self._nav_gen:  # stale result from a cancelled/superseded goal
                return
            try:
                self.nav_status = f.result().status
            except Exception:
                self.nav_status = None
            self._nav_gh = None  # goal terminated; nothing to cancel
            self.nav_done = True

        h.get_result_async().add_done_callback(_res)

    def _begin_spin(self):
        p, y = self.pose()
        self.stop()
        self._capture = False  # stays off until the settle window passes
        self.spin_yaw0 = y
        self.spin_last = y
        self.spin_accum = 0.0
        self.spin_ticks = 0
        with self._det_lock:
            self._proc_mark = self._infer_count
        self.get_logger().info(f"viewpoint {self.vp_i + 1}: SPIN 360deg")
        self.state = "VP_SPIN"

    def _st_VP_SPIN(self):
        p, y = self.pose()
        self.spin_ticks += 1
        max_ticks = (2 * math.pi / max(self.spin_speed, 0.05)) / 0.1 * 2.2 + 40
        if y is None:
            self.stop()  # don't coast on the last spin Twist during a TF dropout
            if self.spin_ticks > 15:
                self.get_logger().warn(f"viewpoint {self.vp_i + 1}: lost TF mid-spin; end")
                self._advance()
            return
        if self.spin_last is not None:
            self.spin_accum += abs(ang_diff(y, self.spin_last))
        self.spin_last = y
        if not self._capture and self.spin_ticks * 0.1 >= self.spin_settle:
            self._capture = True  # settle window passed -> start inferring spin frames
        self._process_inference()  # consume detections while turning
        if self.spin_accum >= 2 * math.pi + self.spin_overlap or self.spin_ticks > max_ticks:
            self.stop()
            done = "full turn" if self.spin_accum >= 2 * math.pi else "watchdog"
            self.get_logger().info(
                f"viewpoint {self.vp_i + 1}: spin {done} ({math.degrees(self.spin_accum):.0f}deg); "
                f"{len(self.objects)} unique objects so far"
            )
            self._advance()
            return
        self.drive(0.0, 0.0, self.spin_speed)

    def _advance(self):
        self._capture = False  # stop inferring the moment a spin ends
        self.vp_i += 1
        if self.vp_i < len(self.viewpoints):
            self._begin_viewpoint()
        else:
            self.stop()
            if self.read_approach:
                self._begin_read_approach()  # detect-then-approach: close read pose per gauge (ADR-017)
            else:
                self._finish()

    # ---------- detect-then-approach: a close, resolution-driven reading pose per gauge (ADR-017) ----------
    def _begin_read_approach(self):
        """The survey/spin DETECTED + 3D-localized gauges at range; now drive to a CLOSE, fronto-parallel,
        REACHABLE standoff in front of each one (distance set by a pixel budget, direction by the wall
        normal) and grab a high-res crop. This is what makes READING (vs detecting) work in big rooms —
        the same navigate-close principle Spot/ANYmal use, computed automatically from the 3D positions."""
        from go2_inspection import inspect_planner as ip

        gauges = [
            o for o in self.objects
            if detect_utils.is_gauge(o.get("class", "")) and o.get("localized") and o.get("world")
        ]
        targets = []
        if gauges and self.K is not None and self._occ_gray is not None:
            # reachability mask = obstacles dilated by the robot footprint clearance (NOT the 1.2 m
            # phantom-rejection radius, which would reject every near-wall reading pose)
            nf = (self._occ_gray < 250).astype(np.uint8)
            rc = max(1, int(self.read_clearance / max(self._map_res, 1e-6)))
            reach = cv2.dilate(nf, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * rc + 1, 2 * rc + 1)))
            is_free = ip.make_is_free(reach, self._map_res, self._map_ox, self._map_oy)
            for o in gauges:
                w = o["world"]
                # ADAPTIVE standoff: distance from the MEASURED gauge size (bbox+depth, _apparent_size),
                # falling back to the nominal read_asset_size only if the measurement was unavailable.
                asset_size = o.get("est_size_m") or self.read_asset_size
                d = ip.standoff_distance(
                    self.K[0], asset_size, self.read_target_px, self.read_dmin, self.read_dmax
                )
                normal = ip.wall_normal(
                    self._occ_gray, (w[0], w[1]), self._map_res, self._map_ox, self._map_oy
                )
                poses = ip.plan_reading_poses(
                    (w[0], w[1]), normal, d, is_free,
                    arc_deg=self.read_arc_deg, max_poses=max(1, self.read_max_attempts),
                )
                if poses:
                    targets.append({"obj": o, "poses": poses, "d": d, "attempt": 0, "best": None})
                    self.get_logger().info(
                        f"read-approach: gauge @ ({w[0]:.1f},{w[1]:.1f}) measured size ~{asset_size:.2f} m "
                        f"-> standoff {d:.2f} m ({len(poses)} candidate view(s))"
                    )
                else:
                    self.get_logger().warn(
                        f"read-approach: no reachable pose for gauge @ ({w[0]:.1f},{w[1]:.1f}); skip"
                    )
            self.get_logger().info(
                f"read-approach: {len(targets)}/{len(gauges)} gauge(s) reachable"
            )
        elif gauges:
            self.get_logger().warn("read-approach: no camera K / occupancy map; cannot plan; skip")
        self._read_targets = targets
        self._read_i = 0
        if not targets:
            self._finish()
            return
        self._begin_read_nav()

    def _begin_read_nav(self):
        self.nav_done = None
        self.nav_status = None
        self.nav_ticks = 0
        self._nav_gen += 1
        self.nav_settle_ticks = int(self.nav_settle / 0.1)
        self.state = "READ_NAV"

    def _cur_read_pose(self):
        t = self._read_targets[self._read_i]
        return t["poses"][min(t["attempt"], len(t["poses"]) - 1)]

    def _st_READ_NAV(self):
        if self.nav_settle_ticks > 0:
            self.nav_settle_ticks -= 1
            self.stop()
            return
        t = self._read_targets[self._read_i]
        x, y, yaw = self._cur_read_pose()
        if self.nav_done is None:
            if not self.nav.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("read-approach: waiting for Nav2 ...", throttle_duration_sec=5.0)
                return
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"
            g.pose.header.stamp = rclpy.time.Time().to_msg()
            g.pose.pose.position.x = float(x)
            g.pose.pose.position.y = float(y)
            g.pose.pose.orientation.z = math.sin(yaw / 2)
            g.pose.pose.orientation.w = math.cos(yaw / 2)
            self.get_logger().info(
                f"read-approach {self._read_i + 1}/{len(self._read_targets)} "
                f"(view {t['attempt'] + 1}/{len(t['poses'])}): NAV -> ({x:.1f},{y:.1f})"
            )
            self.nav_done = False
            gen = self._nav_gen
            self.nav.send_goal_async(g).add_done_callback(lambda fut: self._nav_acc(fut, gen))
            return
        self.nav_ticks += 1
        if self.nav_done == "reject" or (self.nav_done is True and self.nav_status != 4):
            p, _ = self.pose()
            if p is None or np.linalg.norm(p - np.array([x, y])) > 0.8:
                self._read_next_view("nav failed")
                return
            self._begin_read_capture()  # close enough despite a non-SUCCEEDED status
            return
        if self.nav_done is True:
            self._begin_read_capture()
            return
        if self.nav_ticks * 0.1 > self.nav_timeout:
            self._read_next_view("nav timeout")

    def _begin_read_capture(self):
        self.stop()
        self._read_settle = int(max(self.spin_settle, self.read_settle) / 0.1)
        self._read_frames = []
        self.state = "READ_CAPTURE"

    def _st_READ_CAPTURE(self):
        self.stop()  # stationary at the standoff -> no motion blur
        if self._read_settle > 0:
            self._read_settle -= 1
            return
        with self._img_lock:
            img = None if self.img is None else self.img.copy()
        if img is not None:
            self._read_frames.append(img)
        if len(self._read_frames) < self.read_burst and img is not None:
            return
        from go2_inspection import inspect_planner as ip

        t = self._read_targets[self._read_i]
        if self._read_frames:
            best_frame = max(
                self._read_frames, key=lambda im: ip.sharpness(cv2.cvtColor(im, cv2.COLOR_RGB2GRAY))
            )
            sharp = ip.sharpness(cv2.cvtColor(best_frame, cv2.COLOR_RGB2GRAY))
            H, W = best_frame.shape[:2]
            # re-detect on the sharp close frame -> tight crop + how big the gauge is now (read quality)
            bbox, gauge_px = None, 0
            if self.model is not None:
                try:
                    dets = detect_utils.infer(self.model, cv2.cvtColor(best_frame, cv2.COLOR_RGB2BGR))
                    gd = [dd for dd in dets if detect_utils.is_gauge(dd[0])]
                    if gd:
                        bbox = max(gd, key=lambda dd: (dd[2][2] - dd[2][0]) * (dd[2][3] - dd[2][1]))[2]
                        gauge_px = int(bbox[2] - bbox[0])
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f"read-approach re-detect failed: {e}")
            if bbox is None:  # centred fallback (we navigated to face the gauge)
                bbox = [int(W * 0.30), int(H * 0.20), int(W * 0.70), int(H * 0.80)]
            score, ok = ip.read_quality(gauge_px, W, sharp, self.read_target_px)
            if t["best"] is None or score > t["best"]["score"]:  # keep the best view across attempts
                t["best"] = {
                    "score": score, "frame": best_frame, "bbox": bbox, "gauge_px": gauge_px,
                    "sharp": round(sharp, 1), "pose": self._cur_read_pose(),
                }
            self.get_logger().info(
                f"read-approach {self._read_i + 1} view {t['attempt'] + 1}: gauge {gauge_px}px "
                f"sharp {sharp:.0f} -> {'OK' if ok else 'weak'}"
            )
            if ok or t["attempt"] + 1 >= min(len(t["poses"]), self.read_max_attempts):
                self._finalize_read(t)
                self._read_advance()
                return
            self._read_next_view("weak read")
            return
        self._read_advance()

    def _read_next_view(self, reason):
        """Next-best-view: re-approach the SAME gauge from the next candidate angle (the planner's ranked
        list) when a view fails or reads poorly. The best crop across views is always kept, so a retry can
        only improve the result, never worsen it."""
        t = self._read_targets[self._read_i]
        if t["attempt"] + 1 < min(len(t["poses"]), self.read_max_attempts):
            t["attempt"] += 1
            self.get_logger().warn(
                f"read-approach {self._read_i + 1}: {reason}; re-approach (view {t['attempt'] + 1})"
            )
            self._begin_read_nav()
        else:
            if t.get("best"):
                self._finalize_read(t)
            else:
                self.get_logger().warn(f"read-approach {self._read_i + 1}: {reason}; no usable view; skip")
            self._read_advance()

    def _finalize_read(self, t):
        b, o = t.get("best"), t["obj"]
        if not b:
            return
        self._save_read_crop(b["frame"], b["bbox"], o)
        o["read_standoff"] = [round(v, 2) for v in b["pose"]]
        o["read_dist"] = round(t["d"], 2)
        o["read_px"] = b["gauge_px"]
        o["read_sharpness"] = b["sharp"]
        o["read_attempts"] = t["attempt"] + 1
        # CONFIRMATION by re-observation: the robot reached a close, fronto-parallel pose and photographed
        # the spot. If the same model re-detects a gauge there (best gauge_px > 0) the survey hit is
        # CONFIRMED; if every view found nothing, it is REFUTED (a survey false positive). No ground truth.
        o["read_confirmed"] = bool(b["gauge_px"] > 0)
        self.get_logger().info(
            f"read-approach {self._read_i + 1}: BEST {b['gauge_px']}px after {t['attempt'] + 1} view(s) "
            f"-> {o.get('read_crop')}"
        )

    def _read_advance(self):
        self._read_i += 1
        if self._read_i < len(self._read_targets):
            self._begin_read_nav()
        else:
            self.stop()
            self._finish()

    def _save_read_crop(self, img_rgb, bbox, obj):
        H, W = img_rgb.shape[:2]
        x0, y0, x1, y1 = [int(v) for v in bbox]
        pw, ph = int((x1 - x0) * self.capture_pad), int((y1 - y0) * self.capture_pad)
        x0, y0 = max(0, x0 - pw), max(0, y0 - ph)
        x1, y1 = min(W, x1 + pw), min(H, y1 + ph)
        if x1 <= x0 or y1 <= y0:
            return
        crop_bgr = cv2.cvtColor(img_rgb[y0:y1, x0:x1].copy(), cv2.COLOR_RGB2BGR)
        cdir = os.path.join(self.out_dir, self.zone_id, "read_crops")
        os.makedirs(cdir, exist_ok=True)
        fn = f"{obj['id']}.png"
        cv2.imwrite(os.path.join(cdir, fn), crop_bgr)
        obj["read_crop"] = f"read_crops/{fn}"

    # ---------- detection -> map projection -> dedup -> crop ----------
    def _process_inference(self):
        if self.model is None or not self._capture:
            return
        with self._det_lock:
            if self._infer_count <= self._proc_mark:
                return
            self._proc_mark = self._infer_count
            dets = list(self.live_dets)
            img = None if self.live_img is None else self.live_img.copy()
            stamp = self.live_stamp
        if not dets or img is None:
            return
        depth = self.depth
        for name, conf, bbox in dets:
            xyz = self._project(bbox, depth, stamp)
            est_size = self._apparent_size(bbox, depth)  # measured physical width (m) for adaptive standoff
            ok, reason = self._validate_position(name, xyz)
            # record EVERY detection (accepted or rejected, with the reason) so nothing is hidden
            self.detections.append(
                {
                    "class": name,
                    "conf": round(float(conf), 3),
                    "bbox": [int(v) for v in bbox],
                    "world": xyz,
                    "viewpoint": self.vp_i,
                    "accepted": bool(ok),
                    "reject_reason": reason,
                }
            )
            if ok:
                self._accumulate(name, conf, bbox, xyz, img, est_size)

    # ---------- position double-check (zone polygon + occupancy map) ----------
    @staticmethod
    def _is_dynamic(name):
        """Classes that need NOT coincide with a mapped obstacle (they move or weren't present at mapping
        time). 'fire extinguisher'/'fire hydrant' are static safety props despite containing 'fire'."""
        n = name.lower()
        if "extinguisher" in n or "hydrant" in n:
            return False
        tokens = set(n.replace("-", " ").split())  # whole-word match: "trashcan" != "ash"
        return bool(tokens & {"person", "human", "fire", "flame", "fumes", "smoke", "ash"})

    def _in_zone(self, x, y, margin):
        if not self.zone_polygon or len(self.zone_polygon) < 3:
            return True
        poly = np.array(self.zone_polygon, np.float32)
        # signed distance in metres (polygon is in world units); >=0 inside, negative outside
        return cv2.pointPolygonTest(poly, (float(x), float(y)), True) >= -margin

    def _validate_position(self, name, xyz):
        """(ok, reason) for a projected detection. Unlocalized -> accepted (handled weakly by class dedup).
        Localized -> must be inside the zone polygon (+zone_margin); and, unless it is a dynamic class,
        must lie within obstacle_check_radius of a mapped obstacle/unknown cell (else it is floating in
        known-free space == a bad projection / phantom)."""
        if xyz is None:
            return True, ""
        x, y = xyz[0], xyz[1]
        if not self._in_zone(x, y, self.zone_margin):
            return False, "outside_zone"
        if not self.validate_map or self._plausible is None or self._is_dynamic(name):
            return True, ""
        px, py = report_utils.world_to_px(
            x, y, self._map_res, self._map_ox, self._map_oy, self._map_H
        )
        if not (0 <= px < self._map_W and 0 <= py < self._map_H):
            return False, "off_map"
        if self._plausible[py, px] == 0:
            return False, "free_space"
        return True, ""

    def _project(self, bbox, depth, stamp):
        """Depth -> map (x,y,z). Median depth over the inner bbox, deproject via K to camera_link_optical,
        transform to map at the image stamp (fall back to latest)."""
        if self.K is None or depth is None:
            return None
        Hd, Wd = depth.shape[:2]
        x0, y0, x1, y1 = bbox
        dx = int((x1 - x0) * 0.1)
        dy = int((y1 - y0) * 0.1)
        ix0, iy0 = max(0, x0 + dx), max(0, y0 + dy)
        ix1, iy1 = min(Wd, x1 - dx), min(Hd, y1 - dy)
        if ix1 <= ix0 or iy1 <= iy0:
            return None
        patch = depth[iy0:iy1, ix0:ix1]
        valid = patch[np.isfinite(patch) & (patch > 0.2) & (patch < self.max_depth)]
        # enough valid samples for a reliable median: an absolute floor (protects small/distant bboxes)
        # OR a fraction of the patch (guards a large mostly-invalid one) -- whichever is larger.
        need = max(self.min_valid_px, self.min_valid_frac * max(1, patch.size))
        if valid.size == 0 or valid.size < need:
            return None
        Z = float(np.median(valid))
        fx, fy, cx, cy = self.K
        u = 0.5 * (x0 + x1)
        v = 0.5 * (y0 + y1)
        Xc = (u - cx) * Z / fx
        Yc = (v - cy) * Z / fy
        Pc = np.array([Xc, Yc, Z])
        try:
            t = self.tf.lookup_transform(
                "map",
                self.optical_frame,
                stamp if stamp is not None else rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            ).transform
        except Exception:
            try:
                t = self.tf.lookup_transform(
                    "map", self.optical_frame, rclpy.time.Time()
                ).transform
            except Exception:
                return None
        R = quat_to_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        Pm = R @ Pc + np.array([t.translation.x, t.translation.y, t.translation.z])
        return [float(Pm[0]), float(Pm[1]), float(Pm[2])]

    def _apparent_size(self, bbox, depth):
        """Estimate a detection's physical width (m) from its bbox pixel width and median bbox depth via the
        pinhole model: size = px_width * Z / fx. Distance-invariant, so it gives the gauge's REAL size for an
        adaptive read standoff (vs the nominal read_asset_size). Returns None if depth/K unavailable."""
        if self.K is None or depth is None:
            return None
        Hd, Wd = depth.shape[:2]
        x0, y0, x1, y1 = bbox
        dx, dy = int((x1 - x0) * 0.1), int((y1 - y0) * 0.1)
        ix0, iy0 = max(0, x0 + dx), max(0, y0 + dy)
        ix1, iy1 = min(Wd, x1 - dx), min(Hd, y1 - dy)
        if ix1 <= ix0 or iy1 <= iy0:
            return None
        patch = depth[iy0:iy1, ix0:ix1]
        valid = patch[np.isfinite(patch) & (patch > 0.2) & (patch < self.max_depth)]
        if valid.size == 0:
            return None
        Z = float(np.median(valid))
        size = float((x1 - x0) * Z / self.K[0])
        return size if 0.05 <= size <= 1.0 else None  # clamp to a plausible gauge size, else fall back

    def _accumulate(self, name, conf, bbox, xyz, img_rgb, est_size=None):
        """Merge into an existing object if this detection is at the SAME PLACE (within dedup_radius of a
        localized object, REGARDLESS of class -- repeated scans of one spot are the same physical object),
        keeping the HIGHEST-confidence detection's class, position and crop. Unlocalized detections fall
        back to one-entry-per-class. n_observations counts how many frames saw the object (a persistence
        signal used to drop 1-frame phantoms at finish)."""
        localized = xyz is not None
        if localized:
            best, bestd = None, self.dedup_radius
            for o in self.objects:
                w = o.get("world")
                if not (o.get("localized") and w and w[0] is not None):
                    continue
                d = math.dist(xyz[:2], w[:2])
                if d < bestd:
                    bestd, best = d, o
            if best is not None:
                best["n_observations"] += 1
                if conf > best["confidence"]:  # higher-confidence detection wins the identity
                    best["confidence"] = float(conf)
                    best["class"] = name
                    best["world"] = xyz
                    if est_size is not None:
                        best["est_size_m"] = round(est_size, 3)
                    self._save_crop(img_rgb, bbox, best)
                return
        else:
            for o in self.objects:
                if (not o.get("localized")) and o["class"] == name:
                    o["n_observations"] += 1
                    if conf > o["confidence"]:
                        o["confidence"] = float(conf)
                        self._save_crop(img_rgb, bbox, o)
                    return
        obj = {
            "id": f"{self.zone_id}_{name.replace(' ', '_')}_{len(self.objects)}",
            "class": name,
            "confidence": float(conf),
            "world": xyz,
            "localized": localized,
            "viewpoint": self.vp_i,
            "n_observations": 1,
            "crop": None,
            "est_size_m": round(est_size, 3) if est_size is not None else None,
        }
        self._save_crop(img_rgb, bbox, obj)
        self.objects.append(obj)
        self.get_logger().info(
            f"  + {name} ({conf:.2f}) "
            f"{'@ (%.1f,%.1f)' % (xyz[0], xyz[1]) if localized else '(unlocalized)'}"
        )

    def _save_crop(self, img_rgb, bbox, obj):
        H, W = img_rgb.shape[:2]
        x0, y0, x1, y1 = bbox
        pw = int((x1 - x0) * self.capture_pad)
        ph = int((y1 - y0) * self.capture_pad)
        x0 = max(0, x0 - pw)
        y0 = max(0, y0 - ph)
        x1 = min(W, x1 + pw)
        y1 = min(H, y1 + ph)
        if x1 <= x0 or y1 <= y0:
            return
        crop_bgr = cv2.cvtColor(img_rgb[y0:y1, x0:x1].copy(), cv2.COLOR_RGB2BGR)
        cdir = os.path.join(self.out_dir, self.zone_id, "crops")
        os.makedirs(cdir, exist_ok=True)
        fn = f"{obj['id']}.png"
        cv2.imwrite(os.path.join(cdir, fn), crop_bgr)
        obj["crop"] = f"crops/{fn}"

    def _consolidate_duplicates(self):
        """Merge weak localization-noise duplicates into their strong same-class parent. A detection seen
        in only a few frames can have a position ~1 m off (noisy median depth), creating a second marker
        for one physical object that escapes dedup_radius. Absorb the weak one into a much stronger
        same-class one within consolidate_radius, but ONLY when the weak has <= consolidate_obs_frac of the
        strong's observations -- so two comparably-well-seen DISTINCT objects are never merged (recall
        preserved). Observation counts + geometry only; no ground truth."""
        from go2_inspection import inspect_planner as ip

        amap = ip.weak_duplicate_map(
            self.objects, self.consolidate_radius, self.consolidate_obs_frac
        )
        if not amap:
            return
        d = os.path.join(self.out_dir, self.zone_id)
        for wi, si in amap.items():  # fold the weak ghost's frames into its strong parent, drop its crop
            self.objects[si]["n_observations"] += self.objects[wi]["n_observations"]
            if self.objects[wi].get("crop"):
                try:
                    os.remove(os.path.join(d, self.objects[wi]["crop"]))
                except OSError:
                    pass
        self.objects = [o for i, o in enumerate(self.objects) if i not in amap]
        self.get_logger().info(
            f"{self.zone_id}: consolidated {len(amap)} weak duplicate(s) into a stronger "
            f"same-class detection (localization-noise artefact)"
        )

    def _finish(self):
        d = os.path.join(self.out_dir, self.zone_id)
        os.makedirs(d, exist_ok=True)
        # consolidate weak localization-noise duplicates BEFORE the persistence filter, so an absorbed
        # weak detection's frames count toward its strong parent's persistence.
        self._consolidate_duplicates()
        # persistence filter: a real object is seen across many spin frames; drop those seen in fewer than
        # min_observations frames (one-shot misclassifications). Remove their now-orphan crops too.
        if self.min_observations > 1 and self.objects:
            kept, dropped = [], []
            for o in self.objects:
                (kept if o["n_observations"] >= self.min_observations else dropped).append(o)
            for o in dropped:
                if o.get("crop"):
                    try:
                        os.remove(os.path.join(d, o["crop"]))
                    except OSError:
                        pass
            if dropped:
                self.get_logger().info(
                    f"{self.zone_id}: dropped {len(dropped)} object(s) seen "
                    f"< {self.min_observations}x (likely phantoms)"
                )
            self.objects = kept
        # close-approach false-positive rejection (opt-in): drop a gauge the read-approach REACHED +
        # photographed but could not re-detect up close (read_confirmed explicitly False). Independent
        # corroboration by a higher-quality observation -- no ground truth. Objects never approached, or
        # where the approach couldn't reach (read_confirmed absent/None), are left untouched (inconclusive).
        if self.read_drop_unconfirmed:
            unconf = [o for o in self.objects if o.get("read_confirmed") is False]
            for o in unconf:
                for key in ("crop", "read_crop"):
                    if o.get(key):
                        try:
                            os.remove(os.path.join(d, o[key]))
                        except OSError:
                            pass
            if unconf:
                self.get_logger().info(
                    f"{self.zone_id}: dropped {len(unconf)} unconfirmed gauge(s) -- the close approach "
                    f"re-detected nothing there (survey false positive)"
                )
            self.objects = [o for o in self.objects if o.get("read_confirmed") is not False]
        avail = self.model is not None
        json.dump(
            {
                "zone": self.zone_id,
                "label": self.zone_label,
                "available": avail,
                "reason": "" if avail else self.model_reason,
                "failed": self.failed,
                "n_detections": len(self.detections),
                "detections": self.detections,
            },
            open(os.path.join(d, "detections.json"), "w"),
            indent=2,
        )
        json.dump(
            {
                "zone": self.zone_id,
                "label": self.zone_label,
                "available": avail,
                "reason": "" if avail else self.model_reason,
                "failed": self.failed,
                "n_objects": len(self.objects),
                "objects": self.objects,
            },
            open(os.path.join(d, "objects.json"), "w"),
            indent=2,
        )
        try:
            detect_utils.contact_sheet(d, self.objects)
            report_utils.plot_zone_map(
                self.zone_id,
                self.zone_polygon,
                self.viewpoints,
                self.objects,
                os.path.join(d, "zone_map.png"),
                self.map_yaml,
            )
            report_utils.write_zone_report(self.zone_id, self.zone_label, self.objects, d)
        except Exception as e:
            self.get_logger().warn(f"report/plot failed: {e}")
        self._stop_infer = True
        tag = "ABORTED" if self.failed else "DONE"
        self.get_logger().info(
            f"{self.zone_id}: inspection {tag} -- {len(self.objects)} unique objects "
            f"({len(self.detections)} observations) -> {d}"
        )
        self.state = "DONE"


def main(args=None):
    import sys

    rclpy.init(args=args)
    n = ZoneInspector()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    try:
        while rclpy.ok() and n.state != "DONE":
            ex.spin_once(timeout_sec=0.1)
        n.get_logger().info("inspection complete; shutting down")
    except KeyboardInterrupt:
        pass
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
