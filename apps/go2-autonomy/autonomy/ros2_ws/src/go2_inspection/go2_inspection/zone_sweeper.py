#!/usr/bin/env python3
"""zone_sweeper -- Phase 2 of the autonomous gauge inspection.

Given a target ZONE (from go2_zones' zones.yaml), the robot:
  1. NAV_TO_CENTER : Nav2 NavigateToPose to the zone centre, arriving FACING the gauge wall.
                     (skip with -p skip_nav:=true when the robot is already in front of the wall.)
  2. APPROACH      : drive straight at the wall (direct /cmd_vel) until ~approach_dist away (LiDAR front).
                     Records the approach pose; the robot's heading there defines the LATERAL axis.
  3. STRAFE_START  : strafe to one end of the wall (-half_width along the lateral axis).
  4. SWEEP         : strafe to the other end (+half_width), wall-following (lateral vy + a small vx
                     correction holding the distance), grabbing a frame every frame_interval of lateral
                     travel. Lateral position = displacement projected onto the lateral axis -> FRAME-
                     INDEPENDENT (works in SLAM or localization mode, any map orientation).
  5. STITCH        : place each frame's central strip at its lateral position (odometry stitch, no feature
                     matching -> robust on plain walls) -> save zone_<id>/panorama.png. Phase 3 (FastSAM)
                     segments the gauges out of the panorama.

Nav2 owns macro-nav; the approach/strafe publish DIRECT /cmd_vel (the Go2 gait accepts lateral vy). This is
the GLOBAL /cmd_vel that CHAMP consumes (the same topic Nav2's controller publishes) -- safe only because
Nav2's controller is IDLE during the sweep (no NavigateToPose active). The open-loop strafe has NO Nav2
collision check, so SWEEP/STRAFE_START GATE on a /scan lateral-clearance (side_clear) to halt before a
sideways wall collision. Camera frame + odometry only -> no camera extrinsic/TF needed.
"""
import os, json, math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from nav2_msgs.action import NavigateToPose
import tf2_ros


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    """Smallest signed angle a-b in (-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ZoneSweeper(Node):
    def __init__(self):
        super().__init__("zone_sweeper")
        self.zone_id = self.declare_parameter("zone_id", "zone_1").value
        self.zones_file = self.declare_parameter("zones_file", "").value
        self.wall_heading = float(self.declare_parameter("wall_heading", -1.5708).value)
        self.skip_nav = bool(self.declare_parameter("skip_nav", False).value)
        self.half_width = float(self.declare_parameter("half_width", 0.0).value)   # 0 => derive from zone
        self.find_wall = bool(self.declare_parameter("find_wall", True).value)     # rotate+detect gauge wall
        self.approach_dist = float(self.declare_parameter("approach_dist", 1.0).value)
        self.backoff_dist = float(self.declare_parameter("backoff_dist", 1.8).value)   # nav-safe end pose
        self.strafe_speed = float(self.declare_parameter("strafe_speed", 0.15).value)
        self.fwd_speed = float(self.declare_parameter("fwd_speed", 0.15).value)
        self.frame_interval = float(self.declare_parameter("frame_interval", 0.06).value)
        self.margin = float(self.declare_parameter("margin", 0.6).value)
        self.side_clear = float(self.declare_parameter("side_clear", 0.45).value)   # min /scan lateral clearance
        # (m) before the open-loop strafe HALTS -- the only guard against a sideways wall collision when the
        # derived half_width / square-up is wrong. > footprint half-width(0.155)+pad; tune per facility.
        self.out_dir = os.path.expanduser(self.declare_parameter("out_dir", "~/gauges").value)
        # use_sim_time is auto-declared in Jazzy; pass it via -p use_sim_time:=true, don't re-declare.

        self.zone_center = [0.0, 0.0]; self.zone_polygon = []
        if self.zones_file and os.path.exists(self.zones_file):
            self._load_zone()
        elif self.half_width <= 0:
            self.half_width = 4.0
        self.get_logger().info(f"zone={self.zone_id} half_width={self.half_width:.2f}m skip_nav={self.skip_nav}")

        self.K = self.img = self.front = self.scan = None
        self.tf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._ci, 10)
        self.create_subscription(Image, "/camera/image_raw", self._im, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.state = ("FIND_WALL" if self.find_wall else "APPROACH") if self.skip_nav else "NAV"
        self.nav_done = None
        self.approach_pos = self.lat_axis = None; self.sq_ticks = 0; self.bo_ticks = 0
        self.fw_phase = "SCAN"; self.fw_rot = 0.0; self.fw_prev_yaw = None
        self.fw_best_score = -1.0; self.fw_best_yaw = 0.0
        self.frames = []; self.last_cap_lat = None; self.wall_dists = []
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"zone_sweeper ready (start state {self.state})")

    def _load_zone(self):
        z = next((z for z in json.load(open(self.zones_file))["zones"] if z["id"] == self.zone_id), None)
        if z is None:
            self.get_logger().error(f"{self.zone_id} not in {self.zones_file}"); raise SystemExit(1)
        self.zone_center = z["center"]
        self.zone_polygon = z.get("polygon", [])
        # FIND_WALL re-derives half_width along the discovered wall; only fall back to the X-extent
        # heuristic when not finding the wall (and no explicit half_width was given).
        if self.half_width <= 0 and not self.find_wall:
            xs = [p[0] for p in z["polygon"]]
            self.half_width = max(1.0, (max(xs) - min(xs)) / 2 - self.margin)

    def _ci(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5]); self.W, self.H = m.width, m.height
    def _im(self, m):
        self.img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
    def _scan(self, m):
        self.scan = m
        i0 = int(round((0.0 - m.angle_min) / m.angle_increment))
        win = [m.ranges[j] for j in range(i0 - 4, i0 + 5)
               if 0 <= j < len(m.ranges) and m.range_min < m.ranges[j] < m.range_max and math.isfinite(m.ranges[j])]
        self.front = float(np.median(win)) if win else None

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

    def _st_NAV(self):
        if self.nav_done is None:
            if not self.nav.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("waiting for Nav2 ...", throttle_duration_sec=5.0); return
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"; g.pose.header.stamp = self.get_clock().now().to_msg()
            g.pose.pose.position.x, g.pose.pose.position.y = float(self.zone_center[0]), float(self.zone_center[1])
            g.pose.pose.orientation.z = math.sin(self.wall_heading / 2); g.pose.pose.orientation.w = math.cos(self.wall_heading / 2)
            self.get_logger().info(f"NAV -> zone centre {self.zone_center}")
            self.nav_done = False
            self.nav.send_goal_async(g).add_done_callback(self._nav_acc)
        elif self.nav_done is True:
            nxt = "FIND_WALL" if self.find_wall else "APPROACH"
            self.get_logger().info(f"at zone centre; {nxt} ->"); self.state = nxt

    def _nav_acc(self, fut):
        h = fut.result()
        if not h.accepted:
            self.get_logger().error("nav rejected"); self.nav_done = None; return
        h.get_result_async().add_done_callback(lambda f: setattr(self, "nav_done", True))

    def _gauge_score(self):
        """Total area of bright ~circular blobs in the current camera frame (gauge faces are bright white
        discs) + their mean image column -> how strongly the robot is looking at gauges, and where. Fast
        (no NN; FastSAM does the precise segmentation later)."""
        if self.img is None:
            return 0.0, None
        gray = cv2.cvtColor(self.img, cv2.COLOR_RGB2GRAY)
        _, th = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        score, cols = 0.0, []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < 150:
                continue
            per = cv2.arcLength(c, True)
            if per > 0 and 4 * math.pi * a / (per * per) > 0.6:    # circularity ~ a round gauge face
                score += a
                M = cv2.moments(c)
                if M["m00"]:
                    cols.append(M["m10"] / M["m00"])
        return score, (float(np.mean(cols)) if cols else None)

    def _st_FIND_WALL(self):
        """PERCEPTION-DRIVEN wall discovery (no hard-coded direction): rotate 360deg, score gauge-blobs in
        every frame, then turn to face the bearing with the most gauges. Then derive the strafe half-width
        from the zone polygon along that wall. -> works in any world / on the real robot."""
        p, y = self.pose()
        if y is None:
            return
        if self.fw_phase == "SCAN":
            if self.fw_prev_yaw is not None:
                self.fw_rot += abs(ang_diff(y, self.fw_prev_yaw))
            self.fw_prev_yaw = y
            sc, col = self._gauge_score()
            if sc > self.fw_best_score:
                off = 0.0
                if col is not None and self.K is not None:
                    off = math.atan2((self.W / 2 - col), self.K[0])   # gauge bearing offset from front
                self.fw_best_score, self.fw_best_yaw = sc, y + off
            if self.fw_rot < 2 * math.pi + 0.3:
                self.drive(0.0, 0.0, 0.5)                              # rotate in place, scanning
            else:
                self.stop()
                if self.fw_best_score <= 0:
                    self.get_logger().warn("FIND_WALL: no gauges seen in this zone; DONE")
                    self.state = "DONE"; return
                self.get_logger().info(f"FIND_WALL: gauge wall @ {math.degrees(self.fw_best_yaw):.0f}deg "
                                       f"(score {self.fw_best_score:.0f}); FACE ->")
                self.fw_phase = "FACE"
        else:  # FACE the wall
            dy = ang_diff(self.fw_best_yaw, y)
            if abs(dy) > 0.05:
                self.drive(0.0, 0.0, max(-0.5, min(0.5, 1.5 * dy)))
            else:
                self.stop()
                self._derive_half_width(y)
                self.get_logger().info(f"facing wall; half_width={self.half_width:.2f}m; APPROACH ->")
                self.state = "APPROACH"

    def _derive_half_width(self, yaw):
        """Strafe half-width = half the zone's extent ALONG the wall (perpendicular to the approach yaw),
        from the segmented zone polygon -> general for any wall orientation. Falls back to a default."""
        if self.half_width > 0:                                        # explicit override given
            return
        lat = np.array([math.cos(yaw + math.pi / 2), math.sin(yaw + math.pi / 2)])
        c = np.array(self.zone_center)
        if len(self.zone_polygon) >= 3:
            proj = [float(np.dot(np.array(v) - c, lat)) for v in self.zone_polygon]
            self.half_width = max(1.0, (max(proj) - min(proj)) / 2 - self.margin)
        else:
            self.half_width = 3.0

    def _st_APPROACH(self):
        if self.front is None:
            return
        if self.front > self.approach_dist:
            self.drive(self.fwd_speed, 0.0)
        else:
            self.stop()
            self.get_logger().info(f"APPROACH done (wall {self.front:.2f}m); SQUARE_UP ->")
            self.sq_ticks = 0
            self.state = "SQUARE_UP"

    def _wall_offset(self):
        """Bearing (rad) from the robot's front to the wall NORMAL = the min-range beam in the front arc
        (a flat wall is closest along its perpendicular). Used to square up regardless of arrival yaw."""
        m = self.scan
        if m is None:
            return None
        best_a, best_r, arc = None, float("inf"), math.radians(50)
        for j, r in enumerate(m.ranges):
            a = m.angle_min + j * m.angle_increment
            if abs(a) <= arc and m.range_min < r < m.range_max and math.isfinite(r) and r < best_r:
                best_r, best_a = r, a
        return best_a

    def _st_SQUARE_UP(self):
        """Rotate in place until the front faces the wall normal, so the strafe runs PARALLEL to the
        wall (fixes the panorama shear when Nav2 arrives at a slightly off yaw). Then record the axis."""
        off = self._wall_offset()
        self.sq_ticks += 1
        if off is not None and abs(off) > 0.04 and self.sq_ticks < 200:   # 2.3deg deadband, ~20s cap
            self.drive(0.0, 0.0, max(-0.4, min(0.4, 1.5 * off)))          # rotate toward the wall normal
            return
        self.stop()
        p, y = self.pose()
        if p is None:
            return
        self.approach_pos = p
        self.lat_axis = np.array([math.cos(y + math.pi / 2), math.sin(y + math.pi / 2)])  # robot's left
        deg = 0.0 if off is None else math.degrees(off)
        self.get_logger().info(f"SQUARE_UP done (off={deg:.1f}deg); STRAFE_START ->")
        self.state = "STRAFE_START"

    def _lateral(self):
        p, _ = self.pose()
        return None if p is None else float(np.dot(p - self.approach_pos, self.lat_axis))

    def _vx_hold(self):
        return max(-0.1, min(0.1, 0.6 * (self.front - self.approach_dist))) if self.front else 0.0

    def _lateral_clear(self, sign):
        """Min /scan range in a +-20deg arc centred on the CURRENT strafe direction (sign>0 -> robot's
        LEFT=+90deg, SWEEP's +vy; sign<0 -> RIGHT=-90deg, STRAFE_START's -vy). +inf if no scan yet. The
        strafe is open-loop (no Nav2 collision check) -> this is the ONLY thing stopping a sideways wall
        collision when half_width/square-up is wrong. Reuses the already-cached self.scan."""
        m = self.scan
        if m is None:
            return float("inf")
        centre = math.copysign(math.pi / 2, sign)
        arc = math.radians(20)
        best = float("inf")
        for j, r in enumerate(m.ranges):
            a = m.angle_min + j * m.angle_increment
            if abs(ang_diff(a, centre)) <= arc and m.range_min < r < m.range_max and math.isfinite(r):
                best = min(best, r)
        return best

    def _st_STRAFE_START(self):
        lat = self._lateral()
        if lat is None:
            return
        if lat > -self.half_width + 0.05:
            clr = self._lateral_clear(-1.0)
            if clr < self.side_clear:                       # wall to the right -> can't reach the start end
                self.stop()
                self.get_logger().warn(f"STRAFE_START: right obstacle {clr:.2f}m < {self.side_clear:.2f}m; "
                                       f"SWEEP from here")
                self.frames = []; self.last_cap_lat = None; self.state = "SWEEP"; return
            self.drive(self._vx_hold(), -self.strafe_speed)
        else:
            self.stop(); self.frames = []; self.last_cap_lat = None
            self.get_logger().info(f"at start (lat {lat:.2f}); SWEEP ->"); self.state = "SWEEP"

    def _st_SWEEP(self):
        lat = self._lateral()
        if lat is None:
            return
        if lat < self.half_width:
            clr = self._lateral_clear(+1.0)
            if clr < self.side_clear:                       # wall to the left -> halt before collision
                self.stop()
                self.get_logger().warn(f"SWEEP: left obstacle {clr:.2f}m < {self.side_clear:.2f}m; "
                                       f"HALT -> STITCH ({len(self.frames)} frames)")
                self.state = "STITCH"; return
            self.drive(self._vx_hold(), self.strafe_speed)
            if self.img is not None and (self.last_cap_lat is None or abs(lat - self.last_cap_lat) >= self.frame_interval):
                self.frames.append((lat, self.img.copy())); self.last_cap_lat = lat
                if self.front:
                    self.wall_dists.append(self.front)
        else:
            self.stop()
            self.get_logger().info(f"SWEEP done (lat {lat:.2f}): {len(self.frames)} frames; STITCH ->")
            self.state = "STITCH"

    def _st_STITCH(self):
        self.stop()
        if len(self.frames) < 2 or self.K is None:
            self.get_logger().error("not enough frames"); self.state = "BACKOFF"; return
        fx, _, cx, _ = self.K
        wall = float(np.median(self.wall_dists)) if self.wall_dists else self.approach_dist
        ppm = fx / wall
        lats = [f[0] for f in self.frames]; lmin = min(lats)
        strip = max(8, int(self.frame_interval * ppm) + 2)
        pano_w = int((max(lats) - lmin) * ppm) + strip + 4
        pano = np.zeros((self.H, pano_w, 3), np.uint8)
        for lat, im in sorted(self.frames):
            col = int((lat - lmin) * ppm)
            s0 = int(cx - strip / 2); xo = col - strip // 2
            if 0 <= xo and xo + strip <= pano_w and 0 <= s0 and s0 + strip <= self.W:
                pano[:, xo:xo + strip] = im[:, s0:s0 + strip]
        d = os.path.join(self.out_dir, self.zone_id); os.makedirs(d, exist_ok=True)
        path = os.path.join(d, "panorama.png")
        cv2.imwrite(path, cv2.cvtColor(pano, cv2.COLOR_RGB2BGR))
        json.dump({"zone_id": self.zone_id, "wall_dist": wall, "ppm": ppm,
                   "lat_range": [lmin, max(lats)], "n_frames": len(self.frames)},
                  open(os.path.join(d, "panorama_meta.json"), "w"), indent=2)
        # Save the individual source frames + an index. Phase 3 detects each gauge in the (strip-stitched)
        # panorama for dedup + lateral position, then crops it from its SINGLE best-centred frame here --
        # un-sheared, full-resolution -- which is what the model actually reads. ppm/lmin/cx map a panorama
        # column -> a lateral position -> the nearest frame, where the gauge sits near cx.
        fd = os.path.join(d, "frames"); os.makedirs(fd, exist_ok=True)
        index = []
        for k, (lat, im) in enumerate(sorted(self.frames)):
            cv2.imwrite(os.path.join(fd, f"{k:05d}.png"), cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
            index.append({"k": k, "lat": lat, "file": f"frames/{k:05d}.png"})
        json.dump({"ppm": ppm, "lmin": lmin, "cx": cx, "fx": fx, "W": self.W, "H": self.H,
                   "wall_dist": wall, "frames": index},
                  open(os.path.join(d, "frames.json"), "w"), indent=2)
        self.get_logger().info(f"STITCH done: {pano_w}x{self.H} -> {path} (+{len(index)} frames)")
        self.state = "BACKOFF"

    def _st_BACKOFF(self):
        """Back away from the wall to a nav-safe distance before finishing. Without this the robot ends
        ~0.3 m off the wall (inside the costmap inflation), and the orchestrator's NEXT room nav aborts
        ('planner failed to plan from <near-wall>') -- which cascaded SW+NW failures in the first run."""
        self.bo_ticks += 1
        if self.front is not None and self.front < self.backoff_dist and self.bo_ticks < 120:
            self.drive(-self.fwd_speed, 0.0)
        else:
            self.stop()
            self.get_logger().info(f"BACKOFF done (wall {self.front}); DONE")
            self.state = "DONE"


def main(args=None):
    rclpy.init(args=args)
    n = ZoneSweeper()
    try:
        # run-once tool: spin until the sweep reaches DONE, then exit (so an orchestrator can
        # sequence it as a subprocess). Standalone use is unaffected (it just exits when finished).
        while rclpy.ok() and n.state != "DONE":
            rclpy.spin_once(n, timeout_sec=0.1)
        n.get_logger().info("sweep complete; shutting down")
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
