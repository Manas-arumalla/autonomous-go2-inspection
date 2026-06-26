#!/usr/bin/env python3
"""Rasterize the facility SDF walls into an occupancy grid -- a clean, deterministic map for developing
the zone-segmentation node (Phase 1). The real pipeline feeds the live RTAB-Map /map instead; this is
just a noise-free stand-in so we can build + verify the segmentation algorithm offline.

Saves <worlds>/facility_map.npz {grid(int16: -1 unknown / 0 free / 100 occupied), res, origin_x, origin_y}
and facility_map_viz.png. Usage: python3 gen_map.py [facility.sdf]
"""
import os, re, sys
import numpy as np
import cv2

here = os.path.dirname(os.path.abspath(__file__))
worlds = os.path.abspath(os.path.join(here, "..", "worlds"))
sdf = sys.argv[1] if len(sys.argv) > 1 else os.path.join(worlds, "facility.sdf")
out = os.path.join(worlds, "facility_map.npz")

RES = 0.05
XMIN, XMAX, YMIN, YMAX = -16.0, 16.0, -11.0, 11.0
W, H = int((XMAX - XMIN) / RES), int((YMAX - YMIN) / RES)


def w2c(x, y):   # world -> (col, row); row 0 = YMIN (bottom), ROS OccupancyGrid convention
    return int(round((x - XMIN) / RES)), int(round((y - YMIN) / RES))


def main():
    text = open(sdf).read()
    walls = re.findall(r'<collision name="w\d+_c"><pose>([^<]+)</pose><geometry><box><size>([^<]+)</size>', text)
    occ = np.zeros((H, W), np.uint8)
    for pose, size in walls:
        px, py = [float(v) for v in pose.split()][:2]
        sx, sy = [float(v) for v in size.split()][:2]
        x0, y0 = w2c(px - sx / 2, py - sy / 2)
        x1, y1 = w2c(px + sx / 2, py + sy / 2)
        occ[max(0, y0):y1 + 1, max(0, x0):x1 + 1] = 1
    # flood-fill the interior free space from (0,0): exterior free is NOT connected (outer walls block it),
    # so it stays unknown -- exactly like a real SLAM map (only the explored interior is free).
    free = (occ == 0).astype(np.uint8)
    ff = free.copy()
    cv2.floodFill(ff, np.zeros((H + 2, W + 2), np.uint8), w2c(0, 0), 2)
    grid = np.full((H, W), -1, np.int16)
    grid[ff == 2] = 0
    grid[occ == 1] = 100
    np.savez(out, grid=grid, res=RES, origin_x=XMIN, origin_y=YMIN)
    viz = np.full((H, W, 3), 128, np.uint8); viz[grid == 0] = 255; viz[grid == 100] = 0
    cv2.imwrite(os.path.join(worlds, "facility_map_viz.png"), cv2.flip(viz, 0))
    print(f"saved {out}: {W}x{H} @ {RES}m  walls={len(walls)}  free={int((grid==0).sum())}  unknown={int((grid==-1).sum())}")


if __name__ == "__main__":
    main()
