#!/usr/bin/env python3
"""Convert a saved occupancy-grid .npz (grid int16 {-1,0,100}, res, origin_x, origin_y) into the
standard nav2 map_server format (.pgm + .yaml), so a static map_server can serve the FULL facility grid
to Nav2's global costmap (rtabmap's own /map is only a local window in localization mode --).

  python3 npz_to_map.py facility_inspection_map.npz   ->  facility_inspection_map.pgm + .yaml
"""
import sys, os
import numpy as np

npz = sys.argv[1] if len(sys.argv) > 1 else "facility_inspection_map.npz"
base = os.path.splitext(npz)[0]
pgm, yml = base + ".pgm", base + ".yaml"

d = np.load(npz)
g = d["grid"]; res = float(d["res"]); ox = float(d["origin_x"]); oy = float(d["origin_y"])

# occupancy -> pixel: 100 (occupied) -> 0 (black), 0 (free) -> 254 (white), -1 (unknown) -> 205 (gray)
img = np.full(g.shape, 205, np.uint8)
img[g == 0] = 254
img[g == 100] = 0
img = np.flipud(img)   # ROS grid row 0 = min-y (bottom); PGM row 0 = top -> flip vertically

with open(pgm, "wb") as f:
    f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
    f.write(img.tobytes())

with open(yml, "w") as f:
    f.write(f"image: {os.path.basename(pgm)}\n")
    f.write(f"resolution: {res}\n")
    f.write(f"origin: [{ox}, {oy}, 0.0]\n")
    f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

print(f"wrote {pgm} ({img.shape[1]}x{img.shape[0]}) + {yml}  (res={res}, origin=[{ox:.2f},{oy:.2f}])")
