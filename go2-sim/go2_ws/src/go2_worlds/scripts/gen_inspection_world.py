#!/usr/bin/env python3
"""Create facility_inspection.sdf = facility.sdf + analog gauges DISTRIBUTED across multiple rooms.

This is the realistic multi-room inspection scenario: instead of all gauges on one wall (facility_gauges.sdf),
~12 assets of 5 types (PRESSURE / VOLTAGE / TEMPERATURE / CURRENT / FLOW) are spread over 5 rooms so the
robot must genuinely navigate the facility (corridor -> doorway -> room -> wall) to inspect them. A few
gauges read into the RED danger zone on purpose, so anomaly/fault detection has real faults to find.

ADDITIVE + non-destructive: facility.sdf, facility_gauges.sdf, gauge_tex/ and the original generators are
NOT touched. This writes a SEPARATE world (facility_inspection.sdf), a SEPARATE texture set
(inspection_tex/) behind its own no-space symlink, and inspection_groundtruth.json. Reuses draw_gauge.

  python3 gen_inspection_world.py      # re-run per machine to refresh the textures + the symlink

Facility map (from facility.sdf): 30x20 m, central E-W corridor Y in [-1.5,1.5], 6 rooms off doorways
(NW/NC/NE north of the corridor, SW/SC/SE south). HOME (robot start) = (0,0) in the corridor, facing +X.
Gauges are placed on perimeter walls with >=3 m clear floor in front, clear of the obstacles o*/p* and the
doorways, so the (future) autonomous navigation + strafe sweep has room to work. Each gauge panel faces
INTO its room (yaw set per wall): south wall faces +Y, north faces -Y, west faces +X, east faces -X.
"""
import os, sys, json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_gauges import draw_gauge   # reuse the exact dial renderer (no duplication)

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.abspath(os.path.join(HERE, "..", "worlds"))
SRC = os.path.join(WORLDS, "facility.sdf")
DST = os.path.join(WORLDS, "facility_inspection.sdf")
TEX_DIR = os.path.join(WORLDS, "inspection_tex")
LINK = os.path.expanduser("~/.go2_inspection_tex")   # no-space symlink (gz can't resolve the space path)
Z = 0.60                                              # camera height

# Per-wall facing yaw (panel face normal points INTO the room):
YAW = {"south": 0.0, "north": 0.0, "west": 1.5708, "east": 1.5708}

# Distributed inspection assets. Each: room, asset (friendly id), wall, (x,y), type, unit, vmin, vmax,
# reading, danger_frac, anomaly. Positions verified clear of the o*/p* obstacles + doorways in facility.sdf.
LAYOUT = [
    # --- NC : "Thermal / HVAC" (north-centre room, north wall Y=9.85) ---
    dict(room="NC", asset="NC-TEMP-1", wall="north", x=-2.5, y=9.85, type="TEMPERATURE", unit="degC",
         vmin=0, vmax=150, reading=75, danger=0.85, anomaly=False),
    dict(room="NC", asset="NC-TEMP-2", wall="north", x=2.5, y=9.85, type="TEMPERATURE", unit="degC",
         vmin=0, vmax=200, reading=185, danger=0.80, anomaly=True),   # overheat
    # --- NE : "Electrical" (north-east room, east wall X=14.85) ---
    dict(room="NE", asset="NE-VOLT-1", wall="east", x=14.85, y=4.0, type="VOLTAGE", unit="V",
         vmin=0, vmax=300, reading=230, danger=0.85, anomaly=False),
    dict(room="NE", asset="NE-VOLT-2", wall="east", x=14.85, y=6.0, type="VOLTAGE", unit="V",
         vmin=0, vmax=500, reading=470, danger=0.85, anomaly=True),   # over-voltage
    dict(room="NE", asset="NE-CURR-1", wall="east", x=14.85, y=8.0, type="CURRENT", unit="A",
         vmin=0, vmax=50, reading=28, danger=0.85, anomaly=False),
    # --- NW : "Boiler" (north-west room, west wall X=-14.85, south sub-room Y in [1.5,6]) ---
    dict(room="NW", asset="NW-PRES-1", wall="west", x=-14.85, y=3.0, type="PRESSURE", unit="bar",
         vmin=0, vmax=16, reading=9, danger=0.85, anomaly=False),
    dict(room="NW", asset="NW-TEMP-1", wall="west", x=-14.85, y=5.0, type="TEMPERATURE", unit="degC",
         vmin=0, vmax=250, reading=120, danger=0.85, anomaly=False),
    # --- SC : "Hydraulics" (south-centre room, south wall Y=-9.85) ---
    dict(room="SC", asset="SC-PRES-1", wall="south", x=-3.0, y=-9.85, type="PRESSURE", unit="psi",
         vmin=0, vmax=100, reading=40, danger=0.85, anomaly=False),
    dict(room="SC", asset="SC-PRES-2", wall="south", x=2.0, y=-9.85, type="PRESSURE", unit="bar",
         vmin=0, vmax=10, reading=6.5, danger=0.85, anomaly=False),
    # --- SW : "Pump Station" (south-west room, south wall Y=-9.85) ---
    dict(room="SW", asset="SW-PRES-1", wall="south", x=-12.0, y=-9.85, type="PRESSURE", unit="psi",
         vmin=0, vmax=200, reading=90, danger=0.85, anomaly=False),
    dict(room="SW", asset="SW-FLOW-1", wall="south", x=-9.0, y=-9.85, type="FLOW", unit="L/min",
         vmin=0, vmax=500, reading=480, danger=0.85, anomaly=True),   # over-flow
    dict(room="SW", asset="SW-PRES-2", wall="south", x=-7.0, y=-9.85, type="PRESSURE", unit="bar",
         vmin=0, vmax=25, reading=14, danger=0.85, anomaly=False),
]


def gauge_xml(i, g):
    yaw = YAW[g["wall"]]
    return (f'    <model name="gauge_{i:02d}"><static>true</static>\n'
            f'      <pose>{g["x"]} {g["y"]} {Z} 0 0 {yaw}</pose>\n'
            f'      <link name="link"><visual name="face">\n'
            f'        <geometry><box><size>0.26 0.03 0.26</size></box></geometry>\n'
            f'        <material>\n'
            f'          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>\n'
            f'          <pbr><metal>\n'
            f'            <albedo_map>{LINK}/gauge_{i:02d}.png</albedo_map>\n'
            f'            <emissive_map>{LINK}/gauge_{i:02d}.png</emissive_map>\n'
            f'            <metalness>0.0</metalness><roughness>1.0</roughness>\n'
            f'          </metal></pbr>\n'
            f'        </material>\n'
            f'      </visual></link>\n'
            f'    </model>')


def main():
    os.makedirs(TEX_DIR, exist_ok=True)
    # 1) render the gauge faces + ground truth
    gt = []
    for i, g in enumerate(LAYOUT):
        spec = (g["type"], g["unit"], g["vmin"], g["vmax"], g["reading"], g["danger"])
        draw_gauge(spec, os.path.join(TEX_DIR, f"gauge_{i:02d}.png"))
        gt.append({"id": f"gauge_{i:02d}", "asset": g["asset"], "room": g["room"], "wall": g["wall"],
                   "type": g["type"], "unit": g["unit"], "range_min": g["vmin"], "range_max": g["vmax"],
                   "true_reading": g["reading"], "anomaly": g["anomaly"],
                   "pose": [g["x"], g["y"], Z]})
    json.dump(gt, open(os.path.join(TEX_DIR, "inspection_groundtruth.json"), "w"), indent=2)

    # 2) no-space symlink so gz can resolve the texture paths
    if os.path.islink(LINK) or os.path.exists(LINK):
        try:
            os.remove(LINK)
        except OSError:
            pass
    os.symlink(TEX_DIR, LINK)

    # 3) inject the gauge models into a COPY of facility.sdf
    text = open(SRC).read()
    assert "</world>" in text, "no </world> in facility.sdf"
    models = "\n".join(gauge_xml(i, g) for i, g in enumerate(LAYOUT))
    open(DST, "w").write(text.replace("</world>", models + "\n  </world>", 1))

    rooms = {}
    for g in LAYOUT:
        rooms.setdefault(g["room"], []).append(g["asset"])
    print(f"wrote {DST}")
    print(f"  textures via no-space symlink: {LINK} -> {TEX_DIR}")
    print(f"  {len(LAYOUT)} assets across {len(rooms)} rooms, {sum(g['anomaly'] for g in LAYOUT)} anomalies:")
    for r, a in rooms.items():
        print(f"    {r}: {', '.join(a)}")


if __name__ == "__main__":
    main()
