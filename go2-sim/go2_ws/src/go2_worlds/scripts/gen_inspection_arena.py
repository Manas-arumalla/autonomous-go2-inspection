#!/usr/bin/env python3
"""Generate inspection_arena.sdf -- a richly-populated, sim-only multi-room inspection facility for the Go2.

The world populates six rooms with ready-made Gazebo Fuel models (warehouse racks, drums, valves, pumps,
electrical boxes, extinguishers, furniture, clutter), a self-contained fire (particle emitter, no external
asset), and a walking human actor -- the kinds of targets and obstacles a real inspection mission must
perceive and avoid.

Layout reuses the facility.sdf shell (30x20 m, central corridor + 6 rooms, robot spawns at (0,0)) so the
Go2 + Nav2 tuning still applies; only the per-room wall colours (for RGBD/RTAB-Map visual variety) and the
room contents change. All props are <static> to keep physics cheap. Walls match facility.sdf geometry
exactly, just split into per-zone coloured segments.

  python3 gen_inspection_arena.py        # writes ../worlds/inspection_arena.sdf

Every prop is a Fuel <include>; the first `gz sim` launch downloads them to ~/.gz/fuel (network required),
then caches. The human actor mesh (~11 MB) is the single heaviest asset and is included exactly once. Launch
with:
  ros2 launch go2_bringup sim_mapping.launch.py world:=inspection_arena.sdf headless:=false
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.abspath(os.path.join(HERE, "..", "worlds"))
DST = os.path.join(WORLDS, "inspection_arena.sdf")
TEX = os.path.join(WORLDS, "fire_tex")  # generated fire sprite + colour ramps live here
FUEL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models"
H = 1.6  # wall height in metres (matches facility.sdf, which Go2/Nav2 is tuned for)
HZ = H / 2.0  # wall centre Z


def gen_fire_textures(tex_dir):
    """Write the particle textures used by the fire (soft sprite + flame/smoke colour-over-lifetime ramps).

    Mirrors the Fuel 'fog generator' recipe (a 256x256 soft alpha sprite + a 64x1 RGBA lifetime ramp). A
    bare particle_emitter with no albedo_map renders as opaque white squares, so the sprite is required.
    """
    try:
        import numpy as np
        from PIL import Image
    except Exception as e:
        print(
            f"  WARNING: PIL/numpy missing ({e}) -> fire textures NOT generated; fire will look wrong."
        )
        return False
    os.makedirs(tex_dir, exist_ok=True)
    # soft radial sprite (white, gaussian-ish alpha falloff) -> gives particles a soft puff shape
    N = 256
    yy, xx = np.mgrid[0:N, 0:N]
    r = np.sqrt((xx - N / 2) ** 2 + (yy - N / 2) ** 2) / (N / 2)
    alpha = np.clip(1.0 - r, 0, 1) ** 1.7
    spr = np.zeros((N, N, 4), np.uint8)
    spr[..., :3] = 255
    spr[..., 3] = (alpha * 255).astype(np.uint8)
    Image.fromarray(spr, "RGBA").save(os.path.join(tex_dir, "puff.png"))

    def ramp(
        stops, name
    ):  # stops: [(t in 0..1, (r,g,b,a in 0..1)), ...] -> 64x1 RGBA over particle life
        W = 64
        ts = [s[0] for s in stops]
        cs = [np.array(s[1], float) for s in stops]
        out = np.zeros((1, W, 4), float)
        for i in range(W):
            t = i / (W - 1)
            for j in range(len(ts) - 1):
                if ts[j] <= t <= ts[j + 1]:
                    f = (t - ts[j]) / (ts[j + 1] - ts[j] + 1e-9)
                    out[0, i] = cs[j] + (cs[j + 1] - cs[j]) * f
                    break
            else:
                out[0, i] = cs[-1]
        Image.fromarray((np.clip(out, 0, 1) * 255).astype(np.uint8), "RGBA").save(
            os.path.join(tex_dir, name)
        )

    # FLAME: birth bright yellow-white (opaque) -> orange -> red -> death dark-red (transparent)
    ramp(
        [
            (0.0, (1.0, 0.95, 0.55, 1.0)),
            (0.25, (1.0, 0.6, 0.1, 1.0)),
            (0.6, (0.9, 0.2, 0.0, 0.85)),
            (1.0, (0.35, 0.0, 0.0, 0.0)),
        ],
        "flamecolors.png",
    )
    # SMOKE: birth dark grey (semi) -> grey -> death light grey (transparent)
    ramp(
        [
            (0.0, (0.22, 0.22, 0.22, 0.0)),
            (0.15, (0.28, 0.28, 0.28, 0.55)),
            (0.6, (0.45, 0.45, 0.45, 0.4)),
            (1.0, (0.6, 0.6, 0.6, 0.0)),
        ],
        "smokecolors.png",
    )
    return True


# Per-zone wall colours (muted but distinguishable, for RGBD camera + RTAB-Map visual variety).
COL = {
    "nw": "0.70 0.45 0.35",  # terracotta  -- warehouse aisle
    "nc": "0.55 0.62 0.70",  # slate blue  -- office
    "ne": "0.50 0.62 0.52",  # sage green  -- mechanical
    "sw": "0.72 0.40 0.40",  # muted red   -- FIRE hazard zone (semantic warning)
    "sc": "0.74 0.68 0.45",  # warm sand   -- break room
    "se": "0.55 0.55 0.62",  # grey-violet -- logistics
    "cor": "0.82 0.82 0.80",  # off-white   -- corridor spine
    "div": "0.65 0.65 0.67",  # neutral     -- interior dividers
}

# Walls: (name, cx, cy, sx, sy, colour_key). Same geometry as facility.sdf, split for per-zone colour.
WALLS = [
    # north outer wall (Y=10) split into the three north rooms' back walls
    ("nw_back", -10, 10, 10, 0.15, "nw"),
    ("nc_back", 0, 10, 10, 0.15, "nc"),
    ("ne_back", 10, 10, 10, 0.15, "ne"),
    # south outer wall (Y=-10)
    ("sw_back", -10, -10, 10, 0.15, "sw"),
    ("sc_back", 0, -10, 10, 0.15, "sc"),
    ("se_back", 10, -10, 10, 0.15, "se"),
    # east outer wall (X=15) split: NE side / corridor end / SE side
    ("ne_side", 15, 5.75, 0.15, 8.5, "ne"),
    ("cor_e", 15, 0, 0.15, 3.0, "cor"),
    ("se_side", 15, -5.75, 0.15, 8.5, "se"),
    # west outer wall (X=-15)
    ("nw_side", -15, 5.75, 0.15, 8.5, "nw"),
    ("cor_w", -15, 0, 0.15, 3.0, "cor"),
    ("sw_side", -15, -5.75, 0.15, 8.5, "sw"),
    # corridor inner walls (Y=+/-1.5) with doorway gaps -- neutral off-white spine
    ("c_n0", -13.25, 1.5, 3.5, 0.15, "cor"),
    ("c_n1", -5.0, 1.5, 7.0, 0.15, "cor"),
    ("c_n2", 5.0, 1.5, 7.0, 0.15, "cor"),
    ("c_n3", 13.25, 1.5, 3.5, 0.15, "cor"),
    ("c_s0", -13.25, -1.5, 3.5, 0.15, "cor"),
    ("c_s1", -5.0, -1.5, 7.0, 0.15, "cor"),
    ("c_s2", 5.0, -1.5, 7.0, 0.15, "cor"),
    ("c_s3", 13.25, -1.5, 3.5, 0.15, "cor"),
    # interior room dividers (X=+/-5)
    ("div_nw", -5, 5.75, 0.15, 8.5, "div"),
    ("div_ne", 5, 5.75, 0.15, 8.5, "div"),
    ("div_sw", -5, -5.75, 0.15, 8.5, "div"),
    ("div_se", 5, -5.75, 0.15, 8.5, "div"),
    # sub-walls (NW alcove @ Y=6, SE alcove @ Y=-6) -- coloured with their room
    ("sub_nw0", -14.25, 6.0, 1.5, 0.15, "nw"),
    ("sub_nw1", -7.75, 6.0, 5.5, 0.15, "nw"),
    ("sub_se0", 7.75, -6.0, 5.5, 0.15, "se"),
    ("sub_se1", 14.25, -6.0, 1.5, 0.15, "se"),
]

# Fuel props: (instance_name, "Model Name", x, y, z, yaw).
PROPS = [
    # NW -- warehouse storage aisle
    ("nw_rack0", "StorageRack", -13, 8, 0, 0),
    ("nw_rack1", "StorageRack", -7, 8, 0, 0),
    ("nw_pallet0", "Pallet_Standard", -13, 4.5, 0, 0),
    ("nw_pallet1", "Pallet_Standard", -7, 4.5, 0, 0),
    ("nw_crate", "Large Crate", -10, 9, 0, 1.5708),
    ("nw_box0", "Cardboard box", -13, 4.5, 0.15, 0.3),
    ("nw_box1", "Cardboard box", -7, 4.5, 0.15, -0.4),
    ("nw_cone", "Construction Cone", -10, 3, 0, 0),
    # NC -- office / supervisor area
    ("nc_desk0", "Desk", -3.5, 8.5, 0, -1.5708),
    ("nc_chair0", "OfficeChairGrey", -3.5, 7.7, 0, 1.5708),
    ("nc_desk1", "Desk", 3.5, 8.5, 0, -1.5708),
    ("nc_chair1", "OfficeChairGrey", 3.5, 7.7, 0, 1.5708),
    ("nc_cab", "WhiteCabinet", -4.2, 9.4, 0, 0),
    ("nc_shelf", "Bookshelf", 4.2, 9.4, 0, 3.1416),
    ("nc_trash", "TrashBin", -3.5, 6.5, 0, 0),
    # NE -- mechanical / inspection targets
    ("ne_pump", "Pump", 7.5, 8, 0, 0),
    ("ne_valve", "Valve", 9.5, 8, 0, 1.5708),
    ("ne_cab", "MetalCabinet", 12.5, 8.7, 0, -1.5708),  # control cabinet
    ("ne_drum0", "55gal_Drum", 13.5, 4.5, 0, 0),
    ("ne_drum1", "55gal_Drum", 13.5, 5.5, 0, 0),
    ("ne_drum2", "55gal_Drum", 6, 9.2, 0, 0),
    ("ne_ebox", "Electrical Box", 14.6, 7, 1.0, -1.5708),  # wall-mounted, faces -X into the room
    ("ne_barrel", "Construction Barrel", 10, 3, 0, 0),
    # SW -- fire hazard zone (the fire emitter sits over sw_drum; see the FIRE block)
    ("sw_drum", "55gal_Drum", -10, -6, 0, 0),
    ("sw_ext0", "Fire Extinguisher", -12.5, -3, 0, 0),
    ("sw_ext1", "Fire Extinguisher", -7.5, -3, 0, 0),
    ("sw_extcab", "Extinguisher cabinet", -14.6, -6, 1.0, 1.5708),
    ("sw_hydrant", "Fire hydrant", -13, -9, 0, 0),
    ("sw_barrier0", "Jersey Barrier", -10, -4, 0, 0),
    ("sw_barrier1", "Jersey Barrier", -10, -8, 0, 0),
    ("sw_cone0", "Construction Cone", -8.5, -5, 0, 0),
    ("sw_cone1", "Construction Cone", -11.5, -5, 0, 0),
    # SC -- break room (the walking actor loops in the open centre of this room; see the ACTOR block)
    ("sc_table0", "Table", -3, -8, 0, 0),
    ("sc_chair0", "WoodenChair", -3, -7, 0, 3.1416),
    ("sc_chair1", "WoodenChair", -3, -9, 0, 0),
    ("sc_table1", "Table", 3, -8, 0, 0),
    ("sc_chair2", "WoodenChair", 3, -7, 0, 3.1416),
    ("sc_trash", "TrashBin", 4.4, -9.4, 0, 0),
    # SE -- logistics / pallet staging
    ("se_pallet0", "Pallet_Standard", 7, -8, 0, 0),
    ("se_pallet1", "Pallet_Standard", 9, -8, 0, 0),
    ("se_crate", "Large Crate", 7, -8, 0.15, 0),
    ("se_box", "Cardboard box", 9, -8, 0.15, 0.2),
    ("se_cab0", "MetalCabinet", 14.6, -4, 0, -1.5708),
    ("se_cab1", "MetalCabinetYellow", 14.6, -7, 0, -1.5708),
    ("se_box2", "Cardboard box", 11, -9, 0, 0),
    ("se_cone", "Construction Cone", 10, -3, 0, 0),
    ("se_exit", "Exit sign", 14.6, -2, 1.4, -1.5708),
    # corridor compliance props (tight to corridor walls, clear of doorways at x=-10/0/+10)
    ("cor_exit_w", "Exit sign", -14.6, 1.4, 1.4, 1.5708),
    ("cor_ext0", "Fire Extinguisher", -13, -1.3, 0, 0),
    ("cor_ext1", "Fire Extinguisher", 13, 1.3, 0, 3.1416),
]

# Fire: two self-contained particle emitters (orange flames + grey smoke). No external texture/asset.
FIRE = f"""
    <!-- FIRE HAZARD over the SW drum (-10,-6). TEXTURED particle emitters: a soft sprite (puff.png) tinted
         by a colour-over-lifetime ramp. NOTE: a particle_emitter with NO albedo_map renders as opaque WHITE
         squares (the bug that was seen in-sim); the material/pbr albedo_map is what makes soft flames.
         Needs the gz-sim-particle-emitter-system plugin (added in the world header). -->
    <model name="hazard_fire"><static>true</static>
      <pose>-10 -6 0.9 0 0 0</pose>
      <link name="fire_link">
        <particle_emitter name="flames" type="box">
          <emitting>true</emitting>
          <particle_scatter_ratio>0.0</particle_scatter_ratio>  <!-- 0 = LiDAR/depth IGNORE these particles (default 0.65 made smoke a map obstacle) -->
          <size>0.5 0.5 0.1</size>
          <particle_size>0.45 0.45 0.45</particle_size>
          <lifetime>1.3</lifetime>
          <min_velocity>0.5</min_velocity><max_velocity>1.2</max_velocity>
          <scale_rate>0.4</scale_rate><rate>45</rate>
          <material>
            <diffuse>1 1 1</diffuse><specular>1 1 1</specular>
            <pbr><metal><albedo_map>{TEX}/puff.png</albedo_map></metal></pbr>
          </material>
          <color_range_image>{TEX}/flamecolors.png</color_range_image>
        </particle_emitter>
        <particle_emitter name="smoke" type="box">
          <pose>0 0 1.2 0 0 0</pose>
          <emitting>true</emitting>
          <particle_scatter_ratio>0.0</particle_scatter_ratio>  <!-- 0 = LiDAR/depth IGNORE these particles (default 0.65 made smoke a map obstacle) -->
          <size>0.6 0.6 0.1</size>
          <particle_size>0.9 0.9 0.9</particle_size>
          <lifetime>4.0</lifetime>
          <min_velocity>0.6</min_velocity><max_velocity>1.0</max_velocity>
          <scale_rate>1.0</scale_rate><rate>12</rate>
          <material>
            <diffuse>1 1 1</diffuse>
            <pbr><metal><albedo_map>{TEX}/puff.png</albedo_map></metal></pbr>
          </material>
          <color_range_image>{TEX}/smokecolors.png</color_range_image>
        </particle_emitter>
      </link>
    </model>
"""

# Walking human actor: Mingfei Fuel actor (walk.dae). Loops a small rectangle in the open SC break room.
# Z=1.0 is the actor's standard pivot (feet on the floor).
ACTOR = """
    <actor name="inspection_walker">
      <skin>
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walk">
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
        <scale>1.0</scale><interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop><delay_start>0.0</delay_start><auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0.6">
          <waypoint><time>0.0</time><pose>-2 -4 1.0 0 0 0</pose></waypoint>
          <waypoint><time>5.0</time><pose>2 -4 1.0 0 0 0</pose></waypoint>
          <waypoint><time>6.5</time><pose>2 -4 1.0 0 0 -1.5708</pose></waypoint>
          <waypoint><time>9.0</time><pose>2 -6 1.0 0 0 -1.5708</pose></waypoint>
          <waypoint><time>10.5</time><pose>2 -6 1.0 0 0 3.1416</pose></waypoint>
          <waypoint><time>15.5</time><pose>-2 -6 1.0 0 0 3.1416</pose></waypoint>
          <waypoint><time>17.0</time><pose>-2 -6 1.0 0 0 1.5708</pose></waypoint>
          <waypoint><time>19.5</time><pose>-2 -4 1.0 0 0 1.5708</pose></waypoint>
          <waypoint><time>21.0</time><pose>-2 -4 1.0 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>
"""

HEADER = """<?xml version="1.0" ?>
<!-- inspection_arena.sdf  (GENERATED by gen_inspection_arena.py ; edit the generator, not this file)
     Sim-only Go2 INSPECTION facility: facility.sdf shell (30x20 m, central corridor + 6 rooms, robot spawns
     at (0,0)) with PER-ROOM coloured walls + ready-made Fuel props + a fire + a walking human actor.
     Rooms: NW warehouse | NC office | NE mechanical | SW FIRE hazard | SC break-room(+walker) | SE logistics.
     First launch downloads the Fuel models to ~/.gz/fuel (network required). -->
<sdf version="1.10">
  <world name="inspection_arena">
    <physics name="1ms" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system"           name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-contact-system"           name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system"               name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-particle-emitter-system"  name="gz::sim::systems::ParticleEmitter"/>

    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.7 0.7 0.8 1</background><grid>false</grid></scene>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.9</direction></light>

    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
        <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
    </link></model>
"""


def wall_xml(name, cx, cy, sx, sy, ckey):
    c = COL[ckey]
    return (
        f'        <collision name="{name}_c"><pose>{cx} {cy} {HZ} 0 0 0</pose>'
        f"<geometry><box><size>{sx} {sy} {H}</size></box></geometry></collision>\n"
        f'        <visual name="{name}_v"><pose>{cx} {cy} {HZ} 0 0 0</pose>'
        f"<geometry><box><size>{sx} {sy} {H}</size></box></geometry>"
        f"<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse><specular>0.1 0.1 0.1 1</specular></material></visual>"
    )


def include_xml(name, model, x, y, z, yaw):
    uri = f"{FUEL}/{model}"
    return (
        f"    <include><name>{name}</name><static>true</static>\n"
        f"      <pose>{x} {y} {z} 0 0 {yaw}</pose>\n"
        f"      <uri>{uri}</uri>\n"
        f"    </include>"
    )


def main():
    ok = gen_fire_textures(TEX)
    parts = [HEADER]
    # walls: one static model, many coloured segments
    parts.append('\n    <model name="facility"><static>true</static><link name="structure">')
    parts.append("\n".join(wall_xml(*w) for w in WALLS))
    parts.append("    </link></model>\n")
    # Fuel props
    parts.append("\n".join(include_xml(*p) for p in PROPS))
    # fire + actor
    parts.append(FIRE)
    parts.append(ACTOR)
    parts.append("\n  </world>\n</sdf>\n")
    with open(DST, "w") as f:
        f.write("\n".join(parts))

    rooms = {}
    for p in PROPS:
        rooms.setdefault(p[0].split("_")[0].upper(), []).append(p[1])
    print(f"wrote {DST}")
    print(
        f"  fire textures: {'generated in ' + TEX if ok else 'NOT generated (PIL/numpy missing)'}"
    )
    print(
        f"  {len(WALLS)} coloured wall segments, {len(PROPS)} Fuel props, 1 fire, 1 walking actor"
    )
    for r, models in sorted(rooms.items()):
        print(f"    {r}: {len(models)} props -> {', '.join(sorted(set(models)))}")


if __name__ == "__main__":
    main()
