#!/usr/bin/env python3
"""Create facility_gauges.sdf = facility.sdf + analog-gauge models on the SOUTH wall (Y=-10).

facility.sdf is NOT modified. Gauges are textured (emissive, so they're readable regardless of lighting)
thin panels at camera height, spaced so the robot can strafe past them and build a panorama.

GZ TEXTURE-PATH GOTCHA: gz-sim cannot resolve a texture path that resolves through a directory containing
a SPACE (our workspace is ".../EE26 Hackathon/..."), and RGBA PNGs get misread as transparent. So we
(1) flatten textures to RGB (done in gen_gauges.py) and (2) expose them at a NO-SPACE symlink in $HOME
and reference that absolute path. Re-run this script on each machine (like gen_facility.py) to refresh the
symlink + paths.

  python3 gen_gauge_world.py
"""
import os, json

here = os.path.dirname(os.path.abspath(__file__))
worlds = os.path.abspath(os.path.join(here, "..", "worlds"))
src, dst = os.path.join(worlds, "facility.sdf"), os.path.join(worlds, "facility_gauges.sdf")
tex_dir = os.path.join(worlds, "gauge_tex")

# no-space symlink in $HOME -> the (possibly space-containing) textures dir. gz opens the symlink path
# string (no space); the OS resolves it. Persistent across reboots; per-machine (hence: regenerate).
LINK = os.path.expanduser("~/.go2_gauge_tex")
if os.path.islink(LINK) or os.path.exists(LINK):
    try:
        os.remove(LINK)
    except OSError:
        pass
os.symlink(tex_dir, LINK)
TEX = LINK   # absolute, no-space

gt = json.load(open(os.path.join(tex_dir, "gauges_groundtruth.json")))
GAUGE_X = [-3.5, -2.0, -0.5, 1.0, 2.5, 4.0]   # along the south wall
WALL_Y, Z = -9.85, 0.60                        # just in front of the inner wall face, camera height


def gauge_xml(i, x):
    return (f'    <model name="gauge_{i:02d}"><static>true</static>\n'
            f'      <pose>{x} {WALL_Y} {Z} 0 0 0</pose>\n'
            f'      <link name="link"><visual name="face">\n'
            f'        <geometry><box><size>0.26 0.03 0.26</size></box></geometry>\n'
            f'        <material>\n'
            f'          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>\n'
            f'          <pbr><metal>\n'
            f'            <albedo_map>{TEX}/gauge_{i:02d}.png</albedo_map>\n'
            f'            <emissive_map>{TEX}/gauge_{i:02d}.png</emissive_map>\n'
            f'            <metalness>0.0</metalness><roughness>1.0</roughness>\n'
            f'          </metal></pbr>\n'
            f'        </material>\n'
            f'      </visual></link>\n'
            f'    </model>')


def main():
    text = open(src).read()
    models = "\n".join(gauge_xml(i, GAUGE_X[i]) for i in range(len(gt)))
    assert "</world>" in text, "no </world> in facility.sdf"
    text = text.replace("</world>", models + "\n  </world>", 1)
    open(dst, "w").write(text)
    print(f"wrote {dst}")
    print(f"  textures via no-space symlink: {LINK} -> {tex_dir}")
    print(f"  {len(gt)} gauges on the SOUTH wall: Y={WALL_Y}, z={Z}, X={GAUGE_X}")
    for g, x in zip(gt, GAUGE_X):
        print(f"    {g['id']}: {g['type']} {g['true_reading']}{g['unit']} (range {g['range_min']}-{g['range_max']}) @ x={x}")


if __name__ == "__main__":
    main()
