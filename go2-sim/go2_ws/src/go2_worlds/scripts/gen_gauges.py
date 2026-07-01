#!/usr/bin/env python3
"""Generate analog-gauge face textures (PNG) + a ground-truth manifest, for the sim inspection task.

Each gauge is a circular dial (270 deg sweep, 225deg -> -45deg clockwise) with major/minor ticks, numeric
labels, a type+unit caption, a red danger arc near the top of the range, and a red needle at a KNOWN
reading. The known reading/type/unit/range are written to gauges_groundtruth.json so we can later score
the model's readings. Output: <worlds>/gauge_tex/gauge_NN.png  +  gauge_tex/gauges_groundtruth.json

  python3 gen_gauges.py [output_dir]   (default: ../worlds/gauge_tex relative to this script)
"""
import os, sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Arc

# (type, unit, vmin, vmax, reading, danger_starts_at_fraction)
GAUGES = [
    ("PRESSURE",    "psi", 0, 100, 45,  0.85),
    ("VOLTAGE",     "V",   0, 300, 220, 0.90),
    ("TEMPERATURE", "degC",0, 150, 90,  0.80),
    ("PRESSURE",    "bar", 0, 10,  7.0, 0.85),
    ("CURRENT",     "A",   0, 50,  30,  0.85),
    ("PRESSURE",    "psi", 0, 200, 175, 0.80),   # reads into the red zone
]

START_DEG, SWEEP_DEG = 225.0, 270.0   # value=min at 225deg, sweeps clockwise 270deg to -45deg


def val_to_deg(v, vmin, vmax):
    return START_DEG - SWEEP_DEG * (v - vmin) / (vmax - vmin)


def draw_gauge(spec, path):
    gtype, unit, vmin, vmax, reading, danger_frac = spec
    fig, ax = plt.subplots(figsize=(5.12, 5.12), dpi=100)
    ax.set_xlim(-1.1, 1.1); ax.set_ylim(-1.1, 1.1); ax.set_aspect("equal"); ax.axis("off")
    fig.patch.set_facecolor("white")

    ax.add_patch(Circle((0, 0), 1.0, facecolor="white", edgecolor="black", lw=4, zorder=1))
    ax.add_patch(Circle((0, 0), 0.97, facecolor="none", edgecolor="black", lw=1, zorder=1))

    # red danger arc near the top of the range
    d0 = val_to_deg(vmin + danger_frac * (vmax - vmin), vmin, vmax)
    d1 = val_to_deg(vmax, vmin, vmax)
    ax.add_patch(Arc((0, 0), 1.84, 1.84, angle=0, theta1=min(d0, d1), theta2=max(d0, d1),
                     color="red", lw=10, zorder=2))

    # ticks + numbers: 10 major divisions
    nmaj = 10
    for i in range(nmaj + 1):
        v = vmin + (vmax - vmin) * i / nmaj
        th = np.radians(val_to_deg(v, vmin, vmax))
        c, s = np.cos(th), np.sin(th)
        ax.plot([0.78 * c, 0.92 * c], [0.78 * s, 0.92 * s], color="black", lw=3, zorder=3)
        num = f"{v:.0f}" if (vmax - vmin) >= 20 else f"{v:.1f}"
        ax.text(0.66 * c, 0.66 * s, num, ha="center", va="center", fontsize=15,
                fontweight="bold", zorder=3)
        if i < nmaj:  # 4 minor ticks between majors
            for j in range(1, 5):
                vm = v + (vmax - vmin) / nmaj * j / 5
                thm = np.radians(val_to_deg(vm, vmin, vmax))
                cm, sm = np.cos(thm), np.sin(thm)
                ax.plot([0.85 * cm, 0.92 * cm], [0.85 * sm, 0.92 * sm], color="black", lw=1, zorder=3)

    # caption (type + unit)
    ax.text(0, 0.34, gtype, ha="center", va="center", fontsize=20, fontweight="bold", zorder=3)
    ax.text(0, -0.40, unit, ha="center", va="center", fontsize=22, zorder=3)

    # needle at the reading
    thn = np.radians(val_to_deg(reading, vmin, vmax))
    ax.plot([0, 0.72 * np.cos(thn)], [0, 0.72 * np.sin(thn)], color="#cc0000", lw=5, zorder=4,
            solid_capstyle="round")
    ax.add_patch(Circle((0, 0), 0.06, facecolor="black", zorder=5))

    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    fig.savefig(path, dpi=100, facecolor="white"); plt.close(fig)
    # Flatten RGBA -> RGB on white: gz/ogre2 misreads a PNG alpha channel as transparency, which makes
    # the textured panel render see-through (we hit this -- gauges showed as blank gray). RGB fixes it.
    from PIL import Image
    im = Image.open(path).convert("RGBA")
    bg = Image.new("RGB", im.size, (255, 255, 255)); bg.paste(im, mask=im.split()[3]); bg.save(path)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "..", "worlds", "gauge_tex")
    out = os.path.abspath(out); os.makedirs(out, exist_ok=True)
    gt = []
    for i, spec in enumerate(GAUGES):
        name = f"gauge_{i:02d}"
        draw_gauge(spec, os.path.join(out, name + ".png"))
        gt.append({"id": name, "type": spec[0], "unit": spec[1],
                   "range_min": spec[2], "range_max": spec[3], "true_reading": spec[4]})
        print(f"  wrote {name}.png  ({spec[0]} {spec[4]} {spec[1]})")
    with open(os.path.join(out, "gauges_groundtruth.json"), "w") as f:
        json.dump(gt, f, indent=2)
    print(f"{len(GAUGES)} gauges + groundtruth -> {out}")


if __name__ == "__main__":
    main()
