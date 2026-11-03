"""Render the README figures from the recorded drive logs.

Sources
  compute_logs/german_full_drive_sim.json   one simulated lap of the German GP circuit, with the
                                            ground-truth state features and the CNN's predictions
                                            for every logged step (produced by compute_drive.py)
  ../ros/.../car_controller/logs/*.json     real-car test runs, tracked with Vicon

Usage
  python make_figures.py --out ../docs/images

Every figure is written twice, ``<name>.png`` (light) and ``<name>_dark.png`` (dark), so the
README can switch on the viewer's colour scheme.
"""
import argparse
import json
import os

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REAL_LOGS = os.path.join(HERE, "..", "ros", "ros2_ws", "src", "car_controller", "car_controller", "logs")

THEMES = {
    "light": dict(surface="#fcfcfb", text="#0b0b0b", muted="#52514e", faint="#b9b8b2", grid="#e4e3df",
                  s1="#2a78d6", s2="#eb6834", road="#e9e8e4"),
    "dark": dict(surface="#1a1a19", text="#f0efec", muted="#c3c2b7", faint="#5a5955", grid="#33332f",
                 s1="#3987e5", s2="#d95926", road="#262624"),
}

FEATURES = [
    (r"$c_1$", "curvature, near"),
    (r"$c_2$", "curvature, mid"),
    (r"$c_3$", "curvature, far"),
    (r"$o$", "lateral offset"),
    (r"$\theta$", "heading error"),
    (r"$\omega$", "yaw rate"),
    (r"$v_x$", "longitudinal velocity"),
    (r"$v_y$", "lateral velocity"),
]


def style_axes(ax, c, grid=True):
    ax.set_facecolor(c["surface"])
    for side in ("top", "right", "left", "bottom"):
        ax.spines[side].set_visible(False)
    ax.tick_params(colors=c["muted"], length=0, labelsize=9)
    if grid:
        ax.grid(axis="y", color=c["grid"], linewidth=0.8)
        ax.set_axisbelow(True)


def offset_line(points, distance):
    """Offset a polyline perpendicular to its direction (track borders from the centre line)."""
    d = np.gradient(points, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    normal = np.stack([-d[:, 1], d[:, 0]], axis=1)
    return points + normal * distance


def save(fig, out_dir, name, theme, c):
    suffix = "" if theme == "light" else "_dark"
    path = os.path.join(out_dir, f"{name}{suffix}.png")
    fig.savefig(path, facecolor=c["surface"], bbox_inches="tight", pad_inches=0.15)
    plt.close(fig)
    print("saved", path)


def lap_figure(log, out_dir, theme):
    c = THEMES[theme]
    path = np.array(log["path"])
    car = np.array([p["car_pos"] for p in log["datapoints"]])
    heading = np.array([p["car_direction"] for p in log["datapoints"]])
    t = np.array([p["time_stamp"] for p in log["datapoints"]])
    track_len = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))

    fig, ax = plt.subplots(figsize=(11, 5.6), dpi=200)
    fig.patch.set_facecolor(c["surface"])
    style_axes(ax, c, grid=False)

    # road surface (6 m wide) and its borders
    ax.plot(path[:, 0], path[:, 1], color=c["road"], linewidth=9, solid_capstyle="round", zorder=1)
    for side in (3, -3):
        b = offset_line(path, side)
        ax.plot(b[:, 0], b[:, 1], color=c["faint"], linewidth=0.8, zorder=2)

    # heading of the car every few steps
    for i in range(0, len(car), 4):
        p, h = car[i], heading[i] / np.linalg.norm(heading[i])
        ax.plot([p[0], p[0] + h[0] * 4], [p[1], p[1] + h[1] * 4], color=c["muted"], linewidth=0.7, zorder=3)

    ax.plot(car[:, 0], car[:, 1], color=c["s1"], linewidth=2, zorder=4)
    ax.plot(*car[0], "o", color=c["s1"], markersize=8, markeredgecolor=c["surface"], markeredgewidth=2, zorder=5)
    ax.annotate("start", car[0], xytext=(10, -14), textcoords="offset points", color=c["muted"], fontsize=10)

    # 50 m scale bar
    x0, y0 = path[:, 0].min(), path[:, 1].min() - 12
    ax.plot([x0, x0 + 50], [y0, y0], color=c["muted"], linewidth=2)
    ax.text(x0 + 25, y0 - 4, "50 m", color=c["muted"], ha="center", va="top", fontsize=9)

    ax.set_aspect("equal")
    ax.set_xticks([]), ax.set_yticks([])
    ax.set_title(f"One autonomous lap of the German GP circuit   ·   {track_len:.0f} m in {t[-1] - t[0]:.0f} s",
                 loc="left", color=c["text"], fontsize=13, pad=10)
    save(fig, out_dir, "lap_german_gp", theme, c)


def feature_figure(log, out_dir, theme):
    c = THEMES[theme]
    pts = log["datapoints"]
    t = np.array([p["time_stamp"] for p in pts])
    t -= t[0]
    target = np.array([p["target_features"] for p in pts])
    cnn = np.array([p["cnn_features"] for p in pts])
    cnn[:, 4] = 1 - cnn[:, 4]  # θ is predicted with the opposite sign convention (as in create_feature_plots.py)

    fig, axs = plt.subplots(4, 2, figsize=(11, 7.5), dpi=200, sharex=True)
    fig.patch.set_facecolor(c["surface"])
    for i, ax in enumerate(axs.T.flat):
        sym, name = FEATURES[i]
        style_axes(ax, c)
        ax.plot(t, target[:, i], color=c["s1"], linewidth=1.6, label="ground truth (simulator)")
        ax.plot(t, cnn[:, i], color=c["s2"], linewidth=1.2, label="CNN prediction (camera only)")
        mae = np.mean(np.abs(target[:, i] - cnn[:, i]))
        ax.set_title(f"{sym}  {name}", loc="left", color=c["text"], fontsize=10.5, pad=4)
        ax.text(1.0, 1.02, f"MAE {mae:.2f}", transform=ax.transAxes, ha="right", va="bottom",
                color=c["muted"], fontsize=9)
        ax.set_ylim(-0.05, 1.05)
        ax.set_yticks([0, 0.5, 1])
    for ax in axs[-1]:
        ax.set_xlabel("time (s)", color=c["muted"], fontsize=9)

    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", ncol=2, frameon=False, labelcolor=c["text"],
               bbox_to_anchor=(0.005, 0.975), fontsize=10)
    fig.suptitle("What the CNN reads from three stacked camera frames, over the same lap",
                 x=0.012, y=1.0, ha="left", color=c["text"], fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    save(fig, out_dir, "cnn_features", theme, c)


def real_car_figure(out_dir, theme, runs=("20230915-143743.json", "20230915-144410.json")):
    c = THEMES[theme]
    fig, ax = plt.subplots(figsize=(6, 4.6), dpi=200)
    fig.patch.set_facecolor(c["surface"])
    style_axes(ax, c, grid=False)
    for k, (run, col) in enumerate(zip(runs, (c["s1"], c["s2"]))):
        log = json.load(open(os.path.join(REAL_LOGS, run)))
        p = np.array([x["car_position"] for x in log if x["car_position"][0] is not None])
        dur = log[-1]["timestamp"] - log[0]["timestamp"]
        ax.plot(p[:, 0], p[:, 1], color=col, linewidth=2, label=f"run {k + 1}  ({dur:.0f} s)")
        ax.plot(*p[0], "o", color=col, markersize=7, markeredgecolor=c["surface"], markeredgewidth=2)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", color=c["muted"], fontsize=9)
    ax.set_ylabel("y (m)", color=c["muted"], fontsize=9)
    ax.grid(color=c["grid"], linewidth=0.8)
    ax.legend(frameon=False, labelcolor=c["text"], fontsize=9, loc="lower left")
    ax.set_title("Real-car test runs, tracked with Vicon", loc="left", color=c["text"], fontsize=12, pad=8)
    save(fig, out_dir, "real_car_runs", theme, c)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", default=os.path.join(HERE, "compute_logs", "german_full_drive_sim.json"))
    parser.add_argument("--out", default=os.path.join(HERE, "..", "docs", "images"))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans"})
    lap = json.load(open(args.log))
    for theme in THEMES:
        lap_figure(lap, args.out, theme)
        feature_figure(lap, args.out, theme)
        real_car_figure(args.out, theme)
