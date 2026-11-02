#!/usr/bin/env python3
"""plot_scate.py - plots and a 3D replay page for a SCATE closed-loop run.

Reads the replay.json written by scate_online (scate_replay.h) and writes:
  scate_dashboard.png  clearances, obstacle estimate, tool path, solver stats
  scate_joints.png     joint angles per joint: online, blind, offline plan
  scate_controls.png   executed controls per joint, with the limits
  scate_replay.html    the 3D replay viewer with this run built in; opened in
                       your browser unless --no-open (needs viz/ next to this
                       script; three.js loads from a CDN, so it needs internet)

Usage:  python3 plot_scate.py [replay.json] [--out DIR] [--no-open] [--no-html] [--show]
Needs numpy and matplotlib only.
"""

import argparse
import json
import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

# Colors: validated categorical slots 1-2 for the two runs; ink and chrome for
# everything else; the critical status color only for "in collision".
ONLINE = "#2a78d6"
BLIND = "#eb6834"
PLAN = "#898781"      # muted ink: offline plan, replans, reference lines
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SURFACE = "#fcfcfb"
CRITICAL = "#d03b3b"

RUN_COLOR = {"online": ONLINE, "blind": BLIND}


def style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": AXIS,
        "axes.labelcolor": INK2,
        "axes.titlecolor": INK,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.labelsize": 9,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "xtick.color": INK2,
        "ytick.color": INK2,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8,
        "legend.frameon": False,
        "lines.linewidth": 1.8,
        "lines.solid_capstyle": "round",
        "font.family": "sans-serif",
    })


def load(path):
    with open(path) as f:
        d = json.load(f)
    if d.get("format") != "scate-replay":
        sys.exit(f"{path} is not a scate-replay file")
    runs = {r["name"]: r for r in d["runs"]}
    return d, runs


def frames_array(run, key):
    return np.array([np.nan if f[key] is None else f[key] for f in run["frames"]], dtype=float)


def first_detection_time(d, run):
    k = run["summary"]["first_detection"]
    return None if k < 0 else k * d["dt"]


def mark_detection(ax, t, label=True):
    if t is None:
        return
    ax.axvline(t, color=ONLINE, lw=1, ls=":", alpha=0.9)
    if label:
        ax.annotate("obstacle first seen", xy=(t, 1), xycoords=("data", "axes fraction"),
                    xytext=(4, -4), textcoords="offset points", va="top", ha="left",
                    fontsize=8, color=INK2)


def run_label(run):
    s = run["summary"]
    tag = "collides" if s["collided"] else "clear"
    return f'{run["label"]} ({tag})'


def plot_clearance(ax, d, runs):
    standoff = d["obstacle_standoff"]
    for name, run in runs.items():
        t = frames_array(run, "t")
        c = frames_array(run, "clear")
        ax.plot(t, 100 * c, color=RUN_COLOR.get(name, INK2), label=run_label(run))
    ax.axhline(100 * standoff, color=PLAN, lw=1, ls="--")
    ax.annotate(f"standoff {100 * standoff:.0f} cm", xy=(0, 100 * standoff), xytext=(2, 3),
                textcoords="offset points", fontsize=8, color=INK2)
    ymin = min(ax.get_ylim()[0], -1.0)
    ax.axhspan(ymin, 0, color=CRITICAL, alpha=0.08, lw=0)
    ax.axhline(0, color=CRITICAL, lw=1)
    ax.annotate("in collision", xy=(1, 0), xycoords=("axes fraction", "data"), xytext=(-2, -3),
                textcoords="offset points", ha="right", va="top", fontsize=8, color=CRITICAL)
    ax.set_ylim(bottom=ymin)
    if "online" in runs:
        mark_detection(ax, first_detection_time(d, runs["online"]))
    ax.set_title("Obstacle clearance")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("closest sphere to obstacle surface (cm)")
    ax.legend(loc="lower left")


def plot_self(ax, d, runs):
    standoff = d["self_standoff"]
    for name, run in runs.items():
        t = frames_array(run, "t")
        c = frames_array(run, "self")
        ax.plot(t, 100 * c, color=RUN_COLOR.get(name, INK2), label=run["label"])
    ax.axhline(100 * standoff, color=PLAN, lw=1, ls="--")
    ax.annotate(f"self standoff {100 * standoff:.0f} cm", xy=(0, 100 * standoff), xytext=(2, 3),
                textcoords="offset points", fontsize=8, color=INK2)
    ax.axhline(0, color=CRITICAL, lw=1)
    ax.set_ylim(bottom=min(ax.get_ylim()[0], -0.5))
    if "online" in runs:
        mark_detection(ax, first_detection_time(d, runs["online"]), label=False)
    ax.set_title("Self clearance (closest sphere pair)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("gap (cm)")
    ax.legend(loc="lower left")


def plot_estimate(ax, d, runs):
    run = runs.get("online")
    truth = np.array(d["obstacle"]["center"])
    if run is None or run["summary"]["first_detection"] < 0:
        ax.text(0.5, 0.5, "obstacle never detected", ha="center", va="center",
                transform=ax.transAxes, color=INK2)
        ax.set_title("Obstacle estimate")
        return
    ks, err, sig = [], [], []
    for s in run["steps"]:
        if s["l"] is None:
            continue
        ks.append(s["k"] * d["dt"])
        err.append(100 * np.linalg.norm(np.array(s["l"]) - truth))
        if s["l_cov"] is not None:
            cov = np.array(s["l_cov"]).reshape(3, 3)
            sig.append(100 * np.sqrt(np.max(np.linalg.eigvalsh(cov))))
        else:
            sig.append(np.nan)
    ks, err, sig = np.array(ks), np.array(err), np.array(sig)
    if np.isfinite(sig).any():
        ax.fill_between(ks, 0, 3 * sig, color=ONLINE, alpha=0.12, lw=0, step="post",
                        label="3σ (largest axis)")
    ax.step(ks, err, where="post", color=ONLINE, label="|estimate - truth|")
    ax.plot(ks, err, "o", color=ONLINE, ms=4, mec=SURFACE, mew=1)
    ax.set_xlim(0, (d["num_poses"] - 1) * d["dt"])
    ax.set_ylim(bottom=0)
    mark_detection(ax, first_detection_time(d, run), label=False)
    ax.set_title("Obstacle position estimate")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("error (cm)")
    ax.legend(loc="upper right")


def tool_path(run, d):
    tool = d["tool_sphere"]
    if tool < 0:
        return None
    return np.array([f["spheres"][tool] for f in run["frames"]])


def plot_path(ax, d, runs, axes=(0, 1), title="Tool path, top view (x-y)"):
    a, b = axes
    names = "xyz"
    obstacle = np.array(d["obstacle"]["center"])
    r = d["obstacle"]["radius"]
    tool = d["tool_sphere"]
    tool_r = d["sphere_radii"][tool] if tool >= 0 else 0.0
    # Obstacle, and the band the tool-sphere center must stay out of.
    ax.add_patch(Circle((obstacle[a], obstacle[b]), r, color=INK2, alpha=0.18, lw=0))
    ax.add_patch(Circle((obstacle[a], obstacle[b]), r + tool_r + d["obstacle_standoff"],
                        fill=False, ec=PLAN, ls="--", lw=1))
    ax.annotate("obstacle", xy=(obstacle[a], obstacle[b]), ha="center", va="center",
                fontsize=8, color=INK)

    online = runs.get("online")
    if online is not None:
        offline = np.array(online["steps"][0]["plan_tool"])
        if offline.size:
            ax.plot(offline[:, a], offline[:, b], color=PLAN, ls="--", lw=1.2,
                    label="offline plan")
        first = True
        for s in online["steps"][1:]:
            if s["detected"] and len(s["plan_tool"]) > 1:
                p = np.array(s["plan_tool"])
                ax.plot(p[:, a], p[:, b], color=ONLINE, lw=0.8, alpha=0.3,
                        label="replans after detection" if first else None)
                first = False
        est = [s["l"] for s in online["steps"] if s["l"] is not None]
        if est:
            l_final = est[-1]
            ax.plot(l_final[a], l_final[b], "+", color=ONLINE, ms=10, mew=2,
                    label="final obstacle estimate")
    for name, run in runs.items():
        p = tool_path(run, d)
        if p is None:
            continue
        ax.plot(p[:, a], p[:, b], color=RUN_COLOR.get(name, INK2), label=f'{run["label"]} executed')
        ax.plot(p[0, a], p[0, b], "o", color=INK2, ms=5)
        ax.plot(p[-1, a], p[-1, b], "s", color=INK2, ms=5)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(title)
    ax.set_xlabel(f"{names[a]} (m)")
    ax.set_ylabel(f"{names[b]} (m)")
    ax.legend(loc="best")


def plot_solver(ax, d, runs):
    run = runs.get("online")
    if run is None:
        return
    k = np.array([s["k"] for s in run["steps"]])
    ms = np.array([s["solve_ms"] for s in run["steps"]])
    two = np.array([s["solves"] > 1 for s in run["steps"]])
    ax.bar(k[~two], ms[~two], width=0.7, color=ONLINE, label="one solve")
    if two.any():
        ax.bar(k[two], ms[two], width=0.7, color=ONLINE, alpha=0.45, hatch="////",
               ec=SURFACE, lw=0, label="re-solved (estimate moved)")
    ax.set_title("Solve time per step (online)")
    ax.set_xlabel("step k (0 = offline plan)")
    ax.set_ylabel("ms")
    ax.set_xticks(k[::2])
    ax.legend(loc="upper right")
    ax.annotate(f'max {ms.max():.1f} ms', xy=(k[np.argmax(ms)], ms.max()), xytext=(0, 3),
                textcoords="offset points", ha="center", fontsize=8, color=INK2)


def dashboard(d, runs, out):
    fig, axs = plt.subplots(2, 3, figsize=(17, 9.5), constrained_layout=True)
    plot_clearance(axs[0, 0], d, runs)
    plot_self(axs[0, 1], d, runs)
    plot_estimate(axs[0, 2], d, runs)
    plot_path(axs[1, 0], d, runs, (0, 1), "Tool path, top view (x-y)")
    plot_path(axs[1, 1], d, runs, (0, 2), "Tool path, side view (x-z)")
    plot_solver(axs[1, 2], d, runs)
    fig.suptitle(summary_line(d, runs), x=0.01, ha="left", fontsize=10, color=INK2)
    fig.savefig(out, dpi=150)
    plt.close(fig)


def summary_line(d, runs):
    parts = []
    for name, run in runs.items():
        s = run["summary"]
        parts.append(f'{run["label"]}: min clearance {100 * s["min_obstacle_clearance"]:.1f} cm, '
                     f'goal error {s["goal_error"]:.3f} rad')
    return d["source"] + "\n" + "   |   ".join(parts)


def per_joint(d, runs, out, what):
    n = len(d["q_start"])
    rows = int(np.ceil(n / 3))
    fig, axs = plt.subplots(rows, 3, figsize=(15, 3.6 * rows), constrained_layout=True,
                            squeeze=False)
    dt = d["dt"]
    N = d["num_poses"] - 1
    for j in range(n):
        ax = axs[j // 3, j % 3]
        if what == "q":
            online = runs.get("online")
            if online is not None:
                plan = np.array(online["steps"][0]["plan_q"])
                ax.plot(np.arange(len(plan)) * dt, plan[:, j], color=PLAN, ls="--", lw=1.2,
                        label="offline plan")
            for name, run in runs.items():
                ax.plot(frames_array(run, "t"), [f["q"][j] for f in run["frames"]],
                        color=RUN_COLOR.get(name, INK2), label=run["label"])
            ax.set_ylabel("rad")
        else:
            lim = d["u_limit"][j]
            for name, run in runs.items():
                u = np.array(run["u"])[:, j]
                ax.step(np.arange(N + 1) * dt, np.append(u, u[-1]), where="post",
                        color=RUN_COLOR.get(name, INK2), label=run["label"])
            for sgn in (1, -1):
                ax.axhline(sgn * lim, color=PLAN, lw=1, ls="--")
            ax.annotate(f"limit ±{lim:g}", xy=(0, lim), xytext=(2, -10),
                        textcoords="offset points", fontsize=8, color=INK2)
            ax.set_ylabel("rad/s²")
        if "online" in runs:
            mark_detection(ax, first_detection_time(d, runs["online"]), label=(j == 0))
        ax.set_title(f"q{j}" if what == "q" else f"u{j}")
        ax.set_xlabel("time (s)")
        if j == 0:
            ax.legend(loc="lower left" if what == "u" else "lower right")
    for j in range(n, rows * 3):
        axs[j // 3, j % 3].axis("off")
    fig.suptitle("Joint angles" if what == "q" else "Executed controls (joint accelerations)",
                 x=0.01, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.savefig(out, dpi=150)
    plt.close(fig)


VIZ_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "viz")

# The skeleton the hosted artifact gets at publish time; the template is the body.
PAGE_HEAD = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<style>:root{color-scheme:light}body{margin:0;font:14px system-ui,sans-serif}img{max-width:100%}[hidden]{display:none!important}</style>
</head>
<body>
"""


def _inline_json(name, obj):
    # "</" inside a <script> would end it early, so escape the slash.
    text = json.dumps(obj, separators=(",", ":")).replace("</", "<\\/")
    return f"<script>window.{name} = {text};</script>\n"


def write_viewer(replay_path, d, out_html):
    template = os.path.join(VIZ_DIR, "scate_viewer.html")
    meshes = os.path.join(VIZ_DIR, "ur5_meshes.json")
    if not os.path.isfile(template):
        print(f"skipped the 3D page: {template} not found (copy the viz/ folder next to plot_scate.py)")
        return None
    with open(template) as f:
        body = f.read()
    parts = [PAGE_HEAD, _inline_json("SCATE_REPLAY", d)]
    if os.path.isfile(meshes):
        with open(meshes) as f:
            parts.append(_inline_json("SCATE_MESHES", json.load(f)))
    else:
        print(f"note: {meshes} not found; the arm will be drawn from its spheres only")
    parts += [body, "\n</body>\n</html>\n"]
    with open(out_html, "w") as f:
        f.write("".join(parts))
    return out_html


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("replay", nargs="?", default="replay.json")
    p.add_argument("--out", default=".", help="output directory")
    p.add_argument("--no-html", action="store_true", help="skip the 3D replay page")
    p.add_argument("--no-open", action="store_true", help="write the 3D page but do not open it")
    p.add_argument("--show", action="store_true", help="also open the dashboard PNG")
    a = p.parse_args()
    style()
    d, runs = load(a.replay)
    os.makedirs(a.out, exist_ok=True)
    paths = [os.path.join(a.out, f) for f in
             ("scate_dashboard.png", "scate_joints.png", "scate_controls.png")]
    dashboard(d, runs, paths[0])
    per_joint(d, runs, paths[1], "q")
    per_joint(d, runs, paths[2], "u")
    for path in paths:
        print("wrote", path)

    html = None
    if not a.no_html:
        html = write_viewer(a.replay, d, os.path.join(a.out, "scate_replay.html"))
        if html:
            print("wrote", html)
    if html and not a.no_open:
        import pathlib
        import webbrowser
        webbrowser.open(pathlib.Path(html).resolve().as_uri())
    if a.show:
        import subprocess
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        subprocess.run([opener, paths[0]], check=False)


if __name__ == "__main__":
    main()
