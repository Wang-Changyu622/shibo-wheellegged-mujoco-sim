#!/usr/bin/env python3
"""Render auditable overview and step frames from saved MuJoCo trajectories."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import mujoco
import numpy as np

from wheellegged.model import load_model

ROOT = Path(__file__).resolve().parents[1]


def load_trace(path: Path) -> dict:
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key] for key in data.files}


def source_path(path: Path) -> str:
    return str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)


def frame(model, trace, index, *, step_closeup=False):
    data = mujoco.MjData(model)
    data.qpos[:], data.qvel[:], data.ctrl[:] = (
        trace[key][index] for key in ("qpos", "qvel", "ctrl"))
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth, model.vis.global_.offheight = 1000, 650
    camera, options = mujoco.MjvCamera(), mujoco.MjvOption()
    options.geomgroup[3:] = False
    camera.distance, camera.azimuth, camera.elevation = (.31, 100, -12) if step_closeup else (.34, 125, -23)
    camera.lookat[:] = [.205 if step_closeup else data.body("chassis").xpos[0], 0, .045]
    with mujoco.Renderer(model, height=650, width=1000) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=options)
        return renderer.render().copy()


def grid(panels, destination, title, subtitle, columns, font):
    rows = (len(panels) + columns - 1) // columns
    fig, axes = plt.subplots(rows, columns, figsize=(6.0 * columns, 4.4 * rows),
                             facecolor="#101722", squeeze=False)
    for ax, panel in zip(axes.flat, panels):
        ax.imshow(panel["rgb"])
        ax.set_axis_off()
        ax.set_title(panel["title"], fontproperties=font, fontsize=15, color="white", pad=8)
        ax.text(.02, .025, panel["caption"], transform=ax.transAxes, color="white",
                fontproperties=font, fontsize=10,
                bbox={"facecolor": "#111827", "alpha": .88, "edgecolor": "none", "pad": 5})
    for ax in list(axes.flat)[len(panels):]:
        ax.set_axis_off()
    fig.suptitle(title, color="white", fontproperties=font, fontsize=23, y=.985)
    fig.text(.5, .925, subtitle, ha="center", color="#bfcedc", fontproperties=font, fontsize=11)
    fig.subplots_adjust(top=.86, bottom=.02, left=.015, right=.985, hspace=.16, wspace=.035)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destination, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "review-figures")
    parser.add_argument("--baseline", type=Path, default=ROOT / "outputs" / "validated-hardware-v1")
    parser.add_argument("--leg-trace", type=Path,
                        default=ROOT / "outputs" / "leg-motion" / "final" / "full_split_jump.npz")
    args = parser.parse_args()
    args.output, args.baseline, args.leg_trace = (
        path.resolve() for path in (args.output, args.baseline, args.leg_trace))
    font_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    font = FontProperties(fname=str(font_path)) if font_path.exists() else FontProperties()
    panels, records = [], []
    choices = [("balance", "平衡保持", 7.), ("move", "前后移动", 3.),
               ("step", "6 mm 台阶", 2.6775), ("bridge", "6 mm 单边桥", 2.9),
               ("jump", "真实离地", None), ("legs", "左右独立高度", 5.5)]
    for name, title, requested_time in choices:
        source = args.leg_trace if name == "legs" else args.baseline / name / "trace.npz"
        trace = load_trace(source)
        model = load_model(name if name in ("step", "bridge") else "flat")
        index = (int(np.argmax(np.min(trace["wheel_clearance"], axis=1))) if requested_time is None
                 else int(np.argmin(abs(trace["time"] - requested_time))))
        caption = f"t={trace['time'][index]:.3f} s   v={trace['speed'][index]:+.3f} m/s"
        if name == "jump":
            caption += f"\n双轮最小净空 {1000 * min(trace['wheel_clearance'][index]):.2f} mm"
        elif name == "legs":
            caption += f"\n实际横滚 {np.degrees(trace['roll'][index]):+.1f}°"
        panels.append({"rgb": frame(model, trace, index), "title": title, "caption": caption})
        records.append({"panel": name, "trace": source_path(source),
                        "trace_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                        "sample_index": index, "time_s": float(trace["time"][index])})
    grid(panels, args.output / "simulation_overview.png", "世博轮足平衡车 · 当前仿真效果",
         "真实保存轨迹重放 | 2S 7.4 V | 轮半径 18.5 mm | 参数尚待实机标定", 3, font)

    source = args.baseline / "step" / "trace.npz"
    trace, model = load_trace(source), load_model("step")
    stages = [("接近边缘", .15), ("接触边缘", .188), ("爬上台面", .20), ("继续前进", .27)]
    panels = []
    for title, x in stages:
        index = int(np.argmin(abs(trace["axle_xyz"][:, 0] - x)))
        panels.append({"rgb": frame(model, trace, index, step_closeup=True), "title": title,
                       "caption": f"t={trace['time'][index]:.3f} s   v={trace['speed'][index]:+.3f} m/s\n"
                       f"俯仰={np.degrees(trace['pitch'][index]):+.2f}°   轮轴 x={trace['axle_xyz'][index, 0]:.3f} m"})
        records.append({"panel": "step_" + title, "trace": source_path(source),
                        "sample_index": index, "time_s": float(trace["time"][index])})
    grid(panels, args.output / "step_sequence.png", "6 mm 台阶 · 实际通过过程",
         "固定侧视相机与相同比例；四帧的时间间隔不相等。速度来自真实轮轴运动。", 2, font)
    report = {"render_only": True, "physics_states_modified_for_control": False,
              "mujoco_version": mujoco.__version__, "panels": records,
              "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (args.output / "provenance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
