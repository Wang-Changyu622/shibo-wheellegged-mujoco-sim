"""Render saved physical trajectories and export compact experiment figures."""
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def render_video(result: dict, destination: Path, fps: int = 30) -> Path:
    """Replay logged states; never rerun physics or modify measured outcomes."""
    model, trace = result["model"], result["trace"]
    data = mujoco.MjData(model)
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    camera = mujoco.MjvCamera()
    camera.distance, camera.azimuth, camera.elevation = .40, 125, -22
    options = mujoco.MjvOption()
    options.geomgroup[3:] = False
    model.vis.global_.offwidth, model.vis.global_.offheight = 960, 640
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    try:
        title_font = ImageFont.truetype(font_path, 23)
        label_font = ImageFont.truetype(font_path, 17)
    except OSError:
        title_font = label_font = ImageFont.load_default()
    times = trace["time"]
    frame_times = np.arange(times[0], times[-1], 1 / fps)
    indices = np.minimum(np.searchsorted(times, frame_times), len(times) - 1)
    scenario = result["metadata"]["scenario"]
    descriptions = {"balance": "Balance recovery / free base",
                    "move": "Forward and reverse / velocity tracking",
                    "step": "6 mm step / 18.5 mm wheel radius",
                    "bridge": "One-wheel bridge / 6 mm high, 25 mm wide",
                    "jump": ("Torque-limited jump / "
                             f"{model.actuator('L_servo').forcerange[1]:.3f} Nm hip limit")}
    preview_time = {"balance": .6, "move": 3., "step": 3.3, "bridge": 2.9}.get(scenario)
    if scenario == "jump":
        preview_time = float(times[np.argmax(np.min(trace["wheel_clearance"], axis=1))])
    preview_index = int(np.argmin(abs(frame_times - (preview_time or times[-1] / 2))))
    with mujoco.Renderer(model, height=640, width=960) as renderer, imageio.get_writer(
        destination, fps=fps, codec="libx264", quality=8, macro_block_size=16
    ) as writer:
        for frame_number, index in enumerate(indices):
            data.qpos[:], data.qvel[:] = trace["qpos"][index], trace["qvel"][index]
            data.ctrl[:] = trace["ctrl"][index]
            mujoco.mj_forward(model, data)
            camera.lookat[:] = [data.body("chassis").xpos[0], 0, .045]
            renderer.update_scene(data, camera=camera, scene_option=options)
            frame = Image.fromarray(renderer.render().copy())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 960, 76), fill=(18, 24, 33))
            draw.text((22, 14), descriptions.get(scenario, scenario), font=title_font, fill="white")
            profile = result["metadata"].get("hardware_profile", {}).get("name", "reference")
            draw.text((22, 46), f"CAD four-bar | floating base | {profile} | voltage LQR + position servos",
                      font=label_font, fill=(159, 188, 207))
            draw.rectangle((0, 578, 960, 640), fill=(18, 24, 33))
            draw.text((22, 588), f"t = {times[index]:5.2f} s    x = {data.body('chassis').xpos[0]:+.3f} m"
                      f"    pitch = {np.degrees(trace['pitch'][index]):+.2f} deg"
                      f"    phase = {trace['phase'][index]}",
                      font=label_font, fill="white")
            status = "PASS" if result["passed"] else "FAIL"
            draw.text((790, 590), f"{scenario}: {status}", font=label_font,
                      fill=(99, 224, 160) if result["passed"] else (255, 140, 120))
            draw.text((22, 613), "Simulation parameters are not hardware calibration. Video plays at real simulation time.",
                      font=label_font, fill=(159, 188, 207))
            writer.append_data(np.asarray(frame))
            if frame_number == preview_index:
                frame.save(destination.with_suffix(".png"))
    return destination


def plot_trajectory(result: dict, destination: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    trace = result["trace"]
    t = trace["time"]
    # Derive a small stable plotting interface from the full logged state.
    qpos, qvel = trace["qpos"], trace["qvel"]
    model = result["model"]
    base = int(model.joint("floating_base").qposadr[0])
    quaternion = qpos[:, base + 3:base + 7]
    w, x, y, z = quaternion.T
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    with plt.rc_context({"font.size": 10, "axes.grid": True, "grid.alpha": .2}):
        fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True, constrained_layout=True)
        axes[0].plot(t, qpos[:, base], label="body x")
        axes[0].set_ylabel("Position [m]")
        axes[1].plot(t, np.degrees(pitch), label="pitch")
        axes[1].plot(t, np.degrees(roll), label="roll")
        axes[1].set_ylabel("Angle [deg]")
        for side in ("L", "R"):
            aid = model.actuator(side + "_wheel_voltage").id
            axes[2].plot(t, trace["ctrl"][:, aid], label=side)
        axes[2].set_ylabel("Wheel command [V]")
        for side in ("L", "R"):
            jid = int(model.joint(side + "_hip").qposadr[0])
            axes[3].plot(t, qpos[:, jid], label=side)
        axes[3].set_ylabel("Hip angle [rad]")
        axes[3].set_xlabel("Simulation time [s]")
        for axis in axes:
            axis.legend(loc="upper right", ncol=2)
        fig.suptitle(result["metadata"]["scenario"] + " — actual floating-base simulation")
        destination = Path(destination)
        fig.savefig(destination, dpi=160)
        plt.close(fig)
    return destination
