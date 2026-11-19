"""Command line for deterministic evaluation and interactive simulation."""
import argparse
import json
import os
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    run = sub.add_parser("run", help="Run and measure floating-base scenarios")
    scenarios = ("balance", "move", "step", "bridge", "jump")
    run.add_argument("scenario", choices=(*scenarios, "all"))
    run.add_argument("--output", type=Path, default=Path("outputs/baseline"))
    run.add_argument("--dt", type=float, default=.0005)
    run.add_argument("--seed", type=int, default=0)
    run.add_argument("--video", action="store_true")
    run.add_argument("--profile", choices=("reference", "hardware_2s"), default="hardware_2s")
    view = sub.add_parser("view", help="Interactive MuJoCo viewer with keyboard control")
    view.add_argument("--terrain", choices=("flat", "step", "bridge"), default="flat")
    view.add_argument("--profile", choices=("reference", "hardware_2s"), default="hardware_2s")
    view.add_argument("--scenario", choices=scenarios)
    view.add_argument("--auto", action="store_true", help="Play the published scenario command schedule")
    view.add_argument("--duration", type=float, help="Close after this many wall-clock seconds")
    args = parser.parse_args()
    if args.action == "view":
        from .viewer import run_viewer
        run_viewer(args.terrain, args.profile, scenario=args.scenario,
                   automatic=args.auto, duration=args.duration)
        return
    os.environ.setdefault("MUJOCO_GL", "egl")
    from .evaluate import run_scenario
    from .render import plot_trajectory, render_video

    selected = scenarios if args.scenario == "all" else (args.scenario,)
    summary = {}
    for scenario in selected:
        destination = args.output / scenario
        result = run_scenario(scenario, output_dir=destination,
                              timestep=args.dt, seed=args.seed, profile=args.profile)
        summary[scenario] = {"passed": result["passed"], "metrics": result["metrics"],
                             "checks": result["checks"]}
        plot_trajectory(result, destination / "trajectory.png")
        if args.video:
            render_video(result, destination / "simulation.mp4")
        print(f"{scenario}: {'PASS' if result['passed'] else 'FAIL'}", flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    if not all(case["passed"] for case in summary.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
