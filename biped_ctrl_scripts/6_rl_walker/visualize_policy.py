#!/usr/bin/env python3
"""Run and render one deterministic straight-line policy rollout."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import torch

try:
    from .environment import WalkerVelocityEnv
    from .evaluate import load_policy
    from .physics import WalkerParameters
except ImportError:
    from environment import WalkerVelocityEnv
    from evaluate import load_policy
    from physics import WalkerParameters


def simulate(args: argparse.Namespace):
    device = torch.device(args.device)
    model, normalizer, config, checkpoint = load_policy(args.checkpoint, device)
    config.episode_seconds = args.seconds
    env = WalkerVelocityEnv(
        config,
        seed=args.seed,
        parameters=WalkerParameters(
            push_fraction=checkpoint["ppo_config"].get("push_fraction", 0.60)
        ),
    )
    command = np.array([args.command_vx, 0.0, 0.0])
    observation = env.reset(command, randomize=False)
    records = [
        {
            "time": 0.0,
            "state": env.state.copy(),
            "action": np.zeros(8),
            "torque": np.zeros(8),
            "stance": env.stance_leg,
            "foot_strikes": 0,
        }
    ]
    episode_return = 0.0
    done = False
    info = {"termination_reason": None}
    while not done:
        normalized = normalizer.normalize(observation[None]).astype(np.float32)
        with torch.no_grad():
            action = model.act(
                torch.as_tensor(normalized, device=device), deterministic=True
            )[0][0].cpu().numpy()
        observation, reward, done, info = env.step(action)
        episode_return += reward
        records.append(
            {
                "time": env.elapsed,
                "state": env.state.copy(),
                "action": action.copy(),
                "torque": env.last_torque.copy(),
                "stance": env.stance_leg,
                "foot_strikes": env.step_count,
            }
        )
    states = np.stack([row["state"] for row in records])
    velocity_error = states[1:, 14] - args.command_vx
    summary = {
        "checkpoint": str(args.checkpoint.resolve()),
        "checkpoint_training_steps": int(checkpoint["global_step"]),
        "checkpoint_training_episodes": int(checkpoint.get("episodes_completed", 0)),
        "command_vx_mps": args.command_vx,
        "command_vy_mps": 0.0,
        "command_yaw_rate_radps": 0.0,
        "simulated_seconds": float(env.elapsed),
        "distance_x_m": float(states[-1, 0] - states[0, 0]),
        "mean_vx_mps": float(states[1:, 14].mean()),
        "vx_rmse_mps": float(np.sqrt(np.mean(velocity_error**2))),
        "foot_strikes": int(env.step_count),
        "episode_return": float(episode_return),
        "termination_reason": info["termination_reason"],
        "passed_full_horizon": bool(info["termination_reason"] == "time_limit"),
    }
    return env, records, states, summary


def save_data(prefix: Path, records: list[dict], summary: dict) -> None:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    state_names = [
        "x", "y", "z", "roll", "pitch", "yaw",
        "left_hip_roll", "left_hip_pitch", "left_hip_yaw", "left_knee",
        "right_hip_roll", "right_hip_pitch", "right_hip_yaw", "right_knee",
    ]
    fields = ["time", "stance", "foot_strikes"]
    fields += state_names + [f"d_{name}" for name in state_names]
    fields += [f"action_{index}" for index in range(8)]
    fields += [f"torque_{index}_nm" for index in range(8)]
    with prefix.with_suffix(".csv").open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = {
                "time": record["time"],
                "stance": record["stance"],
                "foot_strikes": record["foot_strikes"],
            }
            row.update(zip(state_names, record["state"][:14]))
            row.update(zip((f"d_{name}" for name in state_names), record["state"][14:]))
            row.update({f"action_{i}": value for i, value in enumerate(record["action"])})
            row.update({f"torque_{i}_nm": value for i, value in enumerate(record["torque"])})
            writer.writerow(row)
    prefix.with_name(prefix.name + "_summary").with_suffix(".json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )


def save_tracking_plot(prefix: Path, states: np.ndarray, records: list[dict], summary: dict) -> None:
    times = np.array([record["time"] for record in records])
    figure, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    axes[0, 0].plot(times, states[:, 14], label="actual")
    axes[0, 0].axhline(summary["command_vx_mps"], color="black", linestyle="--", label="command")
    axes[0, 0].set_ylabel("forward velocity (m/s)")
    axes[0, 0].legend()
    axes[0, 1].plot(times, states[:, 0], label="x")
    axes[0, 1].plot(times, states[:, 1], label="y")
    axes[0, 1].set_ylabel("position (m)")
    axes[0, 1].legend()
    axes[1, 0].plot(times, states[:, 2])
    axes[1, 0].axhline(0.55, color="red", linestyle=":", label="fall threshold")
    axes[1, 0].set_ylabel("hip height (m)")
    axes[1, 0].legend()
    axes[1, 1].plot(times, np.rad2deg(states[:, 3]), label="roll")
    axes[1, 1].plot(times, np.rad2deg(states[:, 4]), label="pitch")
    axes[1, 1].set_ylabel("torso angle (deg)")
    axes[1, 1].legend()
    for axis in axes.flat:
        axis.set_xlabel("time (s)")
        axis.grid(alpha=0.25)
    figure.suptitle(
        f"Straight walking test: command {summary['command_vx_mps']:.2f} m/s | "
        f"termination: {summary['termination_reason']}"
    )
    figure.savefig(prefix.with_name(prefix.name + "_tracking").with_suffix(".png"), dpi=170)
    plt.close(figure)


def save_animation(prefix: Path, env: WalkerVelocityEnv, records: list[dict], summary: dict) -> Path:
    points = [env.physics.body_points(record["state"]) for record in records]
    hip_path = np.stack([point["B"] for point in points])
    x_min, x_max = float(hip_path[:, 0].min()), float(hip_path[:, 0].max())
    x_mid = 0.5 * (x_min + x_max)
    x_span = max(2.4, x_max - x_min + 1.0)

    figure, (side, front) = plt.subplots(1, 2, figsize=(11, 6), constrained_layout=True)
    side.set(xlim=(x_mid - x_span / 2, x_mid + x_span / 2), ylim=(0.0, 2.1),
             xlabel="forward x (m)", ylabel="z (m)", title="Side view")
    front.set(xlim=(-0.8, 0.8), ylim=(0.0, 2.1),
              xlabel="lateral y (m)", ylabel="z (m)", title="Front view")
    for axis in (side, front):
        axis.axhline(0.0, color="gray", linewidth=2)
        axis.grid(alpha=0.2)
        axis.set_aspect("equal", adjustable="box")

    def make_artists(axis):
        return (
            axis.plot([], [], "o-", color="#303030", linewidth=5, markersize=6)[0],
            axis.plot([], [], "-", color="#5c5c5c", linewidth=4)[0],
            axis.plot([], [], "o-", color="#2878b5", linewidth=4, markersize=5)[0],
            axis.plot([], [], "o-", color="#d95319", linewidth=4, markersize=5)[0],
            axis.plot([], [], color="#55a868", linewidth=1.5, alpha=0.8)[0],
        )

    side_artists = make_artists(side)
    front_artists = make_artists(front)
    status = figure.text(0.5, 0.98, "", ha="center", va="top")

    def set_line(line, values, horizontal_index):
        array = np.stack(values)
        line.set_data(array[:, horizontal_index], array[:, 2])

    def update(frame):
        point = points[frame]
        record = records[frame]
        segments = (
            [point["B"], point["H"]],
            [point["LH"], point["RH"]],
            [point["LH"], point["LK"], point["LA"]],
            [point["RH"], point["RK"], point["RA"]],
            hip_path[: frame + 1],
        )
        for artists, horizontal_index in ((side_artists, 0), (front_artists, 1)):
            for artist, segment in zip(artists, segments):
                set_line(artist, segment, horizontal_index)
        velocity = record["state"][14]
        terminal = summary["termination_reason"] if frame == len(records) - 1 else "running"
        status.set_text(
            f"Straight-walking policy | command {summary['command_vx_mps']:.2f} m/s | "
            f"t={record['time']:.2f} s | vx={velocity:.2f} m/s | "
            f"strikes={record['foot_strikes']} | status={terminal}"
        )
        return (*side_artists, *front_artists, status)

    frame_interval = 1000.0 * env.config.control_dt
    movie = animation.FuncAnimation(
        figure, update, frames=len(records), interval=frame_interval, blit=False
    )
    output = prefix.with_suffix(".mp4")
    try:
        movie.save(output, writer=animation.FFMpegWriter(fps=round(1.0 / env.config.control_dt), bitrate=2200))
    except (FileNotFoundError, RuntimeError):
        output = prefix.with_suffix(".gif")
        movie.save(output, writer=animation.PillowWriter(fps=round(1.0 / env.config.control_dt)))
    plt.close(figure)
    return output


def main(args: argparse.Namespace) -> None:
    env, records, states, summary = simulate(args)
    save_data(args.output_prefix, records, summary)
    save_tracking_plot(args.output_prefix, states, records, summary)
    movie = save_animation(args.output_prefix, env, records, summary)
    print(json.dumps(summary, indent=2))
    print(f"animation={movie}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--command-vx", type=float, default=0.4)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=Path(__file__).parent / "visualizations" / "straight_0p4ms",
    )
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_args())
