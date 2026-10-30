#!/usr/bin/env python3
"""Rank saved policies on a deterministic held-out velocity command."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

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


def rollout(path: Path, command_vx: float, seconds: float, device: torch.device) -> dict:
    model, normalizer, config, checkpoint = load_policy(path, device)
    config.episode_seconds = seconds
    env = WalkerVelocityEnv(
        config,
        seed=0,
        parameters=WalkerParameters(
            push_fraction=checkpoint["ppo_config"].get("push_fraction", 0.60)
        ),
    )
    observation = env.reset(np.array([command_vx, 0.0, 0.0]), randomize=False)
    velocities = []
    episode_return = 0.0
    done = False
    info = {}
    while not done:
        normalized = normalizer.normalize(observation[None]).astype(np.float32)
        with torch.no_grad():
            action = model.act(
                torch.as_tensor(normalized, device=device), deterministic=True
            )[0][0].cpu().numpy()
        observation, reward, done, info = env.step(action)
        episode_return += reward
        velocities.append(float(env.state[14]))
    velocity = np.asarray(velocities)
    return {
        "checkpoint": str(path),
        "training_steps": int(checkpoint["global_step"]),
        "training_episodes": int(checkpoint.get("episodes_completed", 0)),
        "survival_seconds": float(env.elapsed),
        "foot_strikes": int(env.step_count),
        "mean_vx": float(velocity.mean()),
        "vx_rmse": float(np.sqrt(np.mean((velocity - command_vx) ** 2))),
        "return": float(episode_return),
        "termination_reason": info["termination_reason"],
    }


def main(args: argparse.Namespace) -> None:
    paths = sorted(args.directory.glob("policy_episode_*.pt"))
    if args.stride > 1:
        paths = paths[:: args.stride]
    latest = args.directory / "policy.pt"
    if latest.exists() and latest not in paths:
        paths.append(latest)
    if not paths:
        raise FileNotFoundError(f"No policy checkpoints in {args.directory}")

    device = torch.device(args.device)
    rows = []
    for index, path in enumerate(paths, start=1):
        row = rollout(path, args.command_vx, args.seconds, device)
        rows.append(row)
        print(
            f"[{index:03d}/{len(paths):03d}] ep={row['training_episodes']:5d} "
            f"survival={row['survival_seconds']:.2f}s rmse={row['vx_rmse']:.3f} "
            f"steps={row['foot_strikes']}"
        )

    # Full-horizon survival dominates; among equally long runs prefer velocity
    # accuracy, then more valid hybrid foot strikes.
    rows.sort(key=lambda row: (-row["survival_seconds"], row["vx_rmse"], -row["foot_strikes"]))
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"best={rows[0]['checkpoint']} results={args.csv}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--command-vx", type=float, default=1.0)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--stride", type=int, default=1, help="Evaluate every Nth snapshot")
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--csv", type=Path, default=Path(__file__).parent / "checkpoint_scan_1ms.csv")
    return parser.parse_args()


if __name__ == "__main__":
    main(parse_args())
