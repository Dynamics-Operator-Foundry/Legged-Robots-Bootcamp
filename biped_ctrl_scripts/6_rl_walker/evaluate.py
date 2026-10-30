#!/usr/bin/env python3
"""Evaluate a saved policy on a sequence of velocity commands."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
import torch

try:
    from .environment import EnvConfig, WalkerVelocityEnv
    from .physics import WalkerParameters
    from .policy import ActorCritic, RunningMeanStd
except ImportError:
    from environment import EnvConfig, WalkerVelocityEnv
    from physics import WalkerParameters
    from policy import ActorCritic, RunningMeanStd


DEFAULT_COMMANDS = ((0.0, 0.0, 0.0), (0.4, 0.0, 0.0), (0.8, 0.0, 0.0), (-0.2, 0.0, 0.0))


def load_policy(path: Path, device: torch.device):
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    ppo = checkpoint["ppo_config"]
    env_config = EnvConfig(**checkpoint["env_config"])
    model = ActorCritic(
        WalkerVelocityEnv.observation_size,
        WalkerVelocityEnv.action_size,
        ppo["hidden_size"],
        ppo.get("initial_log_std", -0.5),
    ).to(device)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    normalizer = RunningMeanStd(WalkerVelocityEnv.observation_size)
    normalizer.load_state_dict(checkpoint["observation_normalizer"])
    return model, normalizer, env_config, checkpoint


def evaluate(args: argparse.Namespace) -> None:
    device = torch.device(args.device)
    model, normalizer, env_config, checkpoint = load_policy(args.checkpoint, device)
    env_config.episode_seconds = args.seconds
    env = WalkerVelocityEnv(
        env_config,
        seed=args.seed,
        parameters=WalkerParameters(
            push_fraction=checkpoint["ppo_config"].get("push_fraction", 0.60)
        ),
    )
    rows = []
    for command in DEFAULT_COMMANDS:
        observation = env.reset(np.asarray(command), randomize=False)
        episode_return = 0.0
        while True:
            normalized = normalizer.normalize(observation[None]).astype(np.float32)
            with torch.no_grad():
                action, _, _, _, _ = model.act(torch.as_tensor(normalized, device=device), deterministic=True)
            observation, reward, done, info = env.step(action[0].cpu().numpy())
            episode_return += reward
            rows.append(
                {
                    "command_vx": command[0],
                    "command_vy": command[1],
                    "command_yaw": command[2],
                    "time": env.elapsed,
                    "x": env.state[0],
                    "y": env.state[1],
                    "height": env.state[2],
                    "vx": env.state[14],
                    "vy": env.state[15],
                    "yaw_rate": env.state[19],
                    "return": episode_return,
                    "foot_strikes": info["steps"],
                    "done": done,
                }
            )
            if done:
                print(
                    f"command={command} time={env.elapsed:.2f}s return={episode_return:.1f} "
                    f"vx={env.state[14]:.2f} steps={env.step_count} fallen={info['terminated']}"
                )
                break

    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"checkpoint_steps={checkpoint['global_step']} csv={args.csv}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--csv", type=Path, default=Path(__file__).parent / "evaluation.csv")
    return parser.parse_args()


if __name__ == "__main__":
    evaluate(parse_args())
