#!/usr/bin/env python3
"""Train a command-conditioned walking policy with clipped PPO."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import random
import time

import numpy as np
import torch
from torch import nn

try:
    from .environment import EnvConfig, WalkerVelocityEnv
    from .parallel_environment import ParallelWalkerEnv
    from .physics import WalkerParameters
    from .policy import ActorCritic, RunningMeanStd
except ImportError:
    from environment import EnvConfig, WalkerVelocityEnv
    from parallel_environment import ParallelWalkerEnv
    from physics import WalkerParameters
    from policy import ActorCritic, RunningMeanStd


HERE = Path(__file__).resolve().parent


def save_training_history(output: Path, history: list[dict]) -> None:
    """Persist machine-readable metrics and a compact training-curve figure."""
    csv_path = output / "training_metrics.csv"
    with csv_path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=history[0].keys())
        writer.writeheader()
        writer.writerows(history)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    steps = [row["global_step"] for row in history]
    figure, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    axes[0, 0].plot(steps, [row["mean_episode_return"] for row in history])
    axes[0, 0].set_title("Episode return (last 20)")
    axes[0, 1].plot(steps, [row["velocity_tracking_rmse"] for row in history])
    axes[0, 1].set_title("Velocity tracking RMSE")
    axes[1, 0].plot(steps, [row["mean_episode_length"] for row in history])
    axes[1, 0].set_title("Episode length (control steps)")
    axes[1, 1].plot(steps, [row["policy_loss"] for row in history], label="policy")
    axes[1, 1].plot(steps, [row["value_loss"] for row in history], label="value")
    axes[1, 1].set_title("PPO losses")
    axes[1, 1].legend()
    for axis in axes.flat:
        axis.set_xlabel("environment transitions")
        axis.grid(alpha=0.25)
    figure.savefig(output / "training_curves.png", dpi=160)
    plt.close(figure)


@dataclass
class PPOConfig:
    total_steps: int = 1_000_000_000_000
    num_envs: int = 8
    rollout_steps: int = 256
    update_epochs: int = 10
    minibatch_size: int = 512
    hidden_size: int = 128
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_ratio: float = 0.2
    value_coefficient: float = 0.5
    entropy_coefficient: float = 0.0
    max_grad_norm: float = 0.5
    target_kl: float = 0.02
    initial_log_std: float = -1.5
    command_curriculum_steps: int = 100_000
    checkpoint_every_episodes: int = 10
    push_fraction: float = 0.0
    seed: int = 7


def resolve_device(requested: str) -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(requested)


def save_checkpoint(
    path: Path,
    model: ActorCritic,
    optimizer: torch.optim.Optimizer,
    normalizer: RunningMeanStd,
    ppo_config: PPOConfig,
    env_config: EnvConfig,
    global_step: int,
    episodes_completed: int,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "observation_normalizer": normalizer.state_dict(),
            "ppo_config": asdict(ppo_config),
            "env_config": asdict(env_config),
            "global_step": global_step,
            "episodes_completed": episodes_completed,
        },
        path,
    )


def train(args: argparse.Namespace) -> Path:
    config = PPOConfig(
        total_steps=args.total_steps,
        num_envs=args.num_envs,
        rollout_steps=args.rollout_steps,
        update_epochs=args.update_epochs,
        minibatch_size=args.minibatch_size,
        hidden_size=args.hidden_size,
        learning_rate=args.learning_rate,
        checkpoint_every_episodes=args.checkpoint_every_episodes,
        push_fraction=args.push_fraction,
        seed=args.seed,
    )
    env_config = EnvConfig(integrator=args.integrator)
    device = resolve_device(args.device)
    random.seed(config.seed)
    np.random.seed(config.seed)
    torch.manual_seed(config.seed)

    parallel_envs = ParallelWalkerEnv(
        config.num_envs,
        env_config,
        seed=config.seed,
        parameters=WalkerParameters(push_fraction=config.push_fraction),
    )
    observations = parallel_envs.reset()
    normalizer = RunningMeanStd(WalkerVelocityEnv.observation_size)
    normalizer.update(observations)
    model = ActorCritic(
        WalkerVelocityEnv.observation_size,
        WalkerVelocityEnv.action_size,
        config.hidden_size,
        config.initial_log_std,
    ).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=config.learning_rate, eps=1e-5)
    global_step = 0
    episodes_completed = 0

    if args.resume:
        checkpoint = torch.load(args.resume, map_location=device, weights_only=False)
        model.load_state_dict(checkpoint["model"])
        optimizer.load_state_dict(checkpoint["optimizer"])
        normalizer.load_state_dict(checkpoint["observation_normalizer"])
        global_step = int(checkpoint["global_step"])
        episodes_completed = int(checkpoint.get("episodes_completed", 0))

    batch_size = config.num_envs * config.rollout_steps
    minibatch_size = min(config.minibatch_size, batch_size)
    updates = max(1, int(np.ceil((config.total_steps - global_step) / batch_size)))
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "config.json").write_text(
        json.dumps({"ppo": asdict(config), "environment": asdict(env_config)}, indent=2) + "\n"
    )
    episode_returns = np.zeros(config.num_envs)
    episode_lengths = np.zeros(config.num_envs, dtype=int)
    completed_returns: list[float] = []
    completed_lengths: list[int] = []
    history: list[dict] = []
    next_checkpoint_episode = (
        episodes_completed // config.checkpoint_every_episodes + 1
    ) * config.checkpoint_every_episodes
    started = time.time()

    for update in range(1, updates + 1):
        curriculum = min(global_step / config.command_curriculum_steps, 1.0)
        command_minimum = env_config.command_vx_min * curriculum
        command_maximum = 0.1 + (env_config.command_vx_max - 0.1) * curriculum
        parallel_envs.set_forward_command_range(command_minimum, command_maximum)
        obs_buffer = np.zeros((config.rollout_steps, config.num_envs, WalkerVelocityEnv.observation_size), np.float32)
        raw_action_buffer = np.zeros((config.rollout_steps, config.num_envs, WalkerVelocityEnv.action_size), np.float32)
        log_prob_buffer = np.zeros((config.rollout_steps, config.num_envs), np.float32)
        reward_buffer = np.zeros((config.rollout_steps, config.num_envs), np.float32)
        done_buffer = np.zeros((config.rollout_steps, config.num_envs), np.float32)
        value_buffer = np.zeros((config.rollout_steps, config.num_envs), np.float32)
        rollout_tracking_squared: list[float] = []

        for rollout_step in range(config.rollout_steps):
            normalizer.update(observations)
            normalized = normalizer.normalize(observations).astype(np.float32)
            obs_tensor = torch.as_tensor(normalized, device=device)
            with torch.no_grad():
                actions, raw_actions, log_probs, _, values = model.act(obs_tensor)
            actions_np = actions.cpu().numpy()
            next_observations, rewards, dones, infos = parallel_envs.step(actions_np)
            for index, (reward, done, info) in enumerate(zip(rewards, dones, infos)):
                measured_velocity = np.array(
                    [info["velocity"][0], info["velocity"][1], info["yaw_rate"]]
                )
                rollout_tracking_squared.append(
                    float(np.mean((measured_velocity - info["command"]) ** 2))
                )
                episode_returns[index] += reward
                episode_lengths[index] += 1
                if done:
                    completed_returns.append(float(episode_returns[index]))
                    completed_lengths.append(int(episode_lengths[index]))
                    episodes_completed += 1
                    if episodes_completed == next_checkpoint_episode:
                        save_checkpoint(
                            output / f"policy_episode_{episodes_completed:06d}.pt",
                            model,
                            optimizer,
                            normalizer,
                            config,
                            env_config,
                            global_step + config.num_envs,
                            episodes_completed,
                        )
                        next_checkpoint_episode += config.checkpoint_every_episodes
                    episode_returns[index] = 0.0
                    episode_lengths[index] = 0

            obs_buffer[rollout_step] = normalized
            raw_action_buffer[rollout_step] = raw_actions.cpu().numpy()
            log_prob_buffer[rollout_step] = log_probs.cpu().numpy()
            reward_buffer[rollout_step] = rewards
            done_buffer[rollout_step] = dones
            value_buffer[rollout_step] = values.cpu().numpy()
            observations = next_observations
            global_step += config.num_envs

        with torch.no_grad():
            next_normalized = normalizer.normalize(observations).astype(np.float32)
            next_value = model.value(torch.as_tensor(next_normalized, device=device)).cpu().numpy()
        advantages = np.zeros_like(reward_buffer)
        last_gae = np.zeros(config.num_envs)
        for step in reversed(range(config.rollout_steps)):
            nonterminal = 1.0 - done_buffer[step]
            following_value = next_value if step == config.rollout_steps - 1 else value_buffer[step + 1]
            delta = reward_buffer[step] + config.gamma * following_value * nonterminal - value_buffer[step]
            last_gae = delta + config.gamma * config.gae_lambda * nonterminal * last_gae
            advantages[step] = last_gae
        returns = advantages + value_buffer

        flat_obs = torch.as_tensor(obs_buffer.reshape(batch_size, -1), device=device)
        flat_raw_actions = torch.as_tensor(raw_action_buffer.reshape(batch_size, -1), device=device)
        flat_old_log_probs = torch.as_tensor(log_prob_buffer.reshape(-1), device=device)
        flat_advantages = torch.as_tensor(advantages.reshape(-1), device=device)
        flat_returns = torch.as_tensor(returns.reshape(-1), device=device)
        flat_old_values = torch.as_tensor(value_buffer.reshape(-1), device=device)
        flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std() + 1e-8)

        indices = np.arange(batch_size)
        approximate_kl = torch.tensor(0.0)
        policy_losses: list[float] = []
        value_losses: list[float] = []
        entropies: list[float] = []
        for _ in range(config.update_epochs):
            np.random.shuffle(indices)
            for start in range(0, batch_size, minibatch_size):
                mb = indices[start : start + minibatch_size]
                new_log_prob, entropy, new_value = model.evaluate_raw(flat_obs[mb], flat_raw_actions[mb])
                log_ratio = new_log_prob - flat_old_log_probs[mb]
                ratio = log_ratio.exp()
                with torch.no_grad():
                    approximate_kl = ((ratio - 1.0) - log_ratio).mean()
                unclipped = -flat_advantages[mb] * ratio
                clipped = -flat_advantages[mb] * torch.clamp(
                    ratio, 1.0 - config.clip_ratio, 1.0 + config.clip_ratio
                )
                policy_loss = torch.maximum(unclipped, clipped).mean()
                value_clipped = flat_old_values[mb] + torch.clamp(
                    new_value - flat_old_values[mb], -config.clip_ratio, config.clip_ratio
                )
                value_loss = 0.5 * torch.maximum(
                    (new_value - flat_returns[mb]).square(),
                    (value_clipped - flat_returns[mb]).square(),
                ).mean()
                loss = (
                    policy_loss
                    + config.value_coefficient * value_loss
                    - config.entropy_coefficient * entropy.mean()
                )
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), config.max_grad_norm)
                optimizer.step()
                policy_losses.append(float(policy_loss.detach().cpu()))
                value_losses.append(float(value_loss.detach().cpu()))
                entropies.append(float(entropy.mean().detach().cpu()))
            if approximate_kl.item() > config.target_kl:
                break

        checkpoint_path = output / "policy.pt"
        save_checkpoint(
            checkpoint_path,
            model,
            optimizer,
            normalizer,
            config,
            env_config,
            global_step,
            episodes_completed,
        )
        recent = completed_returns[-20:]
        recent_lengths = completed_lengths[-20:]
        mean_return = float(np.mean(recent)) if recent else float("nan")
        mean_length = float(np.mean(recent_lengths)) if recent_lengths else float("nan")
        tracking_rmse = float(np.sqrt(np.mean(rollout_tracking_squared)))
        history.append(
            {
                "update": update,
                "global_step": global_step,
                "mean_episode_return": mean_return,
                "mean_episode_length": mean_length,
                "velocity_tracking_rmse": tracking_rmse,
                "command_vx_min": command_minimum,
                "command_vx_max": command_maximum,
                "policy_loss": float(np.mean(policy_losses)),
                "value_loss": float(np.mean(value_losses)),
                "entropy": float(np.mean(entropies)),
                "approximate_kl": float(approximate_kl.item()),
            }
        )
        save_training_history(output, history)
        steps_per_second = global_step / max(time.time() - started, 1e-6)
        print(
            f"update={update}/{updates} steps={global_step} return20={mean_return:.2f} "
            f"velocity_rmse={tracking_rmse:.3f} kl={approximate_kl.item():.4f} "
            f"steps/s={steps_per_second:.1f}",
            flush=True,
        )
        if global_step >= config.total_steps:
            break

    parallel_envs.close()
    return output / "policy.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--total-steps", type=int, default=PPOConfig.total_steps)
    parser.add_argument("--num-envs", type=int, default=PPOConfig.num_envs)
    parser.add_argument("--rollout-steps", type=int, default=PPOConfig.rollout_steps)
    parser.add_argument("--update-epochs", type=int, default=PPOConfig.update_epochs)
    parser.add_argument("--minibatch-size", type=int, default=PPOConfig.minibatch_size)
    parser.add_argument("--hidden-size", type=int, default=PPOConfig.hidden_size)
    parser.add_argument("--learning-rate", type=float, default=PPOConfig.learning_rate)
    parser.add_argument(
        "--checkpoint-every-episodes",
        type=int,
        default=PPOConfig.checkpoint_every_episodes,
    )
    parser.add_argument("--seed", type=int, default=PPOConfig.seed)
    parser.add_argument("--push-fraction", type=float, default=PPOConfig.push_fraction)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    parser.add_argument("--integrator", choices=("euler", "rk4"), default="euler")
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--output", type=Path, default=HERE / "checkpoints")
    return parser.parse_args()


if __name__ == "__main__":
    train(parse_args())
