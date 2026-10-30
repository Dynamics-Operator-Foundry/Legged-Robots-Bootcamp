"""Actor-critic network and observation normalization for PPO."""

from __future__ import annotations

import numpy as np
import torch
from torch import nn


class RunningMeanStd:
    def __init__(self, shape: int, epsilon: float = 1e-4):
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count = epsilon

    def update(self, values: np.ndarray) -> None:
        values = np.asarray(values, dtype=np.float64)
        batch_mean = values.mean(axis=0)
        batch_var = values.var(axis=0)
        batch_count = values.shape[0]
        delta = batch_mean - self.mean
        total = self.count + batch_count
        new_mean = self.mean + delta * batch_count / total
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + delta**2 * self.count * batch_count / total
        self.mean, self.var, self.count = new_mean, m2 / total, total

    def normalize(self, values: np.ndarray) -> np.ndarray:
        return np.clip((values - self.mean) / np.sqrt(self.var + 1e-8), -10.0, 10.0)

    def state_dict(self) -> dict:
        return {"mean": self.mean, "var": self.var, "count": self.count}

    def load_state_dict(self, state: dict) -> None:
        self.mean = np.asarray(state["mean"], dtype=np.float64)
        self.var = np.asarray(state["var"], dtype=np.float64)
        self.count = float(state["count"])


def _layer_init(layer: nn.Linear, std: float = np.sqrt(2.0)) -> nn.Linear:
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, 0.0)
    return layer


class ActorCritic(nn.Module):
    def __init__(
        self,
        observation_size: int = 43,
        action_size: int = 8,
        hidden_size: int = 128,
        initial_log_std: float = -1.5,
    ):
        super().__init__()
        self.actor_body = nn.Sequential(
            _layer_init(nn.Linear(observation_size, hidden_size)),
            nn.Tanh(),
            _layer_init(nn.Linear(hidden_size, hidden_size)),
            nn.Tanh(),
        )
        self.actor_mean = _layer_init(nn.Linear(hidden_size, action_size), std=0.01)
        self.log_std = nn.Parameter(torch.full((action_size,), initial_log_std))
        self.critic = nn.Sequential(
            _layer_init(nn.Linear(observation_size, hidden_size)),
            nn.Tanh(),
            _layer_init(nn.Linear(hidden_size, hidden_size)),
            nn.Tanh(),
            _layer_init(nn.Linear(hidden_size, 1), std=1.0),
        )

    def distribution(self, observation: torch.Tensor) -> torch.distributions.Normal:
        mean = self.actor_mean(self.actor_body(observation))
        return torch.distributions.Normal(mean, self.log_std.exp().expand_as(mean))

    def value(self, observation: torch.Tensor) -> torch.Tensor:
        return self.critic(observation).squeeze(-1)

    def act(self, observation: torch.Tensor, deterministic: bool = False):
        distribution = self.distribution(observation)
        raw_action = distribution.mean if deterministic else distribution.rsample()
        action = torch.tanh(raw_action)
        log_prob = distribution.log_prob(raw_action).sum(-1)
        log_prob -= torch.log(1.0 - action.square() + 1e-6).sum(-1)
        return action, raw_action, log_prob, distribution.entropy().sum(-1), self.value(observation)

    def evaluate_raw(self, observation: torch.Tensor, raw_action: torch.Tensor):
        distribution = self.distribution(observation)
        action = torch.tanh(raw_action)
        log_prob = distribution.log_prob(raw_action).sum(-1)
        log_prob -= torch.log(1.0 - action.square() + 1e-6).sum(-1)
        return log_prob, distribution.entropy().sum(-1), self.value(observation)
