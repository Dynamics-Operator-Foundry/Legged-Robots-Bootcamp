"""Velocity-command environment for the custom hybrid 3-D walker."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

try:
    from .physics import Walker3DPhysics, WalkerParameters
except ImportError:  # Allow `python3 environment.py` from this directory.
    from physics import Walker3DPhysics, WalkerParameters


@dataclass
class EnvConfig:
    control_dt: float = 0.02
    # One constrained solve per control tick keeps PPO practical; use 0.002 and
    # ``rk4`` for higher-fidelity evaluation after training.
    physics_dt: float = 0.02
    integrator: str = "euler"
    episode_seconds: float = 8.0
    min_step_seconds: float = 0.10
    nominal_step_seconds: float = 0.45
    foot_clearance: float = 0.06
    torque_limit: float = 180.0
    reset_noise: float = 0.005
    command_vx_min: float = -0.2
    command_vx_max: float = 1.0
    command_vy_min: float = 0.0
    command_vy_max: float = 0.0
    command_yaw_min: float = 0.0
    command_yaw_max: float = 0.0
    reward_tracking: float = 5.0
    reward_upright: float = 0.8
    reward_height: float = 0.4
    reward_alive: float = 0.2
    reward_clearance: float = 0.1
    reward_step: float = 0.2
    penalty_effort: float = 0.04
    penalty_smoothness: float = 0.02
    termination_penalty: float = 50.0
    use_gait_bias: bool = True
    gait_stance_roll: float = 0.25
    gait_swing_pitch: float = 0.40
    gait_speed_scale: float = 0.40
    residual_scale: float = 0.50
    initial_stance_leg: str = "r"


class WalkerVelocityEnv:
    """Small Gym-like API with no Gym dependency.

    Actions are normalized motor position targets. A fixed PD actuator converts
    them to the eight generalized torques used by the original walker model.
    """

    action_size = 8
    observation_size = 43

    def __init__(
        self,
        config: EnvConfig | None = None,
        seed: int | None = None,
        parameters: WalkerParameters | None = None,
    ):
        self.config = config or EnvConfig()
        ratio = self.config.control_dt / self.config.physics_dt
        if not np.isclose(ratio, round(ratio)):
            raise ValueError("control_dt must be an integer multiple of physics_dt")
        self.substeps = int(round(ratio))
        self.physics = Walker3DPhysics(parameters)
        self.rng = np.random.default_rng(seed)

        self.action_scale = np.array([0.35, 0.70, 0.35, 1.10] * 2)
        # Hip roll needs enough authority to move the COM over a single stance
        # foot from the straight double-contact reset.
        self.kp = np.array([240.0, 220.0, 120.0, 180.0] * 2)
        self.kd = 2.0 * np.sqrt(self.kp) * 0.65
        self.velocity_scale = np.array([2.0, 2.0, 2.0, 5.0, 5.0, 5.0] + [8.0] * 8)

        self.state = np.zeros(28)
        self.command = np.zeros(3)
        self.previous_action = np.zeros(8)
        self.stance_leg = "r"
        self.stance_anchor = np.zeros(3)
        self.elapsed = 0.0
        self.step_elapsed = 0.0
        self.swing_was_clear = False
        self.step_count = 0
        self.last_torque = np.zeros(8)

    def sample_command(self) -> np.ndarray:
        c = self.config
        return np.array(
            [
                self.rng.uniform(c.command_vx_min, c.command_vx_max),
                self.rng.uniform(c.command_vy_min, c.command_vy_max),
                self.rng.uniform(c.command_yaw_min, c.command_yaw_max),
            ],
            dtype=np.float64,
        )

    def reset(self, command: np.ndarray | None = None, randomize: bool = True) -> np.ndarray:
        if self.config.initial_stance_leg not in ("l", "r"):
            raise ValueError("initial_stance_leg must be 'l' or 'r'")
        self.stance_leg = self.config.initial_stance_leg
        self.stance_anchor = np.zeros(3)
        self.state = self.physics.standing_state(self.stance_leg)
        if randomize and self.config.reset_noise > 0.0:
            noise = self.config.reset_noise
            self.state[3:14] += self.rng.normal(0.0, noise, 11)
            self.state[14:] += self.rng.normal(0.0, noise, 14)
            self.physics.project_stance(self.state, self.stance_leg, self.stance_anchor)
        self.command = self.sample_command() if command is None else np.asarray(command, dtype=np.float64)
        if self.command.shape != (3,):
            raise ValueError("command must be [forward_velocity, lateral_velocity, yaw_rate]")
        self.previous_action.fill(0.0)
        self.last_torque.fill(0.0)
        self.elapsed = 0.0
        self.step_elapsed = 0.0
        self.swing_was_clear = False
        self.step_count = 0
        return self.observation()

    @property
    def swing_leg(self) -> str:
        return "l" if self.stance_leg == "r" else "r"

    def _phase(self) -> float:
        return min(self.step_elapsed / self.config.nominal_step_seconds, 1.0)

    def observation(self) -> np.ndarray:
        q, qdot = self.state[:14], self.state[14:]
        phase_angle = 2.0 * math.pi * self._phase()
        return np.concatenate(
            (
                np.array([q[2] - 1.0]),
                np.sin(q[3:6]),
                np.cos(q[3:6]),
                q[6:14],
                qdot / self.velocity_scale,
                self.command,
                np.array([1.0 if self.stance_leg == "l" else -1.0]),
                np.array([math.sin(phase_angle), math.cos(phase_angle)]),
                self.previous_action,
            )
        ).astype(np.float32)

    def _torque(self, action: np.ndarray) -> np.ndarray:
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        target = action * self.action_scale * self.config.residual_scale
        if self.config.use_gait_bias:
            target += self._gait_target()
        joint_position = self.state[6:14]
        joint_velocity = self.state[20:28]
        torque = self.kp * (target - joint_position) - self.kd * joint_velocity
        return np.clip(torque, -self.config.torque_limit, self.config.torque_limit)

    def _gait_target(self) -> np.ndarray:
        """Command-scaled nominal target around which PPO learns a residual."""
        target = np.zeros(8, dtype=np.float64)
        stance_index = 4 if self.stance_leg == "r" else 0
        swing_index = 0 if self.stance_leg == "r" else 4
        speed_fraction = np.clip(
            self.command[0] / self.config.gait_speed_scale,
            -1.0,
            1.0,
        )
        target[stance_index] = self.config.gait_stance_roll
        sagittal_sign = -1.0 if self.stance_leg == "r" else 1.0
        target[swing_index + 1] = (
            sagittal_sign * self.config.gait_swing_pitch * speed_fraction
        )
        return target

    def _reward(self, action: np.ndarray, torque: np.ndarray, switched: bool) -> tuple[float, dict[str, float]]:
        q, qdot = self.state[:14], self.state[14:]
        velocity_error = np.array(
            [qdot[0] - self.command[0], qdot[1] - self.command[1], qdot[5] - self.command[2]]
        )
        tracking = math.exp(-3.0 * float(velocity_error @ velocity_error))
        upright = math.exp(-8.0 * float(q[3] ** 2 + q[4] ** 2))
        height = math.exp(-10.0 * float((q[2] - 1.0) ** 2))
        swing_height = max(0.0, float(self.physics.foot_position(self.state, self.swing_leg)[2]))
        clearance = math.exp(-120.0 * (swing_height - self.config.foot_clearance) ** 2)
        effort = float(np.mean((torque / self.config.torque_limit) ** 2))
        smoothness = float(np.mean((action - self.previous_action) ** 2))
        reward_terms = {
            "tracking": self.config.reward_tracking * tracking,
            "upright": self.config.reward_upright * upright,
            "height": self.config.reward_height * height,
            "alive": self.config.reward_alive,
            "clearance": self.config.reward_clearance * clearance,
            "step": self.config.reward_step if switched else 0.0,
            "effort": -self.config.penalty_effort * effort,
            "smoothness": -self.config.penalty_smoothness * smoothness,
        }
        return sum(reward_terms.values()), reward_terms

    def termination_reason(self) -> str | None:
        """Return a machine-readable terminal condition, if one is active."""
        q = self.state[:14]
        if not np.all(np.isfinite(self.state)):
            return "non_finite_state"
        if q[2] < 0.55:
            return "low_hip_height"
        if q[2] > 1.25:
            return "high_hip_height"
        if abs(q[3]) > 0.8:
            return "excessive_roll"
        if abs(q[4]) > 0.8:
            return "excessive_pitch"
        if np.any(np.abs(q[6:14]) > 2.6):
            return "joint_limit"
        return None

    def _fallen(self) -> bool:
        return self.termination_reason() is not None

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict]:
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if action.shape != (8,):
            raise ValueError("action must have shape (8,)")
        torque = self._torque(action)
        switched = False

        for _ in range(self.substeps):
            self.state = self.physics.integrate(
                self.state,
                torque,
                self.stance_leg,
                self.config.physics_dt,
                self.config.integrator,
            )
            self.physics.project_stance(self.state, self.stance_leg, self.stance_anchor)
            self.elapsed += self.config.physics_dt
            self.step_elapsed += self.config.physics_dt

            swing_height = float(self.physics.foot_position(self.state, self.swing_leg)[2])
            self.swing_was_clear = self.swing_was_clear or swing_height > 0.015
            valid_impact = (
                self.swing_was_clear
                and self.step_elapsed >= self.config.min_step_seconds
                and swing_height <= 0.0
            )
            if valid_impact:
                new_anchor = self.physics.foot_position(self.state, self.swing_leg).copy()
                new_anchor[2] = 0.0
                self.state = self.physics.impact(self.state, self.stance_leg)
                self.stance_leg = self.swing_leg
                self.stance_anchor = new_anchor
                self.physics.project_stance(self.state, self.stance_leg, self.stance_anchor)
                self.step_elapsed = 0.0
                self.swing_was_clear = False
                self.step_count += 1
                switched = True
                break

        reward, reward_terms = self._reward(action, torque, switched)
        termination_reason = self.termination_reason()
        terminated = termination_reason is not None
        truncated = self.elapsed >= self.config.episode_seconds
        if terminated:
            reward -= self.config.termination_penalty
        self.previous_action = action.copy()
        self.last_torque = torque.copy()
        info = {
            "terminated": terminated,
            "truncated": truncated,
            "termination_reason": termination_reason if terminated else ("time_limit" if truncated else None),
            "command": self.command.copy(),
            "velocity": self.state[14:17].copy(),
            "yaw_rate": float(self.state[19]),
            "steps": self.step_count,
            "reward_terms": reward_terms,
        }
        return self.observation(), float(reward), bool(terminated or truncated), info


if __name__ == "__main__":
    env = WalkerVelocityEnv(seed=0)
    observation = env.reset(command=np.zeros(3), randomize=False)
    for _ in range(10):
        observation, reward, done, info = env.step(np.zeros(8))
        if done:
            break
    print({"observation_shape": observation.shape, "reward": reward, "info": info})
