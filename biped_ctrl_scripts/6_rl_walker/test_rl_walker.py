"""Focused regression tests for physics loading, reset, environment and NN."""

from __future__ import annotations

import unittest

import numpy as np
import torch

from build_physics import build
from environment import EnvConfig, WalkerVelocityEnv
from physics import Walker3DPhysics
from policy import ActorCritic
from parallel_environment import ParallelWalkerEnv


class WalkerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        build()

    def test_straight_standing_reset(self):
        physics = Walker3DPhysics()
        state = physics.standing_state("r")
        np.testing.assert_allclose(state[3:14], 0.0, atol=1e-12)
        np.testing.assert_allclose(state[14:], 0.0, atol=1e-12)
        np.testing.assert_allclose(physics.foot_position(state, "r"), 0.0, atol=1e-12)
        np.testing.assert_allclose(physics.foot_position(state, "l")[2], 0.0, atol=1e-12)
        self.assertAlmostEqual(state[2], 1.0)

    def test_one_environment_step_is_finite(self):
        env = WalkerVelocityEnv(EnvConfig(reset_noise=0.0), seed=0)
        observation = env.reset(np.array([0.4, 0.0, 0.0]), randomize=False)
        self.assertEqual(observation.shape, (WalkerVelocityEnv.observation_size,))
        observation, reward, _, _ = env.step(np.zeros(8))
        self.assertTrue(np.all(np.isfinite(observation)))
        self.assertTrue(np.isfinite(reward))

    def test_policy_shapes_and_bounds(self):
        model = ActorCritic()
        observation = torch.zeros(3, WalkerVelocityEnv.observation_size)
        action, raw_action, log_prob, entropy, value = model.act(observation)
        self.assertEqual(tuple(action.shape), (3, WalkerVelocityEnv.action_size))
        self.assertEqual(raw_action.shape, action.shape)
        self.assertEqual(tuple(log_prob.shape), (3,))
        self.assertEqual(tuple(entropy.shape), (3,))
        self.assertEqual(tuple(value.shape), (3,))
        self.assertTrue(torch.all(action.abs() <= 1.0))

    def test_parallel_environment(self):
        with ParallelWalkerEnv(2, EnvConfig(reset_noise=0.0), seed=2) as envs:
            observations = envs.reset(randomize=False)
            self.assertEqual(observations.shape, (2, WalkerVelocityEnv.observation_size))
            observations, rewards, dones, infos = envs.step(np.zeros((2, 8)))
            self.assertEqual(observations.shape, (2, WalkerVelocityEnv.observation_size))
            self.assertTrue(np.all(np.isfinite(rewards)))
            self.assertEqual(len(infos), 2)


if __name__ == "__main__":
    unittest.main()
