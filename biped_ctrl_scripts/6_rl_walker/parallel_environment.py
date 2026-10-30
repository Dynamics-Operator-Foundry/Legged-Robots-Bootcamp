"""Process-parallel vector environment for the custom walker simulator."""

from __future__ import annotations

import multiprocessing as mp
from multiprocessing.connection import Connection
from typing import Iterable

import numpy as np

try:
    from .environment import EnvConfig, WalkerVelocityEnv
    from .physics import WalkerParameters
except ImportError:
    from environment import EnvConfig, WalkerVelocityEnv
    from physics import WalkerParameters


def _worker(
    connection: Connection,
    config: EnvConfig,
    parameters: WalkerParameters | None,
    seed: int,
) -> None:
    env = WalkerVelocityEnv(config=config, parameters=parameters, seed=seed)
    try:
        while True:
            operation, payload = connection.recv()
            if operation == "reset":
                command, randomize = payload
                connection.send(env.reset(command=command, randomize=randomize))
            elif operation == "step":
                observation, reward, done, info = env.step(payload)
                if done:
                    info["terminal_observation"] = observation
                    observation = env.reset()
                connection.send((observation, reward, done, info))
            elif operation == "command_range":
                env.config.command_vx_min, env.config.command_vx_max = payload
                connection.send(True)
            elif operation == "close":
                break
            else:
                raise ValueError(f"unknown worker operation: {operation}")
    finally:
        connection.close()


class ParallelWalkerEnv:
    """Run independent custom-physics walkers concurrently in child processes.

    ``spawn`` is used deliberately so CUDA can be initialized safely in the
    trainer process. Workers auto-reset completed episodes and return the final
    observation in ``info['terminal_observation']``.
    """

    def __init__(
        self,
        num_envs: int,
        config: EnvConfig | None = None,
        seed: int = 0,
        parameters: WalkerParameters | None = None,
    ):
        if num_envs < 1:
            raise ValueError("num_envs must be positive")
        self.num_envs = num_envs
        self.config = config or EnvConfig()
        context = mp.get_context("spawn")
        self.connections: list[Connection] = []
        self.processes: list[mp.Process] = []
        for index in range(num_envs):
            parent, child = context.Pipe()
            process = context.Process(
                target=_worker,
                args=(child, self.config, parameters, seed + index),
                daemon=True,
            )
            process.start()
            child.close()
            self.connections.append(parent)
            self.processes.append(process)
        self.closed = False

    def reset(
        self,
        commands: Iterable[np.ndarray | None] | None = None,
        randomize: bool = True,
    ) -> np.ndarray:
        command_list = list(commands) if commands is not None else [None] * self.num_envs
        if len(command_list) != self.num_envs:
            raise ValueError("commands length must equal num_envs")
        for connection, command in zip(self.connections, command_list):
            connection.send(("reset", (command, randomize)))
        return np.stack([connection.recv() for connection in self.connections])

    def step(self, actions: np.ndarray):
        actions = np.asarray(actions)
        if actions.shape != (self.num_envs, WalkerVelocityEnv.action_size):
            raise ValueError(f"actions must have shape ({self.num_envs}, 8)")
        for connection, action in zip(self.connections, actions):
            connection.send(("step", action))
        results = [connection.recv() for connection in self.connections]
        observations, rewards, dones, infos = zip(*results)
        return (
            np.stack(observations),
            np.asarray(rewards, dtype=np.float32),
            np.asarray(dones, dtype=np.float32),
            list(infos),
        )

    def set_forward_command_range(self, minimum: float, maximum: float) -> None:
        for connection in self.connections:
            connection.send(("command_range", (minimum, maximum)))
        for connection in self.connections:
            connection.recv()

    def close(self) -> None:
        if self.closed:
            return
        for connection in self.connections:
            try:
                connection.send(("close", None))
            except (BrokenPipeError, EOFError):
                pass
        for process in self.processes:
            process.join(timeout=5.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
        for connection in self.connections:
            connection.close()
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        if hasattr(self, "closed"):
            self.close()


if __name__ == "__main__":
    with ParallelWalkerEnv(4, seed=0) as envs:
        observations = envs.reset(randomize=False)
        observations, rewards, dones, _ = envs.step(np.zeros((4, 8)))
        print(observations.shape, rewards, dones)
