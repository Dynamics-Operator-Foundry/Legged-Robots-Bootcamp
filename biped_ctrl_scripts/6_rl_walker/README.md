# PPO 3-D walker

This directory contains a self-contained PPO training pipeline around the
physics derived in `../5_walker_3D_control/f_walker_3D/dynamics`. It does not use
MuJoCo, Bullet, Gym, or another rigid-body engine.

The policy observes body orientation, all joint positions and velocities, base
velocity, the commanded forward/lateral/yaw velocity, stance side, gait phase,
and its previous action. It outputs eight bounded joint-position targets. A
fixed PD actuator maps these targets to the eight torques in the original
14-DoF equations. The actuator uses residual RL: a small command-scaled nominal
stance/swing target makes foot strikes reachable at initialization, and PPO
supplies the bounded residual that stabilizes the gait and regulates speed.
Every episode's nominal initial state is torso and legs
exactly straight with zero velocity; training adds small perturbations around
that nominal state.

## Files

- `physics.py`: adapter for the original mass, bias, contact Jacobian, collision,
  and impulse kernels.
- `environment.py`: straight-standing reset, command sampling, rewards,
  termination, contact guard, and stance switching.
- `parallel_environment.py`: spawn-safe process vectorization of independent
  custom-physics walkers; workers feed batches to the CUDA policy learner.
- `policy.py`: two-layer actor/critic and running observation normalization.
- `train_ppo.py`: dependency-light clipped PPO with GAE and checkpoint resume.
- `evaluate.py`: deterministic evaluations at four forward commands and CSV
  output.
- `select_checkpoint.py`: ranks saved snapshots by held-out survival and
  velocity-tracking error.
- `visualize_policy.py`: saves a straight-line rollout as MP4, CSV, JSON, and a
  tracking plot, including the exact termination signal.
- `build_physics.py`: directly compiles the checked-in generated C sources; it
  does not regenerate or replace the user's physics.
- `LIMIT_CYCLE_DIAGNOSIS.md`: detailed diagnosis of the original controller.
- `checkpoints/config.json`: all PPO, network, and environment parameters used
  for the included checkpoint.
- `checkpoints/policy.pt`: included actor, critic, optimizer, observation
  normalization, configuration, and training-step count.
- `checkpoints/training_metrics.csv`: per-update return, episode length,
  velocity-tracking RMSE, losses, entropy, and KL divergence.
- `checkpoints/training_curves.png`: saved return, velocity error, survival, and
  PPO loss curves.

## Run

From the repository root:

```bash
python3 biped_ctrl_scripts/6_rl_walker/build_physics.py
python3 biped_ctrl_scripts/6_rl_walker/test_rl_walker.py
python3 biped_ctrl_scripts/6_rl_walker/train_ppo.py --total-steps 1000000
python3 biped_ctrl_scripts/6_rl_walker/evaluate.py \
  biped_ctrl_scripts/6_rl_walker/checkpoints/policy.pt
python3 biped_ctrl_scripts/6_rl_walker/visualize_policy.py \
  biped_ctrl_scripts/6_rl_walker/checkpoints/policy.pt --command-vx 0.4
```

Training writes `policy.pt` after every rollout, so an interrupted run is
recoverable. It also writes `policy_episode_XXXXXX.pt` every 10 completed
episodes:

```bash
python3 biped_ctrl_scripts/6_rl_walker/train_ppo.py \
  --resume biped_ctrl_scripts/6_rl_walker/checkpoints/policy.pt \
  --total-steps 1000000
```

The default Euler step is one 20 ms constrained solve per control action so PPO
is practical on CPU. For a post-training numerical sensitivity check, construct
`EnvConfig(physics_dt=0.002, integrator="rk4")`; this is much slower. A policy
should only be called successful after it survives the full evaluation horizon,
tracks held-out commands, and gives similar results at the finer integration
setting.

## Reward and conventions

The observation reserves the command vector `[vx, vy, yaw_rate]`; the supplied
training configuration varies forward velocity from -0.2 to 1.0 m/s and holds
the other two entries at zero. A 100k-transition curriculum expands that range
from near-zero commands to the full interval. Velocity tracking is the primary objective
(weight 5.0); uprightness, nominal hip height, swing-foot clearance, and steps
are smaller shaping terms, with torque and action-rate regularization. A fall
costs 50 reward units so a short velocity transient cannot beat sustained
tracking. Falls are detected from height, torso
tilt, joint excursion, or non-finite state. The original controller's
action-independent powered impact impulse is disabled for training
(`push_fraction=0.0`) because its default 0.60 value produces a roughly 6 m/s
jump at contact, incompatible with the requested 0–1 m/s tracking range. The
mass matrix, constraints, guard, and inelastic impact solve remain the
repository's custom physics. The original impulse remains available through
`--push-fraction 0.60` for ablation.

The generated `.so` files are build artifacts ignored by git. The portable
sources and builder are tracked; rerun `build_physics.py` after changing Python,
NumPy, compiler, or platform.
