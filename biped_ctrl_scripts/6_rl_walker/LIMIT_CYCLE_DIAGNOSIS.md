# Why `walker_3D_control.py` does not have a reliable walking limit cycle

## Conclusion

The fixed-point solver is not the primary problem. The return map passed to it
is not a reliable, smooth Poincaré map. It combines a broken generated-module
import contract, a wrong Dormand–Prince coefficient, sample-level collision
detection, asymmetric reference construction, unbounded high-gain inverse
dynamics, and a non-transverse initial section. A root returned by this setup
is therefore not evidence of a stable physical gait.

I audited the progression under `1_hopper_dynamics` through
`5_walker_3D_control`, including the common integration/visualization library,
the 2-D passive-walker return map, the controlled 2-D walker, the 3-D symbolic
derivations, generated functions, C wrappers, tests, controller, and fixed-point
script. The early examples establish useful patterns, but the 3-D script scales
them to a much stiffer and more delicate hybrid system without adding the event
localization, conditioning checks, or section definition that scale-up needs.

## Defects that must be fixed before searching for a fixed point

1. **The generated extensions are imported under the wrong names.**
   `walker_3D_control.py` imports all nine kernels from `wrapper_module_0`.
   Only `B_matrix` is module 0. The checked-in sources declare collision as
   module 1, the Jacobians as 2–5, mass as 6, and push vectors as 7–8. A clean
   build cannot load the controller as written. The RL adapter loads their real
   generated names and includes a direct C build script.

2. **The shared Dormand–Prince implementation has a wrong tableau entry.**
   `dynamics_bootcamp.py:96` uses `a41 = 44 / 55`; Dormand–Prince 5(4) requires
   `44 / 45`. This changes every fourth stage and invalidates the method's order
   and stability properties. The method also returns no embedded error estimate,
   takes no adaptive step, and does not locate events. The 3-D README's warning
   about the naive integration is therefore justified even though the function
   is named `rkdp`.

3. **A sample sign flip is not a robust foot-strike event.**
   The guard is `z_left_ankle - z_right_ankle`, and the loop only checks whether
   its sampled values changed sign. It does not require the swing foot to be
   descending, to have first cleared the ground, or to be in front of the stance
   foot. It accepts the wrong crossing, misses grazing contacts, applies impact
   after penetration, and makes the return map depend strongly on `t_step`.
   There is no root interpolation to the impact time.

4. **The proposed section starts at a degenerate double-contact pose.**
   `x_obj = zeros(22)` creates straight legs with both ankle heights equal. The
   collision guard is exactly zero there, so this is not a transverse post-impact
   Poincaré section. At a valid section, the new stance foot must be constrained,
   the old foot must be separating, and the guard derivative must have the
   correct sign.

5. **Left-support trajectory initialization uses the wrong leg.**
   During left support, the decoupling matrix and controlled coordinates select
   the right (swing) hip. `get_traj_coeff`, however, initializes those reference
   entries from `roll_lh`, `pitch_lh`, `yaw_lh` and their left-hip velocities.
   The desired trajectory jumps at the stance transition. The knee entries are
   also reordered and one is offset by `pitch_lk - 0.4`, so the two half-steps
   are not a clean reflection of one another.

6. **The inverse-dynamics controller is numerically fragile.**
   It explicitly forms `inv(AA)`, then solves an 8x8 decoupling system, with
   `Kp=1000`, no condition-number check, no regularization, no joint limits, and
   no torque limits. Near singular poses this creates huge torque discontinuities.
   Those discontinuities are integrated with a fixed 1 ms step and then fed to
   an instantaneous impact map, which makes finite-difference return-map
   derivatives noisy or meaningless.

7. **The powered impact changes the energy discontinuously.**
   The impact RHS adds `J_stance.T @ P` with
   `F = 0.60 m sqrt(g l)`. This is an active push-off impulse, not merely a
   passive collision. It can be a valid powered-walker design choice, but its
   timing and direction must be part of the controller and fixed-point unknowns;
   hard-coding it while trial-and-error trajectories also inject energy makes
   the gait very sensitive.

8. **The fixed-point residual changes dimension on failure.**
   The normal residual in `f_lala` has 22 entries. When `P` returns NaNs, it
   returns `full_like(result, inf)`, where `result` is the 28-state trajectory.
   A root solver requires a fixed 22-to-22 map. The one-step simulation also has
   no maximum phase time, so a missed collision can hang an optimization call.

9. **The fixed-point search lacks diagnostics and stability validation.**
   It starts from zeros, calls an unconstrained hybrid root solve, and prints a
   norm. It does not constrain section admissibility, cache impact time, scale
   angle and velocity residuals, report singular values/conditioning, verify the
   solution at smaller time steps, or compute Floquet multipliers. Even an exact
   root can be an unstable limit cycle and fall after a small perturbation.

## Recommended deterministic repair sequence

1. Correct `44/55` to `44/45` and add embedded-error adaptive stepping.
2. Build/load the generated kernels by modules 0–8 and regression-test every C
   kernel against its generated Python expression at random nonsingular states.
3. Define a post-impact section with one stance foot fixed, the old foot
   separating, positive hip height, and explicit left/right relabeling.
4. Locate touchdown by solving the guard inside the last integration interval;
   require prior clearance, downward guard velocity, and forward placement.
5. Make left and right references exact mirror images and remove reference jumps.
6. Replace explicit inverses with solves, monitor condition numbers, regularize
   the decoupling solve, and impose torque/joint/rate limits.
7. Treat push-off magnitude and step duration as design variables, or set the
   active impulse to zero while validating the passive impact map.
8. Use bounded least squares/multiple shooting rather than an unconstrained root
   from straight double support. Continue from an already simulated gait.
9. Re-evaluate at half and quarter step size, then finite-difference the corrected
   return map and require all nontrivial Floquet multipliers to lie inside the
   unit circle.

The PPO implementation next to this document avoids depending on the broken
top-level controller. It reuses the same generated equations and impact map,
adds contact-drift projection and guarded impacts, starts from the exact straight
nominal pose, and learns bounded PD motor targets conditioned on velocity.
