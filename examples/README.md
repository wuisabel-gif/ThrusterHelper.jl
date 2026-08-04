# Examples

Each file is a tiny, self-contained script. Run any of them with:

```bash
julia --project=. examples/<name>.jl
```

### Start here
- `demo.jl`: a quick end-to-end tour (allocate a wrench, then report the diagnostics).

### Single-DOF motions
- `forward.jl`: pure surge (Fx).
- `yaw.jl`: pure yaw (τz).
- `hover.jl`: pure heave (Fz) to hold depth.
- `roll.jl`: pure roll (τx).

### Failures & under-actuation
- `failed_thruster.jl`: kill a thruster, compare control authority before/after.
- `underactuated.jl`: the 4-thruster quad can't do heave/roll/pitch.
- `rank_loss.jl`: watch `rank(B)` fall as thrusters fail one by one.

### Numerical analysis
- `condition_number.jl`: how thruster spacing changes conditioning.
- `diagnostics.jl`: full SVD-based design report.

### Solvers
- `weighted_solver.jl`: penalise a weak thruster.
- `minimum_power.jl`: minimum-power vs minimum-norm allocation.
- `qp_solver.jl`: bounded (saturation-aware) allocation.
- `solver_comparison.jl`: run every method on one command, side by side.
- `compare_methods.jl`: the built-in solver-comparison table.

### Design analysis & optimisation
- `reachability.jl`: can the vehicle even produce this wrench?
- `failure_analysis.jl`: rank each thruster by how critical it is.
- `monte_carlo.jl`: robustness to thruster misalignment / position noise.
- `optimize_layout.jl`: re-aim thrusters to improve κ / manipulability.
- `sensitivity.jl`: gradient of a design metric w.r.t. thruster angles (which one to nudge).

### Power & current budgets
- `board_budget.jl`: per-board current limits (`estimate_current`, `group_totals`, `allocate_grouped`).

### Real hardware & dynamics
- `thrust_curve.jl`: nonlinear force ↔ command mapping (a T200 curve).
- `controller.jl`: closed-loop PID driving the vehicle through the allocator.
- `mission_energy.jl`: total energy of a wrench sequence, compared per method.

### Firmware / deployment
- `export_cpp.jl`: bake the allocation into a dependency-free C++ header.

### Beyond AUVs
- `spacecraft_reaction_wheels.jl`: the same pipeline for a satellite.

### Plotting (needs `using Plots`)
- `plot_layout.jl`: render the thruster layout to a PNG.
- `plot_ellipsoid.jl`: draw the 3-D manipulability ellipsoid.
- `animate_allocation.jl`: render the rotating-command allocation GIF.
