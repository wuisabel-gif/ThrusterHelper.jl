# ThrusterHelper.jl

A small, dependency-light Julia toolkit for **thruster / actuator allocation and
control-authority analysis**. A 6-DOF controller asks for a wrench
`τ = [Fx, Fy, Fz, τx, τy, τz]`, but a vehicle only has individual actuators
`f = [f₁, …, fₙ]`. ThrusterHelper builds the allocation matrix `B` (so `τ = B f`),
solves for `f` with a choice of algorithms, and analyses the design itself —
rank, conditioning, redundancy, control authority, failure modes and power.

The core has **no third-party dependencies** (only `LinearAlgebra`, `Printf`,
`Random`). Graphical plotting is an optional `Plots` package extension.

## Install

```julia
using Pkg
Pkg.add("ThrusterHelper")
```

## Quick start

```julia
using ThrusterHelper

vehicle = bluerov_vehicle()                    # 8-thruster BlueROV2 Heavy layout
τ = [1.0, 0.0, 0.0, 0.0, 0.0, 0.5]             # forward + yaw right
result = allocate(vehicle, τ; method = :qp)    # saturation-aware solve
report(result; actuators = vehicle.actuators)

report(diagnostics(vehicle))                   # SVD-based design analysis
```

## What's here

- **Solvers** — `allocate(...; method = :minimum_norm | :weighted | :minimum_power | :qp)`.
- **Diagnostics** — `diagnostics`, `controllable_dofs`: rank, conditioning, manipulability, control authority.
- **Analysis** — `reachable`, `compare_methods`, `rank_failures`, `monte_carlo`.
- **Design** — `optimize_layout`: re-aim/move thrusters to improve the design.
- **Shared budgets** — `estimate_current`, `group_totals`, `allocate_grouped` for power-board current limits.
- **Deploy** — `export_cpp`: a dependency-free, real-time-safe C++17 header.

See the [API reference](@ref) for every exported function, and the
[README](https://github.com/wuisabel-gif/ThrusterHelper.jl) and
[`examples/`](https://github.com/wuisabel-gif/ThrusterHelper.jl/tree/main/examples)
directory for worked scripts.
