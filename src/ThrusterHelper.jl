"""
    ThrusterHelper

An experimental platform for **thruster / actuator allocation and control-
authority analysis**.

A 6-DOF controller asks for a wrench `τ = [Fx, Fy, Fz, τx, τy, τz]`, but a
vehicle only has individual actuators `f = [f₁, …, fₙ]`. ThrusterHelper builds the
allocation matrix `B` (so `τ = B f`), solves for `f` with a choice of algorithms
(`:minimum_norm`, `:weighted`, `:minimum_power`, `:qp`), and analyses the design
itself — rank, conditioning, redundancy, control authority, failure modes and
power.

The math is not underwater-specific: actuators are an `AbstractActuator`
hierarchy ([`Thruster`](@ref), [`ReactionWheel`](@ref), …), so AUVs, spacecraft
and omni-robots all flow through the same pipeline.

```julia
using ThrusterHelper
vehicle = bluerov_vehicle()
τ = [1.0, 0, 0, 0, 0, 0.5]                 # forward + yaw
result = allocate(vehicle, τ; method=:qp)  # respects thruster limits
report(result; actuators=vehicle.actuators)
report(diagnostics(vehicle))               # SVD-based design analysis
```

Graphical plots are available after `using Plots` (`plot_thrusters`,
`plot_vehicle`).

### Pipeline / source layout
    types.jl  geometry.jl              — actuators, Vehicle, geometry
    allocation_matrix.jl  solver.jl  constraints.jl — build B, solve, limits
    diagnostics.jl  visualization.jl   — analysis, reporting
    layouts/                           — ready-made vehicles
"""
module ThrusterHelper

using LinearAlgebra
using Printf
using Random

# --- pipeline ---
include("types.jl")
include("geometry.jl")
include("allocation_matrix.jl")
include("solver.jl")
include("constraints.jl")
include("diagnostics.jl")
include("analysis.jl")
include("optimize.jl")
include("export_cpp.jl")
include("visualization.jl")

# --- ready-made vehicles ---
include("layouts/bluerov.jl")
include("layouts/simple_quad.jl")

# Types
export AbstractActuator, Thruster, ReactionWheel, Vehicle, SVec3
export AllocationResult, AllocationDiagnostics
export nactuators, nthrusters, label

# Geometry
export skew, column, wrench_column, force_contribution, torque_contribution

# Allocation matrix + forward map
export allocation_matrix, wrench, command_bounds, max_thrusts

# Solvers
export allocate, ALLOCATION_METHODS

# Constraints / failures / power
export saturate, scale_to_limits, apply_failures, failed_indices
export estimate_power, total_power, estimate_current, group_totals

# Diagnostics
export diagnostics, controllable_dofs, dominant_dof

# Analysis (reachability, method comparison, failures, robustness)
export reachable, ReachabilityResult
export compare_methods, MethodComparison
export rank_failures, report_failures
export monte_carlo, MonteCarloResult

# Design optimisation
export optimize_layout, OptimizationResult

# Firmware / ROS export
export export_cpp

# Reporting
export describe, report, bar

# Layouts
export bluerov_heavy, bluerov_vehicle, simple_quad, quad_vehicle

# Re-export the LinearAlgebra primitives the docs encourage reaching for, so
# `using ThrusterHelper` is enough to call them on an allocation matrix.
export rank, cond, svd, svdvals, nullspace, eigen, pinv

# Defined by the Plots extension (ThrusterHelperPlotsExt); stubs give a helpful
# error if called before `using Plots`.
function plot_thrusters end
function plot_vehicle end
function plot_manipulability end
export plot_thrusters, plot_vehicle, plot_manipulability

end # module
