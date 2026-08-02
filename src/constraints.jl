# constraints.jl
# Hardware constraints applied to / around the solver: saturation limits,
# direction-preserving scaling, failure injection, and the power model.

"""
    saturate(f, limits) -> (f_sat, saturated)

Hard-clamp each command `f[i]` to `[-limits[i], +limits[i]]` (`limits` may be a
scalar or per-actuator vector). Returns the clamped command and a `Bool` mask of
which actuators hit their limit.

Clamping changes the *direction* of the produced wrench; when that matters,
prefer [`scale_to_limits`](@ref) (preserves direction) or the `:qp` solver
(optimal within bounds).
"""
function saturate(f::AbstractVector, limits)
    lim = limits isa Number ? fill(float(limits), length(f)) : collect(Float64, limits)
    length(lim) == length(f) ||
        throw(ArgumentError("limits length $(length(lim)) ≠ commands length $(length(f))"))
    f_sat = clamp.(f, -lim, lim)
    saturated = abs.(f) .> lim .+ 1e-12
    return f_sat, saturated
end

"""
    scale_to_limits(f, limits) -> (f_scaled, factor)

If any `|f[i]| > limits[i]`, scale the *whole* command vector by the single
worst-case factor so the largest command sits exactly at its limit. Preserves
the direction of the produced wrench (you get less of it, undistorted). Returns
the scaled command and the applied `factor ∈ (0, 1]`.
"""
function scale_to_limits(f::AbstractVector, limits)
    lim = limits isa Number ? fill(float(limits), length(f)) : collect(Float64, limits)
    length(lim) == length(f) ||
        throw(ArgumentError("limits length $(length(lim)) ≠ commands length $(length(f))"))
    worst = maximum(abs.(f) ./ lim)
    factor = worst > 1 ? 1 / worst : 1.0
    return f .* factor, factor
end

"""
    apply_failures(B, failed) -> B_failed

Zero the columns of `B` for failed actuators (a dead actuator produces no wrench
however it is commanded). `failed` is a vector of indices (`[3, 5]`) or a `Bool`
mask of length N. Returns a copy.
"""
function apply_failures(B::AbstractMatrix, failed)
    Bf = copy(Matrix{Float64}(B))
    mask = _failure_mask(failed, size(B, 2))
    @inbounds for i in axes(B, 2)
        mask[i] && (Bf[:, i] .= 0.0)
    end
    return Bf
end

"""
    failed_indices(failed, n) -> Vector{Int}

Normalise a failure spec (index vector or length-`n` `Bool` mask) to a sorted
vector of failed actuator indices.
"""
failed_indices(failed, n::Integer) = findall(_failure_mask(failed, n))

function _failure_mask(failed, n::Integer)
    if eltype(failed) == Bool && length(failed) == n
        return collect(Bool, failed)
    else
        mask = falses(n)
        for i in failed
            (1 <= i <= n) || throw(ArgumentError("failed index $i out of range 1:$n"))
            mask[i] = true
        end
        return mask
    end
end

# ---------------------------------------------------------------------------
# Power model
# ---------------------------------------------------------------------------

"""
    estimate_power(f; k=1.0, p=1.5, idle=0.0) -> Vector{Float64}

Rough per-actuator electrical power (watts). Marine thrusters draw roughly
`power ∝ |thrust|^1.5` (thrust ∝ rpm², power ∝ rpm³), so

    power[i] = idle + k * |f[i]|^p

Tune `k`, `p` to a thruster's bench data. Sum for total draw.
"""
estimate_power(f::AbstractVector; k::Real=1.0, p::Real=1.5, idle::Real=0.0) =
    idle .+ k .* abs.(f) .^ p

"Total estimated electrical power for a command vector."
total_power(f::AbstractVector; kwargs...) = sum(estimate_power(f; kwargs...))

"""
    actuator_commands(forces, actuators) -> Vector{Float64}
    actuator_commands(result::AllocationResult, actuators)

Map allocated per-actuator **forces** (N) to the raw commands you actually send
to the hardware (e.g. PWM µs), through each actuator's [`ThrustCurve`](@ref).
Actuators without a curve pass through unchanged (the force itself). This is the
bridge from an [`allocate`](@ref) result to firmware.

```julia
r = allocate(vehicle, τ; method = :qp)
pwm = actuator_commands(r, vehicle.actuators)   # forces → PWM via each thruster's curve
```
"""
function actuator_commands(forces::AbstractVector, actuators)
    length(forces) == length(actuators) ||
        throw(ArgumentError("forces length $(length(forces)) ≠ actuators length $(length(actuators))"))
    return [(c = thrust_curve(a); c === nothing ? float(f) : force_to_command(c, f))
            for (f, a) in zip(forces, actuators)]
end
actuator_commands(r::AllocationResult, actuators) = actuator_commands(r.commands, actuators)

"""
    estimate_current(f; k=1.0, p=1.5, k_reverse=k) -> Vector{Float64}

Rough per-actuator current draw (amps). Like [`estimate_power`](@ref),
`current ∝ |f|^p`, but with an optional separate coefficient for **reverse**
commands — many thrusters (e.g. the BlueRobotics T200) are less efficient in
reverse and draw more current per unit thrust:

    current[i] = (f[i] ≥ 0 ? k : k_reverse) * |f[i]|^p

Anchor to a bench point with `k = amps / thrust^p` (e.g. 6 A at 20 N forward,
p=1.5 ⇒ `k ≈ 0.067`; 6 A at 15 N reverse ⇒ `k_reverse ≈ 0.103`). Feed the result
to [`group_totals`](@ref) to check a shared power-board budget.

Pass `voltage` (with the `ref_voltage` the anchor was measured at) to account for
battery sag: producing a given thrust takes roughly constant power, so current
scales as `ref_voltage / voltage` — a sagging pack draws *more* current.
"""
estimate_current(f::AbstractVector; k::Real=1.0, p::Real=1.5, k_reverse::Real=k,
                 voltage::Real=1.0, ref_voltage::Real=1.0) =
    (ref_voltage / voltage) .* [(fi >= 0 ? k : k_reverse) * abs(fi)^p for fi in f]

# ---------------------------------------------------------------------------
# Battery voltage derating
# ---------------------------------------------------------------------------

"""
    derate(curve::ThrustCurve; from, to, exponent=2.0) -> ThrustCurve

Scale a [`ThrustCurve`](@ref) from supply voltage `from` to voltage `to`. Thrust
scales as `(to/from)^exponent`; the default `exponent=2` is the textbook
propeller law (thrust ∝ rpm² ∝ V²). Real thrusters are gentler — a BlueRobotics
T200 is closer to `^1.3`–`^1.7` — so **retune `exponent` to your bench data**.
Command samples are unchanged; only the forces scale.
"""
function derate(c::ThrustCurve; from::Real, to::Real, exponent::Real=2.0)
    s = (float(to) / float(from))^exponent
    return ThrustCurve(c.force .* s, c.command)
end

"""
    derate(vehicle::Vehicle; from, to, exponent=2.0) -> Vehicle

A copy of `vehicle` with every thruster's `max_thrust` (and its `ThrustCurve`, if
any) derated from supply voltage `from` to `to`, so `reachable`, the authority
envelope, and `allocate` all reflect the sagging pack. Non-thruster actuators are
left unchanged. See [`derate(::ThrustCurve)`](@ref) on the `exponent`.
"""
function derate(v::Vehicle; from::Real, to::Real, exponent::Real=2.0)
    s = (float(to) / float(from))^exponent
    acts = AbstractActuator[a isa Thruster ?
        Thruster(a.name, a.position, a.direction;
                 max_thrust = a.max_thrust * s,
                 curve = a.curve === nothing ? nothing :
                         derate(a.curve; from=from, to=to, exponent=exponent)) :
        a for a in v.actuators]
    return Vehicle(v.name * " @ $(to)V", acts;
                   mass=v.mass, inertia=v.inertia, center_of_mass=v.center_of_mass)
end

"""
    group_totals(x, groups) -> Vector{Float64}

Sum a per-actuator quantity `x` (current, power, |thrust|, …) over each group of
actuator indices, one total per group in order. `groups` is any collection of
index collections — thrusters sharing a power board, a thermal zone, a hydraulic
circuit. Compare against per-group budgets to check a shared limit:

    boards = [1:4, 5:8]                       # T0–T3, T4–T7
    I = estimate_current(f; k=0.067, k_reverse=0.103)
    all(group_totals(I, boards) .<= 24)       # within the 24 A/board budget?
"""
group_totals(x::AbstractVector, groups) = [sum(x[g]) for g in groups]

"""
    allocate_grouped(B, τ; groups, budgets, bounds, k=1.0, p=1.5, k_reverse=k,
                     maxiter=30, shrink=0.85) -> AllocationResult
    allocate_grouped(vehicle, τ; groups, budgets, bounds=command_bounds(vehicle), ...)

Allocate `τ` while keeping each group's total current within `budgets`, on top of
the per-actuator `bounds`. Use when actuators share a current/thermal budget —
e.g. thrusters ganged on one power board — that per-actuator limits can't express.

`groups` is a collection of index collections (see [`group_totals`](@ref));
`budgets` a per-group cap (scalar ⇒ same for all) in the units of
[`estimate_current`](@ref) (tune `k`, `k_reverse`, `p` to your hardware).

Heuristic: solve the box-constrained `:qp`, and while any group is over budget,
shrink the effective bounds on that group's actuators and re-solve. The `:qp`
inner solve means the per-actuator `bounds` are **always** respected; the
residual grows if `τ` cannot be met within the budgets (check `result.residual`
and `group_totals` on the result). This is a heuristic, not the optimal
constrained QP — good enough until a design needs exactness.
"""
function allocate_grouped(B::AbstractMatrix, τ::AbstractVector;
                          groups, budgets, bounds,
                          k::Real=1.0, p::Real=1.5, k_reverse::Real=k,
                          maxiter::Integer=30, shrink::Real=0.85)
    lo, hi = _resolve_bounds(bounds, size(B, 2))
    lo = copy(lo); hi = copy(hi)                 # effective box, tightened in-loop
    bud = budgets isa Number ? fill(float(budgets), length(groups)) :
          collect(Float64, budgets)
    length(bud) == length(groups) ||
        throw(ArgumentError("budgets must have one entry per group"))
    r = allocate(B, τ; method=:qp, bounds=(lo, hi))
    for _ in 1:maxiter
        over = group_totals(estimate_current(r.commands; k=k, p=p, k_reverse=k_reverse),
                            groups) .> bud
        any(over) || return r
        # ponytail: monotone box-shrink on over-budget groups drives their
        # current down; not the optimal QP, but converges and stays in-bounds.
        for (gi, g) in enumerate(groups), i in g
            over[gi] || continue
            lo[i] *= shrink; hi[i] *= shrink
        end
        r = allocate(B, τ; method=:qp, bounds=(lo, hi))
    end
    @warn "allocate_grouped: group budgets not met in $maxiter iterations; " *
          "returning best effort within bounds (inspect group_totals on the result)."
    return r
end

allocate_grouped(v::Vehicle, τ; bounds=command_bounds(v), kwargs...) =
    allocate_grouped(allocation_matrix(v), τ; bounds=bounds, kwargs...)
