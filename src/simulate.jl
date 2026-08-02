# simulate.jl
# A minimal controller-in-the-loop simulation, so the allocator can be watched
# in a real feedback loop — saturation, redundancy, and failures affecting
# tracking — instead of a single static solve.
#
# The rigid body is modelled per-DOF and DECOUPLED: `M ẍ = τ_achieved − drag`,
# with `M = diag(mass, mass, mass, Ixx, Iyy, Izz)`. This ignores Coriolis /
# gyroscopic cross-coupling and treats the small-angle rotations as independent
# integrators — enough to see allocation behaviour close the loop, not a
# full 6-DOF dynamics engine.
# ponytail: decoupled diagonal dynamics; add cross-coupling only if a use needs it.

"""
    SimResult

Trajectory returned by [`simulate`](@ref). Fields (each 6×N over `t`):
`state`, `velocity`, `wrench_desired`, `wrench_achieved`; DOFs are
`[Fx, Fy, Fz, τx, τy, τz]` (translations then small-angle rotations).
"""
struct SimResult
    t::Vector{Float64}
    state::Matrix{Float64}
    velocity::Matrix{Float64}
    wrench_desired::Matrix{Float64}
    wrench_achieved::Matrix{Float64}
    setpoint::Vector{Float64}
end

_vec6(x, name) = x isa Number ? fill(float(x), 6) :
    (length(x) == 6 ? collect(Float64, x) : throw(ArgumentError("$name must be a scalar or length-6")))

"""
    simulate(vehicle, setpoint; kp, kd, ki=0, drag=zeros(6), drag_lin=zeros(6),
             dt=0.02, duration=10.0, x0=zeros(6), v0=zeros(6),
             method=:qp, bounds=command_bounds(vehicle), failed=Int[]) -> SimResult

Run a PID controller in closed loop through the allocator. Each step the
controller forms a desired wrench from the setpoint error, [`allocate`](@ref)
solves for thruster commands (respecting limits, and with `failed` actuators
zeroed), and the *achieved* wrench drives the decoupled rigid-body dynamics
against quadratic drag `drag` (per-DOF, as in [`top_speeds`](@ref)).

`kp`, `ki`, `kd`, `drag`, `drag_lin` are scalars (applied to every DOF) or
length-6 vectors. The PID is **inertia-normalised** (computed-torque:
`τ = M·(kp·e + ki·∫e − kd·ẋ)`), so the same gains behave identically on every DOF
regardless of mass/inertia. Because saturation and failures make the achieved wrench differ
from the commanded one, this shows whether a design can actually *reach and hold*
a setpoint — and how a thruster failure degrades it.
"""
function simulate(v::Vehicle, setpoint::AbstractVector;
                  kp, kd, ki=0.0, drag=zeros(6), drag_lin=zeros(6),
                  dt::Real=0.02, duration::Real=10.0,
                  x0=zeros(6), v0=zeros(6),
                  method::Symbol=:qp, bounds=command_bounds(v), failed=Int[])
    length(setpoint) == 6 || throw(ArgumentError("setpoint must have 6 elements"))
    isfinite(v.mass) && v.mass > 0 ||
        throw(ArgumentError("simulate needs a positive vehicle mass"))
    M = [v.mass, v.mass, v.mass, v.inertia[1,1], v.inertia[2,2], v.inertia[3,3]]
    all(>(0), M) || throw(ArgumentError("mass and inertia diagonal must be positive"))

    Kp = _vec6(kp, "kp"); Ki = _vec6(ki, "ki"); Kd = _vec6(kd, "kd")
    dq = _vec6(drag, "drag"); dl = _vec6(drag_lin, "drag_lin")
    sp = collect(Float64, setpoint)

    B = allocation_matrix(v)
    isempty(failed) || (B = apply_failures(B, failed))
    lo, hi = _resolve_bounds(bounds, size(B, 2))

    N = round(Int, duration / dt) + 1
    t  = collect(0:N-1) .* float(dt)
    X  = zeros(6, N); V = zeros(6, N)
    Wd = zeros(6, N); Wa = zeros(6, N)
    X[:, 1] = collect(Float64, x0); V[:, 1] = collect(Float64, v0)
    integ = zeros(6)

    for k in 1:N-1
        x = @view X[:, k]; vel = @view V[:, k]
        err = sp .- x
        integ .+= err .* dt
        # Inertia-normalised (computed-torque) PID: gains are in acceleration
        # units, so the same kp/kd behave identically on every DOF regardless of
        # mass/inertia, and stability decouples from the (tiny) rotational inertia.
        acc_cmd = Kp .* err .+ Ki .* integ .- Kd .* vel  # derivative on measurement
        τd = M .* acc_cmd
        r  = allocate(B, τd; method=method, bounds=(lo, hi))
        τa = r.achieved
        # Semi-implicit velocity step with drag treated implicitly:
        #   M(v⁺−v)/dt = τa − (dl·v⁺ + dq·|v⁺|·v⁺)
        # solved per-DOF via terminal_velocity — unconditionally stable, so
        # stiff quadratic drag on tiny rotational inertia can't blow up.
        for j in 1:6
            F = τa[j] + (M[j] / dt) * vel[j]
            V[j, k+1] = terminal_velocity(F, dq[j]; c_lin = dl[j] + M[j] / dt)
        end
        X[:, k+1] = x .+ V[:, k+1] .* dt
        Wd[:, k] = τd; Wa[:, k] = τa
    end
    Wd[:, N] = Wd[:, N-1]; Wa[:, N] = Wa[:, N-1]
    return SimResult(t, X, V, Wd, Wa, sp)
end

function Base.show(io::IO, r::SimResult)
    err = r.state[:, end] .- r.setpoint
    print(io, "SimResult(", length(r.t), " steps, t=", round(r.t[end]; digits=2),
          "s, final |error|∞ = ", round(maximum(abs.(err)); digits=4), ")")
end
