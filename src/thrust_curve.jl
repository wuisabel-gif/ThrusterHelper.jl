# thrust_curve.jl
# Nonlinear force ↔ actuator-command mapping.
#
# Allocation is done in *force* space (the wrench is linear in thruster force),
# so a thruster's nonlinear force→command relationship is an OUTPUT mapping
# applied after `allocate`, not part of the allocation. A `ThrustCurve` captures
# that relationship — including forward/reverse asymmetry and a dead-band — from
# a measured table (e.g. a BlueRobotics T200 at a given voltage).

"""
    ThrustCurve(force, command)

A monotonic map between actuator **force** (N) and the raw **command** it takes
(e.g. PWM microseconds, or RPM). `force` and `command` are equal-length,
**strictly increasing** sample vectors — so the curve is invertible in both
directions. Asymmetry (more thrust forward than reverse) and a dead-band around
neutral are captured simply by the samples; a linear thruster needs no curve.

Use [`force_to_command`](@ref) to turn an allocated force into what you send to
the hardware, and [`command_to_force`](@ref) for the inverse. [`t200_curve`](@ref)
is a ready-made BlueRobotics T200 curve.
"""
struct ThrustCurve
    force::Vector{Float64}
    command::Vector{Float64}

    function ThrustCurve(force, command)
        f = collect(Float64, force)
        c = collect(Float64, command)
        length(f) == length(c) ||
            throw(ArgumentError("force and command must have equal length"))
        length(f) >= 2 || throw(ArgumentError("a ThrustCurve needs at least 2 samples"))
        all(>(0), diff(f)) || throw(ArgumentError("force samples must be strictly increasing"))
        all(>(0), diff(c)) || throw(ArgumentError("command samples must be strictly increasing"))
        return new(f, c)
    end
end

# Piecewise-linear interpolation, clamped at the ends (saturation).
function _interp(xs::Vector{Float64}, ys::Vector{Float64}, x::Real)
    x <= xs[1]   && return ys[1]
    x >= xs[end] && return ys[end]
    i = searchsortedlast(xs, x)
    t = (x - xs[i]) / (xs[i+1] - xs[i])
    return ys[i] + t * (ys[i+1] - ys[i])
end

"Command that produces force `f` (N), clamped to the curve's range."
force_to_command(c::ThrustCurve, f::Real) = _interp(c.force, c.command, f)

"Force (N) produced by raw command `cmd`, clamped to the curve's range."
command_to_force(c::ThrustCurve, cmd::Real) = _interp(c.command, c.force, cmd)

"""
    t200_curve() -> ThrustCurve

A representative BlueRobotics **T200** thruster curve at 16 V: PWM (µs) ↔ thrust
(N), forward/reverse-asymmetric (≈ +51.5 N forward, −40 N reverse) with a
dead-band near the 1500 µs neutral. Tune the samples to your own bench data;
voltage dependence is a separate concern (see the roadmap).
"""
function t200_curve()
    pwm   = [1100.0, 1300.0, 1400.0, 1470.0, 1500.0, 1530.0, 1600.0, 1700.0, 1800.0, 1900.0]
    force = [ -40.0,  -19.5,   -8.8,   -1.0,    0.0,    1.0,   11.8,   28.4,   42.2,   51.5]
    return ThrustCurve(force, pwm)
end
