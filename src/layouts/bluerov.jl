# layouts/bluerov.jl
# An 8-thruster vectored layout in the style of the BlueROV2 *Heavy* /
# RoboSub-class AUV. Convention: x forward, y left, z up.

"""
    bluerov_heavy(; arm=0.22, span=0.12, max_thrust=1.0) -> Vector{Thruster}

Eight-thruster vectored configuration:

- 4 **horizontal** thrusters at 45° in the body XY-plane → surge (Fx), sway
  (Fy), yaw (τz).
- 4 **vertical** thrusters → heave (Fz), roll (τx), pitch (τy).

`arm` is the horizontal distance from centre to each thruster; `span` the
vertical offset of the vertical thrusters. The result is fully actuated
(`rank(B) == 6`). Wrap it in a [`Vehicle`](@ref) with [`bluerov_vehicle`](@ref).
"""
function bluerov_heavy(; arm::Real=0.22, span::Real=0.12, max_thrust::Real=1.0)
    a = float(arm); s = float(span); c = sqrt(2) / 2
    mt = float(max_thrust)
    return [
        Thruster("front-right-horiz", [ a, -a, 0.0], [ c,  c, 0.0]; max_thrust=mt),
        Thruster("front-left-horiz",  [ a,  a, 0.0], [ c, -c, 0.0]; max_thrust=mt),
        Thruster("back-right-horiz",  [-a, -a, 0.0], [-c,  c, 0.0]; max_thrust=mt),
        Thruster("back-left-horiz",   [-a,  a, 0.0], [-c, -c, 0.0]; max_thrust=mt),
        Thruster("front-right-vert",  [ a, -a, s],   [0.0, 0.0, 1.0]; max_thrust=mt),
        Thruster("front-left-vert",   [ a,  a, s],   [0.0, 0.0, 1.0]; max_thrust=mt),
        Thruster("back-right-vert",   [-a, -a, s],   [0.0, 0.0, 1.0]; max_thrust=mt),
        Thruster("back-left-vert",    [-a,  a, s],   [0.0, 0.0, 1.0]; max_thrust=mt),
    ]
end

"""
    bluerov_vehicle(; kwargs...) -> Vehicle

Convenience: the [`bluerov_heavy`](@ref) thrusters wrapped in a named
[`Vehicle`](@ref) with a representative mass/inertia.
"""
function bluerov_vehicle(; mass::Real=11.0, kwargs...)
    thr = bluerov_heavy(; kwargs...)
    inertia = Diagonal([0.16, 0.16, 0.16]) |> Matrix
    return Vehicle("BlueROV2 Heavy", thr; mass=mass, inertia=inertia)
end

"""
    bluerov_standard(; arm=0.22, span=0.12, max_thrust=1.0) -> Vector{Thruster}

The **standard** (non-Heavy) BlueROV2: six thrusters — the same four vectored
horizontal thrusters as [`bluerov_heavy`](@ref), plus **two** vertical thrusters
mounted port/starboard.

Those two verticals give heave (Fz) and roll (τx) but no pitch, so the vehicle is
**under-actuated in pitch** — `rank(B) == 5`, with τy uncontrollable. It's the
real-world middle ground between [`simple_quad`](@ref) (rank 3) and the
fully-actuated Heavy (rank 6). Wrap it with [`bluerov_standard_vehicle`](@ref).
"""
function bluerov_standard(; arm::Real=0.22, span::Real=0.12, max_thrust::Real=1.0)
    a = float(arm); s = float(span); c = sqrt(2) / 2; mt = float(max_thrust)
    return [
        Thruster("front-right-horiz", [ a, -a, 0.0], [ c,  c, 0.0]; max_thrust=mt),
        Thruster("front-left-horiz",  [ a,  a, 0.0], [ c, -c, 0.0]; max_thrust=mt),
        Thruster("back-right-horiz",  [-a, -a, 0.0], [-c,  c, 0.0]; max_thrust=mt),
        Thruster("back-left-horiz",   [-a,  a, 0.0], [-c, -c, 0.0]; max_thrust=mt),
        Thruster("left-vert",         [0.0,  a, s],  [0.0, 0.0, 1.0]; max_thrust=mt),
        Thruster("right-vert",        [0.0, -a, s],  [0.0, 0.0, 1.0]; max_thrust=mt),
    ]
end

"""
    bluerov_standard_vehicle(; mass=10.0, kwargs...) -> Vehicle

The [`bluerov_standard`](@ref) thrusters wrapped in a named [`Vehicle`](@ref).
`rank(B) == 5` — controllable in surge/sway/heave/roll/yaw, **not** pitch.
"""
function bluerov_standard_vehicle(; mass::Real=10.0, kwargs...)
    thr = bluerov_standard(; kwargs...)
    inertia = Diagonal([0.14, 0.14, 0.14]) |> Matrix
    return Vehicle("BlueROV2", thr; mass=mass, inertia=inertia)
end
