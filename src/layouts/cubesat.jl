# layouts/cubesat.jl
# A CubeSat-class 4-reaction-wheel PYRAMID — the standard small-satellite
# attitude-control cluster (the off-the-shelf wheels come from vendors like
# Blue Canyon Technologies' RWP series or CubeSpace's CubeWheel). Wheels produce
# pure torque about their spin axis, so this shows the allocation machinery is
# not underwater-specific.

"""
    cubesat_pyramid(; cant_deg=45.0, max_torque=0.004) -> Vector{ReactionWheel}

Four reaction wheels in the classic **pyramid** configuration: spin axes canted
`cant_deg` up from the body XY-plane, spaced 90° in azimuth. This gives full
3-axis torque control with one redundant wheel, so the cluster is **single-fault
tolerant** — losing any one wheel keeps the three torque axes controllable
(`rank(B) == 3`).

`max_torque` (N·m) is the per-wheel limit; the default ~4 mNm is representative
of a 3U/6U CubeSat wheel. Wrap in a [`Vehicle`](@ref) with [`cubesat_vehicle`](@ref).
"""
function cubesat_pyramid(; cant_deg::Real=45.0, max_torque::Real=0.004)
    α = deg2rad(cant_deg)
    axis(φ) = [sin(α) * cos(φ), sin(α) * sin(φ), cos(α)]
    return [ReactionWheel("RW$i", axis(deg2rad(φ)); max_torque=max_torque)
            for (i, φ) in enumerate((0.0, 90.0, 180.0, 270.0))]
end

"""
    cubesat_vehicle(; mass=4.0, kwargs...) -> Vehicle

The [`cubesat_pyramid`](@ref) wheels wrapped in a named [`Vehicle`](@ref) with a
representative ~4U small-satellite mass/inertia.

A reaction-wheel craft produces **no force**, so `diagnostics` reports the three
force DOFs as uncontrollable and `rank(B) == 3` — and, because the metrics span
the full 6-DOF wrench space, `condition_number` comes back `Inf` and
`manipulability` `0`. That's expected: the meaningful attitude metric is the
conditioning of the **torque sub-block** `B[4:6, :]` (well-conditioned ≈ √2 for
the symmetric pyramid). See `plot_manipulability(v; block=:torque)`.
"""
function cubesat_vehicle(; mass::Real=4.0, kwargs...)
    wheels = cubesat_pyramid(; kwargs...)
    inertia = Diagonal([0.02, 0.02, 0.03]) |> Matrix   # ~4U CubeSat, kg·m²
    return Vehicle("CubeSat (4-wheel pyramid)", wheels; mass=mass, inertia=inertia)
end
