# optimize.jl
# Vehicle layout optimiser — the step from *analyse my AUV* to *design my AUV*.
#
# Given a starting set of thrusters and a frame, search thruster orientations
# (and optionally positions) to optimise a design metric: maximise
# manipulability, minimise condition number, etc. Uses a dependency-free pattern
# search with random restarts, so no optimisation package is required.

"""
    OptimizationResult

Returned by [`optimize_layout`](@ref):

- `vehicle`        : the optimised [`Vehicle`](@ref) (new, immutable thrusters).
- `before`, `after`: [`AllocationDiagnostics`](@ref) for the original / optimised design.
- `score_before`, `score_after` : objective values (lower is better internally).
- `evaluations`    : number of objective evaluations used.
"""
struct OptimizationResult
    vehicle::Vehicle
    before::AllocationDiagnostics
    after::AllocationDiagnostics
    score_before::Float64
    score_after::Float64
    evaluations::Int
end

"""
    optimize_layout(vehicle; objective=:condition_number, free=:directions,
                    position_box=nothing, restarts=4, iterations=4000,
                    step0=0.4, tol=1e-4, seed=1) -> OptimizationResult

Search the thruster layout for a better design.

- `objective` : `:condition_number` (minimise κ), `:manipulability` (maximise),
                or a function `Vehicle -> Real` to **minimise**.
- `free`      : `:directions` (re-aim thrusters, positions fixed — the realistic
                default), `:positions`, or `:both`.
- `position_box` : `(lo, hi)` 3-vectors bounding thruster positions when they are
                free (defaults to ±1.5× the current extent).
- `restarts`, `iterations`, `step0`, `tol`, `seed` : optimiser controls.

Layouts that drop below full rank are penalised, so the optimiser keeps the
vehicle fully actuated. Only `Thruster` actuators are moved.

```julia
res = optimize_layout(bluerov_vehicle(); objective=:condition_number)
report(res)   # before/after κ, manipulability, control authority
```
"""
function optimize_layout(v::Vehicle;
                         objective=:condition_number, free::Symbol=:directions,
                         position_box=nothing, restarts::Integer=4,
                         iterations::Integer=4000, step0::Real=0.4,
                         tol::Real=1e-4, seed::Integer=1)
    thr = Thruster[a for a in v.actuators if a isa Thruster]
    length(thr) == length(v.actuators) ||
        throw(ArgumentError("optimize_layout currently supports all-Thruster vehicles"))
    free in (:directions, :positions, :both) ||
        throw(ArgumentError("free must be :directions, :positions or :both"))

    # default position box from current extent
    if position_box === nothing
        P = reduce(hcat, (t.position for t in thr))
        ext = maximum(abs.(P); dims=2)[:] .* 1.5 .+ 1e-3
        position_box = (-ext, ext)
    end
    lo, hi = collect(Float64, position_box[1]), collect(Float64, position_box[2])

    x0 = _encode(thr, free)
    obj = _objective_fn(objective)
    score(x) = _score(obj, _decode(x, thr, free, lo, hi))

    rng = MersenneTwister(seed)
    xbest = copy(x0); fbest = score(x0); evals = 1
    for r in 0:restarts
        xstart = r == 0 ? copy(x0) : x0 .+ 0.5 .* randn(rng, length(x0))
        x, f, e = _pattern_search(score, xstart; step0=step0, tol=tol, maxevals=iterations)
        evals += e
        if f < fbest
            fbest = f; xbest = x
        end
    end

    newthr = _decode(xbest, thr, free, lo, hi)
    newv = Vehicle(v.name * " (optimised)", newthr;
                   mass=v.mass, inertia=v.inertia, center_of_mass=v.center_of_mass)
    return OptimizationResult(newv, diagnostics(v), diagnostics(newv),
                              score(x0), fbest, evals)
end

# --- objective ----------------------------------------------------------
# Objective callables operate on the decoded Vector{Thruster}. A user-supplied
# Function is wrapped so it still receives a Vehicle.
function _objective_fn(objective)
    objective isa Function && return thr -> objective(Vehicle("candidate", thr))
    objective === :condition_number && return thr -> diagnostics(allocation_matrix(thr)).condition_number
    objective === :manipulability   && return thr -> -diagnostics(allocation_matrix(thr)).manipulability
    throw(ArgumentError("unknown objective :$objective"))
end

# rank-loss penalty keeps the search inside the fully-actuated region
function _score(obj, thr::Vector{Thruster})
    d = diagnostics(allocation_matrix(thr))
    d.rank < 6 && return 1e6 + (6 - d.rank) * 1e5
    s = obj(thr)
    return isfinite(s) ? s : 1e6
end

# --- encode / decode ----------------------------------------------------
# directions as spherical angles (az, el); positions as raw xyz (clamped to box).
function _encode(thr, free)
    x = Float64[]
    for t in thr
        if free !== :positions
            az = atan(t.direction[2], t.direction[1])
            el = asin(clamp(t.direction[3], -1, 1))
            append!(x, (az, el))
        end
        if free !== :directions
            append!(x, t.position)
        end
    end
    return x
end

function _decode(x, thr, free, lo, hi)
    out = Vector{Thruster}(undef, length(thr))
    k = 1
    for (i, t) in enumerate(thr)
        dir = t.direction; pos = t.position
        if free !== :positions
            az = x[k]; el = x[k+1]; k += 2
            dir = [cos(el)*cos(az), cos(el)*sin(az), sin(el)]
        end
        if free !== :directions
            pos = clamp.(x[k:k+2], lo, hi); k += 3
        end
        out[i] = Thruster(t.name, pos, dir; max_thrust=t.max_thrust, curve=t.curve)
    end
    return out
end

# --- pattern search (coordinate descent with shrinking step) ------------
function _pattern_search(f, x0; step0, tol, maxevals)
    x = copy(x0); fx = f(x); h = step0; evals = 1
    while h > tol && evals < maxevals
        improved = false
        for i in eachindex(x)
            for s in (h, -h)
                xt = copy(x); xt[i] += s
                ft = f(xt); evals += 1
                if ft < fx - 1e-12
                    x = xt; fx = ft; improved = true
                    break
                end
                evals >= maxevals && break
            end
            evals >= maxevals && break
        end
        improved || (h /= 2)
    end
    return x, fx, evals
end

function report(r::OptimizationResult; io::IO=stdout)
    println(io, "Layout optimisation")
    println(io, "─"^52)
    @printf(io, "  objective score : %.4g → %.4g  (%.1f%% better)\n",
            r.score_before, r.score_after,
            100 * (r.score_before - r.score_after) / abs(r.score_before + eps()))
    @printf(io, "  condition κ     : %.3f → %.3f\n",
            r.before.condition_number, r.after.condition_number)
    @printf(io, "  manipulability  : %.4g → %.4g\n",
            r.before.manipulability, r.after.manipulability)
    @printf(io, "  rank            : %d → %d\n", r.before.rank, r.after.rank)
    @printf(io, "  evaluations     : %d\n", r.evaluations)
    return nothing
end

# ---------------------------------------------------------------------------
# Sensitivity — gradient of a design metric w.r.t. the thruster layout.
# ---------------------------------------------------------------------------

# The actual metric value (not the minimised form _objective_fn returns).
_metric_fn(objective) =
    objective === :manipulability   ? (thr -> diagnostics(allocation_matrix(thr)).manipulability) :
    objective === :condition_number ? (thr -> diagnostics(allocation_matrix(thr)).condition_number) :
    objective isa Function          ? (thr -> objective(Vehicle("candidate", thr))) :
    throw(ArgumentError("unknown objective :$objective"))

# Type-generic manipulability from direction angles (positions fixed) — the
# differentiable path used by the ForwardDiff extension. B built without the
# Float64-locked `Thruster`, so it accepts Dual numbers.
function _manip_from_params(angles::AbstractVector{T}, positions) where {T}
    n = length(positions)
    B = Matrix{T}(undef, 6, n)
    for i in 1:n
        az = angles[2i-1]; el = angles[2i]
        d = (cos(el) * cos(az), cos(el) * sin(az), sin(el))
        p = positions[i]
        B[1, i] = d[1]; B[2, i] = d[2]; B[3, i] = d[3]
        B[4, i] = p[2]*d[3] - p[3]*d[2]      # (p × d)
        B[5, i] = p[3]*d[1] - p[1]*d[3]
        B[6, i] = p[1]*d[2] - p[2]*d[1]
    end
    return sqrt(max(det(B * B'), zero(T)))
end

# Implemented by ThrusterHelperForwardDiffExt when ForwardDiff is loaded.
_ad_manip_grad(thr, free) =
    error("sensitivity(...; method=:ad) requires `using ForwardDiff`")

"""
    sensitivity(vehicle; objective=:manipulability, free=:directions,
                method=:finitediff, h=1e-6) -> Vector{Float64}

Gradient of a design `objective` (`:manipulability`, `:condition_number`, or a
`Vehicle -> Real` function) with respect to the thruster layout parameters — the
same `(azimuth, elevation)` direction angles (and optionally positions) that
[`optimize_layout`](@ref) searches. Tells you which thruster to nudge, and how
much each one matters.

- `method=:finitediff` (default, always available) — central differences; works
  for every objective.
- `method=:ad` — exact gradient via `ForwardDiff` (needs `using ForwardDiff`).
  Supported for `objective=:manipulability`, `free=:directions` only: the
  condition number's SVD is not differentiable through LAPACK.
"""
function sensitivity(v::Vehicle; objective=:manipulability, free::Symbol=:directions,
                     method::Symbol=:finitediff, h::Real=1e-6)
    thr = Thruster[a for a in v.actuators if a isa Thruster]
    length(thr) == length(v.actuators) ||
        throw(ArgumentError("sensitivity currently supports all-Thruster vehicles"))
    free in (:directions, :positions, :both) ||
        throw(ArgumentError("free must be :directions, :positions or :both"))
    if method === :ad
        objective === :manipulability ||
            throw(ArgumentError("method=:ad supports objective=:manipulability only " *
                                "(the SVD-based condition number is not AD-able); use :finitediff"))
        return _ad_manip_grad(thr, free)
    end
    method === :finitediff ||
        throw(ArgumentError("method must be :finitediff or :ad"))
    metric = _metric_fn(objective)
    P = reduce(hcat, (t.position for t in thr))
    ext = maximum(abs.(P); dims=2)[:] .* 1.5 .+ 1e-3
    lo, hi = -ext, ext
    x0 = _encode(thr, free)
    g = similar(x0)
    for i in eachindex(x0)
        xp = copy(x0); xp[i] += h
        xm = copy(x0); xm[i] -= h
        g[i] = (metric(_decode(xp, thr, free, lo, hi)) -
                metric(_decode(xm, thr, free, lo, hi))) / (2h)
    end
    return g
end
