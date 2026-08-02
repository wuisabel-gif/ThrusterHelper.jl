# analysis.jl
# Higher-level questions a designer actually asks: *can* the vehicle do this?
# which solver is best here? what does each failure cost? how robust is the
# design to manufacturing error? Built on the allocation + diagnostics core.

# ---------------------------------------------------------------------------
# 1. Reachability — can this vehicle even produce this wrench?
# ---------------------------------------------------------------------------

"""
    ReachabilityResult

Returned by [`reachable`](@ref). Fields:

- `reachable::Bool`           : is `τ` achievable within the actuator limits?
- `desired`, `achieved`       : requested vs closest achievable wrench.
- `error`, `max_error`        : `achieved - desired` and its ∞-norm.
- `commands`                  : the limit-respecting commands that get closest.
- `saturated::Vector{Int}`    : actuators pinned at a bound.
- `reason::String`            : human-readable explanation.
"""
struct ReachabilityResult
    reachable::Bool
    desired::Vector{Float64}
    achieved::Vector{Float64}
    error::Vector{Float64}
    max_error::Float64
    commands::Vector{Float64}
    saturated::Vector{Int}
    reason::String
end

"""
    reachable(vehicle, τ; tol=1e-4) -> ReachabilityResult
    reachable(B, τ; bounds, tol=1e-4)

Decide whether the desired wrench `τ` is achievable within the actuator
saturation limits, and if not, report the *closest* achievable wrench (the `:qp`
projection onto the actuation set). Unlike a raw `pinv`, this respects limits, so
it answers the real design question — "Fx = 30 N? The vehicle tops out at 22 N."
"""
function reachable(B::AbstractMatrix, τ::AbstractVector; bounds, tol::Real=1e-4)
    lo, hi = _resolve_bounds(bounds, size(B, 2))
    r = allocate(B, τ; method=:qp, bounds=(lo, hi))
    err = r.achieved .- collect(Float64, τ)
    maxerr = maximum(abs.(err))
    sat = findall(i -> abs(r.commands[i] - hi[i]) < 1e-6 || abs(r.commands[i] - lo[i]) < 1e-6,
                  eachindex(r.commands))
    ok = maxerr <= tol
    reason = if ok
        "reachable within limits"
    else
        worstdof = _DOF_LABELS[argmax(abs.(err))]
        satn = length(sat)
        "UNREACHABLE: $worstdof short by $(round(maxerr; digits=4)); " *
        "$(satn) actuator$(satn == 1 ? "" : "s") saturated"
    end
    return ReachabilityResult(ok, collect(Float64, τ), r.achieved, err, maxerr,
                              r.commands, sat, reason)
end

reachable(v::Vehicle, τ; tol::Real=1e-4) =
    reachable(allocation_matrix(v), τ; bounds=command_bounds(v), tol=tol)
reachable(a::AbstractVector{<:AbstractActuator}, τ; tol::Real=1e-4) =
    reachable(allocation_matrix(a), τ; bounds=command_bounds(a), tol=tol)

function report(r::ReachabilityResult; io::IO=stdout)
    println(io, "Reachability: ", r.reachable ? "REACHABLE ✓" : "NOT REACHABLE ✗")
    println(io, "  ", r.reason)
    @printf(io, "  %-4s %12s %12s %12s\n", "DOF", "desired", "achievable", "error")
    for k in 1:6
        @printf(io, "  %-4s %12.4f %12.4f %12.4f\n",
                _DOF_LABELS[k], r.desired[k], r.achieved[k], r.error[k])
    end
    @printf(io, "  max error: %.5g   saturated actuators: %s\n",
            r.max_error, isempty(r.saturated) ? "none" : string(r.saturated))
    return nothing
end

# ---------------------------------------------------------------------------
# 2. Method comparison — which solver wins on this command?
# ---------------------------------------------------------------------------

"""
    MethodComparison

A table of solver results (one row per method) from [`compare_methods`](@ref);
each row is a NamedTuple `(method, residual, l2, power, saturated, time_s)`.
"""
struct MethodComparison
    rows::Vector{NamedTuple}
end

"""
    compare_methods(vehicle, τ; methods, weights, bounds, repeats=50) -> MethodComparison
    compare_methods(B, τ; limits, ...)

Run every allocation method on the same command and tabulate residual, ‖f‖₂,
estimated power, number of saturated actuators and wall-clock time. This is the
"framework for comparing allocation algorithms" view in one call.
"""
function compare_methods(B::AbstractMatrix, τ::AbstractVector;
                         methods=(:minimum_norm, :weighted, :minimum_power, :qp),
                         weights=ones(size(B, 2)),
                         limits=fill(Inf, size(B, 2)),
                         bounds=nothing,
                         repeats::Integer=50)
    lim = limits isa Number ? fill(float(limits), size(B, 2)) : collect(Float64, limits)
    bnd = bounds === nothing ? lim : bounds
    rows = NamedTuple[]
    for m in methods
        kw = m === :weighted ? (; weights=weights) :
             m === :qp        ? (; bounds=bnd) : (;)
        r = allocate(B, τ; method=m, kw...)
        f = r.commands
        # timing: warm up, then best of `repeats`
        allocate(B, τ; method=m, kw...)
        best = Inf
        for _ in 1:repeats
            best = min(best, @elapsed allocate(B, τ; method=m, kw...))
        end
        nsat = count(i -> abs(f[i]) > lim[i] + 1e-9, eachindex(f))
        push!(rows, (method=m,
                     residual=norm(r.residual),
                     l2=norm(f),
                     power=total_power(f),
                     saturated=nsat,
                     time_s=best))
    end
    return MethodComparison(rows)
end

compare_methods(v::Vehicle, τ; kwargs...) =
    compare_methods(allocation_matrix(v), τ;
                    limits=max_thrusts(v), bounds=command_bounds(v), kwargs...)
compare_methods(a::AbstractVector{<:AbstractActuator}, τ; kwargs...) =
    compare_methods(allocation_matrix(a), τ;
                    limits=max_thrusts(a), bounds=command_bounds(a), kwargs...)

function report(c::MethodComparison; io::IO=stdout)
    println(io, "Allocation method comparison")
    println(io, "─"^64)
    @printf(io, "  %-14s %10s %8s %8s %6s %10s\n",
            "method", "residual", "‖f‖₂", "power", "sat", "time")
    for r in c.rows
        @printf(io, "  %-14s %10.4g %8.4f %8.4f %6d %8.1f μs\n",
                r.method, r.residual, r.l2, r.power, r.saturated, r.time_s * 1e6)
    end
    return nothing
end
Base.show(io::IO, c::MethodComparison) = report(c; io=io)

# ---------------------------------------------------------------------------
# 3. Failure analysis — what does each thruster cost when it dies?
# ---------------------------------------------------------------------------

"""
    rank_failures(vehicle; pairs=false) -> Vector{NamedTuple}

For each single-actuator failure (optionally each *pair* too), report the rank
before/after, which DOFs become uncontrollable, and the condition-number change.
Turns [`apply_failures`](@ref) into a design tool: it ranks how critical each
actuator is.
"""
function rank_failures(B::AbstractMatrix; names=nothing, pairs::Bool=false)
    n = size(B, 2)
    base = diagnostics(B)
    label_for(idx) = names === nothing ? "T$(idx)" : join((names[i] for i in idx), "+")
    rows = NamedTuple[]
    combos = pairs ? [[i] for i in 1:n] ∪ [[i, j] for i in 1:n for j in i+1:n] :
                     [[i] for i in 1:n]
    for idx in combos
        d = diagnostics(apply_failures(B, idx))
        lost = collect(_DOF_LABELS)[findall(base.controllable .& .!d.controllable)]
        push!(rows, (failed=label_for(idx),
                     rank_before=base.rank, rank_after=d.rank,
                     lost_dofs=lost,
                     cond_before=base.condition_number,
                     cond_after=d.condition_number))
    end
    return rows
end

rank_failures(v::Vehicle; kwargs...) =
    rank_failures(allocation_matrix(v); names=[label(a) for a in v.actuators], kwargs...)
rank_failures(a::AbstractVector{<:AbstractActuator}; kwargs...) =
    rank_failures(allocation_matrix(a); names=[label(x) for x in a], kwargs...)

"""
    report_failures(rows; io=stdout)

Pretty-print the table from [`rank_failures`](@ref).
"""
function report_failures(rows; io::IO=stdout)
    println(io, "Failure criticality")
    println(io, "─"^64)
    @printf(io, "  %-22s %10s %18s %12s\n", "failed", "rank", "lost DOFs", "cond Δ")
    for r in rows
        rk = @sprintf("%d→%d", r.rank_before, r.rank_after)
        lost = isempty(r.lost_dofs) ? "—" : join(r.lost_dofs, ",")
        cnd = @sprintf("%.1f→%.1f", r.cond_before, r.cond_after)
        flag = isempty(r.lost_dofs) ? "" : "  ⚠"
        @printf(io, "  %-22s %10s %18s %12s%s\n", r.failed, rk, lost, cnd, flag)
    end
    return nothing
end

# ---------------------------------------------------------------------------
# 4. Monte-Carlo robustness — how does manufacturing error bite?
# ---------------------------------------------------------------------------

"""
    MonteCarloResult

Aggregate of [`monte_carlo`](@ref): per-DOF loss probabilities, and the
distribution of rank / condition number / manipulability over the samples.
"""
struct MonteCarloResult
    samples::Int
    dof_loss_prob::Vector{Float64}        # P(DOF k uncontrollable)
    full_rank_prob::Float64
    cond_mean::Float64
    cond_p95::Float64
    manip_mean::Float64
end

"""
    monte_carlo(vehicle; misalignment_deg=2.0, position_sigma=0.0,
                failure_prob=0.0, samples=10_000, seed=1) -> MonteCarloResult

Perturb every thruster's pointing direction by Gaussian misalignment (and
optionally its position), independently fail each thruster with probability
`failure_prob`, and resample the design `samples` times — reporting how often
each DOF is lost, the chance of staying full-rank, and the condition-number
distribution. Answers "given ±2° misalignment and a 5%-per-thruster failure rate,
what's the chance we lose yaw authority?".

Misalignment alone rarely causes exact rank loss (it degrades conditioning, seen
in `cond_mean`/`cond_p95`); set `failure_prob > 0` to drive DOF-loss events.
Deterministic given `seed` (uses a local RNG; no global state touched).
"""
function monte_carlo(thrusters::AbstractVector{Thruster};
                     misalignment_deg::Real=2.0, position_sigma::Real=0.0,
                     failure_prob::Real=0.0, samples::Integer=10_000, seed::Integer=1)
    rng = MersenneTwister(seed)
    σang = deg2rad(misalignment_deg)
    nDOF_loss = zeros(Int, 6)
    full_rank = 0
    conds = Float64[]; manips = Float64[]
    for _ in 1:samples
        perturbed = Vector{Thruster}(undef, length(thrusters))
        dead = Int[]
        for (i, t) in enumerate(thrusters)
            d = t.direction .+ σang .* randn(rng, 3)        # small-angle misalignment
            p = position_sigma > 0 ? t.position .+ position_sigma .* randn(rng, 3) : t.position
            perturbed[i] = Thruster(t.name, p, d ./ norm(d), t.max_thrust)
            failure_prob > 0 && rand(rng) < failure_prob && push!(dead, i)
        end
        B = allocation_matrix(perturbed)
        isempty(dead) || (B = apply_failures(B, dead))
        d = diagnostics(B)
        d.rank == 6 && (full_rank += 1)
        for k in 1:6
            d.controllable[k] || (nDOF_loss[k] += 1)
        end
        push!(conds, isfinite(d.condition_number) ? d.condition_number : NaN)
        push!(manips, d.manipulability)
    end
    finc = filter(isfinite, conds)
    return MonteCarloResult(samples, nDOF_loss ./ samples, full_rank / samples,
                            isempty(finc) ? Inf : sum(finc)/length(finc),
                            isempty(finc) ? Inf : quantile_sorted(finc, 0.95),
                            sum(manips)/length(manips))
end

function monte_carlo(v::Vehicle; kwargs...)
    thr = Thruster[a for a in v.actuators if a isa Thruster]
    dropped = nactuators(v) - length(thr)
    dropped == 0 || @warn "monte_carlo only perturbs Thruster actuators; " *
        "ignoring $dropped non-Thruster actuator(s) on \"$(v.name)\". " *
        "Reported rank / DOF-loss / conditioning reflect the thrusters alone."
    return monte_carlo(thr; kwargs...)
end

# tiny dependency-free quantile (linear interpolation on the sorted sample)
function quantile_sorted(x, q)
    s = sort(x); n = length(s)
    n == 1 && return s[1]
    h = (n - 1) * q + 1
    lo = floor(Int, h); hi = min(lo + 1, n)
    return s[lo] + (h - lo) * (s[hi] - s[lo])
end

function report(m::MonteCarloResult; io::IO=stdout)
    println(io, "Monte-Carlo robustness  (", m.samples, " samples)")
    println(io, "─"^52)
    @printf(io, "  P(full rank / fully actuated) : %.3f\n", m.full_rank_prob)
    println(io, "  P(lose DOF):")
    for k in 1:6
        bar = round(Int, m.dof_loss_prob[k] * 20)
        @printf(io, "      %-3s : %.3f  %s\n", _DOF_LABELS[k], m.dof_loss_prob[k], "█"^bar)
    end
    @printf(io, "  condition number  : mean %.2f , p95 %.2f\n", m.cond_mean, m.cond_p95)
    @printf(io, "  manipulability    : mean %.4g\n", m.manip_mean)
    return nothing
end
