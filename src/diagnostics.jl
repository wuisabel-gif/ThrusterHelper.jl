# diagnostics.jl
# Tell-me-about-this-design analysis, kept separate from allocation. Rank,
# conditioning, redundancy, control authority and the SVD-derived "weakest
# direction" say far more about a thruster layout than the thrust values alone.

const _DOF_LABELS = ("Fx", "Fy", "Fz", "τx", "τy", "τz")

"""
    AllocationDiagnostics

Numerical health of an allocation matrix `B`. Fields:

- `n_dof`, `n_actuators`     : dimensions of `B` (6 × N).
- `rank`                     : `rank(B)`; 6 ⇒ fully actuated.
- `redundancy`              : `N - rank`, the null-space dimension (spare control).
- `singular_values`         : the σᵢ of `B`.
- `condition_number`        : `σ_max / σ_min` (well-conditioned ≈ small; Inf ⇒ rank loss).
- `manipulability`          : `√det(B Bᵀ)`, volume of the achievable-wrench ellipsoid.
- `controllable`            : `Bool` per DOF — is that wrench axis in range(B)?
- `weakest_direction`       : unit wrench (left singular vector of σ_min) hardest to produce.
- `weakest_gain`            : σ_min, the gain along `weakest_direction`.
"""
struct AllocationDiagnostics
    n_dof::Int
    n_actuators::Int
    rank::Int
    redundancy::Int
    singular_values::Vector{Float64}
    condition_number::Float64
    manipulability::Float64
    controllable::Vector{Bool}
    weakest_direction::Vector{Float64}
    weakest_gain::Float64
end

"""
    diagnostics(B; tol=1e-9) -> AllocationDiagnostics
    diagnostics(vehicle)
    diagnostics(actuators)

Run the full numerical analysis of an allocation matrix.
"""
function diagnostics(B::AbstractMatrix; tol::Real=1e-9)
    size(B, 1) == 6 || throw(ArgumentError("B must have 6 rows"))
    n = size(B, 2)
    F = svd(B)                     # B = U Σ Vᵀ, U is 6×6, σ sorted descending
    σ = F.S
    r = count(>(tol * (σ[1] + eps())), σ)
    if r == 0
        # Degenerate all-zero B (e.g. every actuator failed): nothing is
        # reachable. Return a well-defined result instead of indexing σ[0].
        return AllocationDiagnostics(6, n, 0, n, copy(σ), Inf, 0.0,
                                     fill(false, 6), F.U[:, 1], 0.0)
    end
    cn = σ[r] > 0 ? σ[1] / σ[r] : Inf
    manip = r == 6 ? sqrt(max(det(B * B'), 0.0)) : 0.0

    # Control authority: e_k achievable ⇔ in range(B) = span(U[:, 1:r]).
    Ur = F.U[:, 1:r]
    P = Ur * Ur'                   # projector onto range(B)
    controllable = [norm(P[:, k] - I_col(6, k)) < 1e-6 for k in 1:6]

    weakest_dir = F.U[:, r]        # left singular vec of smallest *nonzero* σ
    weakest_gain = σ[r]

    return AllocationDiagnostics(6, n, r, n - r, copy(σ), cn, manip,
                                 controllable, weakest_dir, weakest_gain)
end

diagnostics(v::Vehicle; kwargs...) = diagnostics(allocation_matrix(v); kwargs...)
diagnostics(a::AbstractVector{<:AbstractActuator}; kwargs...) =
    diagnostics(allocation_matrix(a); kwargs...)

I_col(n, k) = (e = zeros(n); e[k] = 1.0; e)

"""
    controllable_dofs(B; tol=1e-9) -> NamedTuple

Lightweight control-authority check kept for convenience: returns
`(rank, controllable::Vector{Bool}, labels)`. For the full picture use
[`diagnostics`](@ref).
"""
function controllable_dofs(B::AbstractMatrix; tol::Real=1e-9)
    d = diagnostics(B; tol=tol)
    return (rank=d.rank, controllable=d.controllable, labels=_DOF_LABELS)
end
controllable_dofs(v::Vehicle; kwargs...) = controllable_dofs(allocation_matrix(v); kwargs...)
controllable_dofs(a::AbstractVector{<:AbstractActuator}; kwargs...) =
    controllable_dofs(allocation_matrix(a); kwargs...)

"""
    dominant_dof(direction) -> String

Label a wrench direction by its largest-magnitude DOF (e.g. a left singular
vector → `"τy"`).
"""
function dominant_dof(direction::AbstractVector)
    k = argmax(abs.(direction))
    return _DOF_LABELS[k]
end
