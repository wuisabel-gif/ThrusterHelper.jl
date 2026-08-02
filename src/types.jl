# types.jl
# Core data types: the actuator hierarchy, the Vehicle, and result containers.
#
# Everything downstream (allocation matrix, solvers, diagnostics) is written
# against the AbstractActuator interface, so the same machinery works for any
# rigid body that turns scalar actuator commands into a 6-DOF wrench — AUVs,
# spacecraft, drones, omni-robots.

"3-element Float64 vector alias used throughout the package."
const SVec3 = Vector{Float64}

# ---------------------------------------------------------------------------
# Actuator interface
# ---------------------------------------------------------------------------

"""
    AbstractActuator

Anything that converts a single scalar command into a 6-DOF wrench
`[Fx, Fy, Fz, τx, τy, τz]` on the body. Concrete subtypes must implement:

- `wrench_column(a) -> Vector{Float64}` : the 6-vector produced by a unit command.
- `command_limits(a) -> Tuple{Float64,Float64}` : `(lo, hi)` command bounds.
- `label(a) -> String` : a human-readable name.

Provided implementations: [`Thruster`](@ref), [`ReactionWheel`](@ref).
"""
abstract type AbstractActuator end

label(a::AbstractActuator) = a.name
command_limits(a::AbstractActuator) = (-a.max_thrust, a.max_thrust)

"""
    Thruster(name, position, direction; max_thrust=1.0) <: AbstractActuator

A fixed thruster. `position` is the mount point in the body frame (metres,
relative to the centre of mass); `direction` is the **unit** vector along the
force the thruster applies to the body (stored normalised). A unit command
produces force `direction` and torque `position × direction`.

The type is **immutable** — a vehicle does not move its thrusters around at run
time; build a new `Thruster` (or `Vehicle`) to change geometry.
"""
struct Thruster <: AbstractActuator
    name::String
    position::SVec3
    direction::SVec3
    max_thrust::Float64
    curve::Union{Nothing,ThrustCurve}   # optional force↔command map; `nothing` ⇒ linear
end

function Thruster(; name::AbstractString="thruster", position, direction,
                  max_thrust::Real=1.0, curve::Union{Nothing,ThrustCurve}=nothing)
    p = collect(Float64, position)
    d = collect(Float64, direction)
    length(p) == 3 || throw(ArgumentError("position must have 3 elements, got $(length(p))"))
    length(d) == 3 || throw(ArgumentError("direction must have 3 elements, got $(length(d))"))
    n = norm(d)
    n > 0 || throw(ArgumentError("thruster `$name` has a zero direction vector"))
    max_thrust > 0 || throw(ArgumentError("max_thrust must be positive, got $max_thrust"))
    return Thruster(String(name), p, d ./ n, Float64(max_thrust), curve)
end

Thruster(name::AbstractString, position, direction; max_thrust::Real=1.0,
         curve::Union{Nothing,ThrustCurve}=nothing) =
    Thruster(; name=name, position=position, direction=direction,
             max_thrust=max_thrust, curve=curve)

# Backward-compatible 4-arg positional constructor (curve defaults to none).
Thruster(name::AbstractString, position, direction, max_thrust::Real) =
    Thruster(String(name), collect(Float64, position), collect(Float64, direction),
             Float64(max_thrust), nothing)

"""
    thrust_curve(actuator) -> Union{ThrustCurve,Nothing}

The actuator's force↔command [`ThrustCurve`](@ref), or `nothing` if it maps
force to command linearly (the default, and always for non-thrusters).
"""
thrust_curve(::AbstractActuator) = nothing
thrust_curve(t::Thruster) = t.curve

wrench_column(t::Thruster) = vcat(t.direction, cross(t.position, t.direction))

Base.show(io::IO, t::Thruster) = print(io,
    "Thruster(\"", t.name, "\" pos=", round.(t.position; digits=3),
    " dir=", round.(t.direction; digits=3), " max=", t.max_thrust, "N)")

"""
    ReactionWheel(name, axis; max_torque=1.0) <: AbstractActuator

A reaction wheel / control-moment device: produces **pure torque** about a body
axis and no force. Included to show that the allocation machinery is not
underwater-specific — a spacecraft attitude actuator drops straight into the
same `allocation_matrix` / `allocate` pipeline.
"""
struct ReactionWheel <: AbstractActuator
    name::String
    axis::SVec3
    max_thrust::Float64   # reused field name so command_limits works uniformly
end

function ReactionWheel(name::AbstractString, axis; max_torque::Real=1.0)
    a = collect(Float64, axis)
    length(a) == 3 || throw(ArgumentError("axis must have 3 elements"))
    n = norm(a); n > 0 || throw(ArgumentError("axis must be non-zero"))
    max_torque > 0 || throw(ArgumentError("max_torque must be positive"))
    return ReactionWheel(String(name), a ./ n, Float64(max_torque))
end

wrench_column(w::ReactionWheel) = vcat(zeros(3), w.axis)

Base.show(io::IO, w::ReactionWheel) =
    print(io, "ReactionWheel(\"", w.name, "\" axis=", round.(w.axis; digits=3),
          " max=", w.max_thrust, "Nm)")

# ---------------------------------------------------------------------------
# Vehicle
# ---------------------------------------------------------------------------

"""
    Vehicle(name, actuators; mass=NaN, inertia=I(3), center_of_mass=zeros(3))

A rigid body and its set of actuators. Carrying the actuators in one object
makes the rest of the API read naturally:

    allocation_matrix(vehicle)
    allocate(vehicle, τ; method=:qp)
    diagnostics(vehicle)
    describe(vehicle)

`mass`, `inertia` (3×3) and `center_of_mass` are stored for reference and for
future dynamics work; allocation itself only needs the actuator geometry.
Actuator positions are taken to be expressed about `center_of_mass`.
"""
struct Vehicle
    name::String
    actuators::Vector{AbstractActuator}
    mass::Float64
    inertia::Matrix{Float64}
    center_of_mass::SVec3
end

function Vehicle(name::AbstractString, actuators::AbstractVector;
                 mass::Real=NaN,
                 inertia::AbstractMatrix=Matrix{Float64}(I, 3, 3),
                 center_of_mass=zeros(3))
    acts = Vector{AbstractActuator}(actuators)
    isempty(acts) && throw(ArgumentError("a Vehicle needs at least one actuator"))
    size(inertia) == (3, 3) || throw(ArgumentError("inertia must be 3×3"))
    com = collect(Float64, center_of_mass)
    length(com) == 3 || throw(ArgumentError("center_of_mass must have 3 elements"))
    return Vehicle(String(name), acts, Float64(mass), Matrix{Float64}(inertia), com)
end

"Number of actuators on the vehicle."
nthrusters(v::Vehicle) = length(v.actuators)
nactuators(v::Vehicle) = length(v.actuators)

Base.show(io::IO, v::Vehicle) =
    print(io, "Vehicle(\"", v.name, "\", ", length(v.actuators), " actuators)")

# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------

"""
    AllocationResult

Returned by [`allocate`](@ref). Fields:

- `method::Symbol`            : solver used (`:minimum_norm`, `:qp`, …).
- `commands::Vector{Float64}` : command per actuator, `f`.
- `desired::Vector{Float64}`  : requested wrench `τ`.
- `achieved::Vector{Float64}` : `B * commands`, wrench actually produced.
- `residual::Vector{Float64}` : `achieved - desired` (≈0 ⇒ fully realisable).
"""
struct AllocationResult
    method::Symbol
    commands::Vector{Float64}
    desired::Vector{Float64}
    achieved::Vector{Float64}
    residual::Vector{Float64}
end

residual_norm(r::AllocationResult) = norm(r.residual)

function Base.show(io::IO, r::AllocationResult)
    println(io, "AllocationResult (method = :", r.method, ")")
    println(io, "  commands : ", round.(r.commands; digits=4))
    println(io, "  desired  : ", round.(r.desired;  digits=4))
    println(io, "  achieved : ", round.(r.achieved; digits=4))
    print(io,   "  residual : ‖", round(residual_norm(r); digits=6), "‖")
end
