# Contributing to ThrusterHelper.jl

Thanks for your interest! Contributions of all sizes are welcome — from typo
fixes to whole new vehicles.

## We're especially looking for: more AUVs / vehicles

ThrusterHelper is built on an `AbstractActuator` / `Vehicle` abstraction, so it
isn't tied to any one robot. The most valuable contributions right now are
**support for more AUVs, ROVs, and other actuator layouts** — the more real
vehicles the package can analyse out of the box, the more useful it is to
everyone. There are two ways to add one:

### 1. A built-in vehicle layout

Add a function to [`src/layouts/`](src/layouts) that returns the thrusters, plus
a `*_vehicle()` wrapper — exactly the way `bluerov_heavy()` / `bluerov_vehicle()`
and `simple_quad()` / `quad_vehicle()` do. For a well-known platform, that's all
it takes:

```julia
function my_rov(; max_thrust = 40.0)
    return [
        Thruster("front-left",  [ 0.2,  0.15, 0.0], [1, 0, 0]; max_thrust),
        # … the rest of your thrusters (position, direction, limit) …
    ]
end
my_rov_vehicle(; kwargs...) = Vehicle("My ROV", my_rov(; kwargs...); mass = 12.0)
```

### 2. A new actuator type

If your actuator isn't a fixed `Thruster` or a `ReactionWheel`, add an
`AbstractActuator` subtype. You only need two methods — everything downstream
(allocation, diagnostics, reachability, C++ export) then works automatically:

- `wrench_column(a) -> Vector{Float64}` — the 6-vector `[Fx, Fy, Fz, τx, τy, τz]`
  a unit command produces.
- `command_limits(a) -> (lo, hi)` — the command bounds.

Not sure how to model a vehicle? **Open an issue** with its thruster
positions, directions, and thrust limits, and we'll help turn it into a layout.

## Development setup

```bash
git clone https://github.com/wuisabel-gif/ThrusterHelper.jl
cd ThrusterHelper.jl
julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. -e 'using Pkg; Pkg.test()'
```

## Guidelines

- **Keep the core dependency-light.** It depends only on the standard library
  (`LinearAlgebra`, `Printf`, `Random`). Anything heavier (e.g. plotting) goes
  behind a package extension, the way `Plots` does.
- **Add a test.** New behaviour should come with a case in `test/runtests.jl`.
- **Match the surrounding style** — short, readable functions with docstrings.
- Run `Pkg.test()` (and, for a new layout, a quick `report(diagnostics(...))`)
  before opening a PR.

## Pull requests

1. Fork and create a feature branch.
2. Make your change, with tests and docstrings.
3. Open a PR describing what changed and why. CI runs on Julia 1.9, LTS, and the
   current stable release.

By participating you agree to abide by our
[Code of Conduct](CODE_OF_CONDUCT.md).
