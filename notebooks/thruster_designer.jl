### A Pluto.jl notebook ###
# v0.20.3

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    #! format: off
    return quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
    #! format: on
end

# ╔═╡ 11111111-0000-0000-0000-000000000001
md"""
# 🚀 ThrusterHelper: interactive layout designer

Drag the sliders to re-aim and resize the thrusters, or toggle a thruster
**failed**, and watch the design metrics (**rank**, **condition number κ**,
**manipulability**, **control authority**) recompute live.

Then scroll to **"Model your own vehicle"** and paste your sub's geometry in as a
table to see *its* diagnostics.

> Convention: `x` forward, `y` left, `z` up. Directions are given as
> `(azimuth, elevation)` in degrees.
"""

# ╔═╡ 11111111-0000-0000-0000-000000000002
begin
    using ThrusterHelper
    using Plots
    using PlutoUI
end

# ╔═╡ 11111111-0000-0000-0000-000000000003
md"## 1. Live tuning: a vectored 8-thruster layout"

# ╔═╡ 11111111-0000-0000-0000-000000000004
const THRUSTER_NAMES = [
    "front-right-horiz", "front-left-horiz", "back-right-horiz", "back-left-horiz",
    "front-right-vert",  "front-left-vert",  "back-right-vert",  "back-left-vert",
]

# ╔═╡ 11111111-0000-0000-0000-000000000005
@bind geom PlutoUI.combine() do Child
    md"""
    **Arm** (m): $(Child(:arm,  PlutoUI.Slider(0.05:0.01:0.50, default=0.22, show_value=true)))

    **Span** (m): $(Child(:span, PlutoUI.Slider(0.00:0.01:0.30, default=0.12, show_value=true)))

    **Cant** (°): $(Child(:cant, PlutoUI.Slider(10:1:80,        default=45,   show_value=true)))
    """
end

# ╔═╡ 11111111-0000-0000-0000-000000000006
md"**Fail a thruster:** $(@bind failed_names PlutoUI.MultiCheckBox(THRUSTER_NAMES))"

# ╔═╡ 11111111-0000-0000-0000-000000000007
function build_vehicle(arm, span, cant_deg)
    θ = deg2rad(cant_deg)
    ch, sh = cos(θ), sin(θ)
    # sx: +1 front / −1 back ; sy: +1 right (y<0) / −1 left (y>0)
    horiz = [
        Thruster("front-right-horiz", [ arm, -arm, 0.0], [ ch,  sh, 0.0]; max_thrust=51.5),
        Thruster("front-left-horiz",  [ arm,  arm, 0.0], [ ch, -sh, 0.0]; max_thrust=51.5),
        Thruster("back-right-horiz",  [-arm, -arm, 0.0], [-ch,  sh, 0.0]; max_thrust=51.5),
        Thruster("back-left-horiz",   [-arm,  arm, 0.0], [-ch, -sh, 0.0]; max_thrust=51.5),
    ]
    vert = [
        Thruster("front-right-vert",  [ arm, -arm, span], [0.0, 0.0, 1.0]; max_thrust=51.5),
        Thruster("front-left-vert",   [ arm,  arm, span], [0.0, 0.0, 1.0]; max_thrust=51.5),
        Thruster("back-right-vert",   [-arm, -arm, span], [0.0, 0.0, 1.0]; max_thrust=51.5),
        Thruster("back-left-vert",    [-arm,  arm, span], [0.0, 0.0, 1.0]; max_thrust=51.5),
    ]
    return Vehicle("Designer", vcat(horiz, vert); mass=11.0)
end

# ╔═╡ 11111111-0000-0000-0000-000000000008
begin
    vehicle = build_vehicle(geom.arm, geom.span, geom.cant)
    failed = findall(in(failed_names), THRUSTER_NAMES)
    Bfail = copy(allocation_matrix(vehicle));  Bfail[:, failed] .= 0
    diag = diagnostics(Bfail)
end

# ╔═╡ 11111111-0000-0000-0000-000000000009
function diagnostics_panel(d)
    dofnames = ["Fx", "Fy", "Fz", "τx", "τy", "τz"]
    lost = dofnames[.!d.controllable]
    ok = d.rank == 6
    md"""
    | metric | value |
    |---|---|
    | **rank** | $(d.rank) / 6 $(ok ? "✅ fully actuated" : "⚠️ under-actuated") |
    | **condition number κ** | $(isfinite(d.condition_number) ? round(d.condition_number; digits=2) : "∞") |
    | **manipulability** | $(round(d.manipulability; sigdigits=4)) |
    | **redundancy** | $(d.redundancy) |
    | **weakest control authority** | $(round(d.weakest_gain; sigdigits=3)) |
    | **uncontrollable DOFs** | $(isempty(lost) ? "none" : join(lost, ", ")) |
    """
end

# ╔═╡ 11111111-0000-0000-0000-000000000010
diagnostics_panel(diag)

# ╔═╡ 11111111-0000-0000-0000-000000000011
plot(
    plot_vehicle(vehicle; failed=failed, view=:xy),
    plot_vehicle(vehicle; failed=failed, view=:xz);
    size=(820, 380), layout=(1, 2),
)

# ╔═╡ 11111111-0000-0000-0000-000000000012
md"""
## 2. Model your own vehicle

Paste your thrusters below, one per line, as
`x, y, z, azimuth°, elevation°, max_thrust`. Lines starting with `#` are ignored.
The diagnostics update as you type.
"""

# ╔═╡ 11111111-0000-0000-0000-000000000013
@bind spec_text PlutoUI.TextField((60, 10), default = """
# x,     y,     z,    az,   el,  max_thrust   (BlueROV2 Heavy)
 0.22, -0.22, 0.00,   45,   0,   51.5
 0.22,  0.22, 0.00,  135,   0,   51.5
-0.22, -0.22, 0.00,  -45,   0,   51.5
-0.22,  0.22, 0.00, -135,   0,   51.5
 0.22, -0.22, 0.12,    0,  90,   51.5
 0.22,  0.22, 0.12,    0,  90,   51.5
-0.22, -0.22, 0.12,    0,  90,   51.5
-0.22,  0.22, 0.12,    0,  90,   51.5
""")

# ╔═╡ 11111111-0000-0000-0000-000000000014
function parse_spec(text)
    thr = Thruster[]
    for (k, raw) in enumerate(split(text, '\n'))
        line = strip(raw)
        (isempty(line) || startswith(line, "#")) && continue
        vals = parse.(Float64, strip.(split(line, ',')))
        length(vals) == 6 || error("line $k: need 6 values (x,y,z,az,el,max_thrust), got $(length(vals))")
        x, y, z, az, el, mt = vals
        dir = [cosd(el)*cosd(az), cosd(el)*sind(az), sind(el)]
        push!(thr, Thruster("t$k", [x, y, z], dir; max_thrust=mt))
    end
    return Vehicle("Your vehicle", thr)
end

# ╔═╡ 11111111-0000-0000-0000-000000000015
myvehicle = parse_spec(spec_text)

# ╔═╡ 11111111-0000-0000-0000-000000000016
diagnostics_panel(diagnostics(myvehicle))

# ╔═╡ 11111111-0000-0000-0000-000000000017
plot(
    plot_vehicle(myvehicle; view=:xy),
    plot_vehicle(myvehicle; view=:xz);
    size=(820, 380), layout=(1, 2),
)

# ╔═╡ Cell order:
# ╟─11111111-0000-0000-0000-000000000001
# ╠═11111111-0000-0000-0000-000000000002
# ╟─11111111-0000-0000-0000-000000000003
# ╟─11111111-0000-0000-0000-000000000004
# ╠═11111111-0000-0000-0000-000000000005
# ╠═11111111-0000-0000-0000-000000000006
# ╠═11111111-0000-0000-0000-000000000007
# ╠═11111111-0000-0000-0000-000000000008
# ╠═11111111-0000-0000-0000-000000000009
# ╟─11111111-0000-0000-0000-000000000010
# ╟─11111111-0000-0000-0000-000000000011
# ╟─11111111-0000-0000-0000-000000000012
# ╠═11111111-0000-0000-0000-000000000013
# ╠═11111111-0000-0000-0000-000000000014
# ╠═11111111-0000-0000-0000-000000000015
# ╟─11111111-0000-0000-0000-000000000016
# ╟─11111111-0000-0000-0000-000000000017
