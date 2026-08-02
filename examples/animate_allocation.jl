# animate_allocation.jl — the core loop, animated: a rotating surge/sway force
# command (blue arrow), with each thruster's response solved live by :qp and
# drawn as a green/red force vector. Requires `using Plots`. Produces a GIF.
using ThrusterHelper
using Plots

ENV["GKSwstype"] = "100"           # headless GR (no display needed)

thr = simple_quad(; arm = 0.25)    # 4 horizontal thrusters — clean top-down view
frames = 60

anim = @animate for k in 0:frames-1
    θ = 2π * k / frames
    τ = [cos(θ), sin(θ), 0.0, 0.0, 0.0, 0.0]          # unit force, rotating in xy
    r = allocate(thr, τ; method = :qp, bounds = 1.0)   # respect ±1 N limits

    plt = plot_thrusters(thr; commands = r.commands, view = :xy,
                         size = (560, 560), arrowscale = 0.18)
    # bold arrow = the wrench the "controller" asked for
    Plots.quiver!(plt, [0.0], [0.0]; quiver = ([0.30cos(θ)], [0.30sin(θ)]),
                  color = :royalblue, linewidth = 5)
    Plots.title!(plt, "allocate(surge/sway @ $(round(Int, rad2deg(θ)))°)  ·  :qp within ±1 N")
end

out = joinpath(@__DIR__, "allocation_demo.gif")
gif(anim, out; fps = 20)
println("wrote ", out)
