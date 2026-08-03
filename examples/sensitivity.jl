# sensitivity.jl — which thruster should I re-aim, and how much does it matter?
# The gradient of a design metric w.r.t. each thruster's (azimuth, elevation)
# ranks the layout's levers before you spend an optimiser run on them.
using ThrusterHelper

vehicle = bluerov_vehicle(; arm=0.15)

# Finite-difference gradient of manipulability — always available, any metric.
g = sensitivity(vehicle; objective=:manipulability)
println("d(manipulability)/d(angle), 2 params (az, el) per thruster:")
for (i, t) in enumerate(vehicle.actuators)
    println("  $(label(t)):  daz = $(round(g[2i-1]; digits=4))  del = $(round(g[2i]; digits=4))")
end

# Exact gradient via ForwardDiff (manipulability, directions only) — matches
# the finite-difference result but with no step-size error.
using ForwardDiff
g_ad = sensitivity(vehicle; objective=:manipulability, method=:ad)
println("\nAD vs finite-difference max abs error: ", round(maximum(abs.(g_ad .- g)); sigdigits=3))
