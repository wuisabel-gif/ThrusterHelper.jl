# demo.jl — a five-minute tour of Thruster Helper, one section per feature.
# Run: julia --project=. examples/demo.jl
using ThrusterHelper

println("="^60, "\n1. Build a vehicle and look at its geometry\n", "="^60)
vehicle = bluerov_vehicle()
describe(vehicle)

println("\n", "="^60, "\n2. Allocate a command: forward + yaw right\n", "="^60)
report(allocate(vehicle, [1.0, 0, 0, 0, 0, 0.5]; method=:qp); actuators=vehicle.actuators)

println("\n", "="^60, "\n3. Is the design any good? (SVD diagnostics)\n", "="^60)
report(diagnostics(vehicle))

println("\n", "="^60, "\n4. Can it actually do this? (reachability)\n", "="^60)
report(reachable(vehicle, [4.0, 0, 0, 0, 0, 0]))   # beyond the ±1 N thruster limit

println("\n", "="^60, "\n5. What if a thruster dies? (failure criticality)\n", "="^60)
report_failures(rank_failures(vehicle)[1:3])

println("\n", "="^60, "\n6. Which solver should I use? (method comparison)\n", "="^60)
report(compare_methods(vehicle, [1.0, 0.3, 0, 0, 0, 0.4]))

println("\n", "="^60, "\n7. Design, not just analyse: optimise a cramped layout\n", "="^60)
report(optimize_layout(bluerov_vehicle(; arm=0.1); objective=:condition_number, restarts=2))

println("\n", "="^60, "\n8. Ship it: generate a real-time-safe C++ header\n", "="^60)
export_cpp(vehicle; path=joinpath(@__DIR__, "thruster_allocation.hpp"))
println("wrote examples/thruster_allocation.hpp — no Julia dependency at runtime")

println("\nThat's the whole loop: geometry → allocate → diagnose → optimise → ship.")
println("Optional: `using Plots` unlocks plot_thrusters / plot_manipulability.")
