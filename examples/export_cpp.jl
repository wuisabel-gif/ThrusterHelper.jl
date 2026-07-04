# export_cpp.jl — generate a real-time-safe C++ header for any vehicle.
# Not tied to one project: swap in your own Vehicle/thruster layout and this
# still produces a self-contained, dependency-free allocation header.
using ThrusterHelper

vehicle = bluerov_vehicle()
src = export_cpp(vehicle; path = joinpath(@__DIR__, "thruster_allocation.hpp"))
println("wrote examples/thruster_allocation.hpp (", length(vehicle.actuators), " actuators)")
println("\npreview:\n")
println(join(split(src, "\n")[1:20], "\n"))
