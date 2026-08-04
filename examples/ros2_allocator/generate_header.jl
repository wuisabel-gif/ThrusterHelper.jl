# generate_header.jl
# Regenerate include/thruster_allocation.hpp for YOUR vehicle.
#
#   julia --project=/path/to/ThrusterHelper.jl generate_header.jl
#
# Edit the `vehicle` below to match your layout (or build it from your own
# Thrusters), then rebuild the ROS 2 package with `colcon build`.

using ThrusterHelper

vehicle = bluerov_vehicle()   # <-- replace with your Vehicle

here = @__DIR__
export_cpp(vehicle;
           method = :minimum_norm,     # or :weighted (pass weights=...)
           saturate = true,            # direction-preserving clamp to max_thrust
           namespace = "thruster_allocation",
           path = joinpath(here, "include", "thruster_allocation.hpp"))

println("Wrote include/thruster_allocation.hpp for '", vehicle.name, "' (",
        nactuators(vehicle), " actuators)")
