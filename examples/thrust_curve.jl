# thrust_curve.jl — map allocated forces to real actuator commands (PWM).
# Allocation is force-based; a ThrustCurve turns each force into the PWM the
# thruster actually needs, accounting for the T200's nonlinear, asymmetric curve.
using ThrusterHelper, Printf

tc = t200_curve()                                   # BlueRobotics T200 @ 16 V
# BlueROV2 Heavy geometry, but give every thruster a real T200 curve + limit.
thr = [Thruster(t.name, t.position, t.direction; max_thrust = 51.5, curve = tc)
       for t in bluerov_heavy()]
v = Vehicle("BlueROV2 Heavy (T200 curves)", thr)

τ = [40.0, 0, 0, 0, 0, 8.0]                         # surge 40 N + yaw 8 N·m
r = allocate(v, τ; method = :qp, bounds = command_bounds(v))
pwm = actuator_commands(r, v.actuators)             # forces → PWM µs

@printf("%-18s %10s %10s\n", "thruster", "force[N]", "PWM[µs]")
println("-"^40)
for (a, f, p) in zip(v.actuators, r.commands, pwm)
    @printf("%-18s %10.2f %10.0f\n", label(a), f, p)
end

# Note the asymmetry: a reverse force needs less PWM travel from neutral than the
# same-magnitude forward force, because the T200 is stronger forward.
