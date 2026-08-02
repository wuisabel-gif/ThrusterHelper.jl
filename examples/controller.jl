# controller.jl — the allocator in a closed loop: PID → allocate → dynamics.
# Shows setpoint tracking, failure tolerance, and under-actuation, all through
# the same `simulate`.
using ThrusterHelper, Printf

v = bluerov_vehicle(; max_thrust = 51.5)
drag = [40.0, 55.0, 60.0, 8.0, 8.0, 6.0]        # per-DOF quadratic drag
sp = [1.0, 0, 0, 0, 0, deg2rad(30)]              # 1 m surge + 30° yaw

res = simulate(v, sp; kp = 8.0, kd = 12.0, drag = drag, duration = 30.0)
@printf("healthy         : surge %.3f m , yaw %5.1f°   (target 1.000 m, 30.0°)\n",
        res.state[1, end], rad2deg(res.state[6, end]))

# Same command with a dead thruster — the over-actuated Heavy still tracks.
resf = simulate(v, sp; kp = 8.0, kd = 12.0, drag = drag, duration = 30.0, failed = [1])
@printf("thruster 1 dead : surge %.3f m , yaw %5.1f°\n",
        resf.state[1, end], rad2deg(resf.state[6, end]))

# The under-actuated standard BlueROV2 cannot hold a pitch setpoint.
vp = bluerov_standard_vehicle(; max_thrust = 51.5)
resp = simulate(vp, [0, 0, 0, 0, deg2rad(20), 0]; kp = 8.0, kd = 12.0,
                drag = drag, duration = 20.0)
@printf("standard ROV    : pitch cmd 20.0° → reached %4.1f°  (uncontrollable)\n",
        rad2deg(resp.state[5, end]))
