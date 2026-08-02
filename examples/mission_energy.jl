# mission_energy.jl — total energy of a wrench sequence, per allocation method.
# For a fixed sequence, :minimum_power is already the global optimum; this
# quantifies and compares it (and flags steps that saturate).
using ThrusterHelper

v = bluerov_vehicle(; max_thrust = 51.5)

# a demanding, asymmetric mission: hard diagonal push, hard surge, gentle turn.
mission = [[60.0, 25, 40, 0, 0, 10.0],
           [90.0,  0,  0, 0, 0,  0.0],
           [20.0, 20, 20, 3, 3,  3.0]]

# Nominal battery: :minimum_power is the cheapest method (it minimises Σ|f|^1.5,
# which is the energy metric) — nothing saturates.
println("Nominal (16 V):")
report(mission_energy(v, mission; dt = 1.0, idle = 0.3))

# End-of-battery: derated limits mean the limit-ignoring methods now demand
# impossible thrust (saturated steps flagged), while :qp stays feasible but
# can't fully meet the wrench (so its energy is lower — it's giving up, not saving).
println("\nEnd-of-battery (11 V) — same mission:")
report(mission_energy(v, mission; dt = 1.0, idle = 0.3, voltage = 11))
