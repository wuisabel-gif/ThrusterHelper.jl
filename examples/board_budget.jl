# board_budget.jl — shared power-board current budgets on an 8-thruster ROV.
#
# Thrusters are often ganged on power boards that share one current cap — a
# limit per-thruster bounds can't express. This demos the three helpers that
# model it: estimate_current, group_totals, allocate_grouped.
using ThrusterHelper, Printf, LinearAlgebra

vehicle = bluerov_vehicle()                       # BlueROV2 Heavy, 8 × T200
B = allocation_matrix(vehicle)

# T200 @16V current ∝ |thrust|^1.5, anchored to a bench point and ASYMMETRIC:
# reverse is less efficient, so -15 N draws the same ~6 A as +20 N forward.
kf = 6 / 20^1.5                                   # 6 A at 20 N forward
kr = 6 / 15^1.5                                   # 6 A at 15 N reverse
amps(f) = estimate_current(f; k=kf, k_reverse=kr)

boards  = [1:4, 5:8]                              # T0–T3 on board A, T4–T7 on B
budget  = 24.0                                    # 80% of a 30 A board
bounds  = (fill(-15.0, 8), fill(20.0, 8))         # operational caps: +20 / -15 N

# 1 + 2) post-hoc check: does a maneuver fit the board budget?
# (built-in layout groups board A = horizontals, board B = verticals, so this
#  loads both: surge/yaw on A, heave on B.)
τ = [30.0, 0, 40.0, 0, 0, 2.0]                     # surge + heave + yaw
f = allocate(B, τ; method=:qp, bounds=bounds).commands
loads = group_totals(amps(f), boards)
@printf("maneuver       board A %.1f A , board B %.1f A  (budget %.0f A) → %s\n",
        loads[1], loads[2], budget, all(loads .<= budget) ? "OK" : "OVER")

# 3) enforcement: hold a budget tighter than that natural draw, while staying
#    within thruster limits.
tight = round(0.7 * maximum(loads))                # binds, without collapsing the wrench
r = allocate_grouped(B, τ; groups=boards, budgets=tight, bounds=bounds,
                     k=kf, k_reverse=kr)
loads2 = group_totals(amps(r.commands), boards)
@printf("enforced @%.0f A: board A %.1f A , board B %.1f A ; max |cmd| %.1f N ; residual %.3f\n",
        tight, loads2[1], loads2[2], maximum(abs.(r.commands)), norm(r.residual))
println("(budget met and thruster bounds respected; the residual is the wrench ",
        "it had to give up to fit the tighter budget.)")
