using ThrusterHelper
using LinearAlgebra
using Test

@testset "ThrusterHelper.jl" begin

    # -- geometry / types ---------------------------------------------------
    @testset "Thruster construction + immutability" begin
        t = Thruster("a", [1.0, 0.0, 0.0], [0.0, 2.0, 0.0])
        @test norm(t.direction) ≈ 1.0
        @test t.direction ≈ [0.0, 1.0, 0.0]
        @test t.max_thrust == 1.0
        @test !ismutabletype(Thruster)                 # geometry is immutable
        @test_throws ArgumentError Thruster("z", [0,0,0], [0,0,0])
        @test_throws ArgumentError Thruster("z", [0,0], [1,0,0])
        @test_throws ArgumentError Thruster(; name="z", position=[0,0,0],
                                            direction=[1,0,0], max_thrust=-1)
    end

    @testset "skew == cross product" begin
        v = [1.0, 2.0, 3.0]; w = [-4.0, 5.0, 6.0]
        @test skew(v) * w ≈ cross(v, w)
        @test skew(v) ≈ -skew(v)'                       # skew-symmetric
    end

    @testset "wrench column = [dir; pos×dir]" begin
        t = Thruster("x", [0.0, 1.0, 0.0], [1.0, 0.0, 0.0])
        @test column(t) ≈ [1.0, 0, 0, 0, 0, -1.0]       # (0,1,0)×(1,0,0)=(0,0,-1)
        @test force_contribution(t) ≈ [1.0, 0, 0]
        @test torque_contribution(t) ≈ [0, 0, -1.0]
    end

    @testset "ReactionWheel = pure torque" begin
        w = ReactionWheel("rw-z", [0.0, 0.0, 2.0]; max_torque=3.0)
        @test column(w) ≈ [0, 0, 0, 0, 0, 1.0]          # unit axis, no force
        @test w.max_thrust == 3.0
    end

    # -- allocation matrix --------------------------------------------------
    @testset "Allocation matrix shape + known BlueROV values" begin
        thr = bluerov_heavy()
        B = allocation_matrix(thr)
        @test size(B) == (6, 8)
        c = sqrt(2)/2
        @test B[1:3, 1] ≈ [c, c, 0.0]                   # front-right-horiz force
        @test B[1:3, 5] ≈ [0.0, 0.0, 1.0]               # front-right-vert force
        @test rank(B) == 6
        @test_throws ArgumentError allocation_matrix(Thruster[])
    end

    @testset "Vehicle wraps actuators" begin
        v = bluerov_vehicle()
        @test nactuators(v) == 8
        @test allocation_matrix(v) == allocation_matrix(v.actuators)
        @test v.mass == 11.0
    end

    # -- solvers ------------------------------------------------------------
    @testset "Forward/inverse consistency, all solvers (full rank)" begin
        thr = bluerov_heavy()
        B = allocation_matrix(thr)
        for τ in ([1.0,0,0,0,0,0], [0,0,0,0,0,0.5], [0.5,-0.3,0.2,0.0,0.0,0.1])
            for m in (:minimum_norm, :pinv, :minimum_power)
                r = allocate(B, τ; method=m)
                @test r.achieved ≈ τ atol=1e-7
            end
            rq = allocate(thr, τ; method=:qp)           # default bounds = ±1
            @test rq.achieved ≈ τ atol=1e-5
        end
    end

    @testset ":minimum_norm == pinv solution" begin
        thr = bluerov_heavy(); B = allocation_matrix(thr)
        τ = [1.0,0,0,0,0,0.3]
        @test allocate(B, τ; method=:minimum_norm).commands ≈ pinv(B)*τ atol=1e-9
    end

    @testset ":weighted discourages a thruster" begin
        thr = bluerov_heavy(); B = allocation_matrix(thr)
        τ = [1.0,0,0,0,0,0]
        w = ones(8); w[1] = 100.0
        rw = allocate(B, τ; method=:weighted, weights=w)
        rn = allocate(B, τ; method=:minimum_norm)
        @test abs(rw.commands[1]) < abs(rn.commands[1]) + 1e-9
        @test rw.achieved ≈ τ atol=1e-7
        @test_throws ArgumentError allocate(B, τ; method=:weighted)   # needs weights
    end

    @testset ":minimum_power spreads load vs min-norm" begin
        thr = bluerov_heavy(); B = allocation_matrix(thr)
        τ = [1.0,0,0,0,0,0.3]
        rp = allocate(B, τ; method=:minimum_power, p=1.5)
        @test rp.achieved ≈ τ atol=1e-6                 # still hits the command
        @test total_power(rp.commands) <= total_power(allocate(B,τ).commands) + 1e-6
    end

    @testset ":qp respects bounds" begin
        thr = bluerov_heavy(); B = allocation_matrix(thr)
        τ = [3.0, 0, 0, 0, 0, 0]                        # too big for ±1 limits
        rq = allocate(B, τ; method=:qp, bounds=1.0)
        @test maximum(abs.(rq.commands)) <= 1.0 + 1e-6  # never exceeds the box
        @test norm(rq.residual) > 0                     # can't fully achieve it
        # within bounds, qp matches the unconstrained optimum
        rqsmall = allocate(B, [0.2,0,0,0,0,0]; method=:qp, bounds=10.0)
        @test rqsmall.achieved ≈ [0.2,0,0,0,0,0] atol=1e-5
    end

    @testset "unknown method errors" begin
        @test_throws ArgumentError allocate(allocation_matrix(bluerov_heavy()),
                                            zeros(6); method=:nope)
    end

    # -- constraints --------------------------------------------------------
    @testset "Saturation + scaling" begin
        f = [0.5, -1.5, 2.0, -0.2]
        fs, sat = saturate(f, 1.0)
        @test fs == [0.5, -1.0, 1.0, -0.2]
        @test sat == [false, true, true, false]
        fsc, factor = scale_to_limits(f, 1.0)
        @test factor ≈ 0.5
        @test maximum(abs.(fsc)) ≈ 1.0
        @test fsc ≈ f .* factor                          # direction preserved
    end

    @testset "Failure zeros columns" begin
        thr = bluerov_heavy(); B = allocation_matrix(thr)
        Bf = apply_failures(B, [1, 5])
        @test all(Bf[:, 1] .== 0) && all(Bf[:, 5] .== 0)
        @test Bf[:, 2] == B[:, 2]
        @test failed_indices([1,5], 8) == [1, 5]
        @test failed_indices(Bool[1,0,0,0,1,0,0,0], 8) == [1, 5]
        @test norm(allocate(Bf, [1.0,0,0,0,0,0]).residual) < 1e-6   # forward survives
    end

    @testset "ThrustCurve force↔command mapping" begin
        tc = t200_curve()
        # invertible: command → force → command round-trips
        for pwm in (1150, 1400, 1500, 1650, 1850)
            f = command_to_force(tc, pwm)
            @test force_to_command(tc, f) ≈ pwm rtol=1e-6
        end
        @test command_to_force(tc, 1500) ≈ 0.0 atol=1e-9        # neutral ≈ no thrust
        @test command_to_force(tc, 1900) > -command_to_force(tc, 1100)  # forward > reverse (asymmetric)
        # clamps outside the sampled range (saturation)
        @test force_to_command(tc, 1e6) == 1900
        @test force_to_command(tc, -1e6) == 1100
        # construction validation
        @test_throws ArgumentError ThrustCurve([0.0, 0.0], [1500.0, 1600.0])   # force not increasing
        @test_throws ArgumentError ThrustCurve([-1.0, 1.0], [1500.0])          # length mismatch
    end

    @testset "Thruster carries an optional ThrustCurve" begin
        tc = t200_curve()
        t = Thruster("t", [0.1,0,0], [1,0,0]; max_thrust=51.5, curve=tc)
        @test thrust_curve(t) === tc
        @test thrust_curve(Thruster("u", [0.1,0,0], [1,0,0])) === nothing      # default: linear
        @test thrust_curve(ReactionWheel("rw", [0,0,1.0])) === nothing
        # allocation is unchanged (force-based); a curve doesn't alter B
        @test allocation_matrix([t]) == allocation_matrix([Thruster("t",[0.1,0,0],[1,0,0])])
        # actuator_commands maps allocated force → PWM (curve) or passes force through (no curve)
        thr = [Thruster("a",[0.2,0,0],[1,0,0]; max_thrust=51.5, curve=tc),
               Thruster("b",[0.2,0,0],[1,0,0]; max_thrust=51.5)]
        cmds = actuator_commands([28.4, 3.0], thr)
        @test cmds[1] ≈ 1700 rtol=1e-6        # 28.4 N → 1700 µs via the T200 curve
        @test cmds[2] == 3.0                   # no curve → passthrough (force in N)
    end

    @testset "Battery voltage derating" begin
        tc = t200_curve()
        lo = derate(tc; from=16, to=12)                         # (12/16)^2 = 0.5625
        @test command_to_force(lo, 1900) ≈ command_to_force(tc, 1900) * (12/16)^2 rtol=1e-9
        @test command_to_force(lo, 1900) < command_to_force(tc, 1900)   # less thrust when sagging
        @test derate(tc; from=16, to=16).force ≈ tc.force              # no sag ⇒ unchanged
        # derated vehicle: limits shrink, so the authority envelope shrinks
        v = bluerov_vehicle()                                  # ±1 N thrusters
        vd = derate(v; from=16, to=12)
        @test all(a.max_thrust ≈ 0.5625 for a in vd.actuators)
        big = [1e4, 0, 0, 0, 0, 0]
        surge16 = (allocation_matrix(v)  * allocate(v,  big; method=:qp, bounds=command_bounds(v)).commands)[1]
        surge12 = (allocation_matrix(vd) * allocate(vd, big; method=:qp, bounds=command_bounds(vd)).commands)[1]
        @test surge12 < surge16                                 # less surge authority at 12 V
        # estimate_current: same thrust at lower voltage draws more current
        @test estimate_current([20.0]; k=1.0, voltage=12, ref_voltage=16)[1] ≈
              estimate_current([20.0]; k=1.0)[1] * (16/12) rtol=1e-9
    end

    @testset "Power model" begin
        @test estimate_power([2.0]; p=1.5)[1] ≈ 2.0^1.5
        @test estimate_power([0.0]; idle=3.0)[1] ≈ 3.0
        @test all(estimate_power([-1.0, 1.0]) .≈ 1.0)
        @test total_power([1.0, 1.0]) ≈ 2.0
    end

    @testset "estimate_current + group_totals" begin
        # symmetric default mirrors the power law
        @test estimate_current([2.0]; k=1.0)[1] ≈ 2.0^1.5
        # reverse draws more per unit thrust; forward unaffected
        @test estimate_current([-2.0]; k=1.0, k_reverse=3.0)[1] ≈ 3.0 * 2.0^1.5
        @test estimate_current([2.0];  k=1.0, k_reverse=3.0)[1] ≈ 2.0^1.5
        # T200 anchors: 20 N fwd and 15 N rev both ≈ 6 A
        kf = 6/20^1.5; kr = 6/15^1.5
        I = estimate_current([20.0, -15.0]; k=kf, p=1.5, k_reverse=kr)
        @test I[1] ≈ 6.0 && I[2] ≈ 6.0
        # group_totals sums per group; accepts ranges and index vectors
        x = collect(1.0:8.0)
        @test group_totals(x, [1:4, 5:8]) == [10.0, 26.0]
        @test group_totals(x, [[1, 3], [2, 4]]) == [4.0, 6.0]
        # shared-budget check reads as one line
        @test all(group_totals(estimate_current(fill(20.0, 8); k=kf, k_reverse=kr),
                               [1:4, 5:8]) .<= 24)
    end

    @testset "allocate_grouped enforces a binding budget in-bounds" begin
        B = allocation_matrix(bluerov_heavy())
        τ = [2.0, 0, 0, 0, 0, 0]
        groups = [1:4, 5:8]
        # budget set below the unconstrained board draw ⇒ the constraint binds
        plain = allocate(B, τ; method=:qp, bounds=3.0)
        budget = 0.6 * maximum(group_totals(estimate_current(plain.commands), groups))
        r = allocate_grouped(B, τ; groups=groups, budgets=budget, bounds=3.0)
        @test all(group_totals(estimate_current(r.commands), groups) .<= budget + 1e-6)
        @test maximum(abs.(r.commands)) <= 3.0 + 1e-6      # box always respected
        # a budget the plain solution already meets ⇒ returns it unchanged
        loose = allocate_grouped(B, τ; groups=groups, budgets=1e6, bounds=3.0)
        @test loose.commands ≈ plain.commands atol=1e-9
    end

    # -- diagnostics --------------------------------------------------------
    @testset "Diagnostics: full-rank BlueROV" begin
        d = diagnostics(bluerov_heavy())
        @test d.rank == 6
        @test d.redundancy == 2                          # 8 thrusters − 6 DOF
        @test all(d.controllable)
        @test isfinite(d.condition_number)
        @test d.manipulability > 0
        @test length(d.singular_values) == 6
    end

    @testset "Diagnostics: under-actuated quad loses DOFs" begin
        d = diagnostics(simple_quad())
        @test d.rank == 3
        @test d.controllable[1] && d.controllable[2] && d.controllable[6]  # surge/sway/yaw
        @test !d.controllable[3] && !d.controllable[4] && !d.controllable[5]  # heave/roll/pitch
        @test d.condition_number == Inf || !d.controllable[3]
        # demanding heave is impossible
        @test norm(allocate(simple_quad(), [0,0,1.0,0,0,0]).residual) > 0.1
        info = controllable_dofs(simple_quad())
        @test info.rank == 3 && info.labels[3] == "Fz"
    end

    @testset "Diagnostics: rank-0 (all-dead) matrix does not crash" begin
        B = allocation_matrix(bluerov_heavy())
        dead = diagnostics(apply_failures(B, 1:8))        # every actuator failed
        @test dead.rank == 0
        @test dead.redundancy == 8
        @test all(.!dead.controllable)                     # nothing reachable
        @test dead.condition_number == Inf
        @test dead.manipulability == 0.0
        @test dead.weakest_gain == 0.0
        # monte_carlo with failures used to hit the same σ[0] BoundsError
        @test monte_carlo(bluerov_vehicle(); failure_prob=0.6, samples=300,
                          seed=7).full_rank_prob ≥ 0.0
        # single-actuator vehicle: rank_failures zeros B
        solo = Vehicle("solo", [Thruster("t", [0.1, 0, 0], [1.0, 0, 0])])
        @test rank_failures(solo)[1].rank_after == 0
    end

    @testset "condition_number is Inf on rank loss (agrees with cond)" begin
        # Rank-deficient design: κ must be Inf, matching the field docstring
        # and the re-exported cond(B) — not a finite range-only conditioning.
        Bq = allocation_matrix(simple_quad())            # rank 3
        dq = diagnostics(Bq)
        @test dq.rank == 3
        @test dq.condition_number == Inf
        @test cond(Bq) == Inf                            # they now agree
        # Full-rank design still reports a finite, sensible κ
        df = diagnostics(bluerov_heavy())
        @test df.rank == 6
        @test isfinite(df.condition_number)
        @test df.condition_number ≈ cond(allocation_matrix(bluerov_heavy())) rtol=1e-8
    end

    @testset "Standard BlueROV2 (6-thruster, under-actuated in pitch)" begin
        v = bluerov_standard_vehicle()
        @test nactuators(v) == 6
        d = diagnostics(v)
        @test d.rank == 5                                  # between quad (3) and Heavy (6)
        # surge/sway/heave/roll/yaw controllable; pitch (τy) is not
        @test d.controllable == Bool[1, 1, 1, 1, 0, 1]
        @test !d.controllable[5]
        # demanding pitch is impossible
        @test norm(allocate(v, [0,0,0, 0,0.3,0]).residual) > 0.1
        # sanity: the Heavy adds the missing DOF
        @test diagnostics(bluerov_vehicle()).rank == 6
    end

    @testset "CubeSat 4-wheel pyramid (spacecraft layout)" begin
        sat = cubesat_vehicle()
        @test nactuators(sat) == 4
        B = allocation_matrix(sat)
        @test all(B[1:3, :] .== 0)                       # wheels make no force
        d = diagnostics(sat)
        @test d.rank == 3 && d.redundancy == 1
        @test d.controllable[4] && d.controllable[5] && d.controllable[6]  # τx,τy,τz
        @test !any(d.controllable[1:3])                  # no force control
        @test d.condition_number == Inf                  # full 6-DOF view is rank-deficient
        @test cond(B[4:6, :]) ≈ sqrt(2) rtol=1e-6        # torque block is well-conditioned
        # single-fault tolerant: losing ANY one wheel keeps full 3-axis torque control
        for row in rank_failures(sat)
            @test row.rank_after == 3
            @test isempty(row.lost_dofs)
        end
        # a commanded torque is realised
        r = allocate(sat, [0,0,0, 0,0,0.002]; method=:qp, bounds=command_bounds(sat))
        @test r.achieved[6] ≈ 0.002 atol=1e-6
        @test norm(r.residual) < 1e-6
    end

    @testset "dominant_dof labels a direction" begin
        @test dominant_dof([0.1, 0.0, 0.9, 0.0, 0.0, 0.0]) == "Fz"
        @test dominant_dof([0,0,0,0,0,1.0]) == "τz"
    end

    # -- analysis: reachability --------------------------------------------
    @testset "reachable()" begin
        v = bluerov_vehicle()                            # ±1 N thrusters
        r1 = reachable(v, [1.0, 0, 0, 0, 0, 0])
        @test r1.reachable
        @test r1.max_error < 1e-3
        @test isempty(r1.saturated)
        r2 = reachable(v, [4.0, 0, 0, 0, 0, 0])          # beyond ~2.83 N limit
        @test !r2.reachable
        @test r2.max_error > 0.5
        @test !isempty(r2.saturated)
        @test occursin("UNREACHABLE", r2.reason)
        @test all(abs.(r2.commands) .<= 1.0 + 1e-6)      # stays within limits
    end

    # -- analysis: method comparison ---------------------------------------
    @testset "compare_methods()" begin
        v = bluerov_vehicle()
        c = compare_methods(v, [1.0, 0.3, 0, 0, 0, 0.4])
        @test length(c.rows) == 4
        @test all(haskey(r, :power) && haskey(r, :saturated) for r in c.rows)
        qp = first(r for r in c.rows if r.method == :qp)
        @test qp.residual < 1e-6
        buf = IOBuffer(); report(c; io=buf)
        @test occursin("method comparison", String(take!(buf)))
    end

    # -- analysis: failure criticality -------------------------------------
    @testset "rank_failures()" begin
        rows = rank_failures(bluerov_vehicle())
        @test length(rows) == 8
        @test all(r.rank_before == 6 for r in rows)
        @test all(r.rank_after == 6 for r in rows)       # over-actuated: 1 loss ok
        @test all(isempty(r.lost_dofs) for r in rows)
        # the quad has redundancy 1: a single loss is survivable, but some
        # *pair* of failures costs a DOF.
        qsingle = rank_failures(quad_vehicle())
        @test all(r.rank_after == 3 for r in qsingle)    # redundancy absorbs one loss
        qpairs = rank_failures(quad_vehicle(); pairs=true)
        @test any(r.rank_after < r.rank_before for r in qpairs)
        @test any(!isempty(r.lost_dofs) for r in qpairs)
    end

    # -- analysis: Monte-Carlo robustness ----------------------------------
    @testset "monte_carlo()" begin
        v = bluerov_vehicle()
        m = monte_carlo(v; misalignment_deg=2.0, samples=500, seed=42)
        @test m.samples == 500
        @test length(m.dof_loss_prob) == 6
        @test 0.0 <= m.full_rank_prob <= 1.0
        @test m.full_rank_prob > 0.99                    # small error → stays actuated
        @test m.cond_mean > 0
        # deterministic given the seed
        m2 = monte_carlo(v; misalignment_deg=2.0, samples=500, seed=42)
        @test m.cond_mean == m2.cond_mean
    end

    @testset "monte_carlo warns on dropped non-Thruster actuators" begin
        mixed = Vehicle("mixed", AbstractActuator[bluerov_heavy()...,
                                                  ReactionWheel("rw", [0.0, 0.0, 1.0])])
        @test_logs (:warn,) monte_carlo(mixed; samples=50, seed=1)   # wheel dropped ⇒ warn
        @test_logs monte_carlo(bluerov_vehicle(); samples=50, seed=1)  # all thrusters ⇒ silent
    end

    # -- design optimisation -----------------------------------------------
    @testset "optimize_layout()" begin
        v = bluerov_vehicle(; arm=0.1)                   # cramped → poorly conditioned
        before = diagnostics(v).condition_number
        res = optimize_layout(v; objective=:condition_number, restarts=2, iterations=1500)
        @test res.after.condition_number <= before + 1e-6      # no worse
        @test res.after.condition_number < before              # actually improves
        @test res.after.rank == 6                              # stays fully actuated
        @test res.vehicle isa Vehicle
        # manipulability objective should raise manipulability
        resm = optimize_layout(v; objective=:manipulability, restarts=2, iterations=1500)
        @test resm.after.manipulability >= diagnostics(v).manipulability - 1e-9
    end

    # -- C++ export -----------------------------------------------------------
    @testset "export_cpp()" begin
        v = bluerov_vehicle()
        src = export_cpp(v)
        @test occursin("kNumActuators = 8", src)
        @test occursin("THRUSTER_ALLOCATION_HPP", src)
        @test occursin("namespace thruster_allocation", src)
        @test_throws ArgumentError export_cpp(v; method=:qp)

        # saturate flag: default clamps; saturate=false omits the clamp
        @test occursin("f[i] /= worst", src)                       # default: clamp present
        raw = export_cpp(v; saturate=false)
        @test !occursin("/= worst", raw)                           # no clamp emitted
        @test occursin("raw least-squares", raw)

        cxx = Sys.which("c++")
        if cxx === nothing
            @warn "no C++ compiler found; skipping export_cpp numeric round-trip check"
        else
            mktempdir() do dir
                for (method, weights) in ((:minimum_norm, nothing), (:weighted, [1,1,1,1,2,2,2,2.0]))
                    hpp = joinpath(dir, "alloc.hpp")
                    export_cpp(v; method=method, weights=weights, path=hpp)
                    cpp = joinpath(dir, "main.cpp")
                    write(cpp, """
                    #include "alloc.hpp"
                    #include <cstdio>
                    int main() {
                        std::array<float,6> tau1{1.0f,0,0,0,0,0.5f};
                        std::array<float,6> tau2{0,0,1.0f,0.3f,0,0};
                        for (auto tau : {tau1, tau2}) {
                            auto f = thruster_allocation::allocate(tau);
                            for (float v : f) std::printf("%.9f ", v);
                        }
                        return 0;
                    }
                    """)
                    exe = joinpath(dir, "main")
                    run(`$cxx -std=c++17 -O2 -o $exe $cpp`)
                    out = parse.(Float64, split(strip(read(`$exe`, String))))

                    julia_f1 = allocate(v, [1.0,0,0,0,0,0.5]; method=method, weights=weights).commands
                    julia_f2 = allocate(v, [0,0,1.0,0.3,0,0]; method=method, weights=weights).commands
                    @test out[1:8] ≈ julia_f1 atol=1e-5
                    @test out[9:16] ≈ julia_f2 atol=1e-5
                end

                # saturate=false reproduces raw Julia allocate() even for a
                # command large enough to exceed the limits; the default clamp
                # instead keeps commands within kMaxCommand (the two diverge).
                bigτ = [6.0, 0, 0, 0, 0, 0]                        # well beyond ±1 N
                raw_j = allocate(v, bigτ; method=:minimum_norm).commands
                @test maximum(abs.(raw_j)) > 1.0                   # raw genuinely saturates
                for (sat, expect) in ((false, raw_j), (true, nothing))
                    export_cpp(v; saturate=sat, path=joinpath(dir, "a.hpp"))
                    write(joinpath(dir, "m.cpp"), """
                    #include "a.hpp"
                    #include <cstdio>
                    int main(){ std::array<float,6> t{6.0f,0,0,0,0,0};
                        auto f = thruster_allocation::allocate(t);
                        for (float v : f) std::printf("%.9f ", v); return 0; }
                    """)
                    run(`$cxx -std=c++17 -O2 -o $(joinpath(dir,"m")) $(joinpath(dir,"m.cpp"))`)
                    got = parse.(Float64, split(strip(read(`$(joinpath(dir,"m"))`, String))))
                    if sat
                        @test maximum(abs.(got)) <= 1.0 + 1e-6     # clamped within limits
                    else
                        @test got ≈ raw_j atol=1e-5                # matches raw Julia
                    end
                end
            end
        end
    end

    # -- reporting ----------------------------------------------------------
    @testset "report() runs" begin
        thr = bluerov_heavy()
        r = allocate(thr, [1.0,0,0,0,0,0.5]; method=:qp)
        buf = IOBuffer()
        report(r; actuators=thr, io=buf)
        report(diagnostics(thr); io=buf)
        describe(bluerov_vehicle(); io=buf)
        s = String(take!(buf))
        @test occursin("Actuator commands", s)
        @test occursin("condition", s)
        @test occursin("most-loaded", s)
    end
end
