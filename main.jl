using ControlToolbox: rlc_circuit, c2d, lqr_controller, simulate, plot_sim, report_stats

T = 200.0
dt = 0.1
t = collect(0.0:dt:T)

sys = rlc_circuit()
x0 = [50.0; 0.0]
y_ref = 10.0

sys_d = c2d(sys, dt)
K, F = lqr_controller(sys_d)
println("K = ", K)
println("F = ", F)
x, u = simulate(sys_d, t, K, F, x0, y_ref)
plot_sim(sys_d, t, x, u)
report_stats(t, x, u, y_ref, sys_d)
