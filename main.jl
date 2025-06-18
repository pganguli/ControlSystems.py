using ControlSystemsBase: c2d
using ControlToolbox: rlc_circuit, lqr_controller, simulate, plot_sim, report_stats, augment_matrix, deaugment_matrix

T = 200.0
h = 0.1
t = collect(0.0:h:T)

sys = rlc_circuit()
x0 = [50.0; 0.0]
y_ref = 10.0

sys_d, x0 = augment_matrix(sys, h, x0)
K, F = lqr_controller(sys_d)
println("K = ", K)
println("F = ", F)
x, u = simulate(sys_d, t, K, F, x0, y_ref)
x = deaugment_matrix(x)
plot_sim(sys_d, t, x, u)
report_stats(t, x, u, y_ref, sys)
