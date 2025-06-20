using ControlToolbox
using Plots: display

T = 20.0
h = 0.1
t = collect(0.0:h:T)

# F1/10 Car
sys = f1tenth_car()
x0 = [10.0; 0.8]
y_ref = [1000.0;;]
sys_d, x0_aug = augment_matrix(sys, h, x0)
K, F = lqr_controller(sys_d)
x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
x = deaugment_matrix(x)
display(plot_sim(sys_d, t, x, u))
println(convergence_time(t, x, y_ref, sys))
println(max_u(u))
