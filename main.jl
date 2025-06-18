using ControlToolbox: rlc_circuit, lqr_controller, simulate, plot_sim, max_u, convergence_time, augment_matrix, deaugment_matrix

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

# Print statistics
println("\n--- Simulation Statistics ---")
conv_time = convergence_time(t, x, y_ref, sys)
max_control = max_u(u)

if conv_time > 0.0
  println("Time to converge to reference: $(round(conv_time, digits=2)) s")
else
  println("Did not converge to reference.")
end

println("Maximum control input exerted: $(round(max_control, digits=3))")
