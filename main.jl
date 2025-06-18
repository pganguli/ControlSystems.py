using ControlToolbox: rlc_circuit, spring_mass, cruise_control, dc_motor, inv_pendulum, lqr_controller, simulate, plot_sim, augment_matrix, deaugment_matrix

T = 200.0
h = 0.1
t = collect(0.0:h:T)

# RLC Circuit
sys = rlc_circuit()
x0 = [50.0; 0.0]
y_ref = 10.0
sys_d, x0_aug = augment_matrix(sys, h, x0)
K, F = lqr_controller(sys_d)
x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
x = deaugment_matrix(x)
plot_sim(sys_d, t, x, u)

# Spring-Mass
sys = spring_mass()
x0 = [1.0; 0.0]
y_ref = 0.5
sys_d, x0_aug = augment_matrix(sys, h, x0)
K, F = lqr_controller(sys_d)
x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
x = deaugment_matrix(x)
plot_sim(sys_d, t, x, u)

# Cruise Control
sys = cruise_control()
x0 = [0.0]
y_ref = 20.0
sys_d, x0_aug = augment_matrix(sys, h, x0)
K, F = lqr_controller(sys_d)
x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
x = deaugment_matrix(x)
plot_sim(sys_d, t, x, u)

# DC Motor
sys = dc_motor()
x0 = [0.0; 0.0; 0.0]
y_ref = 1.0
sys_d, x0_aug = augment_matrix(sys, h, x0)
K, F = lqr_controller(sys_d)
x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
x = deaugment_matrix(x)
plot_sim(sys_d, t, x, u)

# Inverted Pendulum
sys = inv_pendulum()
x0 = [0.0; 0.0; 0.1; 0.0]
y_ref = 0.0
sys_d, x0_aug = augment_matrix(sys, h, x0)
K, F = lqr_controller(sys_d)
x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
x = deaugment_matrix(x)
plot_sim(sys_d, t, x, u)
