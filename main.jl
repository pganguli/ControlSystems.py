using ControlToolbox

T = 200.0
h = 0.1
t = collect(0.0:h:T)
H = length(t)

# --- Plant Model Initialization ---
# DC Motor
sys_dc = dc_motor()
x0_dc = [0.0; 0.0; 0.0]
y_ref_dc = [1.0;;]
sysd_dc, x0_aug_dc = augment_matrix(sys_dc, h, x0_dc)
K_dc, F_dc = lqr_controller(sysd_dc)

# Cruise Control
sys_cc = cruise_control()
x0_cc = [0.0]
y_ref_cc = [20.0;;]
sysd_cc, x0_aug_cc = augment_matrix(sys_cc, h, x0_cc)
K_cc, F_cc = lqr_controller(sysd_cc)

# RLC Circuit
sys_rlc = rlc_circuit()
x0_rlc = [10.0; 0.0]
y_ref_rlc = [50.0;;]
sysd_rlc, x0_aug_rlc = augment_matrix(sys_rlc, h, x0_rlc)
K_rlc, F_rlc = lqr_controller(sysd_rlc)

# --- Task List Initialization ---
tasks = [
  ControlTask("cruise", 1.0, sysd_cc, sys_cc, K_cc, F_cc, zeros(sysd_cc.nx, H), zeros(sysd_cc.nu, H), y_ref_cc, 0.045, 0.005, 0.0),
  ControlTask("dc_motor", 2.0, sysd_dc, sys_dc, K_dc, F_dc, zeros(sysd_dc.nx, H), zeros(sysd_dc.nu, H), y_ref_dc, 0.045, 0.005, 0.0),
  ControlTask("rlc", 3.0, sysd_rlc, sys_rlc, K_rlc, F_rlc, zeros(sysd_rlc.nx, H), zeros(sysd_rlc.nu, H), y_ref_rlc, 0.045, 0.005, 0.0)
]

# Set initial states
for task in tasks
  task.x_hist[:, 1] = if task.name == "dc_motor"
    x0_aug_dc
  elseif task.name == "cruise"
    x0_aug_cc
  else
    x0_aug_rlc
  end
  # Initial control input
  task.u_hist[:, 1] .= 0.0
end

# --- Simulation Loop (delegated) ---
multitask_simulation!(tasks, t, h)

# --- Plot and Print Results (delegated) ---
report_results(tasks, t)
