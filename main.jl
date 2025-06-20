using ControlToolbox
using Plots: display

T = 200.0
h = 0.1
t = collect(0.0:h:T)
H = length(t)

# --- Plant Model Initialization ---
# F1/10 Car
sys_f1 = f1tenth_car()
x0_f1 = [10.0; 0.8]
y_ref_f1 = [1000.0;;]
sysd_f1, x0_aug_f1 = augment_matrix(sys_f1, h, x0_f1)
K_f1, F_f1 = lqr_controller(sysd_f1)

# Cruise Control
sys_cc = cruise_control()
x0_cc = [0.0]
y_ref_cc = [20.0;;]
sysd_cc, x0_aug_cc = augment_matrix(sys_cc, h, x0_cc)
K_cc, F_cc = lqr_controller(sysd_cc)

# RLC Circuit
sys_rlc = rlc_circuit()
x0_rlc = [50.0; 0.0]
y_ref_rlc = [10.0;;]
sysd_rlc, x0_aug_rlc = augment_matrix(sys_rlc, h, x0_rlc)
K_rlc, F_rlc = lqr_controller(sysd_rlc)

# --- ControlTask Struct ---
mutable struct ControlTask
  name::String
  priority::Int
  sysd
  K
  F
  x_hist::Matrix{Float64}
  u_hist::Matrix{Float64}
  y_ref::Matrix{Float64}
  exec_mean::Float64
  exec_std::Float64
  exec_time::Float64
end

# --- Task List Initialization ---
tasks = [
  ControlTask("f1tenth", 1, sysd_f1, K_f1, F_f1, zeros(sysd_f1.nx, H), zeros(sysd_f1.nu, H), y_ref_f1, 0.03, 0.01, 0.0),
  ControlTask("cruise", 2, sysd_cc, K_cc, F_cc, zeros(sysd_cc.nx, H), zeros(sysd_cc.nu, H), y_ref_cc, 0.04, 0.01, 0.0),
  ControlTask("rlc", 3, sysd_rlc, K_rlc, F_rlc, zeros(sysd_rlc.nx, H), zeros(sysd_rlc.nu, H), y_ref_rlc, 0.05, 0.01, 0.0)
]

# For convergence_time, keep a mapping from name to sys (continuous)
sys_map = Dict(
    "f1tenth" => sys_f1,
    "cruise" => sys_cc,
    "rlc" => sys_rlc
)

# Set initial states
for task in tasks
  task.x_hist[:, 1] = if task.name == "f1tenth"
    x0_aug_f1
  elseif task.name == "cruise"
    x0_aug_cc
  else
    x0_aug_rlc
  end
  # Initial control input
  task.u_hist[:, 1] .= 0.0
end

# --- Simulation Loop ---
for k in 2:H
  # 1. Draw execution times
  for task in tasks
    task.exec_time = max(0, randn() * task.exec_std + task.exec_mean)
  end
  # 2. Sort by priority
  sorted_tasks = sort(tasks, by=t -> t.priority)
  # 3. Schedule
  total_time = 0.0
  missed = String[]
  for task in sorted_tasks
    x_prev = task.x_hist[:, k-1]
    u_prev = task.u_hist[:, k-1]
    if total_time + task.exec_time <= h
      # Update control input
      u = -task.K * x_prev + task.F * task.y_ref
      total_time += task.exec_time
      println("[t=$(t[k])s] $(task.name) (exec_time=$(round(task.exec_time, digits=4))) [u=$(join(round.(u; digits=4), ", "))*]")
    else
      # Hold previous control input
      u = u_prev
      push!(missed, task.name)
      println("[t=$(t[k])s] $(task.name) (exec_time=$(round(task.exec_time, digits=4))) [u=$(join(round.(u_prev; digits=4), ", "))]")
    end
    # Simulate one step
    x = task.sysd.A * x_prev + task.sysd.B * u
    task.x_hist[:, k] = x
    task.u_hist[:, k] = u
  end
  if !isempty(missed)
    println("[t=$(t[k])s] Missed tasks: ", join(missed, ", "))
  end
end

# --- Plot and Print Results ---
for task in tasks
  x = deaugment_matrix(task.x_hist)
  u = task.u_hist
  println("\nResults for $(task.name):")
  display(plot_sim(task.sysd, t, x, u))
  println("Convergence time: ", convergence_time(t, x, task.y_ref, sys_map[task.name]))
  println("Max control: ", max_u(u))
end
