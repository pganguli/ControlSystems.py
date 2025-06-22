function simulate(sys::StateSpace, t::Vector{Float64}, K::Matrix{Float64}, F::Matrix{Float64}, x0::Vector{Float64}, y_ref::Matrix{Float64})
  H = length(t)
  x = zeros(sys.nx, H)
  u = zeros(sys.nu, H)

  x[:, 1] = x0

  for k in 2:H
    u[:, k-1] = -K * x[:, k-1] + F * y_ref
    x[:, k] = sys.A * x[:, k-1] + sys.B * u[:, k-1]
  end

  x, u
end

function calculate_priority(x::Vector{Float64}, y_ref::Matrix{Float64}, sys::StateSpace)
  # Calculate current output
  y = sys.C * x
  # Calculate percentage distance from reference
  percent_distance = abs.(y .- y_ref) ./ abs.(y_ref)
  # Return maximum percentage distance (highest priority for farthest away)
  return maximum(percent_distance)
end

function multitask_simulation!(tasks::Vector{ControlTask}, t::Vector{Float64}, h::Float64)
  H = length(t)
  for k in 2:H
    # 1. Draw execution times
    for task in tasks
      task.exec_time = max(0, randn() * task.exec_std + task.exec_mean)
    end
    
    # 2. Calculate dynamic priorities based on current distance from reference
    println("[t=$(t[k])s] Priority calculations:")
    for task in tasks
      x_current = task.x_hist[:, k-1]
      # Deaugment the state before calculating priority with original system
      x_deaug = deaugment_matrix(x_current)
      task.priority = calculate_priority(x_deaug, task.y_ref, task.sys_orig)
      
      # Print task name and percentage distance
      println("  $(task.name): $(round(task.priority * 100, digits=4))%")
    end
    
    # 3. Sort by priority (highest priority first)
    sorted_tasks = sort(tasks, by=t -> t.priority, rev=true)
    
    # 4. Schedule
    total_time = 0.0
    missed = String[]
    for task in sorted_tasks
      x_prev = task.x_hist[:, k-1]
      u_prev = task.u_hist[:, k-1]
      if total_time + task.exec_time <= h
        # Update control input
        u = -task.K * x_prev + task.F * task.y_ref
        total_time += task.exec_time
      else
        # Hold previous control input
        u = u_prev
        push!(missed, task.name)
      end
      # Simulate one step
      x = task.sysd.A * x_prev + task.sysd.B * u
      task.x_hist[:, k] = x
      task.u_hist[:, k] = u
    end
    if !isempty(missed)
      println("[t=$(t[k])s] Missed tasks: ", join(missed, ", "))
    end
    println()  # Add blank line for readability
  end
  return tasks
end