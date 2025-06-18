function max_u(u::Matrix{Float64})
  return maximum(abs.(u), init=0.0)
end

function convergence_time(t::Vector{Float64}, x::Matrix{Float64}, y_ref::Float64, sys::StateSpace, tol::Float64=0.01)
  # Check if we have any data to analyze
  if isempty(t) || isempty(x)
    return 0.0
  end

  # Calculate system output and check convergence
  y = sys.C * x
  within_tol = all(abs.(y .- y_ref) .< tol, dims=1)

  # Find the last time step where system was not converged
  last_not_converged_col = findlast(!, within_tol)[2]

  # Return convergence time (next time step after last non-converged point)
  if last_not_converged_col !== nothing && last_not_converged_col < length(t)
    return t[last_not_converged_col+1]
  else
    return 0.0  # No convergence detected
  end
end
