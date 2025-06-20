mutable struct ControlTask
  name::String
  priority::Int
  sysd::StateSpace
  K::AbstractMatrix{Float64}
  F::AbstractMatrix{Float64}
  x_hist::Matrix{Float64}
  u_hist::Matrix{Float64}
  y_ref::Matrix{Float64}
  exec_mean::Float64
  exec_std::Float64
  exec_time::Float64
end