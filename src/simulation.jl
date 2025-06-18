function simulate(sys::StateSpace, t::Vector{Float64}, K::Matrix{Float64}, F::Vector{Float64}, x0::Vector{Float64}, y_ref::Float64)
  H = length(t)
  x = zeros(sys.nx, H)
  u = zeros(sys.nu, H)

  x[:, 1] = x0
  u[:, 1] = -K * x0 + F * y_ref

  for k in 2:H
    x[:, k] = sys.A * x[:, k-1] + sys.B * u[:, k-1]
    u[:, k] = -K * x[:, k] + F * y_ref
  end

  x, u
end
