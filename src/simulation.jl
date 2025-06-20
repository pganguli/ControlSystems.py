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
