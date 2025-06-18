using ControlSystemsBase: c2d, delay

function augment_matrix(sys::StateSpace, h::Float64, x0::Vector{Float64})
  c2d(sys * delay(h), h), vcat(x0, 0.0)
end

function deaugment_matrix(x::Matrix{Float64})
  x[1:end-1, :]
end
