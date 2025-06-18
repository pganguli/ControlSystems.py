module ControlToolbox

using ControlSystemsBase: StateSpace
using LinearAlgebra: I

include("plant.jl")
export spring_mass, rlc_circuit, cruise_control, dc_motor, inv_pendulum

include("controller.jl")
export lqr_controller

include("statespace.jl")
export augment_matrix, deaugment_matrix

include("simulation.jl")
export simulate

include("statistics.jl")
export max_u, convergence_time

include("plot.jl")
export plot_sim

end # module ControlToolbox
