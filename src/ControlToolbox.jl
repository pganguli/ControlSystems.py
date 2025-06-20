module ControlToolbox

using ControlSystemsBase: StateSpace
using LinearAlgebra: I

include("task.jl")
export ControlTask

include("plant.jl")
export spring_mass, rlc_circuit, cruise_control, dc_motor, inv_pendulum, f1tenth_car

include("controller.jl")
export lqr_controller

include("statespace.jl")
export augment_matrix, deaugment_matrix

include("simulation.jl")
export simulate, multitask_simulation!

include("statistics.jl")
export max_u, convergence_time, report_results

include("plot.jl")
export plot_sim

end # module ControlToolbox
