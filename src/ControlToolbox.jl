module ControlToolbox

include("plant.jl")
export spring_mass, rlc_circuit, cruise_control, dc_motor, inv_pendulum

include("controller.jl")
export lqr_controller

include("simulation.jl")
export simulate

include("statistics.jl")
export report_stats

include("plot.jl")
export plot_sim

end # module ControlToolbox
