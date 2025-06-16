using ControlSystemsBase: StateSpace
using Plots: plot

function plot_sim(sys::StateSpace, t::Vector{Float64}, x::Matrix{Float64}, u::Matrix{Float64})
    p1 = plot(t, x', labels=reshape(sys.state_labels, 1, :), ylabel="States", legend=:topright)
    p2 = plot(t, u', labels=reshape(sys.input_labels, 1, :), ylabel="Inputs", xlabel="Time (s)", legend=:topright)
    plot(p1, p2, layout=(2, 1), size=(800, 800), plot_title="Simulation Results")
    # Plots are automatically displayed in Julia environments like Pluto or VS Code.
    # If running from a terminal, you might need to explicitly save or display.
end
