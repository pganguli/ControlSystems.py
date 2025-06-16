using LinearAlgebra: I, inv
using ControlSystems: StateSpace, lqr, c2d
using Plots
include("model.jl")

function controller(sys::StateSpace)
    K, _, _ = lqr(sys, I(sys.nx), I(sys.nu))
    # Feedforward term for reference tracking
    F = [1 / (sys.C * inv(I(sys.nx) - sys.A + sys.B * K) * sys.B)[1]]
    return K, F
end

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
    return x, u
end

function plot_sim(sys::StateSpace, t::Vector{Float64}, x::Matrix{Float64}, u::Matrix{Float64})
    p1 = plot(t, x', labels=reshape(sys.state_labels, 1, :), ylabel="States", legend=:topright)
    p2 = plot(t, u', labels=reshape(sys.input_labels, 1, :), ylabel="Inputs", xlabel="Time (s)", legend=:topright)
    plot(p1, p2, layout=(2, 1), size=(800, 800), plot_title="Simulation Results")
    # Plots are automatically displayed in Julia environments like Pluto or VS Code.
    # If running from a terminal, you might need to explicitly save or display.
end

function report_stats(t::Vector{Float64}, x::Matrix{Float64}, u::Matrix{Float64}, y_ref::Float64, sys::StateSpace, tol::Float64 = 0.01)
    println("\n--- Simulation Statistics ---")
    try
        y = sys.C * x
        y_target = y_ref
        within_tol = all(abs.(y .- y_target) .< tol .* abs.(y_target), dims=1)
        
        # Convert within_tol from a 1xT matrix to a 1D boolean array
        within_tol_flat = vec(within_tol)

        # Find the first index where the condition is true and stays true
        # This logic is adapted from the Python np.cumprod(within_tol[::-1])[::-1]
        converge_idx = -1
        for i in 1:length(within_tol_flat)
            if within_tol_flat[i]
                all_after_this_true = true
                for j in (i+1):length(within_tol_flat)
                    if !within_tol_flat[j]
                        all_after_this_true = false
                        break
                    end
                end
                if all_after_this_true
                    converge_idx = i
                    break
                end
            end
        end

        if converge_idx != -1
            converge_time = t[converge_idx]
            println("Time to converge to reference (within $(tol*100):.1f%): $(converge_time):.2f s")
        else
            println("Did not converge to reference within $(tol*100):.1f% tolerance.")
        end

    catch e
        if isa(e, BoundsError) # Equivalent to IndexError for empty results
            println("Did not converge to reference within $(tol*100):.1f% tolerance.")
        else
            rethrow(e)
        end
    end
    max_u = maximum(abs.(u))
    println("Maximum control input exerted: $(max_u):.3f")
end

function main()
    T = 200.0
    dt = 0.1
    t = collect(0.0:dt:T)

    sys = rlc_circuit()
    x0 = [50.0; 0.0]
    y_ref = 10.0

    sys_d = c2d(sys, dt)
    K, F = controller(sys_d)
    println("K = ", K)
    println("F = ", F)
    x, u = simulate(sys_d, t, K, F, x0, y_ref)
    plot_sim(sys_d, t, x, u)
    report_stats(t, x, u, y_ref, sys_d)
end

# Entry point
main() 