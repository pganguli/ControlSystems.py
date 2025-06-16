using ControlSystemsBase: StateSpace

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
