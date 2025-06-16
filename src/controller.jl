using ControlSystemsBase: StateSpace, lqr

function lqr_controller(sys::StateSpace)
    K, _, _ = lqr(sys, I(sys.nx), I(sys.nu))
    # Feedforward term for reference tracking
    F = [1 / (sys.C * inv(I(sys.nx) - sys.A + sys.B * K) * sys.B)[1]]
    return K, F
end
