using ControlSystemsBase: lqr

function lqr_controller(sys::StateSpace, Q::AbstractMatrix, R::AbstractMatrix)
    K = lqr(sys, Q, R)
    F = reshape(1 ./ (sys.C * inv(I(sys.nx) - sys.A + sys.B * K) * sys.B), 1, :)
    K, F
end

lqr_controller(sys::StateSpace) = lqr_controller(sys, I(sys.nx), I(sys.nu))