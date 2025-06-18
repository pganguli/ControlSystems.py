using ControlSystemsBase: lqr

function lqr_controller(sys::StateSpace)
  K = lqr(sys, I(sys.nx), I(sys.nu))
  F = [1 / (sys.C*inv(I(sys.nx) - sys.A + sys.B * K)*sys.B)[1]]
  K, F
end
