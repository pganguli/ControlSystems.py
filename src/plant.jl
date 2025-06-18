using ControlSystemsBase: ss

function spring_mass(m::Float64=1.0, k::Float64=1.0, b::Float64=0.2) # Mass-Spring-Damper System
  # m: mass (kg)
  # k: spring constant (N/m)
  # b: damping constant (Ns/m)
  A = [0.0 1.0; -k/m -b/m] # State matrix
  B = [0.0; 1 / m] # Input matrix
  C = [1.0 0.0] # Output matrix
  D = 0.0 # Feedforward matrix
  # Input: F (force)
  # Output: x (position)
  # States:
  #   x (position)
  #   v (velocity)
  return ss(A, B, C, D)
end

function rlc_circuit(R::Float64=10.0, L::Float64=100.0, C::Float64=0.01) # RLC Circuit
  # R: electric resistance (Ohm)
  # L: electric inductance (H)
  # C: electric capacitance (F)
  A = [0.0 1.0; -1/(L*C) -R/L] # State matrix
  B = [0.0; 1 / L] # Input matrix
  C = [1.0 0.0] # Output matrix
  D = 0.0 # Feedforward matrix
  # Input: V (voltage)
  # Output: q (charge)
  # States:
  #   q (charge)
  #   i (current)
  return ss(A, B, C, D)
end

function cruise_control(m::Float64=1000.0, b::Float64=50.0) # Cruise Control
  # m: vehicle mass (kg)
  # b: damping constant (N.s/m)
  A = -b / m # State matrix
  B = 1 / m # Input matrix
  C = 1.0 # Output matrix
  D = 0.0 # Feedforward matrix
  # Input: F (force)
  # Output: v (velocity)
  # States:
  #   v (velocity)
  return ss(A, B, C, D)
end

function dc_motor(J::Float64=3.2284e-6, b::Float64=3.5077e-6, Kb::Float64=0.0274, Kt::Float64=0.0274, R::Float64=4.0, L::Float64=2.75e-6) # DC Motor Position
  # J: moment of inertia of the rotor (kg.m^2)
  # b: motor viscous friction constant (N.m.s)
  # Kb: electromotive force constant (V/rad/sec)
  # Kt: motor torque constant (N.m/A)
  # R: electric resistance (Ohm)
  # L: electric inductance (H)
  A = [0.0 1.0 0.0; 0.0 -b/J -Kt/J; 0.0 -Kb/L -R/L] # State matrix
  B = [0.0; 0.0; 1 / L] # Input matrix
  C = [1.0 0.0 0.0] # Output matrix
  D = 0.0 # Feedforward matrix
  # Input: V (voltage)
  # Output: θ (angle)
  # States:
  #   θ (angle)
  #   ω (angular velocity)
  #   i (current)
  return ss(A, B, C, D)
end

function inv_pendulum(M::Float64=0.5, m::Float64=0.2, b::Float64=0.1, l::Float64=0.3, I::Float64=0.006, g::Float64=9.8) # Inverted Pendulum
  # M: mass of the cart (kg)
  # m: mass of the pendulum (kg)
  # b: coefficient of friction for cart (N/m/s)
  # l: length to pendulum center of mass (m)
  # I: moment of inertia of the pendulum (kg.m^2)
  # g: acceleration due to gravity (m/s^2)
  p = I * (M + m) + M * m * (l^2) # denominator for the A and B matrices
  A = [
    0.0 1.0 0.0 0.0;
    0.0 -(I + m * (l^2))*b/p ((m^2)*g*(l^2))/p 0.0;
    0.0 0.0 0.0 1.0;
    0.0 -(m * l * b)/p m*g*l*(M+m)/p 0.0
  ] # State matrix
  B = [0.0; (I + m * (l^2)) / p; 0.0; m * l / p] # Input matrix
  C = [1.0 0.0 0.0 0.0; 0.0 0.0 1.0 0.0] # Output matrix
  D = [0.0; 0.0] # Feedforward matrix
  # Input: F (force)
  # Outputs:
  #   x (position)
  #   φ (angle)
  # States:
  #   x (position)
  #   v (velocity)
  #   φ (angle)
  #   ω (angular velocity)
  return ss(A, B, C, D)
end
