using Test
using ControlToolbox
using LinearAlgebra: I

T = 200.0
h = 0.1
t = collect(0.0:h:T)

@testset "RLC Circuit" begin
  sys = rlc_circuit()
  x0 = [50.0; 0.0]
  y_ref = [10.0;;]
  sys_d, x0_aug = augment_matrix(sys, h, x0)
  K, F = lqr_controller(sys_d)
  x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
  x = deaugment_matrix(x)
  conv_time = convergence_time(t, x, y_ref, sys)
  @test conv_time ≈ 164.0 atol = 0.001
  max_control = max_u(u)
  @test max_control ≈ 1001.8432304116046 atol = 0.001
end

@testset "Spring-Mass" begin
  sys = spring_mass()
  x0 = [1.0; 0.0]
  y_ref = [0.5;;]
  sys_d, x0_aug = augment_matrix(sys, h, x0)
  K, F = lqr_controller(sys_d)
  x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
  x = deaugment_matrix(x)
  conv_time = convergence_time(t, x, y_ref, sys)
  @test conv_time ≈ 7.5 atol = 0.001
  max_control = max_u(u)
  @test max_control ≈ 0.7044279242636569 atol = 0.001
end

@testset "Cruise Control" begin
  sys = cruise_control()
  x0 = [0.0]
  y_ref = [20.0;;]
  sys_d, x0_aug = augment_matrix(sys, h, x0)
  K, F = lqr_controller(sys_d)
  x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
  x = deaugment_matrix(x)
  conv_time = convergence_time(t, x, y_ref, sys)
  @test conv_time ≈ 152.2 atol = 0.001
  max_control = max_u(u)
  @test max_control ≈ 1000.0997449885296 atol = 0.001
end

@testset "DC Motor" begin
  sys = dc_motor()
  x0 = [0.0; 0.0; 0.0]
  y_ref = [1.0;;]
  sys_d, x0_aug = augment_matrix(sys, h, x0)
  K, F = lqr_controller(sys_d)
  x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
  x = deaugment_matrix(x)
  conv_time = convergence_time(t, x, y_ref, sys)
  @test conv_time ≈ 4.8 atol = 0.001
  max_control = max_u(u)
  @test max_control ≈ 0.025582016854240382 atol = 0.001
end

@testset "Inverted Pendulum" begin
  sys = inv_pendulum()
  x0 = [0.0; 0.0; 1.57; 0.0]
  y_ref = [0.78;;]
  sys_d, x0_aug = augment_matrix(sys, h, x0)
  K, F = lqr_controller(sys_d)
  x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
  x = deaugment_matrix(x)
  conv_time = convergence_time(t, x, y_ref, sys)
  @test conv_time ≈ 9.5 atol = 0.001
  max_control = max_u(u)
  @test max_control ≈ 37.42677403013481 atol = 0.001
end

@testset "F1/10 Car" begin
  sys = f1tenth_car()
  x0 = [1000.0; 0.8]
  y_ref = [10.0;;]
  sys_d, x0_aug = augment_matrix(sys, h, x0)
  Q = I(3)           # Focus on both states equally
  R = [20.0;;]       # Increase R to penalize control effort more
  K, F = lqr_controller(sys_d, Q, R)
  x, u = simulate(sys_d, t, K, F, x0_aug, y_ref)
  x = deaugment_matrix(x)
  conv_time = convergence_time(t, x, y_ref, sys)
  @test conv_time ≈ 2.8 atol = 0.001
  max_control = max_u(u)
  @test max_control ≈ 141.26107233918506 atol = 0.001
end
