clc
clear all
close all

dt = 0.01; T = 60;

% Vehicle parameters
m = 1500; rho = 1.225;
Cd = 0.30; A_front = 2.2;
Cr = 0.015; g = 9.81;

% Driving conditions
Ft = 1000; v0 = 20;

N = round(T / dt);
t = (0:N) * dt;

v = zeros(1, N + 1);
v(1) = v0;

% Rolling resistance
Fr = Cr * m * g;

% Forward Euler simulation
for k = 1:N
    Fd = 0.5 * rho * Cd * A_front * v(k)^2;
    a = (Ft - Fd - Fr) / m;
    v(k+1) = v(k) + a*dt;
end

plot(t, v, 'LineWidth', 2)
grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Vehicle Velocity with Aerodynamic Drag')

% Confirm equilibrium velocty, final velocity
v_eq = sqrt(2 * (Ft - Fr) / (rho * Cd * A_front));

fprintf('Final velocity: %.2f m/s\n', v(end));
fprintf('Equilibrium velocity: %.2f m/s\n', v_eq);
