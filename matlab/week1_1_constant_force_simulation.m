clc
clear all
close all

dt = 0.01; T = 10;

% Vehicle condition 
m = 1000; Ft = 2500;
F_resistance = 500; v0 = 20;

N = T / dt; t = 0:dt:T;

v = zeros(1, N + 1); 
v(1) = v0;

for k = 1:N
    a = (Ft - F_resistance)/m;
    v(k+1) = v(k) + a*dt;
end

plot(t, v, 'LineWidth', 2)
grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Vehicle Velocity')