%% Example 1: Motion of a block on a frictionless surface
% F = m*a => a = F/m
% Integrating: v(t) = v0 + (F/m)*t
% Integrating: x(t) = x0 + v0*t + 0.5*(F/m)*t^2

clear; clc; close all;

%% Parameters
m = 5;          % Mass (kg)
F = 10;         % Applied horizontal force (N)

%% Initial Conditions
x0 = 0;         % Initial position (m)
v0 = 0;         % Initial velocity (m/s)

%% Time vector
t = linspace(0, 10, 1000);   % 0 to 10 seconds

%% Equations of Motion (analytical solution)
a = F / m;                          % Acceleration (m/s^2) - constant
v = v0 + a * t;                     % Velocity (m/s)
x = x0 + v0*t + 0.5 * a * t.^2;   % Position (m)

%% Plot Results
figure('Name', 'Block Motion on Frictionless Surface', 'NumberTitle', 'off');

subplot(3,1,1)
plot(t, x, 'b-', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Position x (m)')
title('Position vs Time')
grid on

subplot(3,1,2)
plot(t, v, 'r-', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Velocity v (m/s)')
title('Velocity vs Time')
grid on

subplot(3,1,3)
plot(t, a * ones(size(t)), 'g-', 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Acceleration a (m/s²)')
title('Acceleration vs Time')
ylim([0, a*2])
grid on

sgtitle(sprintf('Frictionless Block Motion  |  m = %.1f kg,  F = %.1f N', m, F))