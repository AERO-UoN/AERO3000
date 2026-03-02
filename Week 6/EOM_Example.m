% =========================================================================
% Tutorial 6 - Equations of Motion: Analytical Solution (1-DOF)
% =========================================================================
% PURPOSE:
%   Introduce Newton's second law as the foundation of aircraft equations
%   of motion (EOM) by solving the simplest possible case: a single mass
%   accelerated by a constant force.  The analytical solution is then
%   plotted to build intuition before the full 6-DOF aircraft EOM are
%   introduced in lectures.
%
% CONCEPTS TAUGHT:
%   Newton's second law:   F = m * a   =>   a = F / m
%   Integration of acceleration to obtain velocity and position:
%       a(t) = F/m              (constant — force and mass are fixed)
%       v(t) = v0 + a*t         (velocity grows linearly with time)
%       x(t) = x0 + v0*t + 0.5*a*t^2   (position grows quadratically)
%
%   Connection to aircraft EOM:
%     An aircraft in 3-D has six degrees of freedom (3 translational,
%     3 rotational).  The full 6-DOF equations in the body frame are:
%       m*(du/dt + q*w - r*v) = Fx   (force in x, axial)
%       m*(dv/dt + r*u - p*w) = Fy   (force in y, lateral)
%       m*(dw/dt + p*v - q*u) = Fz   (force in z, normal)
%       Ixx*dp/dt - Ixz*dr/dt + (Izz-Iyy)*q*r = L   (rolling moment)
%       Iyy*dq/dt + (Ixx-Izz)*p*r + Ixz*(p^2-r^2) = M  (pitching moment)
%       Izz*dr/dt - Ixz*dp/dt + (Iyy-Ixx)*p*q = N   (yawing moment)
%     This 1-DOF example isolates one force/one acceleration to keep the
%     mathematics tractable before tackling the full coupled system.
%
% KEY ASSUMPTIONS:
%   - Single degree of freedom (translation along x only)
%   - Constant applied force (no aerodynamic or gravitational variation)
%   - Frictionless surface (no drag or resistance)
%   - Zero initial velocity and position
%
% EXPECTED OUTPUT:
%   - Figure with three subplots: position x(t), velocity v(t),
%     and acceleration a(t) over the simulation interval 0–10 s
%
% Reference: Nelson, R.C. (1998) Flight Stability and Automatic Control,
%            Ch.2 (Equations of Motion); Stevens & Lewis (2016)
%            Aircraft Control and Simulation, Ch.2.
% =========================================================================

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