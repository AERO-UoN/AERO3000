% =========================================================================
% Tutorial 7 - Numerical Integration: Runge-Kutta 4th Order (RK4)
% =========================================================================
% PURPOSE:
%   Implement the 4th-order Runge-Kutta (RK4) method to numerically
%   integrate a first-order ODE representing simplified aircraft pitch
%   dynamics, then validate the result against MATLAB's built-in ode45
%   solver.
%
% CONCEPTS TAUGHT:
%   Why numerical integration?
%     The equations of motion for an aircraft cannot generally be solved
%     analytically.  Numerical integration steps the solution forward in
%     time, one small increment dt at a time, approximating the continuous
%     differential equation with discrete update rules.
%
%   RK4 algorithm:
%     Given theta_dot = f(t, theta), the RK4 update rule is:
%       k1 = f(t,          theta)
%       k2 = f(t + dt/2,   theta + dt/2 * k1)
%       k3 = f(t + dt/2,   theta + dt/2 * k2)
%       k4 = f(t + dt,     theta + dt   * k3)
%       theta(i+1) = theta(i) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
%     RK4 is 4th-order accurate (local error O(dt^5)) and requires four
%     function evaluations per step.  It is the most widely used fixed-
%     step integrator in flight dynamics and simulation.
%
%   Pitch dynamics ODE used here:
%     theta_dot = -0.5*theta + 10*sin(t)
%     This is a first-order linear ODE with a sinusoidal forcing term,
%     representative of a simplified, decoupled pitch angle response.
%
%   Validation with ode45:
%     MATLAB's ode45 uses an adaptive-step Runge-Kutta (4,5) method.
%     Comparing RK4 (fixed dt = 0.1 s) against ode45 confirms that the
%     manual implementation is correct.
%
% KEY ASSUMPTIONS:
%   - First-order scalar ODE (single state: pitch angle theta)
%   - Fixed time step dt = 0.1 s (sufficiently small for this ODE)
%   - Zero initial pitch angle (theta0 = 0 deg)
%
% EXPECTED OUTPUT:
%   - Figure showing RK4 solution (blue solid) overlaid with ode45
%     solution (red dashed); curves should be nearly indistinguishable
%
% Reference: Stevens, B.L. & Lewis, F.L. (2016) Aircraft Control and
%            Simulation, Ch.2 (Simulation and Numerical Integration).
% =========================================================================

% Given initial conditions
theta0 = 0;  % initial pitch angle (deg)
t0 = 0;      % initial time (s)
tf = 10;     % final time (s)
dt = 0.1;    % time step (s)
time = t0:dt:tf;
N = length(time);
theta_RK4 = zeros(1, N);
theta_RK4(1) = theta0;

% Define the differential equation as a function handle
pitch_ode = @(t, theta) -0.5*theta + 10*sin(t);

% Runge-Kutta 4th Order integration loop
for i = 1:N-1
    t = time(i);
    theta = theta_RK4(i);

    k1 = pitch_ode(t, theta);
    k2 = pitch_ode(t + dt/2, theta + (dt/2)*k1);
    k3 = pitch_ode(t + dt/2, theta + (dt/2)*k2);
    k4 = pitch_ode(t + dt, theta + dt*k3);

    theta_RK4(i+1) = theta + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% Plot RK4 solution
figure;
plot(time, theta_RK4, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pitch Angle \theta (deg)');
title('RK4 Numerical Solution of Aircraft Pitch Dynamics');
grid on;

% Comparison with MATLAB ode45 solver
[t_ode45, theta_ode45] = ode45(pitch_ode, [t0 tf], theta0);
hold on;
plot(t_ode45, theta_ode45, 'ro--', 'LineWidth', 1.5);
legend('RK4 Solution', 'ode45 Solution');

