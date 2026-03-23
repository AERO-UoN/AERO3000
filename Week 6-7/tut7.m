% RK4 Method Implementation Tutorial

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

