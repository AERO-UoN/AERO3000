% Define parameters
t = 0:1:60;   % Time vector (0 to 60 seconds, step = 1 sec)
h0 = 500;     % Initial altitude (m)
vc1 = 10;     % Climb rate scenario 1 (m/s)
vc2 = 15;     % Climb rate scenario 2 (m/s)

% Compute altitude profiles
h1 = h0 + vc1 * t; % Constant climb (10 m/s)
h2 = h0 + vc2 * t; % Faster climb (15 m/s)
h3 = h0 + vc1 * t + 5*sin(0.2*t); % Climb with turbulence

% ------------ Part 1: Basic Climb Profile ------------
figure;
plot(t, h1, 'b', 'LineWidth', 2); % Blue line
xlabel('Time (seconds)');
ylabel('Altitude (m)');
title('Aircraft Climb Profile (10 m/s)');
grid on;

% ------------ Part 2: Multiple Climb Profiles ------------
figure;
hold on;
plot(t, h1, 'b', 'LineWidth', 2); % Scenario 1
plot(t, h2, 'r--', 'LineWidth', 2); % Scenario 2 (dashed red)
xlabel('Time (seconds)');
ylabel('Altitude (m)');
title('Comparison of Climb Profiles');
legend('v_c = 10 m/s', 'v_c = 15 m/s');
grid on;
hold off;

% ------------ Part 3: Subplots for Different Climb Conditions ------------
figure;

% Subplot 1: Constant Climb
subplot(2,1,1);
plot(t, h1, 'g', 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Altitude (m)');
title('Constant Climb (10 m/s)');
grid on;

% Subplot 2: Climb with Turbulence
subplot(2,1,2);
plot(t, h3, 'm', 'LineWidth', 2);
xlabel('Time (seconds)');
ylabel('Altitude (m)');
title('Climb with Turbulence');
grid on;
