% Given experimental data (Example: Temperature vs. Pressure)
temperature = [20, 25, 30, 35, 40, 45, 50]; % Temperature in Celsius
pressure = [101.3, 102.5, 104.0, 105.8, 107.6, 110.2, 113.1]; % Pressure in kPa

% Perform linear curve fitting (1st-degree polynomial)
p_linear = polyfit(temperature, pressure, 1);

% Perform quadratic curve fitting (2nd-degree polynomial)
p_quadratic = polyfit(temperature, pressure, 2);

% Generate finer points for smooth plotting
T_fit = linspace(min(temperature), max(temperature), 100);
P_linear_fit = polyval(p_linear, T_fit);
P_quadratic_fit = polyval(p_quadratic, T_fit);

% Plot original data
figure;
plot(temperature, pressure, 'bo', 'MarkerSize', 8, 'DisplayName', 'Experimental Data'); hold on;

% Plot linear fit
plot(T_fit, P_linear_fit, 'r-', 'LineWidth', 2, 'DisplayName', 'Linear Fit');

% Plot quadratic fit
plot(T_fit, P_quadratic_fit, 'g--', 'LineWidth', 2, 'DisplayName', 'Quadratic Fit');

% Formatting the plot
xlabel('Temperature (°C)');
ylabel('Pressure (kPa)');
title('Curve Fitting Using polyfit');
legend; grid on;

% Display polynomial coefficients
disp('Linear Fit Coefficients (y = mx + c):');
disp(['Slope (m): ', num2str(p_linear(1))]);
disp(['Intercept (c): ', num2str(p_linear(2))]);

disp('Quadratic Fit Coefficients (y = ax^2 + bx + c):');
disp(['a: ', num2str(p_quadratic(1))]);
disp(['b: ', num2str(p_quadratic(2))]);
disp(['c: ', num2str(p_quadratic(3))]);