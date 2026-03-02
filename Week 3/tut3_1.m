% =========================================================================
% Tutorial 3.1 - Data Analysis: Polynomial Curve Fitting
% =========================================================================
% PURPOSE:
%   Fit linear and quadratic polynomials to experimental data using
%   MATLAB's polyfit/polyval functions.  In aeronautics, this technique
%   is used to build analytical approximations from wind-tunnel or
%   flight-test data — for example, fitting a lift curve (CL vs alpha)
%   or a drag polar (CD vs CL^2) so the result can be used in equations.
%
% CONCEPTS TAUGHT:
%   - polyfit(x, y, n): least-squares polynomial fit of degree n.
%       Degree 1 gives y = m*x + c  (linear)
%       Degree 2 gives y = a*x^2 + b*x + c  (quadratic)
%   - polyval(p, x): evaluate a fitted polynomial at new x values
%   - Choosing fit degree: linear fits are appropriate when data are nearly
%     straight; quadratic fits capture curvature.  Higher-degree fits risk
%     overfitting (Runge's phenomenon).
%   - Visualising the fit against the raw data to assess quality
%
% KEY ASSUMPTIONS:
%   - Temperature-pressure data are synthetic illustrative values
%   - No outlier detection or cross-validation is applied here
%   - Fit quality is assessed visually; for rigorous use compute R^2 or
%     residuals
%
% EXPECTED OUTPUT:
%   - Figure showing data points with linear (red) and quadratic (green)
%     fits overlaid
%   - Printed polynomial coefficients for each fit
%
% Reference: MATLAB Documentation — polyfit, polyval
% =========================================================================

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