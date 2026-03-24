function plotData(varargin)
%   Plots the full simulation output.
%
%   Figures are ordered in cause-effect sequence:
%     1 — Control inputs          (what the pilot commanded)
%     2 — Aerodynamic angles      (immediate aerodynamic response)
%     3 — Body angular rates      (rotational response)
%     4 — Euler attitude angles   (resulting orientation)
%     5 — Body velocities         (translational response)
%     6 — Position and altitude   (where the aircraft went)
%     7 — Flight condition        (airspeed and altitude in aviation units)
%     8 — 3-D flight path         (spatial trajectory)
%
% Usage:
%   plotData(X, U, time)            — standard call from main.m
%   plotData(X, U, time, true, sfx) — as above plus save figures to disk
%   plotData(X, time)               — state only (no control figures)

% -----------------------------------------------------------------------
% Parse inputs
% -----------------------------------------------------------------------
if length(varargin) == 5
    X             = varargin{1};
    U             = varargin{2};
    time          = varargin{3};
    toSave        = varargin{4};
    controlSuffix = varargin{5};
    plotU         = true;
    if toSave
        currentFolder = pwd;
        saveLoc = uigetdir(currentFolder, 'Select save folder');
    end
elseif length(varargin) == 2
    X      = varargin{1};
    time   = varargin{2};
    toSave = false;
    plotU  = false;
else
    X      = varargin{1};
    U      = varargin{2};
    time   = varargin{3};
    toSave = false;
    plotU  = true;
end

% -----------------------------------------------------------------------
% Pre-compute derived quantities used across multiple figures
% -----------------------------------------------------------------------
u_b = X(1,:);
v_b = X(2,:);
w_b = X(3,:);
V_hist     = sqrt(u_b.^2 + v_b.^2 + w_b.^2);
alpha_hist = rad2deg(atan(w_b ./ u_b));       % angle of attack (deg)
beta_hist  = rad2deg(asin(v_b ./ V_hist));    % sideslip angle  (deg)
euler      = rad2deg(quat2euler(X(7:10,:)));  % [phi; theta; psi] (deg)
V_kts      = V_hist * 1.94384;               % m/s -> knots
alt_ft     = -X(13,:) * 3.28084;             % m   -> feet


% -----------------------------------------------------------------------
% Figure 1 — Control inputs
% Shown first so students can link every feature in the response plots
% directly back to the control input that caused it.
% -----------------------------------------------------------------------
if plotU
    fig1 = figure(1);
    set(fig1, 'Name', 'Control Inputs', 'NumberTitle', 'on', 'Color', [1 1 1])

    subplot(2,2,1)
    plot(time, U(1,:), 'LineWidth', 2)
    title('Throttle')
    xlabel('Time (s)');  ylabel('Throttle fraction')
    grid on;  set(gca, 'Color', [1 1 1])

    subplot(2,2,2)
    plot(time, rad2deg(U(2,:)), 'LineWidth', 2)
    title('Elevator \delta_e')
    xlabel('Time (s)');  ylabel('Deflection (\circ)')
    grid on;  set(gca, 'Color', [1 1 1])

    subplot(2,2,3)
    plot(time, rad2deg(U(3,:)), 'LineWidth', 2)
    title('Aileron \delta_a')
    xlabel('Time (s)');  ylabel('Deflection (\circ)')
    grid on;  set(gca, 'Color', [1 1 1])

    subplot(2,2,4)
    plot(time, rad2deg(U(4,:)), 'LineWidth', 2)
    title('Rudder \delta_r')
    xlabel('Time (s)');  ylabel('Deflection (\circ)')
    grid on;  set(gca, 'Color', [1 1 1])

    if toSave
        print(fig1, [saveLoc filesep 'fig1_controls_' controlSuffix], '-depsc')
    end
end

% -----------------------------------------------------------------------
% Figure 2 — Aerodynamic angles (alpha and beta)
% The most physically informative outputs — directly connect the control
% input to the aerodynamic forces and moments it produces.
% -----------------------------------------------------------------------
fig2 = figure(2);
set(fig2, 'Name', 'Aerodynamic Angles', 'NumberTitle', 'on', 'Color', [1 1 1])

subplot(2,1,1)
plot(time, alpha_hist, 'LineWidth', 2)
title('Angle of Attack \alpha')
xlabel('Time (s)');  ylabel('\alpha (\circ)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(2,1,2)
plot(time, beta_hist, 'LineWidth', 2)
title('Sideslip Angle \beta')
xlabel('Time (s)');  ylabel('\beta (\circ)')
grid on;  set(gca, 'Color', [1 1 1])

if toSave
    print(fig2, [saveLoc filesep 'fig2_aeroAngles_' controlSuffix], '-depsc')
end

% -----------------------------------------------------------------------
% Figure 3 — Body angular rates (p, q, r)
% Shows the rotational response.  For an elevator doublet, q dominates.
% For aileron, p.  For rudder, r (with coupled p and q from cross-terms).
% -----------------------------------------------------------------------
fig3 = figure(3);
set(fig3, 'Name', 'Body Angular Rates', 'NumberTitle', 'on', 'Color', [1 1 1])

subplot(3,1,1)
plot(time, rad2deg(X(4,:)), 'LineWidth', 2)
title('Roll rate p')
xlabel('Time (s)');  ylabel('p (\circ/s)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(3,1,2)
plot(time, rad2deg(X(5,:)), 'LineWidth', 2)
title('Pitch rate q')
xlabel('Time (s)');  ylabel('q (\circ/s)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(3,1,3)
plot(time, rad2deg(X(6,:)), 'LineWidth', 2)
title('Yaw rate r')
xlabel('Time (s)');  ylabel('r (\circ/s)')
grid on;  set(gca, 'Color', [1 1 1])

if toSave
    print(fig3, [saveLoc filesep 'fig3_bodyRates_' controlSuffix], '-depsc')
end

% -----------------------------------------------------------------------
% Figure 4 — Euler attitude angles (phi, theta, psi)
% Shown in three separate subplots so that small angles are not hidden
% by a larger one on a shared axis.
% -----------------------------------------------------------------------
fig4 = figure(4);
set(fig4, 'Name', 'Euler Attitude Angles', 'NumberTitle', 'on', 'Color', [1 1 1])

subplot(3,1,1)
plot(time, euler(1,:), 'LineWidth', 2)
title('Bank angle \phi')
xlabel('Time (s)');  ylabel('\phi (\circ)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(3,1,2)
plot(time, euler(2,:), 'LineWidth', 2)
title('Pitch angle \theta')
xlabel('Time (s)');  ylabel('\theta (\circ)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(3,1,3)
plot(time, euler(3,:), 'LineWidth', 2)
title('Yaw angle \psi')
xlabel('Time (s)');  ylabel('\psi (\circ)')
grid on;  set(gca, 'Color', [1 1 1])

if toSave
    print(fig4, [saveLoc filesep 'fig4_attitude_' controlSuffix], '-depsc')
end

% -----------------------------------------------------------------------
% Figure 5 — Body-axis velocity components (u, v, w)
% Shows the translational state.  Total airspeed in aviation units is
% in Figure 7; here the focus is on the body-axis decomposition.
% -----------------------------------------------------------------------
fig5 = figure(5);
set(fig5, 'Name', 'Body Velocities', 'NumberTitle', 'on', 'Color', [1 1 1])

subplot(3,1,1)
plot(time, u_b, 'LineWidth', 2)
title('Forward velocity u')
xlabel('Time (s)');  ylabel('u (m/s)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(3,1,2)
plot(time, v_b, 'LineWidth', 2)
title('Lateral velocity v')
xlabel('Time (s)');  ylabel('v (m/s)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(3,1,3)
plot(time, w_b, 'LineWidth', 2)
title('Heave velocity w')
xlabel('Time (s)');  ylabel('w (m/s)')
grid on;  set(gca, 'Color', [1 1 1])

if toSave
    print(fig5, [saveLoc filesep 'fig5_bodyVelocities_' controlSuffix], '-depsc')
end

% -----------------------------------------------------------------------
% Figure 6 — Position and altitude (SI units)
% Altitude is separated from the horizontal position so that small
% altitude changes are not hidden by the much larger position range.
% -----------------------------------------------------------------------
fig6 = figure(6);
set(fig6, 'Name', 'Position and Altitude', 'NumberTitle', 'on', 'Color', [1 1 1])

subplot(2,1,1)
plot(time, -X(13,:), 'LineWidth', 2)
title('Altitude')
xlabel('Time (s)');  ylabel('Altitude (m)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(2,1,2)
plot(time, X(11,:), time, X(12,:), 'LineWidth', 2)
title('Horizontal position')
xlabel('Time (s)');  ylabel('Position (m)')
legend('North (x)', 'East (y)', 'Location', 'best')
grid on;  set(gca, 'Color', [1 1 1])

if toSave
    print(fig6, [saveLoc filesep 'fig6_position_' controlSuffix], '-depsc')
end

% -----------------------------------------------------------------------
% Figure 7 — Flight condition in aviation units
% Useful for comparing results against published PC-9 performance data
% and for building intuition about the magnitude of speed/altitude change.
% -----------------------------------------------------------------------
fig7 = figure(7);
set(fig7, 'Name', 'Flight Condition', 'NumberTitle', 'on', 'Color', [1 1 1])

subplot(2,1,1)
plot(time, V_kts, 'LineWidth', 2)
title('Airspeed')
xlabel('Time (s)');  ylabel('Airspeed (kn)')
grid on;  set(gca, 'Color', [1 1 1])

subplot(2,1,2)
plot(time, alt_ft, 'LineWidth', 2)
title('Altitude')
xlabel('Time (s)');  ylabel('Altitude (ft)')
grid on;  set(gca, 'Color', [1 1 1])

if toSave
    print(fig7, [saveLoc filesep 'fig7_flightCond_' controlSuffix], '-depsc')
end

% -----------------------------------------------------------------------
% Figure 8 — 3-D flight path
% A spatial view of the trajectory.  Rotate interactively in MATLAB
% (View menu -> Rotate 3D, or drag with the mouse) to inspect the path.
% -----------------------------------------------------------------------
fig8 = figure(8);
set(fig8, 'Name', '3-D Flight Path', 'NumberTitle', 'on', 'Color', [1 1 1])

plot3(X(11,:), X(12,:), -X(13,:), 'LineWidth', 2)
% Mark the start and end points
hold on
plot3(X(11,1),   X(12,1),   -X(13,1),   'go', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'g')
plot3(X(11,end), X(12,end), -X(13,end), 'rs', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'r')
hold off
xlabel('North (m)');  ylabel('East (m)');  zlabel('Altitude (m)')
title('3-D Flight Path')
legend('Path', 'Start', 'End', 'Location', 'best')
grid on;  axis equal
set(gca, 'Color', [1 1 1])

if toSave
    print(fig8, [saveLoc filesep 'fig8_flightPath_' controlSuffix], '-depsc')
end

end
