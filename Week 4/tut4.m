% =========================================================================
% Tutorial 4 - Longitudinal Static Stability: Neutral Point and Static Margin
% =========================================================================
% PURPOSE:
%   Calculate the stick-fixed neutral point (x_NP) and static margin (K_n)
%   for a conventional aircraft configuration, and determine whether the
%   aircraft is statically stable in the longitudinal (pitch) axis.
%
% CONCEPTS TAUGHT:
%   Static stability (pitch):
%     An aircraft is statically stable in pitch if a nose-up disturbance
%     generates a restoring (nose-down) pitching moment.  The key parameter
%     governing this is the location of the CG relative to the neutral point.
%
%   Neutral point (x_NP):
%     The CG location at which the aircraft is neutrally stable — the
%     pitching moment does not change with angle of attack (dCm/dalpha = 0).
%     Locations are expressed as fractions of the mean aerodynamic chord.
%     x_NP is computed from wing and tail contributions:
%       x_NP = x_ac + eta_t * V_bar * (C_L_alpha_t / C_L_alpha) * (1 - deps_dalpha)
%     where V_bar = S_t * l_t_bar / (S * c_bar) is the tail volume coefficient.
%
%   Static margin (K_n):
%     K_n = x_NP - x_cg
%     K_n > 0  =>  CG is ahead of the neutral point  =>  STABLE
%     K_n = 0  =>  CG is at the neutral point         =>  NEUTRALLY STABLE
%     K_n < 0  =>  CG is behind the neutral point     =>  UNSTABLE
%     Typical civil aircraft: K_n = 5–15% MAC (positive margin for safety).
%     Fighter aircraft may be designed with small or negative margins
%     (relaxed static stability) for agility, relying on active control.
%
%   Downwash (deps_dalpha):
%     The wing generates a downward-deflected wake (downwash) that reduces
%     the effective angle of attack seen by the horizontal tail.  This
%     reduces the tail's stabilising contribution by the factor (1 - deps_dalpha).
%
% KEY ASSUMPTIONS:
%   - Stick-fixed analysis (elevator is fixed; pilot not moving the stick)
%   - Linear aerodynamics (constant lift curve slopes)
%   - Symmetric flight (no sideslip or roll)
%   - Fuselage and power effects on Cm are neglected
%
% EXPECTED OUTPUT:
%   - C_L_alpha   [1/rad]: total aircraft lift curve slope
%   - x_NP        [fraction MAC]: stick-fixed neutral point location
%   - K_n         [fraction MAC]: static margin (positive = stable)
%   - Stability verdict printed to the command window
%
% Reference: Nelson, R.C. (1998) Flight Stability and Automatic Control,
%            Ch.3 (Static Stability and Control).
% =========================================================================

% Given data
x_cg = 0.235; % Center of gravity location (fraction of mean aerodynamic chord)
x_ac = 0.25;  % Aerodynamic center location (fraction of mean aerodynamic chord)
c_bar = 1.652; % Mean aerodynamic chord (m)
eta = 0.8; % Horizontal tail efficiency
C_L_alpha_w = 5.827; % Lift curve slope of main wing (per radian)
S = 16.3; % Wing area (m^2)
l_t = 4.257; % Tail moment arm (m)
deps_dalpha = 0.6; % Downwash gradient
eta_t = 1; % Tail efficiency factor
C_L_alpha_t = 4.120; % Lift curve slope of tail (per radian)
S_t = 2.09; % Tail area (m^2)

% Compute total lift curve slope of the complete aircraft
C_L_alpha = C_L_alpha_w + (eta_t * S_t / S) * C_L_alpha_t * (1 - deps_dalpha);

% Compute stick-fixed neutral point location (x_NP)
l_t_bar = l_t + (x_cg-x_ac)*c_bar;
V_bar =  (S_t*l_t_bar)/(S*c_bar);
x_NP = x_ac + eta_t*V_bar* (C_L_alpha_t / C_L_alpha) * (1 - deps_dalpha);

% Compute static margin
K_n = x_NP - x_cg;

% Display results
disp(['Total Aircraft Lift Curve Slope (C_L_alpha_total): ', num2str(C_L_alpha, 4), ' per radian']);
disp(['Stick-Fixed Neutral Point (x_NP): ', num2str(x_NP, 4), ' * c_bar']);
disp(['Static Margin (K_n): ', num2str(K_n, 4)]);

% Stability interpretation
if K_n > 0
    disp('The aircraft is STABLE (Positive Static Margin).');
elseif K_n == 0
    disp('The aircraft is NEUTRALLY STABLE.');
else
    disp('The aircraft is UNSTABLE (Negative Static Margin).');
end

