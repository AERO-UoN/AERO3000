% =========================================================================
% Tutorial 5 - Lateral-Directional Equilibrium and Damaged Aircraft Control
% =========================================================================
% PURPOSE:
%   Determine the control surface deflections (aileron delta_a, rudder
%   delta_r) and sideslip angle (beta) required to maintain equilibrium
%   flight for a STOL aircraft with structural damage to one wing.
%   Two scenarios are analysed: wings-level flight and a standard-rate turn.
%
% CONCEPTS TAUGHT:
%   Lateral-directional equilibrium:
%     In steady flight with constant heading and no roll, the lateral force,
%     rolling moment, and yawing moment must all be zero.  This gives three
%     simultaneous linear equations in three unknowns (beta, delta_a, delta_r):
%       [CYb   CYda  CYdr] [beta   ]   [ 0    ]
%       [Clb   Clda  Cldr] [delta_a] = [ 0    ]
%       [Cnb   Cnda  Cndr] [delta_r]   [-Cn0  ]
%     where Cn0 is the residual yawing moment due to wing damage.
%
%   Stability derivatives (all per radian):
%     CYb  - side force due to sideslip
%     Clb  - rolling moment due to sideslip (dihedral effect)
%     Cnb  - yawing moment due to sideslip (weathercock stability)
%     Clda - rolling moment due to aileron deflection
%     Cndr - yawing moment due to rudder deflection  (etc.)
%
%   Standard-rate turn (Part 2):
%     A standard-rate turn completes 360 deg in 2 minutes, giving a yaw
%     rate r = pi/60 rad/s.  The non-dimensional yaw rate is:
%       r_hat = r * b / (2V)
%     Cross-coupling from the yaw rate (Clr, Cnr terms) must be balanced
%     by the control surfaces.
%
%   Damaged aircraft (Part 3):
%     Wing structural damage creates asymmetric drag, modelled as a
%     constant yawing moment Cn0.  The sign of Cn0 reveals which wing:
%       Cn0 < 0  =>  net yaw is nose-left  =>  LEFT wing damaged
%       Cn0 > 0  =>  net yaw is nose-right =>  RIGHT wing damaged
%
% KEY ASSUMPTIONS:
%   - Steady flight: all time derivatives and accelerations are zero
%   - p-hat = 0 (no roll rate) throughout
%   - Linear aerodynamics (constant stability derivatives)
%   - Damage is modelled as a fixed offset moment Cn0; no change to other
%     derivatives is assumed
%
% EXPECTED OUTPUT:
%   - Part 1: beta [deg], delta_a [deg], delta_r [deg] for wings-level flight
%   - Part 2: delta_a [deg], delta_r [deg] for the standard-rate turn
%   - Part 3: which wing was damaged, inferred from the sign of Cn0
%
% Reference: Nelson, R.C. (1998) Flight Stability and Automatic Control,
%            Ch.5 (Lateral-Directional Stability and Control).
% =========================================================================

% Load aircraft stability data from provided script
FlightData = load_stol_data();
Aero = FlightData.Aero;
Geom = FlightData.Geom;
Oper = FlightData.Oper;

% Given yawing moment due to damage
C_n0 = -0.023;

%% PART 1: Wings-Level Flight (Solving for beta, delta_a, delta_r)
% Lateral-Directional Equilibrium Equations
% Assumptions: p-hat = 0, r-hat = 0, phi = 0

% Define coefficient matrix
A1 = [Aero.CYb, Aero.CYda, Aero.CYdr;
      Aero.Clb, Aero.Clda, Aero.Cldr;
      Aero.Cnb, Aero.Cnda, Aero.Cndr];

% Define right-hand side vector (accounting for yawing moment due to damage)
B1 = [0; 0; -C_n0];

% Solve for unknowns (beta, delta_a, delta_r)
X1 = A1 \ B1;
beta_deg = rad2deg(X1(1));
delta_a_deg = rad2deg(X1(2));
delta_r_deg = rad2deg(X1(3));

%% PART 2: Standard-Rate Turn (Solving for delta_a, delta_r)
% Assumptions: p-hat = 0, beta = 0, r-hat ≠ 0 (nonzero yaw rate)

% Compute standard-rate yaw rate (rad/s)
b = Geom.b; % Wingspan
V = Oper.V; % Airspeed
r_hat = (pi / 60) * (b / (2 * V)); % Using r = pi/60 rad/s for a 2-min turn

% Define coefficient matrix
A2 = [Aero.Clda, Aero.Cldr;
      Aero.Cnda, Aero.Cndr];


% Define right-hand side vector (including yawing moment effect)
B2 = [-Aero.Clr * r_hat;
      -C_n0 - Aero.Cnr * r_hat];

% Solve for unknowns (delta_a, delta_r)
X2 = A2 \ B2;
delta_a_turn_deg = rad2deg(X2(1));
delta_r_turn_deg = rad2deg(X2(2));

%% PART 3: Determine Damaged Wing Side
% The residual yawing moment C_n0 is caused by asymmetric drag from the damaged wing.
% Sign convention: positive yaw = nose-right.
%   C_n0 < 0  →  net yaw is nose-left  →  extra drag on the LEFT wing  →  LEFT wing damaged
%   C_n0 > 0  →  net yaw is nose-right →  extra drag on the RIGHT wing →  RIGHT wing damaged
if C_n0 < 0
    damage_side = 'Left';
elseif C_n0 > 0
    damage_side = 'Right';
else
    damage_side = 'Unknown (C_n0 = 0, no asymmetric damage detected)';
end

%% Display Results
disp('Wings-Level Flight:');
disp(['Sideslip Angle (beta) = ', num2str((beta_deg)), ' degrees']);
disp(['Aileron Deflection (delta_a) = ', num2str((delta_a_deg)), ' degrees']);
disp(['Rudder Deflection (delta_r) = ', num2str((delta_r_deg)), ' degrees']);

disp(' ');
disp('Standard-Rate Turn:');
disp(['Aileron Deflection (delta_a) = ', num2str((delta_a_turn_deg)), ' degrees']);
disp(['Rudder Deflection (delta_r) = ', num2str((delta_r_turn_deg)), ' degrees']);

disp(' ');
%% Determine Which Wing Was Damaged
disp(['The ', damage_side, ' wing was damaged.']);
