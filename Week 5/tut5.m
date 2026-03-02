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

% PART 3: Dameged side
% Pick one option from belwo and delete the other
damage_side = 'Left';
% damage_side = 'Right';

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
