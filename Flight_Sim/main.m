%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PC-9 Flight Dynamics Simulator
% AERO3000 — Assignment 3
%
% This script:
%   1. Loads the PC-9 aircraft data
%   2. Verifies the trim solver at two flight conditions
%   3. Runs a nonlinear time-domain simulation with a pre-defined control input
%   4. Plots all results
%
% See README.txt for a full description of the code package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear

%% Setup
addpath('Functions\');
params = LoadFlightDataPC9();


%% ======================================================================
%  STUDENT CONFIGURATION — change these settings to run different cases
%  ======================================================================

% Flight condition
%   '100'  — 100 kn at 20 m altitude
%   '180'  — 180 kn at 1000 ft altitude
SPEED = '100';

% Control input sequence
%   Each file contains a (4 x N) matrix called U_filter: the deviation
%   from the trim input applied at each time step.  All files use T_END = 150 s.
%
%   control_input_E.mat       — elevator doublet      (longitudinal modes)
%   control_input_A.mat       — aileron  doublet      (lateral-roll modes)
%   control_input_R.mat       — rudder   doublet      (lateral-yaw modes)
%   control_input_Phugoid.mat — phugoid excitation    (slow long-period mode)
CONTROL_FILE = 'control_input_E.mat';

% Simulation time settings
T_END = 150;    % total simulation duration (s)
DT    = 0.01;   % RK4 integration time step (s)

%  ======================================================================


%% Trim verification
% Build a minimal initial state (wings-level, zero angular rates) at each
% flight condition, trim the aircraft, and confirm that all state rates
% are driven to near zero.  This is a sanity check for the trim solver.

% Test case 1: 100 kn at 20 m
u_test1   = 100 * 0.5144;                    % kn -> m/s
X_test1   = [u_test1,0,0, 0,0,0, 1,0,0,0, 0,0,-20]';

% Test case 2: 180 kn at 1000 ft
u_test2   = 180 * 0.5144;                    % kn -> m/s
alt_test2 = convlength(1000, 'ft', 'm');     % ft -> m
X_test2   = [u_test2,0,0, 0,0,0, 1,0,0,0, 0,0,-alt_test2]';

[X_trimmedTest1, U_trimmedTest1] = trim(params, X_test1);
[X_trimmedTest2, U_trimmedTest2] = trim(params, X_test2);

Xdot_Test1 = getstaterates(params, X_trimmedTest1, U_trimmedTest1);
Xdot_Test2 = getstaterates(params, X_trimmedTest2, U_trimmedTest2);

fprintf('\n============================================================\n')
fprintf('Trim verification — all state rates should be near zero\n')
fprintf('============================================================\n')
fprintf('Test case 1 (100 kn, 20 m)\n')
fprintf('  u_dot = %+.2e   v_dot = %+.2e   w_dot = %+.2e  (m/s^2)\n', ...
    Xdot_Test1(1), Xdot_Test1(2), Xdot_Test1(3))
fprintf('  p_dot = %+.2e   q_dot = %+.2e   r_dot = %+.2e  (rad/s^2)\n', ...
    Xdot_Test1(4), Xdot_Test1(5), Xdot_Test1(6))
fprintf('Test case 2 (180 kn, 1000 ft)\n')
fprintf('  u_dot = %+.2e   v_dot = %+.2e   w_dot = %+.2e  (m/s^2)\n', ...
    Xdot_Test2(1), Xdot_Test2(2), Xdot_Test2(3))
fprintf('  p_dot = %+.2e   q_dot = %+.2e   r_dot = %+.2e  (rad/s^2)\n', ...
    Xdot_Test2(4), Xdot_Test2(5), Xdot_Test2(6))
fprintf('============================================================\n\n')


%% Load initial state for the selected flight condition
switch SPEED
    case '100'
        load InitialCondition1_100Kn
        fcLabel = '100 kn / 20 m';
    case '180'
        load InitialCondition2_180Kn
        fcLabel = '180 kn / 1000 ft';
    otherwise
        error('SPEED must be ''100'' or ''180''.')
end

% The .mat files store attitude as Euler angles [phi, theta, psi].
% Convert to the quaternion representation used by this simulator.
X_initial = [X0(1:6); euler2quat(X0(7:9)); X0(10:end)];


%% Trim the aircraft at the selected flight condition
[X_trimmed, U_trimmed] = trim(params, X_initial);

% Print a readable trim summary
euler_trim = rad2deg(quat2euler(X_trimmed(7:10)));
fprintf('Trimmed flight condition: %s\n', fcLabel)
fprintf('  Airspeed        : %5.1f kn   (%5.2f m/s)\n', ...
    norm(X_trimmed(1:3)) * 1.94384, norm(X_trimmed(1:3)))
fprintf('  Altitude        : %5.0f ft   (%5.1f m)\n', ...
    -X_trimmed(13) * 3.28084, -X_trimmed(13))
fprintf('  Angle of attack : %+6.3f deg\n', ...
    rad2deg(atan(X_trimmed(3) / X_trimmed(1))))
fprintf('  Pitch angle     : %+6.3f deg\n', euler_trim(2))
fprintf('  Throttle        : %6.3f\n',      U_trimmed(1))
fprintf('  Elevator        : %+6.3f deg\n', rad2deg(U_trimmed(2)))
fprintf('\n')


%% Nonlinear simulation
time = 0:DT:T_END;
nSteps = length(time);

% Preallocate state and control history matrices for speed
X = zeros(13, nSteps);
U = zeros(4,  nSteps);

X(:,1) = X_trimmed;
U(:,1) = U_trimmed;

% Load the chosen control perturbation sequence
load(CONTROL_FILE)

fprintf('Running nonlinear simulation  (%g s,  dt = %g s) ...\n', T_END, DT)

for i = 2:nSteps
    % Add the pre-recorded perturbation to the trim control input
    U_manoeurve = U_trimmed + U_filter(1:4, i);

    % Advance the state one step with 4th-order Runge-Kutta
    X(:,i) = rungeKutta4(params, X(:,i-1), U_manoeurve, DT);
    U(:,i) = U_manoeurve;
end

fprintf('Simulation complete.\n\n')


%% Plot results
plotData(X, U, time)
