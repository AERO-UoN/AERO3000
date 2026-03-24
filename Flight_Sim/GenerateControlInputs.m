%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GenerateControlInputs.m
% AERO3000 Flight Dynamics — Assignment 3
%
% Generates (or regenerates) all control-input .mat files used by main.m.
%
% Each file saves one variable:
%   U_filter  (4 x N)  — control perturbation from trim at every time step
%       Row 1: delta_t   throttle perturbation  (fraction of full range, 0-1)
%       Row 2: delta_e   elevator perturbation  (rad, +ve = trailing-edge down)
%       Row 3: delta_a   aileron  perturbation  (rad, +ve = right aileron down)
%       Row 4: delta_r   rudder   perturbation  (rad, +ve = trailing-edge left)
%
%   N = T_END/DT + 1  (must match the time vector produced by main.m)
%
% Files generated — all use T_END = 150 s in main.m:
%   control_input_E.mat       — elevator doublet       T_END = 150 s
%   control_input_A.mat       — aileron  doublet       T_END = 150 s
%   control_input_R.mat       — rudder   doublet       T_END = 150 s
%   control_input_Phugoid.mat — phugoid excitation     T_END = 150 s
%
% Background — doublet inputs
%   A doublet is the standard flight-test excitation for mode identification.
%   It consists of a positive pulse followed immediately by an equal-and-
%   opposite negative pulse, producing zero net displacement.  This excites
%   the natural modes without causing a steady offset in heading or altitude.
%   Cosine-shaped half-cycles (raised cosine) are used here for smooth onset
%   and offset, avoiding sharp corners that would artificially inject energy
%   across all frequencies.
%
% Background — phugoid excitation
%   The phugoid is a slow (~23 s period at 100 kn) exchange between kinetic
%   and potential energy.  A brief elevator push disturbs the aircraft speed
%   and flight-path angle without strongly exciting the short-period mode
%   (which has a much shorter period and is well-damped).  A 150 s simulation
%   shows approximately six complete phugoid cycles.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
fprintf('=================================================================\n')
fprintf('  GenerateControlInputs.m — AERO3000 Assignment 3\n')
fprintf('=================================================================\n\n')

DT = 0.01;      % time step — must match DT in main.m

% =========================================================================
% Common doublet parameters  (shared by control_input_E, A, R)
% =========================================================================
T_END  = 150;               % total simulation time (s) — consistent across all files
time   = 0 : DT : T_END;
N      = length(time);

t_on   = 10;                % doublet starts at t = 10 s (10 s of trim settling)
T_half = 2;                 % half-period: 2 s positive + 2 s negative = 4 s total
amp    = deg2rad(5);        % peak amplitude ±5 deg (surface limit is ±25 deg)

% =========================================================================
% 1.  Elevator doublet  (longitudinal response)
%
%     Excites two longitudinal modes simultaneously:
%
%     Short-period mode
%       Period ~1-3 s, well-damped.  Seen as the fast pitch (q) and
%       angle-of-attack (alpha) oscillation immediately after the input.
%       Converges within ~5-10 s.
%
%     Phugoid mode
%       Period ~23 s at 100 kn, very lightly damped.  Seen as the slow,
%       long-period oscillation in airspeed (V) and altitude (h) that
%       persists for the remainder of the simulation.  With 136 s of free
%       response (t = 14-150 s), approximately 5-6 cycles are visible.
%
%     Observation tip: Figure 2 (alpha) shows the short-period decay;
%     Figure 7 (V, h) shows the phugoid oscillation.
% =========================================================================
U_filter = zeros(4, N);
U_filter(2, :) = cos_pulse(time, t_on,          T_half,  amp) ...
               + cos_pulse(time, t_on + T_half,  T_half, -amp);

save('control_input_E.mat', 'U_filter')
fprintf('  control_input_E.mat       saved  (elevator doublet +/-%.0f deg, T_END = %g s)\n', ...
    rad2deg(amp), T_END)

% =========================================================================
% 2.  Aileron doublet  (lateral-roll response)
%
%     Excites three lateral-directional modes:
%
%     Roll subsidence
%       An aperiodic (non-oscillatory), heavily-damped mode.  Seen as the
%       fast rise and decay in roll rate p immediately after the input.
%       The roll rate time constant is typically 0.5-1 s for the PC-9.
%
%     Dutch-roll mode
%       Period ~3-5 s, lightly damped.  A coupled rolling/yawing oscillation
%       visible in roll rate p, yaw rate r, and sideslip beta.  The aileron
%       doublet excites this mode through the roll-to-Dutch-roll coupling.
%
%     Spiral mode
%       Very slow (time constant >> 150 s), often near-neutral.  Not clearly
%       visible in a 150 s simulation but may appear as a very slow drift in
%       bank angle phi after the Dutch-roll decays.
%
%     Observation tip: Figure 3 (p, r) and Figure 2 (beta) show the modes.
% =========================================================================
U_filter = zeros(4, N);
U_filter(3, :) = cos_pulse(time, t_on,          T_half,  amp) ...
               + cos_pulse(time, t_on + T_half,  T_half, -amp);

save('control_input_A.mat', 'U_filter')
fprintf('  control_input_A.mat       saved  (aileron  doublet +/-%.0f deg, T_END = %g s)\n', ...
    rad2deg(amp), T_END)

% =========================================================================
% 3.  Rudder doublet  (lateral-yaw response)
%
%     Excites the same lateral-directional modes as the aileron doublet but
%     with yaw-first coupling rather than roll-first:
%
%     Dutch-roll mode
%       The yaw rate r leads, and a coupled roll rate p follows through the
%       Clr cross-derivative (roll moment due to yaw rate).  Sideslip beta
%       oscillates at the same frequency.
%
%     Comparison with aileron doublet
%       Both inputs excite the Dutch-roll mode but from opposite ends: the
%       aileron doublet drives roll first (then yaw couples in), while the
%       rudder doublet drives yaw first (then roll couples in).  Comparing
%       the two responses reinforces the cross-coupling nature of lateral-
%       directional dynamics.
%
%     Observation tip: Figure 3 (p, r) and Figure 4 (phi) reveal the
%     roll-yaw coupling.
% =========================================================================
U_filter = zeros(4, N);
U_filter(4, :) = cos_pulse(time, t_on,          T_half,  amp) ...
               + cos_pulse(time, t_on + T_half,  T_half, -amp);

save('control_input_R.mat', 'U_filter')
fprintf('  control_input_R.mat       saved  (rudder   doublet +/-%.0f deg, T_END = %g s)\n', ...
    rad2deg(amp), T_END)

% =========================================================================
% 4.  Phugoid excitation  (T_END = 150 s)
%
%     A brief +3 deg elevator push (1.5 s wide) at t = 10 s increases lift
%     momentarily, which disturbs both the airspeed and flight-path angle.
%     The input is deliberately shorter than the short-period period so that
%     the short-period mode is not strongly excited.  The remaining 138 s of
%     free response is dominated by the phugoid oscillation.
%
%     Key observable quantities:
%       Period    Read T_p directly from the airspeed or altitude plot.
%                 Theory: T_p = 2*pi*V / (g*sqrt(2)) ~ 23 s at 100 kn.
%       Damping   Measure the ratio of successive peak amplitudes.
%       Phase lag Airspeed leads altitude by ~pi/2 (quarter cycle): when
%                 speed is maximum, altitude is near its mean and climbing.
% =========================================================================
amp_ph  = deg2rad(3);       % +3 deg — gentle, avoids exciting short-period strongly
t_ph    = 10;               % pulse starts at t = 10 s (after trim settling)
T_dur   = 1.5;              % pulse width 1.5 s  (< short-period period ~1-3 s)

U_filter = zeros(4, N);
U_filter(2, :) = cos_pulse(time, t_ph, T_dur, amp_ph);

save('control_input_Phugoid.mat', 'U_filter')
fprintf('  control_input_Phugoid.mat saved  (phugoid, +%.0f deg pulse, T_END = %g s)\n', ...
    rad2deg(amp_ph), T_END)

fprintf('\n=================================================================\n')
fprintf('  Done — four files saved in the current directory.\n')
fprintf('  Open main.m, set CONTROL_FILE (T_END is already 150 s), then run.\n')
fprintf('=================================================================\n')


% =========================================================================
% Local helper — cosine-shaped smooth pulse (raised cosine)
%
%   Shape:  y(t) = amplitude * 0.5 * (1 - cos(2*pi*(t-t_start)/duration))
%
%   The output rises smoothly from zero to AMPLITUDE and returns to zero
%   over DURATION seconds.  Outside [t_start, t_start+duration] the output
%   is identically zero.
%
%   Inputs
%     t         1 x N time vector (s)
%     t_start   time at which the pulse begins (s)
%     duration  total pulse width (s)
%     amplitude signed peak value
% =========================================================================
function pulse = cos_pulse(t, t_start, duration, amplitude)
    pulse      = zeros(1, length(t));
    idx        = (t >= t_start) & (t < t_start + duration);
    pulse(idx) = amplitude * 0.5 .* (1 - cos(2*pi * (t(idx) - t_start) / duration));
end
