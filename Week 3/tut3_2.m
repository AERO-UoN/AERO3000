% =========================================================================
% Tutorial 3.2 - Aircraft Lateral Equilibrium Analysis
% =========================================================================
% PURPOSE:
%   Compute the mass, thrust, and roll angle of an aircraft in trimmed
%   (equilibrium) flight using aerodynamic force coefficients and a simple
%   force balance.
%
% CONCEPTS TAUGHT:
%   Equilibrium (trim) flight: net forces and moments acting on the
%   aircraft are zero.  For this steady flight condition the force balance
%   simplifies to three scalar equations:
%       Lift  = Weight        =>  L = mg          (vertical equilibrium)
%       Thrust = Drag         =>  T = D           (longitudinal equilibrium)
%       Side force + W*sin(phi) = 0               (lateral equilibrium)
%
%   Linear aerodynamic model:
%     CL = CL1 + CL_alpha*(alpha - alpha_1)   (lift curve, linearised)
%     CD = CD1 + CD_alpha*(alpha - alpha_1)   (drag polar, linearised)
%     CY = CY_beta * beta                     (side force from sideslip)
%
%   Roll angle from sideslip:
%     A non-zero sideslip angle beta generates a side force Y.  In lateral
%     equilibrium the weight component W*sin(phi) must cancel this force:
%       sin(phi) = -Y / (m*g)   =>   phi = asin(-Y / mg)
%     This requires |Y| < mg; the code checks this condition explicitly.
%
% KEY ASSUMPTIONS:
%   - Steady, level flight (constant altitude and speed, no acceleration)
%   - Longitudinal and lateral equilibrium are decoupled
%   - Linear aerodynamic model is valid near the reference condition alpha_1
%
% EXPECTED OUTPUT:
%   - m   [kg]  : aircraft mass from the lift = weight condition
%   - T   [N]   : thrust required to overcome aerodynamic drag
%   - phi [deg] : bank angle required to balance the aerodynamic side force
%
% Reference: Nelson, R.C. (1998) Flight Stability and Automatic Control,
%            Ch.2 (Equations of Motion and Equilibrium).
% =========================================================================

% Given conditions
V = 67; % True airspeed (m/s)
rho = 1.225; % Air density at sea level (kg/m^3)
S = 10; % Wing planform area (m^2)
g = 9.81; % Gravity (m/s^2)

% Aerodynamic angles (convert degrees to radians)
alpha = deg2rad(7); % Angle of attack
beta = deg2rad(5); % Sideslip angle
alpha_1 = deg2rad(5.5); % Reference AoA

% Aerodynamic coefficients
CL1 = 0.3;
CD1 = 0.0295;
CL_alpha = 5.16;
CD_alpha = 0.214;
CY_beta = -0.176;

% Compute lift coefficient
CL = CL1 + CL_alpha * (alpha - alpha_1);

% Compute drag coefficient
CD = CD1 + CD_alpha * (alpha - alpha_1);

% Compute side force coefficient
CY = CY_beta * beta;

% Compute lift force
L = 0.5 * rho * V^2 * S * CL;

% Compute mass from equilibrium condition (L = W = mg)
m = L / g;

% Compute drag force (Thrust required for equilibrium flight)
T = 0.5 * rho * V^2 * S * CD;

% Compute side force
Y = 0.5 * rho * V^2 * S * CY;

% Compute roll angle (phi) from lateral equilibrium: Y + W * sin(phi) = 0
ratio = -Y / (m * g);
if abs(ratio) > 1
    error('Lateral equilibrium is not possible: |Y| exceeds weight (|Y/(mg)| = %.4f > 1).', abs(ratio));
end
phi = asin(ratio); % Roll angle in radians
phi_deg = rad2deg(phi); % Convert to degrees

% Display results
disp('Aircraft Equilibrium Properties:');
disp(['Mass (kg): ', num2str(m)]);
disp(['Thrust Required (N): ', num2str(T)]);
disp(['Roll Angle (degrees): ', num2str(phi_deg)]);
