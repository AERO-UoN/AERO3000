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
phi = asin(-Y / (m * g)); % Roll angle in radians
phi_deg = rad2deg(phi); % Convert to degrees

% Display results
disp('Aircraft Equilibrium Properties:');
disp(['Mass (kg): ', num2str(m)]);
disp(['Thrust Required (N): ', num2str(T)]);
disp(['Roll Angle (degrees): ', num2str(phi_deg)]);
