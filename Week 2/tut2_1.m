% Given Euler angles (degrees)
phi = deg2rad(10);    % Roll angle
theta = deg2rad(20);  % Pitch angle
psi = deg2rad(45);    % Yaw angle

% Velocity vector in body frame
V_b = [100; 5; -2]; % m/s

% Compute Aerospace Rotation Matrix (3-2-1: Yaw-Pitch-Roll)
R_be = [ cos(theta)*cos(psi),  cos(theta)*sin(psi), -sin(theta);
         sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), ...
         sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta);
         cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), ...
         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)];

% Transform velocity vector from body frame to earth frame
Reb = R_be';
V_e = Reb * V_b;

% Display results
disp('Velocity vector in the Earth frame:');
disp(V_e);
