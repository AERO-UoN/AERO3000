% Given Euler angles (degrees)
phi = deg2rad(15);   % Roll angle
theta = deg2rad(30); % Pitch angle
psi = deg2rad(60);   % Yaw angle

% Compute half angles
half_phi = phi / 2;
half_theta = theta / 2;
half_psi = psi / 2;

% Compute quaternion components
q0 = cos(half_phi) * cos(half_theta) * cos(half_psi) + sin(half_phi) * sin(half_theta) * sin(half_psi);
q1 = sin(half_phi) * cos(half_theta) * cos(half_psi) - cos(half_phi) * sin(half_theta) * sin(half_psi);
q2 = cos(half_phi) * sin(half_theta) * cos(half_psi) + sin(half_phi) * cos(half_theta) * sin(half_psi);
q3 = cos(half_phi) * cos(half_theta) * sin(half_psi) - sin(half_phi) * sin(half_theta) * cos(half_psi);

% Normalize quaternion
q = [q0; q1; q2; q3];
q_norm = q / norm(q);

% Display results
disp('Quaternion representation:');
disp(q_norm);
