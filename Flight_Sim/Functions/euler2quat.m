function quaternion = euler2quat(euler_angles)
% Function Name: euler2quat
%
% Function Description:
%   Returns the quaternion components for a given Euler angle input matrix
%
% Inputs:
%   euler_angles: matrix containing Euler angles (radians). The angles are
%                 stored in order: [phi; theta; psi] = [roll; pitch; yaw]
%
% Outputs:
%   quaternion: matrix of quaternion components [q0; q1; q2; q3]
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Unpack Euler angles matrix
phi     = euler_angles(1,:);
theta   = euler_angles(2,:);
psi     = euler_angles(3,:);

% Calculate quaternion components
q0  = cos(psi/2).*cos(theta/2).*cos(phi/2) + ...
    sin(psi/2).*sin(theta/2).*sin(phi/2);
q1  = cos(psi/2).*cos(theta/2).*sin(phi/2) - ...
    sin(psi/2).*sin(theta/2).*cos(phi/2);
q2  = cos(psi/2).*sin(theta/2).*cos(phi/2) + ...
    sin(psi/2).*cos(theta/2).*sin(phi/2);
q3  = -cos(psi/2).*sin(theta/2).*sin(phi/2) + ...
    sin(psi/2).*cos(theta/2).*cos(phi/2);

% Return quaternion vector
quaternion = [q0; q1; q2; q3];
end