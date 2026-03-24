function euler_angles = quat2euler(quaternion)
%   Returns the Euler angles for a given quaternion input matrix
%
% Inputs:
%   quaternion: matrix containing quaternions. They are stored in the
%               order: [q0; q1; q2; q3]
%
% Outputs:
%   euler_angles: matrix of Euler angles (radians): [phi; theta; psi] =
%                 [roll; pitch; yaw]
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Unpack quaternion matrix
q0 = quaternion(1,:);
q1 = quaternion(2,:);
q2 = quaternion(3,:);
q3 = quaternion(4,:);

% Calculate roll angle (phi) (radians)
num1    = q2.*q3 + q0.*q1;
denom1  = q0.^2 + q3.^2 - 1/2;
phi     = atan2(num1,denom1);

% Calculate pitch angle (theta) (radians)
num2    = q0.*q2 - q1.*q3;
denom2  = ((q0.^2 + q1.^2-1/2).^2 + (q1.*q2+q0.*q3).^2).^(1/2);
theta   = atan2(num2,denom2);

% Calculate yaw angle (psi) (radians)
num3    = q1.*q2 + q0.*q3;
denom3  = q0.^2 + q1.^2 - 1/2;
psi     = atan2(num3,denom3);

% Return euler angle vector (radians)
euler_angles = [phi; theta; psi];
end