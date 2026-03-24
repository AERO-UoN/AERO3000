function [V, alpha, beta] = aeroangles(X)
% Function Description:
%   Converts body velocity vector into flight vector components
%
% Inputs:
%   X:  Vector containing the aircraft state. The order is:
%           - u   = X(1)    (m/s)
%           - v   = X(2)    (m/s)
%           - w   = X(3)    (m/s)
%           - p   = X(4)    (rad/s)
%           - q   = X(5)    (rad/s)
%           - r   = X(6)    (rad/s)
%           - q0  = X(7)    -
%           - q1  = X(8)    -
%           - q2  = X(9)    -
%           - q3  = X(10)   -
%           - x   = X(11)   (m)
%           - y   = X(12)   (m)
%           - z   = X(13)   (m)
%
% Outputs:
%   V:      Total velocity magnitude (m/s)
%   alpha:  Angle of attack (rad)
%   beta:   Side slip angle (rad)
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Unpack state vector
u   = X(1);
v   = X(2);
w   = X(3);

% Calculate total velocity
V = sqrt(u.^2 + v.^2 + w.^2);

% Calculate angle of attack
alpha = atan(w./u);

% Calculate side slip angle
beta = asin(v/V);
end