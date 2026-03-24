function [Fgx, Fgy, Fgz] = gravity(Params, X)
%   Determines the force due to gravity and converts it to the body frame
%
% Inputs:
%   Params: Struct containing all characteristics of the aircraft
%   X:      Vector containing the aircraft state. The order is:
%               - u   = X(1)    (m/s)
%               - v   = X(2)    (m/s)
%               - w   = X(3)    (m/s)
%               - p   = X(4)    (rad/s)
%               - q   = X(5)    (rad/s)
%               - r   = X(6)    (rad/s)
%               - q0  = X(7)    -
%               - q1  = X(8)    -
%               - q2  = X(9)    -
%               - q3  = X(10)   -
%               - x   = X(11)   (m)
%               - y   = X(12)   (m)
%               - z   = X(13)   (m)
%
% Outputs:
%   Fgx: Body x-component of weight (N)
%   Fgy: Body y-component of weight (N)
%   Fgz: Body z-component of weight (N)
%
% Other m-files required:
%   rotate321quat.m
%
% Subfunctions:
%   rotate321quat
%
% MAT-files required: none
%
% TODO: none

% Extract parameters
g = Params.Inertial.g;
m = Params.Inertial.m;

% Calculate weight force in Earth Frame
W = [0; 0; m*g];

% Create transformation matrix from Earth to Body 
C_be = rotate321quat(X(7:10));

% Transform force
Fg  = C_be*W;
Fgx = Fg(1);
Fgy = Fg(2);
Fgz = Fg(3);
end