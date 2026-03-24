function [BodyForces, gravForces, thrust, CL] = calculateForces(Params, X, U, angle_rates)
%   Returns all forces acting on the aircraft given its physical
%   characteristics, present state and control inputs
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
%   U:      Vector containing all aircraft control settings. The order is:
%               - delta_t = U(1)    -
%               - delta_e = U(2)    (rad)
%               - delta_a = U(3)    (rad)
%               - delta_r = U(4)    (rad)
%   angle_rates: attack and sideslip rates in rad/s
%
% Outputs:
%   BodyForces: Struct containing two fields, "Force" and "Moment", for the
%               aircraft's body forces and moments (N and Nm)
%   gravForces: Cell array containing the cartesian components of the
%               aircraft's weight when projected into the body frame (N)
%   thrust:     Thrust of the aircraft (N)
%
% Other m-files required:
%   aeroangles.m, flowproperties.m, gravity.m, windforces.m, bodyforces.m,
%   gravForces.m, propforce.m
%
% Subfunctions:
%   aeroangles, flowproperties, gravity, windforces, bodyforces,
%   gravForces, propforce
%
% MAT-files required: none
%
% TODO: none

% Get aerodynamic angles
[V, alpha, beta] = aeroangles(X);

% Get flow properties
[rho, Q]  = flowproperties(X, V);

% Calculate gravity forces
[Fgx, Fgy, Fgz] = gravity(Params, X);

% Calculate wind forces
[Cfa_z, Cfa_x, CL] = windforces(Params, alpha, X, U, V, angle_rates);

% Calculate body forces
[F_body, M_body] = bodyforces(Params, X, U, Cfa_x, Cfa_z, ...
    Q, alpha, beta, angle_rates(1), angle_rates(2),V);

% Concatenate gravity forces into cell array
gravForces = {Fgx, Fgy, Fgz};

% Calculate thrust force
thrust = propforce(Params, X, U, rho);

% Save body forces to struct
BodyForces.Force    = F_body;
BodyForces.Moment   = M_body;
end