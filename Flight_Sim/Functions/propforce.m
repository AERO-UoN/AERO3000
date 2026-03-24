function thrust = propforce(Params, X, U, rho)
%   Determines the thrust produced by the propellor in the body frame
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
%   rho:    Density of air at aircraft altitude (kg/m^3)
%
% Outputs:
%   thrust: Thrust of the aircraft (N)
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Extract aircraft parameters
PmaxSL = Params.Prop.P_max;
eta = Params.Prop.eta;

% Extract state
u = X(1);

% Extract control
delta_t = U(1);

% Calculate available thrust (AERO2000)
rho_SL = 1.2256;
sigma = rho/rho_SL;
Pmax = PmaxSL*(1.1324*sigma - 0.1324);

% Calculate thrust produced
thrust = (Pmax*eta*delta_t)/u;

end