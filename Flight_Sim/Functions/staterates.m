function [Xdot, CL, Fa_y] = staterates(Params, X, U, angle_rates)
% Computes the rates of changes of all state variables
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
%
% Outputs:
%   Xdot:   Vector containing the aircraft state rates. The order is:
%               - u_dot   = X(1)    (m/s^2)
%               - v_dot   = X(2)    (m/s^2)
%               - w_dot   = X(3)    (m/s^2)
%               - p_dot   = X(4)    (rad/s^2)
%               - q_dot   = X(5)    (rad/s^2)
%               - r_dot   = X(6)    (rad/s^2)
%               - q0_dot  = X(7)    -
%               - q1_dot  = X(8)    -
%               - q2_dot  = X(9)    -
%               - q3_dot  = X(10)   -
%               - x_dot   = X(11)   (m/s)
%               - y_dot   = X(12)   (m/s)
%               - z_dot   = X(13)   (m/s)
%   angle_rates:    Rates of change of angle attack and sideslip (rad/s)
%
% Other m-files required:
%   calculateForces.m, aeroangles.m, flowproperties.m, gravity.m,
%   windforces.m, bodyforces.m, gravForces.m, propforce.m
%
% Subfunctions:
%   calculateForces, aeroangles, flowproperties, gravity, windforces,
%   bodyforces, gravForces, propforce
%
% MAT-files required: none
%
% TODO: none

% Unpack state vector
u   = X(1);
v   = X(2);
w   = X(3);
p   = X(4);
q   = X(5);
r   = X(6);
q0  = X(7);
q1  = X(8);
q2  = X(9);
q3  = X(10);

% Calculate forces
[BodyForces, gravForces, thrust, CL] = calculateForces(Params, X, U, ...
    angle_rates);
F_body = BodyForces.Force;
M_body = BodyForces.Moment;
[Fgx, Fgy, Fgz] = gravForces{:};
Ft = thrust;

% Unpack forces
Fa_x = F_body(1);
Fa_y = F_body(2);
Fa_z = F_body(3);
La = M_body(1);
Ma = M_body(2);
Na = M_body(3);

% Extract parameters
m = Params.Inertial.m;
Ixx = Params.Inertial.Ixx;
Iyy = Params.Inertial.Iyy;
Izz = Params.Inertial.Izz ;
Ixz = Params.Inertial.Ixz;

% Calculate inertial constants
c0 = Ixx*Izz - Ixz^2;
c1 = Izz/c0;
c2 = Ixz/c0;
c3 = c2*(Ixx - Iyy + Izz);
c4 = c1*(Iyy - Izz) - c2*Ixz;
c5 = 1/Iyy;
c6 = c5*Ixz;
c7 = c5*(Izz - Ixx);
c8 = Ixx/c0;
c9 = c8*(Ixx - Iyy) + c2*Ixz;

% Calculate body accelerations
udot = r.*v - q.*w + Fgx/m + (Fa_x + Ft)./m;
vdot = -r.*u + p.*w + Fgy/m + Fa_y./m;
wdot = q.*u - p.*v + Fgz/m + Fa_z./m;

% Calculate manoeuvring rates
pdot = c3.*p.*q + c4.*q.*r + c1.*La + c2.*Na;
qdot = c7.*p.*r - c6.*(p.^2 - r.^2) + c5.*Ma;
rdot = c9.*p.*q - c3.*q.*r + c2.*La + c8.*Na;

% Calculate quaternion rates
q0dot = -0.5.*(q1.*p + q2.*q + q3.*r);
q1dot = 0.5.*(q0.*p - q3.*q + q2.*r);
q2dot = 0.5.*(q3.*p + q0.*q - q1.*r);
q3dot = -0.5.*(q2.*p - q1.*q - q0.*r);

% Calculate position rates
Cbe = rotate321quat([q0;q1;q2;q3]);
Ceb = Cbe';
position = Ceb*[u;v;w];

% Create output
Xdot = [udot;vdot;wdot;pdot;qdot;rdot;q0dot;q1dot;q2dot;q3dot;...
    position(1);position(2);position(3)];
end