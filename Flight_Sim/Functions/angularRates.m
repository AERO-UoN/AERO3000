function [alpha_dot, beta_dot] = angularRates(Xdot,X)
% Function Description:
%   Estimates the angle of attack and sideslip rates in rad/s
%
% Inputs:
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
%
% Outputs:
%   alpha_dot:  Rate of change of angle of attack (rad/s)
%   beta_dot:   Rate of change of side slip (rad/s)
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Unpack state vector
u = X(1);
v = X(2);
w = X(3);

% Unpack state rate vector
u_dot = Xdot(1);
v_dot = Xdot(2);
w_dot = Xdot(3);

V = aeroangles(X);

% Calcualte alpha_dot
% alpha_dot = (w_dot*u - w*u_dot)/(u^2);
alpha_dot = (w_dot*u - w*u_dot)/(u^2+w^2);

% Calculate beta_dot 
% beta_dot = (v_dot*u - u_dot*v)/(u^2);
beta_dot = (v_dot*V - v*u_dot)/(V^2);
end