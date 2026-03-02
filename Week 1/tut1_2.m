function V = computeAirspeed(u, v, w)
% COMPUTEAIRSPEED  Compute total airspeed from body-frame velocity components.
%
%   V = computeAirspeed(u, v, w) returns the total airspeed V (m/s) given
%   the three orthogonal velocity components resolved in the aircraft body
%   frame:
%       u  - axial velocity component   (positive forward)    [m/s]
%       v  - lateral velocity component (positive starboard)  [m/s]
%       w  - normal velocity component  (positive downward)   [m/s]
%
%   The airspeed is the Euclidean magnitude of the velocity vector:
%       V = sqrt(u^2 + v^2 + w^2)
%
%   From these same components, the aerodynamic angles can also be derived:
%       alpha = atan2(w, u)   (angle of attack, positive nose-up)
%       beta  = asin(v / V)   (sideslip angle,  positive nose-right)
%
%   All inputs may be scalars or equal-length arrays (element-wise).
%
%   Example:
%       V = computeAirspeed(100, 5, -2)   % returns ~100.14 m/s

    V = sqrt(u.^2 + v.^2 + w.^2);
end
