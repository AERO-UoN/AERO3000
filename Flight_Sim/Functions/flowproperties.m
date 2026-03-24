function [rho, Q] = flowproperties(X, V)
%   Returns the air density and dynamic pressure, calculated as a function
%   of aircraft altitude and speed
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
%   V:      Total velocity magnitude (m/s)
%
% Outputs:
%   rho:    Density of air at aircraft altitude (kg/m^3)
%   Q:      Dynamic pressure at aircraft altitude (Pa)
%
% Other m-files required: none
%
% Subfunctions: none
%
% MAT-files required: none
%
% TODO: none

% Constants
T0  = 288.15;           % ISA SSL temperature (K)
P0  = 101.325;          % ISA SSL pressure (kPa)
R   = 0.287058;         % Ideal gas constant for air (kJ/kg.K)

% Extract altitude (m) from state vector
z   = X(13);
h   = -z;

% Calculate atmospheric conditions based on altitude

% Troposphere
if h <= 11000

    % Temperature (K)
    temp = T0 - 0.0065*h;

    % Pressure (kPa)
    pressure = P0*(temp/T0)^5.256;

    % Lower stratosphere
elseif h <= 25000

    % Temperature (K)
    temp = -56.46 + 273.15;

    % Pressure (kPa)
    pressure = 22.65*exp(1.73 - 0.000157*h);

    % Upper stratosphere
else

    % Stop simulation and display error message
    string = ['Altitude places aircraft in the upper stratosphere!',...
        ' Re-evaluate calculation of z in Earth axes!'];
    error(string);
end

% Calculate density (kg/m^3)
rho = pressure/(R*temp);

% Calculate dynamic pressure (Pa)
Q = (1/2)*rho*V^2;
end