function FlightData = load_stol_data()
% LOAD_STOL_DATA  Return a structure of geometry, inertia, and aerodynamic
%                 data for an example STOL transport aircraft.
%
%   FlightData = load_stol_data() returns a scalar struct with four fields:
%
%   FlightData.Geom      — Geometric properties (SI units)
%     .S      Wing planform area                    [m^2]
%     .b      Wing span                             [m]
%     .cbar   Wing mean aerodynamic chord (MAC)     [m]
%     .AR     Wing aspect ratio                     [-]
%     .lf     Fuselage length                       [m]
%     .qcw    Wing quarter-chord location from nose [m]
%     .cg     Nominal CG location from leading edge [m]
%     .St     Horizontal tail planform area         [m^2]
%     .bt     Horizontal tail span                  [m]
%     .ctbar  Horizontal tail MAC                   [m]
%     .ARt    Horizontal tail aspect ratio          [-]
%     .lt     Tail moment arm (with nominal CG)     [m]
%     .Se     Elevator area                         [m^2]
%     .wf     Fuselage width                        [m]
%     .zv     Vertical tail vertical offset         [m]
%     .lv     Vertical tail horizontal offset       [m]
%     .Sv     Vertical tail planform area           [m^2]
%     .Vv     Vertical tail volume coefficient      [-]
%
%   FlightData.Inertial  — Mass and inertia properties (SI units)
%     .m      Aircraft mass                         [kg]
%     .Ixx    Roll  moment of inertia               [kg m^2]
%     .Iyy    Pitch moment of inertia               [kg m^2]
%     .Izz    Yaw   moment of inertia               [kg m^2]
%
%   FlightData.Oper      — Nominal operating condition
%     .rho    Air density                           [kg/m^3]
%     .M      Mach number                           [-]
%     .V      True airspeed                         [m/s]
%     .g      Gravitational acceleration            [m/s^2]
%
%   FlightData.Aero      — Aerodynamic stability derivatives (all per radian)
%     Longitudinal:  CLa, CDa, Cma, CLadot, Cmadot, CLq, Cmq, CLde, Cmde
%     Lateral-dir.:  CYb, Clb, Cnb, Clp, Cnp, Clr, Cnr,
%                    CYda, Clda, Cnda, CYdr, Cldr, Cndr
%
%   All values are originally from Nelson (1998) Table B.6, converted from
%   US customary units (ft, lbm, slug) to SI units internally.
%
%   Example:
%       fd = load_stol_data();
%       disp(fd.Geom.b)    % wing span in metres
%       disp(fd.Aero.Cnb)  % weathercock stability derivative
%
% Reference: Nelson, R.C. (1998) Flight Stability and Automatic Control,
%            2nd ed., Appendix B, Table B.6.

%% Example STOL aircraft data (Nelson, 1998)

% Conversion factors
ft2m = 0.3048;
lbm2kg = 0.453592;
slug2kg = 14.5939;

%% Geometric properties
Geom.S = 945*ft2m^2;            % Wing planform area
Geom.b = 96*ft2m;               % Wing span
Geom.cbar = 10.1*ft2m;          % Wing mean aerodynamic chord
Geom.AR = 9.75;                 % Wing aspect ratio
Geom.lf = 76*ft2m;              % Fuselage length
Geom.qcw = 0.316*Geom.lf;       % Wing 1/4 chord location (from nose)
Geom.cg = 0.4*Geom.cbar;        % Nominal centre of gravity location (from leading edge)
Geom.St = 233*ft2m^2;           % Horizontal tail planform area
Geom.bt = 32*ft2m;              % Horizontal tail span
Geom.ctbar = 7*ft2m;            % Horizontal tail mean aerodynamic chord
Geom.ARt = 4.4;                 % Horizontal tail aspect ratio
Geom.lt = 35*ft2m;              % Tail length (with nominal CG location)
Geom.Se = 81.5*ft2m^2;          % Elevator area
Geom.wf = 9.4*ft2m;             % Fuselage width
Geom.zv = 11.5*ft2m;            % Vertical tail vertical offset
Geom.lv = Geom.lt;              % Vertical tail horizontal offset
Geom.Sv = 105*ft2m;             % Vertical tail planform area
Geom.Vv = Geom.Sv*Geom.lv/Geom.S/Geom.b;    % Vertical tail volume coefficient

%% Inertial properties
Inertial.m = 40000*lbm2kg;              % Mass
Inertial.Ixx = 273000*slug2kg*ft2m^2;   % Mass moment of inertia about x axis
Inertial.Iyy = 215000*slug2kg*ft2m^2;   % Mass moment of inertia about y axis
Inertial.Izz = 447000*slug2kg*ft2m^2;   % Mass moment of inertia about z axis

%% Nominal operating condition
Oper.rho = 1.225;               % Air density
Oper.M = 0.14;                  % Mach number
Oper.V = Oper.M*340;            % True airspeed
Oper.g = 9.81;                  % Acceleration due to gravity

%% Aerodynamic properties (at nominal operating condition)
Aero.aw = 5.2;                  % Wing lift curve slope
Aero.at = 3.5;                  % Horizontal tail lift curve slope
Aero.e = 0.75;                  % Span efficiency factor

% Longitudinal derivatives
Aero.Cmafp = 0.93;              % Pitching moment angle of attack derivative due to fuselage and power
Aero.CLa = 5.24;
Aero.CDa = 0.67;
Aero.Cma = -0.78;
Aero.CLadot = 1.33;
Aero.Cmadot = -6.05;
Aero.CLq = 7.83;
Aero.Cmq = -35.6;
Aero.CLde = 0.465;
Aero.Cmde = -2.12;

% Lateral-directional derivatives
Aero.CYb = -0.362;
Aero.Clb = -0.125;
Aero.Cnb = 0.101;
Aero.Clp = -0.53;
Aero.Cnp = -0.283;
Aero.Clr = 0.410;
Aero.Cnr = -0.188;
Aero.CYda = 0.0;
Aero.Clda = -0.08;
Aero.Cnda = 0.0;
Aero.CYdr = 0.233;
Aero.Cldr = 0.024;
Aero.Cndr = -0.127;

%% Create output structure
FlightData.Geom = Geom;
FlightData.Aero = Aero;
FlightData.Inertial = Inertial;
FlightData.Oper = Oper;

end