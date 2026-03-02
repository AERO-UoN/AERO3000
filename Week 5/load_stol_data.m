function FlightData = load_stol_data()
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