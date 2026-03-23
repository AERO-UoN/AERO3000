% =========================================================================
% Tutorial 2.1 - Coordinate Frames and the Direction Cosine Matrix
% =========================================================================
% PURPOSE:
%   Construct the Direction Cosine Matrix (DCM) for the standard aerospace
%   3-2-1 Euler angle sequence and use it to transform a velocity vector
%   from the aircraft Body frame to the Earth (NED) frame.
%
% CONCEPTS TAUGHT:
%   Body frame (B):  origin at the aircraft centre of gravity (CG);
%                    x-axis points forward (nose), y-axis points starboard
%                    (right wing), z-axis points downward.  This frame is
%                    fixed to and rotates with the aircraft.
%
%   Earth frame (E): North-East-Down (NED) frame fixed to Earth's surface;
%                    x-axis points North, y-axis points East, z-axis points
%                    Down.  Treated as inertial for most flight dynamics.
%
%   Euler angles:    phi (roll), theta (pitch), psi (yaw) describe how to
%                    rotate frame E into alignment with frame B.
%
%   3-2-1 sequence:  Starting from E, rotate:
%                      1st: by psi   about the z-axis  (rotation 3)
%                      2nd: by theta about the new y-axis  (rotation 2)
%                      3rd: by phi   about the new x-axis  (rotation 1)
%                    This order (3-2-1 = yaw → pitch → roll) is the
%                    standard convention in aerospace engineering because
%                    it corresponds to the natural way an aircraft manoeuvres.
%
%   DCM (R_be):      The resulting 3x3 matrix encodes the orientation.
%                    As constructed here, R_be transforms a vector from
%                    the Earth frame into the Body frame (Earth → Body).
%                    Its transpose, Reb = R_be', therefore transforms from
%                    Body → Earth.
%
% KEY ASSUMPTIONS:
%   - Full trigonometric form; no small-angle approximations
%   - Singularity (gimbal lock) occurs at theta = ±90 deg; use quaternions
%     (see tut2_2) to avoid this limitation in simulation
%   - Rotation sequence is strictly 3-2-1; other sequences (e.g. 3-1-3 for
%     spacecraft) produce different matrices
%
% EXPECTED OUTPUT:
%   - V_e: 3x1 velocity vector expressed in the Earth (NED) frame [m/s]
%
% Reference: Nelson, R.C. (1998) Flight Stability and Automatic Control,
%            Ch.1; Stevens & Lewis (2016) Aircraft Control and Simulation,
%            Appendix A.
% =========================================================================

% Given Euler angles (degrees)
phi = deg2rad(10);    % Roll angle
theta = deg2rad(20);  % Pitch angle
psi = deg2rad(45);    % Yaw angle

% Velocity vector in body frame
V_b = [100; 5; -2]; % m/s

% Compute Direction Cosine Matrix using the 3-2-1 (Yaw-Pitch-Roll) sequence.
% R_be transforms vectors FROM the Earth frame TO the Body frame.
% Derivation: R_be = R1(phi) * R2(theta) * R3(psi), where Ri is an
% elementary rotation matrix about axis i.
R_be = [ cos(theta)*cos(psi),  cos(theta)*sin(psi), -sin(theta);
         sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), ...
         sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), sin(phi)*cos(theta);
         cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), ...
         cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)];

% Reb = R_be' transforms vectors FROM the Body frame TO the Earth frame.
% This is the direction we need to express the aircraft velocity in NED.
Reb = R_be';
V_e = Reb * V_b;

% Display results
disp('Velocity vector in the Earth (NED) frame [m/s]:');
disp(V_e);
