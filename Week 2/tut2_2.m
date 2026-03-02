% =========================================================================
% Tutorial 2.2 - Attitude Representation: Euler Angles to Quaternions
% =========================================================================
% PURPOSE:
%   Convert a set of 3-2-1 Euler angles (phi, theta, psi) to the
%   equivalent unit quaternion representation of the same orientation.
%
% CONCEPTS TAUGHT:
%   Why quaternions?
%     Euler angles suffer from a singularity called gimbal lock when the
%     pitch angle reaches theta = ±90 deg — one degree of freedom is lost
%     and the representation becomes undefined.  Quaternions avoid this
%     singularity entirely, making them the preferred attitude state for
%     numerical integration in flight simulations.
%
%   Quaternion structure (scalar-first / Hamilton convention):
%     q = [q0; q1; q2; q3]
%     q0  — scalar (real) part;  q0 = cos(angle/2) for a single-axis rotation
%     q1, q2, q3 — vector (imaginary) parts encoding the rotation axis
%     A unit quaternion satisfies: q0^2 + q1^2 + q2^2 + q3^2 = 1
%
%   Conversion from 3-2-1 Euler angles (phi, theta, psi):
%     The formula below is the closed-form result of composing three
%     elementary quaternion rotations in the 3-2-1 order.  Each component
%     is a product of half-angle sines and cosines of phi, theta, psi.
%
%   Normalisation:
%     A correctly computed quaternion from exact Euler angles is already a
%     unit quaternion.  In practice, numerical drift during integration
%     causes small norm errors, so periodic renormalisation is essential.
%
% KEY ASSUMPTIONS:
%   - Scalar-first convention: q = [q0; q1; q2; q3]
%   - 3-2-1 rotation sequence (same convention as tut2_1)
%   - Input angles are converted to radians before computation
%
% EXPECTED OUTPUT:
%   - q_norm: 4x1 unit quaternion (norm = 1.0) representing the same
%             orientation as the input Euler angles
%
% Reference: Stengel, R.F. (2004) Flight Dynamics, Ch.2;
%            Stevens & Lewis (2016) Aircraft Control and Simulation, App.A.
% =========================================================================

% Given Euler angles (degrees)
phi = deg2rad(15);   % Roll angle
theta = deg2rad(30); % Pitch angle
psi = deg2rad(60);   % Yaw angle

% Compute half angles — the quaternion formula uses half-angle identities
half_phi   = phi   / 2;
half_theta = theta / 2;
half_psi   = psi   / 2;

% Compute quaternion components from 3-2-1 Euler angles (scalar-first convention)
q0 = cos(half_phi) * cos(half_theta) * cos(half_psi) + sin(half_phi) * sin(half_theta) * sin(half_psi);
q1 = sin(half_phi) * cos(half_theta) * cos(half_psi) - cos(half_phi) * sin(half_theta) * sin(half_psi);
q2 = cos(half_phi) * sin(half_theta) * cos(half_psi) + sin(half_phi) * cos(half_theta) * sin(half_psi);
q3 = cos(half_phi) * cos(half_theta) * sin(half_psi) - sin(half_phi) * sin(half_theta) * cos(half_psi);

% Normalise to enforce the unit-quaternion constraint ||q|| = 1.
% For exact arithmetic this step is redundant, but it guards against
% accumulated floating-point error in simulation use.
q = [q0; q1; q2; q3];
q_norm = q / norm(q);

% Display results
disp('Quaternion (scalar-first) [q0; q1; q2; q3]:');
disp(q_norm);
disp(['Norm check (should be 1.0): ', num2str(norm(q_norm))]);
