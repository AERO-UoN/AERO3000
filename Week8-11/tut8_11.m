% =========================================================================
% Tutorial 8-11 - Longitudinal Dynamic Stability: Eigenvalue Analysis
% =========================================================================
% PURPOSE:
%   Compute the eigenvalues of the linearised longitudinal state-space
%   matrix A to identify the two longitudinal dynamic modes (short-period
%   and phugoid), apply standard mode approximations, and determine whether
%   the aircraft is dynamically stable.
%
% CONCEPTS TAUGHT:
%   State-space form:
%     The linearised longitudinal equations of motion are written as:
%       x_dot = A*x + B*u
%     where the state vector is x = [u; w; q; theta]:
%       u     — perturbation forward speed  (m/s)
%       w     — perturbation heave speed    (m/s)  (related to alpha)
%       q     — perturbation pitch rate     (rad/s)
%       theta — perturbation pitch angle    (rad)
%     and A is the 4x4 dimensional stability derivative matrix.
%
%   Eigenvalues and dynamic modes:
%     Each complex eigenvalue pair sigma ± j*omega_d corresponds to one
%     oscillatory mode with:
%       Natural frequency:  omega_n = |lambda| = sqrt(sigma^2 + omega_d^2)
%       Damping ratio:      zeta    = -sigma / omega_n
%       Period:             T       = 2*pi / omega_d
%     The two longitudinal modes are:
%       Short-period — fast, well-damped pitch oscillation (w and q states)
%       Phugoid      — slow, lightly damped speed/altitude exchange (u and theta states)
%
%   Short-period approximation:
%     Since u and theta change slowly relative to w and q during the
%     short-period mode, the approximation fixes u ≈ 0 and theta ≈ 0,
%     reducing A to the 2x2 sub-matrix of the fast states [w, q]:
%       Asp = A(fastIdx, fastIdx)    where fastIdx = [2, 3]
%
%   Phugoid approximation (Schur complement):
%     The fast [w, q] states are assumed to reach quasi-steady state
%     instantaneously relative to the slow [u, theta] dynamics.  Setting
%     x_fast_dot = 0 and eliminating the fast states gives:
%       Aph = A11 - A12 * inv(A22) * A21
%     where A11, A12, A21, A22 are sub-blocks of A partitioned by
%     slowIdx = [1, 4] and fastIdx = [2, 3].
%
%   Stability criterion:
%     A mode is stable if its eigenvalue has a strictly negative real part
%     (Re(lambda) < 0).  All modes must be stable for the aircraft to be
%     dynamically stable overall.
%
% KEY ASSUMPTIONS:
%   - Linear, time-invariant (LTI) model — valid for small perturbations
%     from a trimmed equilibrium condition
%   - Dimensional stability derivatives (entries of A have units of 1/s,
%     m/s/rad, etc.)
%   - Longitudinal and lateral-directional dynamics are decoupled
%   - Level, unaccelerated trim flight (symmetric flight)
%
% EXPECTED OUTPUT:
%   - Full 4x4 A matrix printed to the command window
%   - Four eigenvalues of A (two complex pairs: short-period and phugoid)
%   - Short-period eigenvalues from the 2x2 approximation
%   - Phugoid eigenvalues from the Schur-complement approximation
%   - Stability verdict (stable / unstable) based on real parts
%
% Reference: Nelson, R.C. (1998) Flight Stability and Automatic Control,
%            Ch.4 (Longitudinal Dynamic Stability and Response).
% =========================================================================

% State vector: x = [u (m/s); w (m/s); q (rad/s); theta (rad)]
% Row i of A contains the dimensional stability derivatives for x_dot(i)
A = [-0.022   0.052   0.0012  -9.81;
     -0.64   -4.85   -0.61     0;
     -0.0048 -0.070  -1.79     0;
      0       0       1        0];

disp('Longitudinal state-space matrix A:');
disp(A);

%% Full eigenvalue analysis
eigenvalues = eig(A);
disp('Eigenvalues of A (full system):');
disp(eigenvalues);

% Partition state indices into slow (phugoid) and fast (short-period) groups
slowIdx = [1, 4];   % u and theta — phugoid states
fastIdx = [2, 3];   % w and q    — short-period states

%% Short-period approximation
% Retain only the fast [w, q] sub-system; assumes u and theta are frozen
Asp = A(fastIdx, fastIdx);
eigenvalues_sp = eig(Asp);
disp('Short-period eigenvalues (2x2 approximation):');
disp(eigenvalues_sp);

%% Phugoid approximation (Schur complement)
% Partition A into slow/fast sub-blocks
A11 = A(slowIdx, slowIdx);   % slow-slow coupling
A12 = A(slowIdx, fastIdx);   % slow-fast coupling
A21 = A(fastIdx, slowIdx);   % fast-slow coupling
A22 = A(fastIdx, fastIdx);   % fast-fast coupling

% Eliminate the fast states by assuming x_fast_dot = 0 (quasi-steady state)
Aph = A11 - A12 * (A22 \ A21);

eigenvalues_ph = eig(Aph);
disp('Phugoid eigenvalues (Schur-complement approximation):');
disp(eigenvalues_ph);

%% Stability verdict
real_parts = real(eigenvalues);
if all(real_parts < 0)
    disp('The aircraft is dynamically STABLE (all eigenvalues have negative real parts).');
else
    disp('The aircraft is dynamically UNSTABLE (one or more eigenvalues have non-negative real parts).');
end