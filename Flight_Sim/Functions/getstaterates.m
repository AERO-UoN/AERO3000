function [Xdot, CL, Y] = getstaterates(Params, X, U)
%   Iterates on state rates until angle of attack and sideslip rates
%   converge, giving an accurate estimate of the state
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
%               - delta_t = U(1)    (0-1)
%               - delta_e = U(2)    (rad)
%               - delta_a = U(3)    (rad)
%               - delta_r = U(4)    (rad)
%
% Outputs:
%   Xdot:   State rate vector with after iteration of alpha and beta dot
%   CL:     Lift coefficient
%   Y:      Side force
%
% Other m-files required: none
%
% Subfunctions:
%   staterates, calculateForces, aeroangles, flowproperties, gravity,
%   windforces, bodyforces, gravForces, propforce
%
% MAT-files required:
%   staterates.m, calculateForces.m, aeroangles.m, flowproperties.m,
%   gravity.m, windforces.m, bodyforces.m, gravForces.m, propforce.m
%
% TODO: none

% Initial guess for alpha_dot and beta_dot (rad/s).
alpha_dot_old = 0;
beta_dot_old  = 0;

% Convergence tolerance (rad/s) and iteration bookkeeping.
% Absolute tolerance is used instead of relative tolerance because
% alpha_dot and beta_dot converge to near-zero during trimmed flight,
% making a relative criterion ill-conditioned (small/small -> large error).
tolerance = 1e-9;   % rad/s — matches the accuracy of the aero model
iterLim   = 100;
iterCount = 0;

% Iterate until alpha_dot and beta_dot converge.
% These rates appear in the unsteady aerodynamic derivatives (CLad, Cmad,
% etc.) and must be self-consistent with the resulting state accelerations.
while true

    % Pack rates into a 2-vector for staterates()
    angle_rates = [alpha_dot_old, beta_dot_old];

    % Estimate state rates using current angle-of-attack and sideslip rates
    [Xdot, CL, Y] = staterates(Params, X, U, angle_rates);

    % Recompute alpha_dot and beta_dot implied by the new state rates
    [alpha_dot, beta_dot] = angularRates(Xdot, X);

    % Absolute error — robust whether values are near-zero or finite
    err = max(abs(alpha_dot - alpha_dot_old), abs(beta_dot - beta_dot_old));

    % Store estimates for next iteration
    alpha_dot_old = alpha_dot;
    beta_dot_old  = beta_dot;

    % Exit conditions
    if err < tolerance
        break
    end
    if iterCount >= iterLim
        warning('getstaterates: iteration limit reached for alpha/beta rates.');
        break
    end

    iterCount = iterCount + 1;
end
end