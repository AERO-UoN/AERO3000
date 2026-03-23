%% Parameters
phi0 = deg2rad(0);     % Initial roll angle (rad)
theta0 = deg2rad(0);   % Initial pitch angle (rad)
psi0 = deg2rad(90);    % Initial yaw angle (rad)

p = deg2rad(0);        % Body roll rate (rad/s)
q = deg2rad(5);   % Body pitch rate (rad/s)
r = deg2rad(3);      % Replace 3 with your student number's last two digits

omega = [p; q; r];

T = 30;                % Total simulation time (s)
dt = 0.01;             % Time step (s)
N = T/dt;              % Number of steps
time = (0:N-1) * dt;

%% (a) Euler Angle Integration
phi = zeros(1, N); theta = zeros(1, N); psi = zeros(1, N);
phi(1) = phi0; theta(1) = theta0; psi(1) = psi0;

for k = 1:N-1
    % Transformation matrix from body rates to Euler angle rates
    T_eul = [1, sin(phi(k))*tan(theta(k)), cos(phi(k))*tan(theta(k));
             0, cos(phi(k)), -sin(phi(k));
             0, sin(phi(k))/cos(theta(k)), cos(phi(k))/cos(theta(k))];

    eul_dot = T_eul * omega;

    phi(k+1) = phi(k) + dt * eul_dot(1);
    theta(k+1) = theta(k) + dt * eul_dot(2);
    psi(k+1) = psi(k) + dt * eul_dot(3);
end

% Convert to degrees and wrap
phi_deg = wrapTo180(rad2deg(phi));
theta_deg = wrapTo180(rad2deg(theta));
psi_deg = wrapTo180(rad2deg(psi));

% Plot Euler angles
figure;
plot(time, phi_deg, 'r', time, theta_deg, 'g', time, psi_deg, 'b');
xlabel('Time (s)'); ylabel('Euler Angles (deg)');
title('Euler Angle Integration'); legend('\phi', '\theta', '\psi'); grid on;

%% (b) Quaternion Integration
quat = angle2quat(psi0, theta0, phi0); % initial quaternion [w x y z]
quat_hist = zeros(N, 4); quat_hist(1, :) = quat;

for k = 1:N-1
    % Quaternion rate matrix
    omega_mat = [  0,    -omega(1), -omega(2), -omega(3);
                  omega(1), 0,       omega(3), -omega(2);
                  omega(2), -omega(3), 0,       omega(1);
                  omega(3), omega(2), -omega(1), 0];

    dquat = 0.5 * (omega_mat * quat');
    quat = quat + (dquat' * dt);
    quat = quat / norm(quat); % Normalize
    quat_hist(k+1, :) = quat;
end

% Convert back to Euler angles
[psi_q, theta_q, phi_q] = quat2angle(quat_hist);
phi_q = (rad2deg(phi_q));
theta_q = (rad2deg(theta_q));
psi_q = (rad2deg(psi_q));

% Plot Quaternion results
figure;
plot(time, phi_q, 'r--', time, theta_q, 'g--', time, psi_q, 'b--');
xlabel('Time (s)'); ylabel('Euler Angles (deg)');
title('Quaternion Integration'); legend('\phi_q', '\theta_q', '\psi_q'); grid on;

%% (c) Compare Small r Cases
r_vals = deg2rad([0.1, 0.01]);
colors = {'m', 'k'};

figure; hold on;
for i = 1:2
    r = r_vals(i);
    omega_small = [p; q; r];
    quat = angle2quat(psi0, theta0, phi0);
    q_hist_small = zeros(N, 4);
    q_hist_small(1,:) = quat;

    for k = 1:N-1
        omega_mat_small = [  0,    -omega_small(1), -omega_small(2), -omega_small(3);
                          omega_small(1), 0,       omega_small(3), -omega_small(2);
                          omega_small(2), -omega_small(3), 0,       omega_small(1);
                          omega_small(3), omega_small(2), -omega_small(1), 0];

        dquat = 0.5 * (omega_mat_small * quat');
        quat = quat + (dquat' * dt);
        quat = quat / norm(quat);
        q_hist_small(k+1,:) = quat;
    end
    [psi_small, ~, ~] = quat2angle(q_hist_small);
    plot(time, (rad2deg(psi_small)), colors{i}, 'DisplayName', ['r = ', num2str(rad2deg(r)), ' deg/s']);
end
xlabel('Time (s)'); ylabel('\psi (deg)');
title('Yaw Angle for Small r Values (Quaternion Integration)'); grid on; legend;

function [out] = wrapTo180(in)
%NORM_ANGLE Normalizes input angles to range [-180..180]
%   Input vector is expected to be degree angles that are unwrapped.
    out = mod(in + 180, 360) - 180;
end

function q = angle2quat(z, y, x)
%ANGLE2QUAT Convert Euler angles to a quaternion.
%   Q = ANGLE2QUAT(Z, Y, X) converts Euler angles Z, Y, X, into an
%   equivalent quaternion Q.

thetas = [z(:) y(:) x(:)];

c = cos(thetas/2);
s = sin(thetas/2);

q = [c(:,1).*c(:,2).*c(:,3) + s(:,1).*s(:,2).*s(:,3), ...
     c(:,1).*c(:,2).*s(:,3) - s(:,1).*s(:,2).*c(:,3), ...
     c(:,1).*s(:,2).*c(:,3) + s(:,1).*c(:,2).*s(:,3), ...
     s(:,1).*c(:,2).*c(:,3) - c(:,1).*s(:,2).*s(:,3)];
end 

function [z y x] = quat2angle(q)
%QUAT2ANGLE Convert quaternion to Euler angles.
%   [Z Y X] = QUAT2ANGLE(Q) calculates the Euler angles, Z, Y, X, for a
%   quaternion, Q.

qn = bsxfun(@rdivide, q, sqrt(sum(q.^2, 2)));

z = atan2(2.*(qn(:,2).*qn(:,3) + qn(:,1).*qn(:,4)), ...
          qn(:,1).^2 + qn(:,2).^2 - qn(:,3).^2 - qn(:,4).^2);
y = asin(-2.*(qn(:,2).*qn(:,4) - qn(:,1).*qn(:,3)));
x = atan2(2.*(qn(:,3).*qn(:,4) + qn(:,1).*qn(:,2)), ...
          qn(:,1).^2 - qn(:,2).^2 - qn(:,3).^2 + qn(:,4).^2);
end
