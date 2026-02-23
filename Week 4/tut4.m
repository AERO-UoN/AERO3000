% Given data
x_cg = 0.235; % Center of gravity location (fraction of mean aerodynamic chord)
x_ac = 0.25;  % Aerodynamic center location (fraction of mean aerodynamic chord)
c_bar = 1.652; % Mean aerodynamic chord (m)
eta = 0.8; % Horizontal tail efficiency
C_L_alpha_w = 5.827; % Lift curve slope of main wing (per radian)
S = 16.3; % Wing area (m^2)
l_t = 4.257; % Tail moment arm (m)
deps_dalpha = 0.6; % Downwash gradient
eta_t = 1; % Tail efficiency factor
C_L_alpha_t = 4.120; % Lift curve slope of tail (per radian)
S_t = 2.09; % Tail area (m^2)

% Compute total lift curve slope of the complete aircraft
C_L_alpha = C_L_alpha_w + (eta_t * S_t / S) * C_L_alpha_t * (1 - deps_dalpha);

% Compute stick-fixed neutral point location (x_NP)
l_t_bar = l_t + (x_cg-x_ac)*c_bar;
V_bar =  (S_t*l_t_bar)/(S*c_bar);
x_NP = x_ac + eta_t*V_bar* (C_L_alpha_t / C_L_alpha) * (1 - deps_dalpha);

% Compute static margin
K_n = x_NP - x_cg;

% Display results
disp(['Total Aircraft Lift Curve Slope (C_L_alpha_total): ', num2str(C_L_alpha, 4), ' per radian']);
disp(['Stick-Fixed Neutral Point (x_NP): ', num2str(x_NP, 4), ' * c_bar']);
disp(['Static Margin (K_n): ', num2str(K_n, 4)]);

% Stability interpretation
if K_n > 0
    disp('The aircraft is STABLE (Positive Static Margin).');
elseif K_n == 0
    disp('The aircraft is NEUTRALLY STABLE.');
else
    K_n('The aircraft is UNSTABLE (Negative Static Margin).');
end

