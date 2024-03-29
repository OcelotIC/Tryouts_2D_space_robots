function [ tau ] = floating_base_control_manisat(model, t, x)
% Implements a hierarchical-QP-based controller which realizes a desired 
% base motion.
% 
% Outputs:
%   tau   : [4x1] torques [tau_L, tau_R]'
% 
% Inputs:
%   model : The Multi-body dynamics and kinematics model of the system.
%   t     : Current time [s]
%   x     : Current state of the system (position, velocity)
% 
% Where:
%	tau_L : [2x1] torques [hip, knee]' (front leg)
%	tau_R: [2X1] torques [hip, knee]' (hind leg)

% 

%% System State

% Extract generalized positions and velocities
q = x(1:7);    % [7x1] Generalized coordinates [q_b, q_L, q_R,7]'
dq = x(8:14);  % [7X1] Generalized velocities

%% Model Parameters 

% Extract dynamics at current state
params = model.parameters.values;
M = model.dynamics.compute.M(q,dq,[],[],params); % [7X7] Inertia matrix
b = model.dynamics.compute.b(q,dq,[],[],params); % [7X1] Nonlinear-dynamics vector
g = model.dynamics.compute.g(q,dq,[],[],params); % [7X1] Gravity vector
S = [zeros(4, 3), eye(4)];                       % [7x4] Selection matrix all actuated joint are active

% Get Jacobians and Derivatives at current state
J_B  = eval_jac(model.body( 1).kinematics.compute.I_J_IBi, q, [], params);    % [3X7] body position and orientation Jacobian
J_RF = eval_jac(model.body( 7).kinematics.compute.I_J_IBi, q, [], params);	% [3X7] Right foot position and orientation Jacobian
J_LF = eval_jac(model.body( 4).kinematics.compute.I_J_IBi, q, [], params);	% [3x7] Left foot position and orientation Jacobian

dJ_B  = eval_jac(model.body( 1).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X7] body position and orientation Jacobian derivative
dJ_RF = eval_jac(model.body( 7).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X7] Right foot position and orientation Jacobian derivative
dJ_LF = eval_jac(model.body( 4).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3x7] Left foot position and orientation Jacobian derivative


% Extract forward kinematics of the base
p_B = q(1:3);
w_B = dq(1:3);

% Assemble constraint Jacobian -> Only constrain linear velocity at feet
J_c = [J_RF(1:2,:) ; J_LF(1:2,:)];
dJ_c = [dJ_RF(1:2,:); dJ_LF(1:2,:)];


%% Control References

% Call the base trajectory planner
[p_star_B, w_star_B] = base_motion_trajectory(model, t, x);

%% Optimization Tasks

% x_opt = [ddq', f_c_xz', tau'] dim is 7 + 4 +4 = 15 

% Equations of motions
A_eom = [M, -J_c', -S'];
b_eom = -b -g;%dim=7

% No foot-contact motions
A_c = [J_c, zeros(4, 4), zeros(4, 7)];
b_c = -dJ_c*dq;%dim=4

% Body motion
kp_b = 10;
kd_b = 2*sqrt(kp_b);
dw_star_B = kp_b*(p_star_B - p_B) + kd_b*(w_star_B - w_B);
A_b = [J_B, zeros(3, 4), zeros(3, 7)];
b_b = dw_star_B - dJ_B*dq;%dim=3

%% Additional Tasks

% Kinematic null space position control
q0 = [0 0.50 0 0.9 -1.5 0.9 -1.5]'; % [7x1] Default generalized coordinates
kp_0 = 1;
kd_0 = 2*sqrt(kp_0);
A_0 = [eye(7), zeros(7, 4), zeros(7, 7)];
b_0 = kp_0*(q0 - q) - kd_0*dq;%dim=7

% Torque minimization
A_tau = [zeros(4, 7), zeros(4, 4), eye(4)];
b_tau = zeros(4,1);%dim=4

% Contact force minimization
A_f_c = [zeros(4, 7), eye(4), zeros(4, 4)];
b_f_c = zeros(4,1);%dim=4

%% Inequality Constraints

% Torque limits
tau_max = 50;
C_tau_up = [zeros(4, 7), zeros(4, 4), eye(4)];
d_tau_up = tau_max*ones(4,1);
C_tau_low = -C_tau_up;%dim={0,..,7}
d_tau_low = d_tau_up;%dim={0,..,7}

% Friction Cone necessary at the moment 
mu = 0.5;
C_cone = [0 -1; 1 -mu; -1 -mu];
C_f_c = [zeros(6, 10), blkdiag(C_cone, C_cone), zeros(6, 7)];
d_f_c = zeros(6, 1);%dim={0, 1, 2}

%% Assemble and solve

% Define hierarchy of tasks
A = {A_eom, A_c, A_b, A_0, A_tau, A_f_c};
b = {b_eom, b_c, b_b, b_0, b_tau, b_f_c};

% Define inequality constraints
C = [C_tau_up; C_tau_low; C_f_c];
d = [d_tau_up; d_tau_low; d_f_c];

% Solve hierarchical QPs
print_solution = 0;
x_opt = hopt(A, b, C, d, print_solution);
ddq = x_opt(1:7);
f_c = x_opt(11:14);
tau = x_opt(15:end);

end

%% Helper functions

function jac = eval_jac(f, q, dq, params)
jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end

%% EOF
