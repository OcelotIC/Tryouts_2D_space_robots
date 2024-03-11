function [ddq, F_C] = space_dynamics(model, x, u)
% Implements Constraint-Consistent Forward Dynamics of a simplified 
% planar Manisat robotic system.
% 
% Outputs:
%   ddq   : [7x1] Vector of joint accelerations. base + joints
%   F_C   : [6x1] Vector of constraint forces acting at the feet.
% 
% Inputs:
%   model : The Multi-body dynamics and kinematics model of the system.
%   x     : [14x1] State vector of the system (positions, velocities).
%   u     : [4x1] Input vector of the system [tau_F, tau_H, tau_A]' .

% 

persistent I_p_Ff0 I_p_Hf0;

% Map system quantities to multi-body physical quantities
q = x(1:7);    % [7x1] Generalized coordinates [q_b, q_F, q_H]'
dq = x(8:14);  % [7X1] Generalized velocities
tau = u;        % [4X1] Control input mapping directly to joint torques

% Extract dynamical parameters at current state
params = model.parameters.values;
M_d = model.dynamics.compute.M(q,dq,[],[],params); % [7X7] Inertia matrix
Q_v = model.dynamics.compute.b(q,dq,[],[],params); % [7X1] Nonlinear forces Qv
g_d = model.dynamics.compute.g(q,dq,[],[],params); % [7X1] Gravity forces empty in this case

% Compute Jacobians and respective derivatives at current state
I_J_Ff = eval_jac(model.body( 7).kinematics.compute.I_J_IBi, q, [], params);	% [3X7] Front foot position and orientation Jacobian
I_J_Hf = eval_jac(model.body( 4).kinematics.compute.I_J_IBi, q, [], params);	% [3x7] Hind foot position and orientation Jacobian
I_Jd_Ff = eval_jac(model.body( 7).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3X7] Front foot position and orientation Jacobian derivative
I_Jd_Hf = eval_jac(model.body( 4).kinematics.compute.I_dJ_IBi, q, dq, params);  % [3x7] Hind foot position and orientation Jacobian derivative

% Compute forward kinematics at current state
I_T_Ff = model.body(7).kinematics.compute.T_IBi(q, dq, [], params);
I_p_Ff = I_T_Ff([1,3], 4);
I_T_Hf = model.body(4).kinematics.compute.T_IBi(q, dq, [], params);
I_p_Hf = I_T_Hf([1,3], 4);

% TODO
if isempty(I_p_Ff0)
    I_p_Ff0 = I_p_Ff;
    I_p_Hf0 = I_p_Hf;
end

% Assemble foot constraints -> no linear velocity (no rotational velocity
% is too hard)
I_J_c = [I_J_Ff(1:2,:) ; I_J_Hf(1:2,:)];
I_Jd_c = [I_Jd_Ff(1:2,:); I_Jd_Hf(1:2,:)];
num_constr = size(I_J_c,1);

% Selection Matrix
%S = [zeros(7, 3), eye(7)]; 
S = [zeros(4, 3), eye(4)]; 

% Drift correction from nikravesh book 
kp = 10;
kd = 2*sqrt(kp);
wd_drift = [kp*(I_p_Ff0 - I_p_Ff); kp*(I_p_Hf0 - I_p_Hf)] - kd*I_J_c*dq;

% Define matrix and vector of the linear system used to solve the dynamics
A = [M_d,    -I_J_c'; ...
     -I_J_c, zeros(num_constr)];
b = [S'*tau - Q_v - g_d; ...
     I_Jd_c * dq - wd_drift];

% Solve for acceleration and constraint force simultaniously
xd = A \ b; 
ddq = xd(1:7);
F_C = xd(8:11);

end

function jac = eval_jac(f, q, dq, params)
jac = f(q,dq,[],params);
jac = jac(1:2:end,:);
end

