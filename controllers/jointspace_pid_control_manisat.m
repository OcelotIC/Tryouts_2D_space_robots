function [ tau ] = jointspace_pid_control(model, t, x)
% Implements a trivial joint-space PID controller on zero velocity references.
% 
% Outputs:
% 	tau : 4x1] torques [tau_L, tau_R]'
% 
% Inputs:
%   model : The Multi-body dynamics and kinematics model of the system.
%   t     : Current time [s]
%   x     : Current state of the system (position, velocity)
% 
% Where:
%	tau_F : [2x1] torques [hip, knee]' (left leg)
%	tau_H : [2X1] torques [hip, knee]' (right leg)
%	
% 

%% System State

% Extract generalized positions and velocities
q = x(1:7); % [7x1] Generalized coordinates [q_b, q_L, q_R]'
dq = x(8:14); % [7X1] Generalized velocities

%% Model Parameters 

% Extract model parameters 
params = model.parameters.values;

% Extract dynamics at current state
g = model.dynamics.compute.g(q,dq,[],[],params); % [7X1] Gravity vector
S = [zeros(4, 3), eye(4)];                       % [7X4] Selection matrix

%% Zero-Velocity Joint-Space PID Controller

% Joint configurations and velocities
q_j = q(4:7);
dq_j = dq(4:7);

% Reference joint configurations and velocities
%q_star_j = [0.5  -1 0.7  0.5  ].';

q_star_j = [ 0.8  -1.1   -0.8  1.3].';
dq_star_j = zeros(4,1);

% PD Gains
Kp = [1*ones(1,4)].';
Kd = [0.5*ones(1,4) ].';

% Joint torque command
tau = Kp .* (q_star_j - q_j) + Kd .* (dq_star_j - dq_j);

% Gravity compensation
tau = tau + S*g;

end

%% EOF