%% Setup Simulation

try % Try if the visualization instance is still open
    
    % Initialize the state in the visualization.
    robotviz.update(xinit(1:10), zeros(10,1), zeros(3,1));
    
catch % If it doesn't work, open a new visualization instance
   
    % Clear the entire MATLAB workspace     
    close all; clearvars; clear classes;
    
    % Load the pre-generated model of the robot

    load('modelsManisat_easy.mat');
    load('modelssc_easy.mat');
    
    % Set model parameters
    % Def:     [grav, h_B, l_B, l_FS, l_FT, l_RS, l_RT, m_B, m_LS, m_LT,  m_RS, m_RT,  r_FF, r_FT,    r_RF, r_RT,   w_B]
    mparams  = [0     0.1  0.6  0.3   0.3   0.3   0.3   10.0  0.5   0.5   0.5   0.5    0.02  0.015    0.02  0.015   0.2].';
    robotmdl.parameters.values = mparams;
    
    % Visualization configurations
    fontsize = 10;
    csfscale = 0.1;
    posoffset = [0; 0; 0];
    zoom = 3.0;
    
    % Generate 3D Visualization instance
    robotviz = RobotVisualization(robotmdl, worldmdl, fontsize, csfscale, zoom, posoffset);
    robotviz.open();
    robotviz.load();
    
end

%% Configure Experiment

% Set the initial state of the system
% Def:  [xb   yb    thb   left : q1   q2   right :  q3   q4       dot(q)
xinit = [0.0  0.5  0             0.9  -1.5          -0.9  1.5    zeros(1,7)].';

% Set distance of wall in positive X-direction in world coordinates.
dx = 5;

% Set the controller to use
controller = @jointspace_pid_control_manisat;
% controller = @floating_base_control;
% controller = @floating_base_control_manisat;
% controller = @floating_base_control_test;
% controller = @hybrid_force_motion_control;
%controller = @hybrid_force_motion_control_v2;

% Control frequency (Hz)
ctrlfreq = 200;

%% Run Simulation

% Initialize the system state and input.
x = xinit;
u = zeros(4,1);

% Sets the total duration of the simulation experiment.
total_sim_time = 60.0;

% Set to < 1.0 for running simulation in slow-motion.
real_time_factor = 1.0;

% Simulation timing configurations.
dt_ctrl = 1/ctrlfreq;   % Control rate (inverse of frequencey)
dt_sim = 1e-3;          % Physics simulation time-step
dt_viz = 1/30;          % Visualization frames-per-second (1/FPS)

% Initializes visualization
robotviz.setcallback(@forceviz);
robotviz.update(x(1:7), zeros(7,1), zeros(3,1));

% Exectue a simulation using a fixed-step integration scheme.
vizT = tic;
totT = tic;
ctrlT = 0;
for t = 0:dt_sim:total_sim_time
    
    % Compute interfaction forces with the spacecraft (deprecated)
    %F_EE = wall(robotmdl, x, dx);
    
    % Integrate the system dynamics - updates state at dt_sim increments.

    ddq = space_dynamics(robotmdl, x, u);
    x = x + [x(8:end); ddq]*dt_sim;

    % Update torques from controller - applies zero-order hold.
    if t-ctrlT > dt_ctrl
        u = controller(robotmdl, t, x);
        ctrlT = t;
    end
    F_EE = zeros(3,1);
    % Update visualization and configured FPS
    if toc(vizT) > dt_viz * real_time_factor
        robotviz.update(x(1:7), x(8:14), F_EE);
        vizT = tic;
        if t > toc(totT)
            pause(t - toc(totT));
        end
    end
    
end

%% EOF
