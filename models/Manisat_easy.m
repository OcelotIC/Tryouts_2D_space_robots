% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% World Gravity

% Gravitational potential field in inertial world-frame
syms grav real;
e_I_g = [0 0 1].';
I_a_g = grav*e_I_g;

%% System Parameters & Variables

% Body masses
syms m_B m_LT m_LS m_RT m_RS real;

% Body geometry parameters
syms l_B w_B h_B real;
syms r_LT l_HT real;
syms r_LS l_LS real;
syms r_LF real;
syms r_RT l_RT real;
syms r_RS l_RS real;
syms r_RF real;

% Declare all system variables
% State variables
syms x_B y_B theta_B phi_L1 phi_L2 phi_R1 phi_R2 real;
% Applicable joint torques
syms tau_L1 tau_L2 tau_R1 tau_R2  real;
% Feet contact forces and torques (6D wrenches)
syms f_LFx f_LFy f_LFz t_LFx t_LFy t_LFz 
syms f_RFx f_RFy f_RFz t_RFx t_RFy t_RFz 


%% Multi-Body System Description (Kinematic Tree)

% Base
i=1;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'Base';
body(i).ktree.nodeid        = 1;
body(i).ktree.parents       = 0;
body(i).cs.P_r_PB           = [x_B 0 y_B].';
body(i).cs.C_PB             = getRotationMatrixY(theta_B);
body(i).param.m             = m_B;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 0].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cuboid';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [l_B w_B h_B] ; 
body(i).geometry.values     = [0.6 0.2 0.1];
body(i).geometry.offsets.r  = [0 0 0].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [197 172 202]/255;

% Left thigh
i=2;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'LT';
body(i).ktree.nodeid        = 2;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [-l_B/2 0 -h_B/2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_L1);
body(i).param.m             = m_LT;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_HT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_LT l_HT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Left Shank (LS)
i=3;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'LS';
body(i).ktree.nodeid        = 3;
body(i).ktree.parents       = 2;
body(i).cs.P_r_PB           = [0 0 -l_HT].';
body(i).cs.C_PB             = getRotationMatrixY(phi_L2);
body(i).param.m             = m_LS;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_HT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_LT l_HT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Left Foot (LF)
i=4;
body(i) = RigidBodyDescription_v2;
body(i).name = 'LF';
body(i).ktree.nodeid = 4;
body(i).ktree.parents = 3;
body(i).cs.P_r_PB = [0 0 -l_LS].';
body(i).cs.C_PB = sym(eye(3));
body(i).geometry.type = 'sphere';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_LF]; 
body(i).geometry.values = [0.02];
body(i).geometry.offsets.r = [0 0 0].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.8 0.6 0.6];

% Right Thigh (RT)
i=5;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'RT';
body(i).ktree.nodeid        = 5;
body(i).ktree.parents       = 1;
body(i).cs.P_r_PB           = [l_B/2 0 -h_B/2].';
body(i).cs.C_PB             = getRotationMatrixY(phi_R1);
body(i).param.m             = m_RT;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_RT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_RT l_RT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Right Shank (FS)
i=6;
body(i)                     = RigidBodyDescription_v2;
body(i).name                = 'RS';
body(i).ktree.nodeid        = 6;
body(i).ktree.parents       = 5;
body(i).cs.P_r_PB           = [0 0 -l_RT].';
body(i).cs.C_PB             = getRotationMatrixY(phi_R2);
body(i).param.m             = m_RS;
body(i).param.B_Theta       = [];
body(i).param.B_r_BCoM      = [0 0 -l_RT/2].'; 
body(i).param.C_BCoM        = sym(eye(3));
body(i).geometry.type       = 'cylinder';
body(i).geometry.issolid    = true;
body(i).geometry.params     = [r_RT l_RT]; 
body(i).geometry.values     = [0.015 0.3];
body(i).geometry.offsets.r  = [0 0 0.15].';
body(i).geometry.offsets.C  = eye(3);
body(i).geometry.color      = [0.6 0.6 0.6];

% Right Foot (RF)
i=7;
body(i) = RigidBodyDescription_v2;
body(i).name = 'RF';
body(i).ktree.nodeid = 7;
body(i).ktree.parents = 6;
body(i).cs.P_r_PB = [0 0 -l_RS].';
body(i).cs.C_PB = sym(eye(3));
body(i).geometry.type = 'sphere';
body(i).geometry.issolid = false;
body(i).geometry.params = [r_RF]; 
body(i).geometry.values = [0.02];
body(i).geometry.offsets.r = [0 0 0].';
body(i).geometry.offsets.C = eye(3);
body(i).geometry.color = [0.8 0.6 0.6];



%% Definition of External Forces & Torques

%
% Joint Torques
%

j=1;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_RHFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 2;
ftel(j).B_T     = [0 0 tau_L1].';

j=2;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_RKFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 2; 
ftel(j).body_B  = 3;
ftel(j).B_T     = [0 0 tau_L2].';

j=3;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_FHFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 1; 
ftel(j).body_B  = 5;
ftel(j).B_T     = [0 0 tau_R1].';

j=4;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'tau_FKFE';
ftel(j).type    = 'rotational';
ftel(j).body_P  = 5; 
ftel(j).body_B  = 6;
ftel(j).B_T     = [0 0 tau_R2 ].';


%
% Contact Wrenches
%

j=5;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'w_LF';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 4;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 -r_LF].');
ftel(j).I_F     = [f_LFx f_LFy f_LFz].';
ftel(j).I_T     = [t_LFx t_LFy t_LFz].';

j=6;
ftel(j) = ForceTorqueDescription_v2;
ftel(j).name    = 'w_RF';
ftel(j).type    = 'wrench';
ftel(j).body_P  = 0; 
ftel(j).body_B  = 7;
ftel(j).P_r_R   = sym([0 0 0].');
ftel(j).B_r_A   = sym([0 0 -r_RF].');
ftel(j).I_F     = [f_RFx f_RFy f_RFz].';
ftel(j).I_T     = [t_RFx t_RFy t_RFz].';



%% System Definitions

% Definition of the joint DoFs of 2-link system
q_j  = [x_B y_B theta_B phi_L1 phi_L2 phi_R1 phi_R2 ].';

% Controllable joint forces/torques
tau_j = [tau_L1 tau_L2 tau_R1 tau_R2 ].';

% External forces/torques
tau_env = [f_LFx f_LFy f_LFz t_LFx t_LFy t_LFz f_RFx f_RFy f_RFz t_RFx t_RFy t_RFz].';

%% Generate Full System Model using proNEu.v2

% Give a name to the model
robot_name = 'Manisat_easy';

% Generate the model object
robotmdl = RobotModel(body, ftel, q_j, tau_j, tau_env, I_a_g, 'name', robot_name, 'type', 'fixed', 'method', 'proneu', 'symsteps', 100);

%% Symbolic Simplifications (Advanced)

% Set number of symbolic simplification steps
ns = 100;

% Test symbolic simplifciations of M,b,g - use serial computations
robotmdl.dynamics.symbols.simplifydynamics('elementwise', ns);

%% Generate MATLAB numerical functions

% Generate numerical functions
robotmdl.generatefunctions();

%% Save to MAT File

% Generate file and directory paths
[fpath,fname,fext] = fileparts(mfilename('fullpath'));
dpath = strrep(fpath, 'generators', 'models/');

% Store generated model in the appropriate directory
save(strcat(dpath,robotmdl.name), 'robotmdl');

%% 
% EOF