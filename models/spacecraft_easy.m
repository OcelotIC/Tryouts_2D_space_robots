% Optional calls for workspace resetting/cleaning
close all; clear all; clear classes; clc;

%% Define World Model Parameters

% spacecraft parameters
syms w_sc h_sc real;

%%

k=1;
element(k) = WorldElementDescription;
element(k).name = 'ground';
element(k).geometry.type = 'plane';
element(k).geometry.issolid = false;
element(k).geometry.params = [w_sc h_sc]; 
element(k).geometry.values = [10.0 10.0];
element(k).geometry.offsets.r = [0 0 0].';
element(k).geometry.offsets.C = eye(3);
element(k).geometry.color = [0.96 0.96 0.96];



%% Generate a Model of the World

% Set the world's name
world_name = 'sc_easy';

% Generate the world model object
worldmdl = WorldModel(element, @feet_arm_contacts, 'name', world_name);

%% Save to MAT File

% Generate file and directory paths
[fpath,fname,fext] = fileparts(mfilename('fullpath'));
dpath = strrep(fpath, 'generators', 'models/');

% Store generated model in the appropriate directory
save(strcat(dpath,worldmdl.name), 'worldmdl');

%%
% EOF
