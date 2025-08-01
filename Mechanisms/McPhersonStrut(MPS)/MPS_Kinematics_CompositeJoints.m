%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the McPherson Suspension
% using two composite joints
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]

%% Reset and Dependencies
clear
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Functions')         % Adds all defined functions to our search path
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\BC_Formulation')    % Adds all scripts to our search path

%% Defined Parameters
% number of bodies
nb = 1;          
nbc = nb*3;
nj = 2;
np = 4;
dof = 1;
OG = [0;0];                                             % Origin array

% Displacement and Unit Vectors
sA2_l = [0;-0.07];                          % Point A Displacements [m]
sB2_l = [-0.17;0.25];                       % Point B Displacements [m]
sC2_l = [0.11;-0.02];                       % Point C Displacements [m]
rO0 = [0.41;0.13];                          % Point O Displacements [m]
rQ0 = [0.12;-0.41];                         % Point Q Displacements [m]
s_l = {0, sA2_l; ...                        % Point A
        0, sB2_l; ...                       % Point B
        rO0, 0; ...                         % Point O
        rQ0, 0};                            % Point Q
u2_l = [0.17;-0.32];                        % Unit Vector of body 2
u_l = {OG,u2_l};                    

% Initial Guess
q_0 = [0.5826;-0.3405;6.08];

% Joint Definition
Joints = {'rev-rev', [1,4],[2,1],0.45;...
    'rev-tran',[3,2],[1,2],0};...
% General strucuture of joint definition:
% Joints = {'JointType', [point array], [body array], length;...
%           ...}

% Simple Driver Constraint
syms t
syms y1
Phi_driver = y1 + 0.1*t + 0.12;         % Super simple driver constraint
rhsv_driver = -0.1;                     % RHS of driver velocity constraint
rhsa_driver = 0;                        % RHS of driver acc. constraint

% Time Information
end_time = 1;
t_step = 0.1;

%% Coordinate Setup
BC_CoordinateSetup;

%% Constraint Equations Formulation
BC_ConstraintEquations;

%% Solving Constraint Equations 
BC_KinematicAnalysis;

%% Display
% Setting up data for plotting
r_val = cell(1,nb);
for i = 1:nb
    r_val{i} = [pos_record(:,3*i-2)';...
        pos_record(:,3*i-1)'];
end

% Plotting/Animation done in another script...