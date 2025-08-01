%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the Double Arm Suspension on
%               Position, Velocity and Acceleration levels with a simple
%               driver constraint
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Dependencies: A_matrix.m, rev_pin_phi.m, rev_pin_gamma.m, tran_phi.m, tran_gamma.m, revrev_phi.m, revrev_gamm, revtran_phi.m, revtran_gamma.m, NRfunc.m
%% Reset and Dependencies
clear
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Functions')         % Adds all defined functions to our search path
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\BC_Formulation')    % Adds all scripts to our search path

%% Defined Parameters (Model Input / Manual)
% number of bodies
nb = 3;          
nbc = nb*3;
nj = 4;
np = 4;
OG = [0;0];

% Displacement Vectors
sA1_l = [0.18;0]; sA2_l = [-0.07;-0.10];    % Joint A Displacements [m]
sB2_l = [-0.10;0.12]; sB3_l = [0.13;0];     % Joint B Displacements [m]
rO0 = [0.32;0.02]; sO3_l = [-0.13;0];       % Joint O Displacements [m]
rQ0 = [0.20;-0.14]; sQ1_l = [-0.24;0];      % Joint Q Displacements [m]
s_l = {0, sA1_l, sA2_l, 0; ...              % Joint A
        0, 0, sB2_l, sB3_l; ...             % Joint B
        rO0, 0, 0, sO3_l; ...               % Joint O
        rQ0, sQ1_l, 0, 0};                  % Joint Q

% Kinematic Joints
Joints = {'rev',[1],[2,3];...               % Joint A
    'rev',[2],[3,4];...                     % Joint B
    'rev',[3],[1,4];...                     % Joint O
    'rev',[4],[1,2]};                       % Joint Q

% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340);     % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0);       % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350);      % Body 3 initial guess
q_0 = [r1_0;phi1_0;r2_0;phi2_0;r3_0;phi3_0];    % Initial Guess Array

% Initialising Velocity and Acceleration Arrays
rhsv = zeros(nbc,1);                          % RHS of velocity constraints is always zero
rhsa = [];                                    % RHS of acceleration constraints

% Simple Driver Constraint
syms t
syms y2
Phi_driver = y2 + 0.1*t + 0.12;         % Super simple driver constraint
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

%% Displaying Results
% Setting up data for plotting
r_val = cell(1,nb); rd_val = cell(1,nb); rdd_val = cell(1,nb);
for i = 1:nb
    r_val{i} = [pos_record(:,3*i-2)';...
        pos_record(:,3*i-1)'];
    rd_val{i} = [vel_record(:,3*i-2)';...
        vel_record(:,3*i-1)'];
    rdd_val{i} = [acc_record(:,3*i-2)';...
        acc_record(:,3*i-1)'];
end

% Plotting/Animation done in another script...