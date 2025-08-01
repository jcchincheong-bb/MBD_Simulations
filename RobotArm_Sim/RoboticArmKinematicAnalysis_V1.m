%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is to do a 2D simulation of a basic robotic arm
% (does not work, see more from this project in my other github project
% page Robotic Arm Simulations)
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 29.07.25
% Version: 1.0
% Revision History: 
    % 1.0 - using BCF from MBD to do the simulation
% Dependencies: A_matrix.m, rev_pin_phi.m, rev_pin_gamma.m, NRfunc.m
%% Reset and Dependencies
clear
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Functions')         % Adds all defined functions to our search path
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\BC_Formulation')    % Adds all scripts to our search path

%% Defined Parameters (Model Input / Manual)
% number of bodies
nb = 5;          
nbc = nb*3;
nj = 5;
np = 4;
dof = 5;
OG = [0;0];

% Displacement Vectors
sA0_l = OG; sA1_l = [-0.35;0];              % Joint A Displacements [m]
sB1_l = [0.35;0]; sB2_l = [-0.20;0];        % Joint B Displacements [m]
sC2_l = [0.20;0]; sC3_l = [-0.20;0];        % Joint C Displacements [m]
sD3_l = [0.20;0]; sD4_l = [-0.10;0];        % Joint D Displacements [m]
s_l = {0, sA1_l, 0, 0, 0, 0; ...            % Joint A
        0, sB1_l, sB2_l, 0, 0, 0; ...       % Joint B
        0, 0, sC2_l, sC3_l, 0, 0; ...       % Joint C
        0, 0, 0, sD3_l, sD4_l, 0};          % Joint D

% Unit Vectors
u4_l = [1;0]; u5_l = [1;0];                 % Joint T
u_l = {0, 0, 0, 0, u4_l, u5_l};

% Kinematic Joints
Joints = {'rev',[1],[1,2];...                 % Joint A
    'rev',[2],[2,3];...                     % Joint B
    'rev',[3],[3,4];...                     % Joint C
    'rev',[4],[4,5];...                     % Joint D
    'tran',[0],[6,5]};                      % Joint T

% Initial Guess
q_0 = [0.303; 0.175; deg2rad(30);...            % Body 1 initial guess
        0.794; 0.418; deg2rad(20);...           % Body 2 initial guess
        1.182; 0.487; 0;...                     % Body 3 initial guess
        1.476; 0.453; -deg2rad(20);...          % Body 4 initial guess
        1.504; 0.442; -deg2rad(20)];            % Body 5 initial guess

% Initialising Velocity and Acceleration Arrays
rhsv = zeros(nbc,1);                          % RHS of velocity constraints is always zero
rhsa = [];                                    % RHS of acceleration constraints

% Simple Driver Constraint
syms t
syms phi1
syms phi2
syms phi4
syms x5
syms y5
Phi_driver = [phi1 - 0.4*t^2;...
                phi2 + 0.2*t;...
                phi4 + 0.15*t;...
                x5 + 0.4*t;...
                y5 - 0.3*t];            
rhsv_driver = [0.8*t;...
                -0.2;...
                -0.15;...
                -0.4;...
                 0.3];                     % RHS of driver velocity constraint
rhsa_driver = [0.8;...
                0;...
                0;...
                0;...
                0];                        % RHS of driver acc. constraint

% Time Information
end_time = 1;
t_step = 0.1;

%% Coordinate Setup
BC_CoordinateSetup;

%% Constraint Equations Formulation
BC_ConstraintEquations;

%% Solving Constraint Equations 
% BC_KinematicAnalysis;
Phi = subs(Phi,t,0);
[q_sol,steps] = NRfunc(Phi,q,q_0,0.001,10);
Phi_0 = vpa(subs(Phi,q,q_0)); Phi_sol = vpa(subs(Phi,q,q_sol));
T = table(q_0,Phi_0,q_sol,Phi_sol,'VariableNames',{'q0','Phi0','q_sol','Phi_sol'});
disp(T)
