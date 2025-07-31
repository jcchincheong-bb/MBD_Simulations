%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the McPherson Suspension on
%               Position levels
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 15.06.25
% Version: 5.0
% Revision History: 
%               implemented both the rev-rev composite joint and rev-tran
%               composite joint
% Dependencies: A_matrix.m, rev_pin_phi.m, s_rot.m, tran_phi.m

%% Reset and Dependencies
clear
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\Functions')         % Adds all defined functions to our search path
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\BC_Formulation')    % Adds all scripts to our search path

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
% q_0 = [0.345;-0.41;0;...
%     0.5826;-0.3405;6.08;...
%     0.4525;-0.0138;5];

q_0 = [0.5826;-0.3405;6.08];

% Joint Definition
Joints = {'rev-rev', [1,4],[2,1],0.45;...
    'rev-tran',[3,2],[1,2],0};...
% General strucuture of joint definition:
% Joints = {'JointType', [point array], [body array], length;...
%           ...}
%% Coordinate Setup
BC_CoordinateSetup;

%% Constraint Equations Formulation
BC_ConstraintEquations;

%% Evaluating Constraint Equations
Phi_sol = subs(Phi,q,q_0);
T = table(vpa(Phi_sol),'VariableNames',{'Constraint'},'RowNames',{'rev-rev','rev-tran'});
disp(T)