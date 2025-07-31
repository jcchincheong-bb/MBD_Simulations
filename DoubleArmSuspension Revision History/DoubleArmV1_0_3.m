%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the Double Arm Suspension on
%               Position level
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 16.05.25
% Version: 1.0.3
% Revision History: 
    % 1.0 - evaluating constraint equations for some initial guess on position level
    % 1.0.1 - using syms instead of function handles 
    % 1.0.2 - using a single equation for Phi and subbing in all values
    % 1.0.3 - using rev_pin() function to form the constraint equations,
    %          and cell arrays for the r and A arrays
% Dependencies: A_matrix.m, rev_pin_old.m
%% Reset and Dependencies
clear
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\Functions')  % Adds all defined functions to our search path
%% Defined Parameters
rO0 = [0.32;0.02]; sO3_l = [-0.13;0];       % Joint O Displacements [m]
rQ0 = [0.20;-0.14]; sQ1_l = [-0.24;0];      % Joint Q Displacements [m]
sA1_l = [0.18;0]; sA2_l = [-0.07;-0.10];    % Joint A Displacements [m]
sB2_l = [-0.10;0.12]; sB3_l = [0.13;0];     % Joint B Displacements [m]

%% Constraint Equations
% Constraint Equations (using syms)
% Initialising the symbols
m = 3;          % number of bodies
syms x [m 1]
syms y [m 1]
syms phi [m 1]

% Initialising our coordinate arrays
syms q [3*m 1]
r = cell(1,m);
A = cell(1,m);

% Setting up the position coordinate arrays
for i = 1:m
    q(3*i-2)=x(i);q(3*i-1)=y(i);q(3*i)=phi(i);      % coordinate array
    r{i} = [x(i);y(i)];                             % position arrays
    % A{i} = A_matrix(phi(i));                      % A matrices
end

% Syms equations for the constraints
Phi = [rev_pin_old([0;0],r{3},0,phi3,rO0,sO3_l);...           % Joint O
        rev_pin_old([0;0],r{1},0,phi1,rQ0,sQ1_l);...          % Joint Q
        rev_pin_old(r{3},r{2},phi3,phi2,sB3_l,sB2_l);...      % Joint B
        rev_pin_old(r{1},r{2},phi1,phi2,sA1_l,sA2_l)];        % Joint A

%% Evaluating Constraint Equations for Initial Guess
% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340);     % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0);       % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350);      % Body 3 initial guess
q_0 = [r1_0;phi1_0;r2_0;phi2_0;r3_0;phi3_0];    % Initial Guess Array

% Evaluating
Phi_0 = subs(Phi,q,q_0);

% Displaying
T = table(double(Phi_0),'VariableNames',{'Phi_0'});
disp(T)