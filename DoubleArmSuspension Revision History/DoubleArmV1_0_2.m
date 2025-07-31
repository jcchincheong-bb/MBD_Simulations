%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the 
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 16.05.25
% Version: 1.0.1
% Revision History: 
    % 1.0 - evaluating constraint equations for some initial guess on position level
    % 1.0.1 - using syms instead of function handles 
    % 1.0.2 - using a single equation for Phi and subbing in all values  
% Dependencies: A_matrix.m
%% Reset
clear
%% Defined Parameters
rQ0 = [0.20;-0.14]; rO0 = [0.32;0.02];% Body 0 positions (global attached)
sQ1_l = [-0.24;0]; sA1_l = [0.18;0]; % Body 1 displacements [m]
sB2_l = [-0.10;0.12]; sA2_l = [-0.07;-0.10]; % Body 2 displacements [m]
sO3_l = [-0.13;0]; sB3_l = [0.13;0]; % Body 3 displacements [m]

%% Constraint Equations
% Constraint Equations (using syms)
% Initialising the symbols
syms x [3 1]
syms y [3 1]
syms phi [3 1]
q = [x1;y1;phi1;x2;y2;phi2;x3;y3;phi3];    % I really don't like this, I need to find a more generalised way to set this up
% Setting up the position coordinate arrays
r1 = [x1;y1]; r2 = [x2;y2]; r3 = [x3;y3];
% Setting up the A Matrices 
A1 = A_matrix(phi1); A2 = A_matrix(phi2); A3 = A_matrix(phi3); 

% Syms functions for the equations
Phi = [rO0 - (r3 + A3*sO3_l);...            % Joint O
        (r1 + A1*sQ1_l)- rQ0;...            % Joint Q
        r3 + A3*sB3_l - (r2 + A2*sB2_l);... % Joint B
        r1 + A1*sA1_l - (r2 + A2*sA2_l)];   % Joint A

%% Evaluating Constraint Equations for Initial Guess
% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340); % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0); % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350); % Body 3 initial guess
q_0 = [r1_0;phi1_0;r2_0;phi2_0;r3_0;phi3_0]; % Initial Guess Array

% Evaluating
Phi_0 = subs(Phi,q,q_0);

% Displaying
T = table(double(Phi_0),'VariableNames',{'Phi_0'});
disp(T)