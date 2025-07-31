%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the 
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 16.05.25
% Version: 1.0.1
% Revision History: 
    % 1.0 - evaluating constraint equations for some initial guess on position level
    % 1.0.1 - using syms instead of function handles 
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
% Setting up the position coordinate arrays
r1 = [x1;y1]; r2 = [x2;y2]; r3 = [x3;y3];
% Setting up the A Matrices 
A1 = A_matrix(phi1); A2 = A_matrix(phi2); A3 = A_matrix(phi3); 

% Syms functions for the equations
PhiO = rO0 - (r3 + A3*sO3_l);
PhiQ = (r1 + A1*sQ1_l)- rQ0;
PhiB = r3 + A3*sB3_l - (r2 + A2*sB2_l);
PhiA = r1 + A1*sA1_l - (r2 + A2*sA2_l);

%% Evaluating Constraint Equations for Initial Guess
% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340); % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0); % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350); % Body 3 initial guess
c_0 = [[r1_0;phi1_0] [r2_0;phi2_0] [r3_0;phi3_0]]; % Initial Guess Array
x_0 = c_0(1,:); y_0 = c_0(2,:); phi_0 = c_0(3,:);

% Evaluating
PhiO_00 = PhiO;
PhiQ_00 = PhiQ;
PhiB_00 = PhiB;
PhiA_00 = PhiA;
for i = 1:3
    % subs in the ith x,y and phi values into the equation
    PhiO_0 = subs(PhiO_00,{x(i),y(i),phi(i)},{x_0(i),y_0(i),phi_0(i)}); 
    % updates the temp equation so it can be used in the next iteration
    % (psudeo-recursion) 
    PhiO_00 = PhiO_0;
    % The other equations, same process
    PhiQ_0 = subs(PhiQ_00,{x(i),y(i),phi(i)},{x_0(i),y_0(i),phi_0(i)}); PhiQ_00 = PhiQ_0;
    PhiA_0 = subs(PhiA_00,{x(i),y(i),phi(i)},{x_0(i),y_0(i),phi_0(i)}); PhiA_00 = PhiA_0;
    PhiB_0 = subs(PhiB_00,{x(i),y(i),phi(i)},{x_0(i),y_0(i),phi_0(i)}); PhiB_00 = PhiB_0;
end


% Displaying
T = table(double(PhiO_0),double(PhiQ_0),double(PhiB_0),double(PhiA_0),'VariableNames',{'PhiO','PhiQ','PhiB','PhiA'});
disp(T)