%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the 
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 16.05.25
% Version: 1.0
% Revision History: 
    % 1.0 - evaluating constraint equations for some initial guess on position level
% Dependencies: A_matrix.m

%% Defined Parameters
rQ0 = [0.20;-0.14]; rO0 = [0.32;0.02];% Body 0 positions (global attached)
sQ1_l = [-0.24;0]; sA1_l = [0.18;0]; % Body 1 displacements [m]
sB2_l = [-0.10;0.12]; sA2_l = [-0.07;-0.10]; % Body 2 displacements [m]
sO3_l = [-0.13;0]; sB3_l = [0.13;0]; % Body 3 displacements [m]

%% Constraint Equations
% Constraint Equations (using function handles)
PhiO = @(r3,A3)(rO0 - (r3 + A3*sO3_l));
PhiQ = @(r1,A1)((r1 + A1*sQ1_l) - rQ0);
PhiB = @(r3,r2,A3,A2)( (r3 + A3*sB3_l) - (r2 + A2*sB2_l) );
PhiA = @(r1,r2,A1,A2)( (r1 + A1*sA1_l) - (r2 + A2*sA2_l) );

%% Evaluating Constraint Equations for Initial Guess
% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340); % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0); % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350); % Body 3 initial guess
c_0 = [[r1_0;phi1_0] [r2_0;phi2_0] [r3_0;phi3_0]]; % Initial Guess Array

% Evaluating
PhiO_0 = PhiO(r3_0,A_matrix(phi3_0));
PhiQ_0 = PhiQ(r1_0,A_matrix(phi1_0));
PhiB_0 = PhiB(r3_0,r2_0,A_matrix(phi3_0),A_matrix(phi2_0));
PhiA_0 = PhiA(r1_0,r2_0,A_matrix(phi1_0),A_matrix(phi2_0));

% Displaying
T = table(PhiO_0,PhiQ_0,PhiB_0,PhiA_0,'VariableNames',{'PhiO','PhiQ','PhiB','PhiA'});
disp(T)