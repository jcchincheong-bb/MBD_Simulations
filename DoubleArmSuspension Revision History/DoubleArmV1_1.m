%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the 
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 16.05.25
% Version: 1.0
% Revision History: 
    % 1.0 - evaluating constraint equations for some initial guess on position level
    % 1.1 - solving using fsolve(), required 9x1 Phi function handle using
    %       a 9x1 input array q. 
    %     - Also redefined the input of function
    %       handles for the phi and not the A matrix. 
    %     - Added a a basic driver constraint, will need to choose a better one for 
    %       better results
    %     - removed script for initial guess (not useful) 
% Dependencies: A_matrix.m

%% Defined Parameters
rQ0 = [0.20;-0.14]; rO0 = [0.32;0.02];% Body 0 positions (global attached)
sQ1_l = [-0.24;0]; sA1_l = [0.18;0]; % Body 1 displacements [m]
sB2_l = [-0.10;0.12]; sA2_l = [-0.07;-0.10]; % Body 2 displacements [m]
sO3_l = [-0.13;0]; sB3_l = [0.13;0]; % Body 3 displacements [m]

% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340); % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0); % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350); % Body 3 initial guess
q_0 = [r1_0;phi1_0;r2_0;phi2_0;r3_0;phi3_0]; % Initial Guess Array
%% Constraint Equations
% Constraint Equations (using function handles)
PhiO = @(r3,phi3)(rO0 - (r3 + A_matrix(phi3)*sO3_l)); % Joint O
PhiQ = @(r1,phi1)((r1 + A_matrix(phi1)*sQ1_l) - rQ0); % Joint Q
PhiB = @(r3,r2,phi3,phi2)( (r3 + A_matrix(phi3)*sB3_l) - (r2 + A_matrix(phi2)*sB2_l) ); % Joint B
PhiA = @(r1,r2,phi1,phi2)( (r1 + A_matrix(phi1)*sA1_l) - (r2 + A_matrix(phi2)*sA2_l) ); % Joint A
Phi = @(q)(vertcat(PhiO([q(7);q(8)],q(9)),PhiQ([q(1);q(2)],q(3)),PhiB([q(7);q(8)],[q(4);q(5)],q(9),q(6)),PhiA([q(1);q(2)],[q(4);q(5)],q(3),q(6)), ...
    q(1)-0.6));       % Driver constraint 


%% Solving for using fsolve()
q_sol = fsolve(Phi,q_0);
% Displaying
T = table(q_0,Phi(q_0),q_sol,Phi(q_sol),'VariableNames',{'q0','Phi(q0)','qsol','Phi(qsol)'},'RowNames',{'x1','y1','phi1','x2','y2','phi2','x3','y3','phi3'});
fprintf("All x and y in meters and all phi in radians\n")
disp(T)