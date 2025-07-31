%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the Double Arm Suspension on
%               Position level
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 16.05.25
% Version: 1.0.4
% Revision History: 
    % 1.0 - evaluating constraint equations for some initial guess on position level
    % 1.0.1 - using syms instead of function handles 
    % 1.0.2 - using a single equation for Phi and subbing in all values
    % 1.0.3 - using rev_pin() function to form the constraint equations,
    %          and cell arrays for the r and A arrays
    % 1.0.4 - adjusted the rev_pin() function to take global inputs and
    %           added velocity level constraints
% Dependencies: A_matrix.m, rev_pin.m
%% Reset and Dependencies
clear
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\Functions')  % Adds all defined functions to our search path
%% Defined Parameters
rO0 = [0.32;0.02]; sO3_l = [-0.13;0];       % Joint O Displacements [m]
rQ0 = [0.20;-0.14]; sQ1_l = [-0.24;0];      % Joint Q Displacements [m]
sA1_l = [0.18;0]; sA2_l = [-0.07;-0.10];    % Joint A Displacements [m]
sB2_l = [-0.10;0.12]; sB3_l = [0.13;0];     % Joint B Displacements [m]

%% Coordinate Formulation
% Initialising the symbols
m = 3;          % number of bodies
syms t
syms x [m 1]
syms y [m 1]
syms phi [m 1]
syms xd [m 1]
syms yd [m 1]
syms phid [m 1]

% Initialising our coordinate arrays
syms q [3*m 1]
syms qd [3*m 1]
r = cell(1,m);
rd = cell(1,m);
A = cell(1,m);

% Setting up the position coordinate arrays
for i = 1:m
    q(3*i-2)=x(i);q(3*i-1)=y(i);q(3*i)=phi(i);          % coordinate array
    qd(3*i-2)=xd(i);qd(3*i-1)=yd(i);qd(3*i)=phid(i);    % coordinate velocity array
    r{i} = [x(i);y(i)];                                 % position arrays
    rd{i} = [xd(i);yd(i)];                              % velocity arrays
    A{i} = A_matrix(phi(i));                            % A matrices
end

% Globalising 
sA1 = A{1}*sA1_l; sQ1 = A{1}*sQ1_l;
sB2 = A{2}*sB2_l; sA2 = A{2}*sA2_l;
sO3 = A{3}*sO3_l; sB3 = A{3}*sB3_l;


%% Constraint Equations
% Initialising Velocity and Acceleration Arrays
rhsv = zeros(3*m,1);                          % RHS of velocity constraints is always zero
% rhsa =  requires the gamma function 

% Simple Driver Constraint
Phi_driver = y2 - 0.01*t;               % Super simple driver constraint
rhsv_driver = 0.01;                     % RHS of driver velocity constraint
rhsa_driver = 0;                        % RHS of driver acc. constraint

% Syms equations for the constraints
Phi = [rev_pin(r{1},r{2},sA1,sA2);...      % Joint A
        rev_pin(r{3},r{2},sB3,sB2);...      % Joint B
        rev_pin([0;0],r{3},rO0,sO3);...      % Joint O
        rev_pin([0;0],r{1},rQ0,sQ1);...     % Joint Q
        Phi_driver];                        % Driver Constraint

% Jacobian
D = jacobian(Phi,q);                        % Constraint Jacobian

%% Evaluating Constraint Equations for Initial Guess
% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340);     % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0);       % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350);      % Body 3 initial guess
q_0 = [r1_0;phi1_0;r2_0;phi2_0;r3_0;phi3_0];    % Initial Guess Array

% Evaluating Position
Phi_0 = subs(Phi,q,q_0);
Phi_0 = subs(Phi_0,t,0);   % At t= 0;

% Evaluating Velocity
rhsv(3*m) = rhsv_driver;   % At t= 0;
D_0 = subs(D,q,q_0);
qd_0 = inv(D_0)*rhsv;
Phid_0 = D_0*qd_0;

% Displaying
T = table(double(Phi_0),double(Phid_0),'VariableNames',{'Phi_0','Phid_0'});
disp(T)