%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the Double Arm Suspension on
%               Position, Velocity and Acceleration levels with a simple
%               driver constraint
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 04.06.25
% Version: 2.1
% Revision History: 
    % 1.0 - evaluating constraint equations for some initial guess on position level
    % 1.0.1 - using syms instead of function handles 
    % 1.0.2 - using a single equation for Phi and subbing in all values
    % 1.0.3 - using rev_pin() function to form the constraint equations,
    %          and cell arrays for the r and A arrays
    % 1.0.4 - adjusted the rev_pin() function to take global inputs and
    %           added velocity level constraints
    % 1.0.5 - even more generalising, make it so that the user can add
    %           info directly to the manual section and everything else is automatic
    % 2.0 - using NRfunc, solving the position and velocity constraints w/
    %       initial guess
    % 2.1 - solving for acceleration 
% Dependencies: A_matrix.m, rev_pin_phi.m, rev_pin_gamma.m, NRfunc.m
%% Reset and Dependencies
clear
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\Functions')  % Adds all defined functions to our search path

%% Defined Parameters (Model Input / Manual)
% number of bodies
nb = 3;          
nbc = nb*3;
nj = 4;

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
Joints = {'rev','rev','rev','rev'};         % A, B, O, Q

% Initial Guess
r1_0 = [0.51;-0.20]; phi1_0 = deg2rad(340);     % Body 1 initial guess
r2_0 = [0.75;-0.07]; phi2_0 = deg2rad(0);       % Body 2 initial guess
r3_0 = [0.49;0.02]; phi3_0 = deg2rad(350);      % Body 3 initial guess
q_0 = [r1_0;phi1_0;r2_0;phi2_0;r3_0;phi3_0];    % Initial Guess Array

% Simple Driver Constraint
syms t
syms y2
Phi_driver = y2 - 0.01*t;               % Super simple driver constraint
rhsv_driver = 0.01;                     % RHS of driver velocity constraint
rhsa_driver = 0;                        % RHS of driver acc. constraint

%% Coordinate Setup
% Initialising the symbols
syms t
syms x [nb 1]
syms y [nb 1]
syms phi [nb 1]
syms xd [nb 1]
syms yd [nb 1]
syms phid [nb 1]

% Initialising our coordinate arrays
syms q [nbc 1]
syms qd [nbc 1]
r = cell(1,nb);                                         % Positions arrays
A = cell(1,nb);                                         % A matrices
rd = cell(1,nb);

% Setting up the position coordinate arrays
for i = 1:nb
    q(3*i-2)=x(i);q(3*i-1)=y(i);q(3*i)=phi(i);          % coordinate array
    qd(3*i-2)=xd(i);qd(3*i-1)=yd(i);qd(3*i)=phid(i);    % coordinate velocity array
    r{i} = [x(i);y(i)];                                 % position arrays
    rd{i} = [xd(i);yd(i)];                              % velocity arrays
    A{i} = A_matrix(phi(i));                            % A matrices
end

% Globalising 
s = cell(nj,nb+1);                            % Global displacement array
for i = 1:nj
    for j = 1:nb+1
        % nb + 1 since the we only do coordinates for 3 bodies and the
        % ground attached body is neglected, so we need to account for it
        % here
        if s_l{i,j} == 0
            s{i,j} = [];
        elseif j == 1    % First row is body zero which is not moving 
            s{i,j} = s_l{i,j};
        else
            s{i,j} = A{j-1}*s_l{i,j};
        end
    end
end

%% Constraint Equations Formulation
% Initialising Velocity and Acceleration Arrays
rhsv = zeros(nbc,1);                          % RHS of velocity constraints is always zero
rhsa = [];                                    % RHS of acceleration constraints

% Syms equations for the constraints (not generalised as yet, only rev-pin)
Phi = [];           % Empty constraint equation array
gamma = [];         % Empty gamma function array
for i = 1:(nj)
    k = 1;
    switch (Joints{i})
        case ('rev')                            % Revolut Pin Joint
            for j = 1:nb+1
                if (~isempty(s{i,j}))           % Checks for empty arrays (ie point doesnt exist)
                    s_temp{k} = s{i,j};         % stores the point displacement array
                    if (j==1)                   % checks if body 0 (grounded body)
                        r_temp{k} = [0;0];      % stores the body position array
                        phid_temp{k} = 0;       % stores the angular accleration variable
                    else
                        r_temp{k} = r{j-1};     % stores the body position array
                        phid_temp{k} = phid(j-1); % stores the angular accleration variable
                    end
                    k = k+1;                    % updates only when the arrays of a joint are found
                end
            end
            % Adding the rev-pin constraint equation and gamma function to the array
            Phi = vertcat(Phi,...
                rev_pin_phi(r_temp{1},r_temp{2},s_temp{1},s_temp{2}));
            gamma = vertcat(gamma,...
                rev_pin_gamma(phid_temp{1},phid_temp{2},s_temp{1},s_temp{2}));
    end
end

Phi = vertcat(Phi,Phi_driver);              % Adding the driver constraint

% Jacobian
D = jacobian(Phi,q);                        % Constraint Jacobian

%% Solving Constraint Equations 
% Evaluating Position Coordinates
Phi_sol = subs(Phi,t,0);                            % At t= 0;
[steps, q_sol] = NRfunc(Phi_sol,q,q_0,0.001,10);    % Solving our position coordinates
Phi_sol = subs(Phi_sol,q,q_sol);                    % Evaluating Phi at q_sol

% Evaluating Velocity Constraints
rhsv(nbc) = rhsv_driver;                            % At t= 0;
D_sol = subs(D,q,q_sol);                            % Evaluating jacobian at q_sol
qd_sol = inv(D_sol)*rhsv;                           % Solving our velocity coordinates
Phid_sol = D_sol*qd_sol;                            % Evaluating velocity constraints

% Evaluating Acceleration Constraints
gamma_sol = subs(gamma,qd,qd_sol);                  % Sub in velocity values
gamma_sol = subs(gamma_sol,q,q_sol);                % Sub in position values
rhsa = -gamma_sol; rhsa = [rhsa;-rhsa_driver];      % Set RHS of acceleration constraint
qdd_sol = inv(D_sol)*rhsa;                          % Solving our acceleration coordinates
Phidd_sol = D_sol*qdd_sol - rhsa;                   % Evaluating acceleration constraints

% Displaying
T = table(double(q_sol),double(Phi_sol),double(qd_sol),double(Phid_sol),double(qdd_sol),double(Phidd_sol),'VariableNames',{'q_sol','Phi_sol','qd_sol','Phid_sol','qdd_sol','Phidd_sol'});
disp(T)