%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the Double Arm Suspension on
%               Position level
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 04.06.25
% Version: 1.0.5
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
% Dependencies: A_matrix.m, rev_pin.m
%% Reset and Dependencies
clear
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\Functions')  % Adds all defined functions to our search path

%% Defined Parameters (Model Input / Manual)
% number of bodies
nb = 3;          
nbc = nb*3;
nj = 4;
% Displacement Vectors
rO0 = [0.32;0.02]; sO3_l = [-0.13;0];       % Joint O Displacements [m]
rQ0 = [0.20;-0.14]; sQ1_l = [-0.24;0];      % Joint Q Displacements [m]
sA1_l = [0.18;0]; sA2_l = [-0.07;-0.10];    % Joint A Displacements [m]
sB2_l = [-0.10;0.12]; sB3_l = [0.13;0];     % Joint B Displacements [m]
s_l = {0, sA1_l, sA2_l, 0; ...
        0, 0, sB2_l, sB3_l; ...
        rO0, 0, 0, sO3_l; ...
        rQ0, sQ1_l, 0, 0};                  % Displacement Cell Array
Joints = {'rev','rev','rev','rev'};

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
rd = cell(1,nb);                                        % Velocity arrays
A = cell(1,nb);                                         % A matrices

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
% rhsa =  requires the gamma function 

% Syms equations for the constraints (not generalised as yet, only rev-pin)
Phi = [];
for i = 1:(nj)
    k = 1;
    switch (Joints{i})
        case ('rev')                            % Revolut Pin Joint
            for j = 1:nb+1
                if (~isempty(s{i,j}))           % Checks for empty arrays (ie point doesnt exist)
                    s_temp{k} = s{i,j};         % stores the point displacement array
                    if (j==1)                   % checks if body 0 (grounded body)
                        r_temp{k} = [0;0];      % stores the body position array
                    else
                        r_temp{k} = r{j-1};     % stores the body position array
                    end
                    k = k+1;                    % updates only when the arrays of a joint are found
                end
            end
            Phi = vertcat(Phi,rev_pin(r_temp{1},r_temp{2},s_temp{1},s_temp{2}));
    end
end

Phi = vertcat(Phi,Phi_driver);

% Jacobian
D = jacobian(Phi,q);                        % Constraint Jacobian

%% Evaluating Constraint Equations for Initial Guess
% Evaluating Position Constraints
Phi_0 = subs(Phi,q,q_0);
Phi_0 = subs(Phi_0,t,0);   % At t= 0;

% Evaluating Velocity Constraints
rhsv(nbc) = rhsv_driver;   % At t= 0;
D_0 = subs(D,q,q_0);
qd_0 = inv(D_0)*rhsv;
Phid_0 = D_0*qd_0;

% Displaying
T = table(double(Phi_0),double(qd_0),double(Phid_0),'VariableNames',{'Phi_0','qd_0','Phid_0'});
disp(T)