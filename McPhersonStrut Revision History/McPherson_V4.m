%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to the McPherson Suspension on
%               Position levels
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 15.06.25
% Version: 4.0
% Revision History: 
%               implemented both the rev-rev composite joint and rev-tran
%               composite joint
% Dependencies: A_matrix.m, rev_pin_phi.m, s_rot.m, tran_phi.m

%% Reset and Dependencies
clear
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\Functions')  % Adds all defined functions to our search path

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
s = cell(np,nb+1);                            % Global displacement array
u = cell(1,nb+1);                             % Global unit vec array
for i = 1:np
    for j = 1:nb+1
        % nb + 1 since the we only do coordinates for nb bodies and the
        % ground attached body is neglected, so we need to account for it
        % here
        if s_l{i,j} == 0
            s{i,j} = [];
        elseif j == 1    % First row is body zero which is not moving 
            s{i,j} = s_l{i,j};
        else
            s{i,j} = A{j-1}*s_l{i,j};
        end

        if u_l{j} == OG
            u{j} = [];
        else
            u{j} = A{j-1}*u_l{j};
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
    switch (Joints{i})
        case ('rev')                            % Revolut Pin Joint
            points = Joints{i,2}; bodies = Joints{i,3}; 
            for b = 1:length(bodies)
                s_temp{b} = s{points,bodies(b)};            % stores the point displacement array
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    r_temp{b} = OG;                         % stores the body position array
                    phid_temp{b} = 0;                       % stores the angular accleration variable
                else
                    r_temp{b} = r{bodies(b)-1};             % stores the body position array
                    phid_temp{b} = phid(bodies(b)-1);       % stores the angular veloctiy variable
                end
            end
            % Adding the rev-pin constraint equation and gamma function to the array
            Phi = vertcat(Phi,...
                rev_pin_phi(r_temp{1},r_temp{2},s_temp{1},s_temp{2}));
            gamma = vertcat(gamma,...
                rev_pin_gamma(phid_temp{1},phid_temp{2},s_temp{1},s_temp{2}));
        
        case ('tran')
            points = Joints{i,2}; bodies = Joints{i,3}; 
            for b = 1:length(bodies)
                s_temp{b} = s{points(b),bodies(b)};         % stores the point displacement array
                u_temp{b} = u{bodies(b)};
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    r_temp{b} = OG;                         % stores the body position array 
                else
                    r_temp{b} = r{bodies(b)-1};             % stores the body position array
                end
            end
            % Adding the translational constraint equation to the array
            Phi = vertcat(Phi,...
                tran_phi(r_temp{1},r_temp{2},u_temp{1},u_temp{2},s_temp{1},s_temp{2}));

        case('rev-rev')
            points = Joints{i,2}; bodies = Joints{i,3}; L = Joints{i,4};
            for b = 1:length(bodies)
                s_temp{b} = s{points(b),bodies(b)};         % stores the point displacement array
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    r_temp{b} = OG;                         % stores the body position array
                else
                    r_temp{b} = r{bodies(b)-1};             % stores the body position array
                end
            end

            % Adding the rev-pin constraint equation and gamma function to the array
            Phi = vertcat(Phi,...
                revrev_phi(r_temp{1},r_temp{2},s_temp{1},s_temp{2},L));
        
        case('rev-tran')
            points = Joints{i,2}; bodies = Joints{i,3}; 
            L = Joints{i,4};
            u_temp = u{bodies(2)};
            for b = 1:length(bodies)
                s_temp{b} = s{points(b),bodies(b)};         % stores the point displacement array
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    r_temp{b} = OG;                         % stores the body position array
                else
                    r_temp{b} = r{bodies(b)-1};             % stores the body position array
                end
            end

            % Adding the rev-pin constraint equation and gamma function to the array
            Phi = vertcat(Phi,...
                revtran_phi(r_temp{1},r_temp{2},s_temp{1},s_temp{2},u_temp,L));
    end
end

% Phi = vertcat(Phi,Phi_driver);              % Adding the driver constraint

% Jacobian
D = jacobian(Phi,q);                        % Constraint Jacobian

%% Evaluating Constraint Equations
Phi_sol = subs(Phi,q,q_0);
T = table(vpa(Phi_sol),'VariableNames',{'Constraint'},'RowNames',{'rev-rev','rev-tran'});
disp(T)