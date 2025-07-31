%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is to test the tran_phi function against manual
%               computation
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 13.06.25
% Version: 1.0
% Revision History: 
% Dependencies: A_matrix.m, rev_pin_phi.m, s_rot.m, tran_phi.m

%% Reset and Dependencies
clear
addpath('G:\My Drive\HSRW\Semester 4\Multi body Dynamics\MBD_MATLAB\Functions')  % Adds all defined functions to our search path

%% Defined Parameters
% number of bodies
nb = 3;          
nbc = nb*3;
nj = 4;
OG = [0;0];                                 % Null array

% Displacement and Unit Vectors
sA1_l = [0.225;0]; sA2_l = [0;-0.07];       % Joint A Displacements [m]
sB2_l = [-0.17;0.25];                       % Joint B Displacements [m]
sC2_l = [0.11;-0.02];                       % Point C Displacements [m]
rO0 = [0.41;0.13]; sO3_l = [-0.15;0];       % Joint O Displacements [m]
rQ0 = [0.12;-0.41]; sQ1_l = [-0.225;0];     % Joint Q Displacements [m]
s_l = {0, sA1_l, sA2_l, 0; ...              % Joint A
        0, 0, sB2_l, 0; ...                 % Joint B
        rO0, 0, 0, sO3_l; ...               % Joint O
        rQ0, sQ1_l, 0, 0};                  % Joint Q
u2_l = [0.17;-0.32];                        % Unit Vector of body 2
u3_l = [1;0];                               % Unit Vector of body 3
u_l = {OG,OG,u2_l,u3_l};                    

% Initial Guess
q_0 = [0.345;-0.41;0;...
    0.5826;-0.3405;6.08;...
    0.4525;-0.0138;5];

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
u = cell(1,nb+1);                             % Global unit vec array
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

        if u_l{j} == OG
            u{j} = [];
        else
            u{j} = A{j-1}*u_l{j};
        end
    end
end

%% Constraint Formulation and Evaluation
% Using tran_phi()
PhiB = tran_phi(r{2},r{3},u{3},u{4},s{2,3},s{3,4});  % Translational Joint
PhiB_func = vpa(subs(PhiB,q,q_0));

% Manually
% Get A matrices 
A_2 = A_matrix(q_0(6)); A_3 = A_matrix(q_0(9));
% Globalise s vectors
sB2 = A_2*sB2_l; sO3 = A_3*sO3_l;
% Find position vectors
r2 = [q_0(4);q_0(5)]; r3 = [q_0(7);q_0(8)];
rB2 = r2 + sB2; rO3 = r3  + sO3;
% Unit Vectors
u2 = A_2*u2_l; u3 = A_3*u3_l;
ub2 = s_rot(u2); ub3 = s_rot(u3);
% Constraint evaluation
PhiB_man_23 = [ub2'*u3;...
    ub2'*(rB2-rO3)];
PhiB_man_32 = [ub3'*u2;...
    ub3'*(rO3-rB2)];
PhiB_man_func = tran_phi(r2,r3,u2,u3,sB2,sO3);
% Display
%T = table(PhiB_func,PhiB_man_23,PhiB_man_32,PhiB_man_func,'VariableNames',{'Using function','Manual 2 first','Manual 3 first','Numeric Function'});
T = table(PhiB_man_23,PhiB_man_32,PhiB_man_func,'VariableNames',{'Manual 2 first','Manual 3 first','Numeric Function'});
disp(T)
