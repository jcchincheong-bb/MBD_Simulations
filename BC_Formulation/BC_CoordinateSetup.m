%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script sets up the coordinates for body-coordinate
% formatultion using syms. It defines all relevant coordinates, their
% arrays and globalises body measurements
% Dependencies: A_matrix.m

%% Dependencies
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Functions')  % Adds all defined functions to our search path

%% Coordinate Setup
% Initialising the symbols
syms t
syms x [nb 1]
syms y [nb 1]
syms phi [nb 1]
syms xd [nb 1]
syms yd [nb 1]
syms phid [nb 1]
syms xdd [nb 1]
syms ydd [nb 1]
syms phidd [nb 1]

% Initialising our coordinate arrays
syms q [nbc 1]
syms qd [nbc 1]
syms qdd [nbc 1]
r = cell(1,nb);                                         % Positions arrays
A = cell(1,nb);                                         % A matrices
rd = cell(1,nb);                                        % Velocity Arrays
rdd = cell(1,nb);                                       % Acceleration Arrays

% Setting up the position coordinate arrays
for i = 1:nb
    q(3*i-2)=x(i);q(3*i-1)=y(i);q(3*i)=phi(i);          % coordinate array
    qd(3*i-2)=xd(i);qd(3*i-1)=yd(i);qd(3*i)=phid(i);    % coordinate velocity array
    r{i} = [x(i);y(i)];                                 % position arrays
    rd{i} = [xd(i);yd(i)];                              % velocity arrays
    rdd{i} = [xdd(i);ydd(i)];                           % acceleration arrays
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
        
        % We have to check if the unit vector array exists 
        % cause if there's no tran joints that we wouldn't define them
        if exist('u_l','var') == 1   
            if u_l{j} == OG
                u{j} = [];
            else
                u{j} = A{j-1}*u_l{j};
            end
        end 
    end
end
