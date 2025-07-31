%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is just tests out the basic pp actuators
% Sources: [Brandt, 2908 Multibody Dynamics] , [Nikravesh, Planar Multibody Dynamics]
% Last Edit: 20.06.25
% Version: 1.0
% Revision History: 
    % 1.0 -
% Dependencies:fe_s.m, A_matrix.m, s_rot.m
%% Define Parameters
sA1_l = [0.15;0]; sB2_l = [0;0.1];
r1 = [-0.1;0.05]; r2 = [0.1;-0.05];
phi1 = 0.785; phi2 = 0.262;
rd1 = [0.1;0.2]; rd2 = [-0.2;0.1];
phid1 = -0.25; phid2 = 0.12;

% Force stuff
l0 = 0.15; ks = 10;
kd = 5;
f_a = -2;
%% Set up Point Vectors
% globalising
sA1 = A_matrix(phi1)*sA1_l; 
sB2 = A_matrix(phi2)*sB2_l; 
% Position vectors
rA1 = r1 + sA1;
rB2 = r2 + sB2;
d = rB2-rA1;
% Velocity Vectors
rdA1 = rd1 + s_rot(sA1)*phid1; 
rdB2 = rd2 + s_rot(sB2)*phid2; 
dd = rdB2 - rdA1;
%% Evaluate 
fA = fe_s(ks,l0,d) + fe_d(kd,d,dd) + fe_a(f_a,d);
fB = -fA;

%% display
rA1
rB2
rdA1
rdB2
d
dd
fA
fB