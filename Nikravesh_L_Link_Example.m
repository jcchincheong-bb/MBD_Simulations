%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to examples 3.1, 4.3, etc from
% Nikravesh, Planar Multibody Dynamics
% Last Edit: 18.04.25


%% Defined Parameters
% Body/System Parameters
m = 2.5;            % Body Mass [kg]
J = 1.2;            % Mass MoI [kgm^2]
M = zeros(3);       % Initializing the Inertial Matrix/Tensor [kg,kgm^2]
M(1,1) = m; M(2,2) = m; M(3,3) = J; % Inertial Array [kg,kgm^2]
r = [2.5;1.2];      % Global Centroid Position [m]
phi = deg2rad(325); % Orientation of Xi Axis [rad]
A = [cos(phi),-sin(phi);sin(phi),cos(phi)]; % Rotation Matrix [-]
f_A = [2;-1]; f_B = [-3;2];      % Force at A & B [N] (Global)


% Local Displacement Vectors [m]
s_A_l = [2.18;0]; s_B_l = [-1.8;1.3]; 


%% Calculated Parameters
% Global Displacement Vectors [m]
s_A = A*s_A_l; s_B = A*s_B_l;

% Global Position Vectors [m]
r_A = r + s_A; r_B = r + s_B;

% Global Relative Displacement Vectors [m]
s_B_A = s_B - s_A;

% Force Array [N,Nm]
f = f_A + f_B; 
n = (s_rot(s_A))'*f_A + (s_rot(s_B))'*f_B;
h = [f;n];

% Accelerations [m/s^2, rad/s^2]
c_dd = inv(M)*h;
r_dd = [c_dd(1);c_dd(2)]; phi_dd=c_dd(3);

