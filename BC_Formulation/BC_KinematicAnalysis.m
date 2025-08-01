%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script finds calculates and stores the kinematic
% properties of a system over a period of time. It requires that the system
% is formulated using syms via BC_ConstraintEquations and
% BC_CoordinateSetup. 
% Dependencies: NRfunc()

%% Dependencies
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Functions')  % Adds all defined functions to our search path

%% Solving Constraint Equations 
% Setting up time variables
tspan = 0:t_step:end_time;
n_steps = length(tspan);

% Arrays to store all values  (position, velocity and acceleration)
pos_record = zeros(n_steps,nbc);
vel_record = zeros(n_steps,nbc);
acc_record = zeros(n_steps,nbc);

% Kinematic Loop
for n = 1:n_steps
    % Evaluating Position Coordinates
    Phi_sol = subs(Phi,t,tspan(n));                     % Current time
    q_sol = NRfunc(Phi_sol,q,q_0,0.001,10);             % Solving our position coordinates
    Phi_sol = subs(Phi_sol,q,q_sol);                    % Evaluating Phi at q_sol
    
    % Evaluating Velocity Constraints
    if (exist('rhsv','var')==1)
        rhsv(nbc) = subs(rhsv_driver,t,tspan(n));           % Current time
        D_sol = subs(D,q,q_sol);                            % Evaluating jacobian at q_sol
        qd_sol = inv(D_sol)*rhsv;                           % Solving our velocity coordinates
        Phid_sol = D_sol*qd_sol;                            % Evaluating velocity constraints
    end
    
    % Evaluating Acceleration Constraints
    if (exist('rhsa','var')==1)
        gamma_sol = subs(gamma,q,q_sol);                % Sub in position values
        gamma_sol = subs(gamma_sol,qd,qd_sol);                  % Sub in velocity values               
        rhsa_driver_sol = subs(rhsa_driver,t,tspan(n));     % Current time
        rhsa = [gamma_sol;rhsa_driver_sol];                 % Setting up RHS of acceleration constraint
        qdd_sol = inv(D_sol)*rhsa;                          % Solving our acceleration coordinates
        Phidd_sol = D_sol*qdd_sol - rhsa;                   % Evaluating acceleration constraints
    end
    % Storing all the Values
    pos_record(n,:) = q_sol';
    vel_record(n,:) = qd_sol';
    acc_record(n,:) = qdd_sol';
end 
