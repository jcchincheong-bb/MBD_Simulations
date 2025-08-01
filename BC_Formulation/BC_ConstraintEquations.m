%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script sets up the constraint equations of kinematic joints
% for body-coordinate formatultion using syms. It requires globalised data
% to be initialised alongside necessary driver or simple constraints. 
% It puts all constraints in a Phi syms column vector and also finds the
% Jacobian (D)
% Dependencies: rev_pin_phi(),tran_phi(),revrev_phi(),revtran_phi(),
%               rev_pin_gamma(),tran_gamma(),revrev_gamma(),revtran_gamma().

%% Instructions
% Requires Joint array to be properly formatted
% Joints = {'JointType', [point array], [body array], parameter;...
%           ...}
% Revolut Joints - 'rev', only 1 point, 2 bodies, no parameter
% Translation Joints - 'tran', 2 points, 2 bodies*, no parameter
% Revolut-Revolut Joint - 'rev-rev', 2 points, 2 bodies, length
% Tran-Revolut Joint - 'rev-rev', 2 points, 2 bodies*, length 
% *ensure 2nd body is where the main translational axis is defined

%% Dependencies
addpath('G:\My Drive\Projects\MBD_Simulations\MBD_MATLAB\Functions')  % Adds all defined functions to our search path

%% Constraint Equations Formulation
% Syms equations for the constraints (not generalised as yet, only rev-pin)
Phi = [];           % Empty constraint equation array
gamma = [];         % Empty gamma function array
for i = 1:(nj)
    switch (Joints{i})
        case ('rev')                            % Revolut Pin Joint
            points = Joints{i,2}; bodies = Joints{i,3}; 
            for b = 1:length(bodies)
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    s_temp{b} = OG;
                    r_temp{b} = OG;                         % stores the body position array
                    phid_temp{b} = 0;                       % stores the angular velocity variable
                else
                    s_temp{b} = s{points,bodies(b)};        % stores the point displacement array
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
                u_temp{b} = u{bodies(b)};
                if (points == 0)                            % checks if points defined as COGs
                    s_temp{b} = OG;
                else
                    s_temp{b} = s{points(b),bodies(b)};     % stores the point displacement array
                end
                    
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    r_temp{b} = OG;                         % stores the body position array
                    rd_temp{b} = OG;                        % stores the body velocity array
                    phid_temp{b} = 0;                       % stores the angular velocity variable
                else
                    r_temp{b} = r{bodies(b)-1};             % stores the body position array
                    rd_temp{b} = r{bodies(b)-1};            % stores the body velocity array
                    phid_temp{b} = phid(bodies(b)-1);       % stores the angular veloctiy variable
                end
            end
            % Adding the translational constraint equation to the array
            Phi = vertcat(Phi,...
                tran_phi(r_temp{1},r_temp{2},u_temp{1},u_temp{2},s_temp{1},s_temp{2}));
            gamma = vertcat(gamma,...
                tran_gamma(r_temp{1},r_temp{2},u_temp{2},rd_temp{1},rd_temp{2},phid_temp{1}));

        case('rev-rev')
            points = Joints{i,2}; bodies = Joints{i,3}; L = Joints{i,4};
            for b = 1:length(bodies)
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    r_temp{b} = OG;
                    r_temp{b} = OG;                         % stores the body position array
                    rd_temp{b} = OG;                        % stores the body velocity array
                    phid_temp{b} = 0;                       % stores the angular velocity variable
                else
                    s_temp{b} = s{points(b),bodies(b)};     % stores the point displacement array
                    r_temp{b} = r{bodies(b)-1};             % stores the body position array
                    rd_temp{b} = r{bodies(b)-1};            % stores the body velocity array
                    phid_temp{b} = phid(bodies(b)-1);       % stores the angular veloctiy variable
                end
            end

            % Adding the rev-pin constraint equation and gamma function to the array
            Phi = vertcat(Phi,...
                revrev_phi(r_temp{1},r_temp{2},s_temp{1},s_temp{2},L));
            gamma = vertcat(gamma,...
                revrev_gamma(r_temp{1},r_temp{2},rd_temp{1},rd_temp{2},phid_temp{1},phid_temp{2},s_temp{1},s_temp{2}));
        
        case('rev-tran')
            points = Joints{i,2}; bodies = Joints{i,3}; 
            L = Joints{i,4};
            u_temp = u{bodies(2)};
            for b = 1:length(bodies)
                if (bodies(b)==1)                           % checks if body 0 (grounded body)
                    s_temp{b} = OG;
                    r_temp{b} = OG;                         % stores the body position array
                    rd_temp{b} = OG;                        % stores the body velocity array
                    phid_temp{b} = 0;                       % stores the angular velocity variable
                else
                    s_temp{b} = s{points(b),bodies(b)};     % stores the point displacement array
                    r_temp{b} = r{bodies(b)-1};             % stores the body position array
                    rd_temp{b} = r{bodies(b)-1};            % stores the body velocity array
                    phid_temp{b} = phid(bodies(b)-1);       % stores the angular veloctiy variable
                end
            end

            % Adding the rev-pin constraint equation and gamma function to the array
            Phi = vertcat(Phi,...
                revtran_phi(r_temp{1},r_temp{2},s_temp{1},s_temp{2},u_temp,L));
            gamma = vertcat(gamma,...
                revtran_gamma(r_temp{1},r_temp{2},rd_temp{1},rd_temp{2},phid_temp{1},phid_temp{2},s_temp{1},s_temp{2},u_temp));
    end
end

if (exist('Phi_driver','var') == 1)
    Phi = vertcat(Phi,Phi_driver);              % Adding the driver constraint
    D = jacobian(Phi,q);                        % Constraint Jacobian
end