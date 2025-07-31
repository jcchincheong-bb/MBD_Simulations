%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is just plotting the values of the DoubleArm
% Sources: [ChatGPT]
% Last Edit: 29.07.25
% Version: 1.0
% Dependencies: DoubleArm.m (current version)

%% Time Domain Graphs
% Requires that the Analysis Script is run first
% DoubleArmV4.m;

% Double Arm Data
pos_data = cell(1,nb); vel_data = cell(1,nb); acc_data = cell(1,nb);
for b = 1:nb
    for n = 1:n_steps
        pos_data{b}(n) = norm(r_val{b}(:,n));
        vel_data{b}(n) = norm(rd_val{b}(:,n));
        acc_data{b}(n) = norm(rdd_val{b}(:,n));
    end
end

%% Plots
figure
for b = 1:nb
    subplot(3,3,b)
    plot(tspan,pos_data{b},"k-")
    ylim([0.4 0.7])
end
for b = 1:nb
    subplot(3,3,b+3)
    plot(tspan,vel_data{b},"b-")
    ylim([0.04 0.12])
end
for b = 1:nb
    subplot(3,3,b+6)
    plot(tspan,acc_data{b},"r-")
    ylim([0.01 0.05])
end

