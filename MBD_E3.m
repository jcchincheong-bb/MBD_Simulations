%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is the solution to Exercise 3
% Brandt, 2908 Multibody Dynamics
% Last Edit: 16.05.25

%% Defined Parameters
r = [2.5;1.2]; phi = 5.6723;  % Global body position [m,rad]
sA_l = [2.18;0]; sB_l = [-1.8;1.3]; % Local point displacement [m]
rd = [1;-2]; phid = 1; % Global body velocities [m/s,rad/s]
rdd = [1;-2]; phidd = 4.65; % Global body accelerations [m/s^2,rad/s^2]

%% Initial Conditions
% Global point displacement
sA = A_matrix(phi)*sA_l; sB = A_matrix(phi)*sB_l;
sAB = sA - sB;  % relative displacement [m]

% Global point position
rA = r_Point(r,sA);
rB = r_Point(r,sB);

% Global point velocity (manual)
sAd = s_rot(sA)*phid; sBd = s_rot(sB)*phid;
rAd = rd + sAd;
rBd = rd + sBd;

% Global point velocity (using function r_Point_d()) 
rAd2 = r_Point_d(rd,sA,phid);
rBd2 = r_Point_d(rd,sB,phid);

% Global point accelerations (using function r_Point_dd())
rAdd = r_Point_dd(rdd,sA,phid,phidd);
rBdd = r_Point_dd(rdd,sB,phid,phidd);

% Display 
fprintf("Initial Conditions\n")
T = table(rA,rB,rAd2,rBd2,rAdd,rBdd,'VariableNames',{'rA [m]','rB [m]','rAd [m/s]','rBd [m/s]','rAdd [m/s^2]','rBdd [m/s^2]'});
disp(T)

% %% Kinematics (Unconstrained) 
% % This obviously don't make sense since the points are part of a rigid
% % body, so this is more like two independent particles 
% % Equation of Motion (assuming const acc) 
% r_A = @(t)((1/2)*rAdd*t^2+rAd*t+rA);
% r_B = @(t)((1/2)*rBdd*t^2+rBd*t+rB);
% 
% % Time setup
% n = 2;     % Duration (seconds)
% h = 0.01;    % Step size
% time = 0:h:n;
% 
% % Initialize figure
% figure;
% axis equal;
% axis([-10 10 -10 10]); % Adjust as needed
% grid on;
% hold on;
% 
% % Plot handles for animation
% hA = plot(0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
% hB = plot(0, 0, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
% trajA = animatedline('Color', 'r', 'LineStyle', '--');
% trajB = animatedline('Color', 'b', 'LineStyle', '--');
% hLine = plot([0 0], [0 0], 'k-', 'LineWidth', 2); % Line between A and B
% legend('A','B')
% % Animation loop
% for i = 1:length(time)
%     rA_val = r_A(time(i));
%     rB_val = r_B(time(i));
% 
%     % Update point positions
%     set(hA, 'XData', rA_val(1), 'YData', rA_val(2));
%     set(hB, 'XData', rB_val(1), 'YData', rB_val(2));
% 
%     % Update connecting line
%     set(hLine, 'XData', [rA_val(1) rB_val(1)], 'YData', [rA_val(2) rB_val(2)]);
% 
%     % Add to trajectory which updates the line
%     addpoints(trajA, rA_val(1), rA_val(2));
%     addpoints(trajB, rB_val(1), rB_val(2));
% 
%     % Update the graph
%     drawnow;
%     pause(h);  % Controls speed
% end

%% Kinematics (Constrained) 
% Initial conditions of B 
rB_val = rB;
rBd_val = rBd;
rBdd_val = rBdd;

% Equations of Motion
r_A = @(t)((1/2)*rAdd*t^2+rAd*t+rA);   % Position of A (static eqn)
rd_A = @(t)(rAdd*t+rAd);               % Velocity of A (static eqn)
r_B = @(t)((1/2)*rBdd_val*t^2+rBd_val*t+rB_val);   % Position of B (dynamic eqn)
rd_B = @(t)(rBdd_val*t+rBd_val);               % Velocity of B (dynamic eqn)
rdd_B = @(lambda,delta_r)(rAdd + lambda * delta_r);    % Constrained acceleration of B (dynamic eqn)

% Time setup
n = 2;     % Duration (seconds)
h = 0.01;    % Step size
time = 0:h:n;

% Initialize figure
figure;
axis equal;
axis([-10 10 -10 10]); % Adjust as needed
grid on;
hold on;

% Plot handles for animation
hA = plot(0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
hB = plot(0, 0, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
trajA = animatedline('Color', 'r', 'LineStyle', '--');
trajB = animatedline('Color', 'b', 'LineStyle', '--');
hLine = plot([0 0], [0 0], 'k-', 'LineWidth', 2); % Line between A and B
legend('A','B')
% Animation loop
for i = 1:length(time)
    rA_val = r_A(time(i));
    rAd_val = rd_A(time(i));
    rBd_val = rd_B(time(i));
    rB_val = r_B(time(i));
    
    % Compute constraint-based acceleration for B
    delta_r = rB_val - rA_val;
    delta_v = rBd_val - rAd_val;

    % Compute lambda
    lambda = (-dot(delta_v, delta_v) - dot(delta_r, -rAdd)) / dot(delta_r, delta_r);
    
    % Update the acceleration of B
    rBdd_val = rdd_B(delta_r,lambda);

    % Update point positions
    set(hA, 'XData', rA_val(1), 'YData', rA_val(2));
    set(hB, 'XData', rB_val(1), 'YData', rB_val(2));
    
    % Update connecting line
    set(hLine, 'XData', [rA_val(1) rB_val(1)], 'YData', [rA_val(2) rB_val(2)]);
    
    % Add to trajectory which updates the line
    addpoints(trajA, rA_val(1), rA_val(2));
    addpoints(trajB, rB_val(1), rB_val(2));
    
    % Update the graph
    drawnow;
    pause(h);  % Controls speed
end