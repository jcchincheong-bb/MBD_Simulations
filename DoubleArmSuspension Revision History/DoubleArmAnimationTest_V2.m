%% Meta
% This is complete dummy data from ChatGPT to test how to create these 2D
% animations
%% Animatin

% Define pivots
pivot_1 = [0, 0];      % Q
pivot_2 = [6, 0];      % A
pivot_3 = [3, 4];      % O
 
% Define shapes (relative to pivot)
body1_shape = [0, 0; 1.5, 2];            % Body (1): Q-bar
body2_shape = [0, 0; 1.2, 2; 2, 0];      % Body (2): triangle ABC
body3_shape = [0, 0; 3, 0];              % Body (3): OB bar

% Time and angles
t = linspace(0, 2*pi, 300);
theta1 = 0.3 * sin(t);             % rotation of body (1)
theta2 = 0.5 * sin(t + pi/4);      % rotation of body (2)
theta3 = 0.2 * sin(t - pi/6);      % rotation of body (3)

% Set up figure
figure;
axis equal;
axis([-2, 10, -2, 7]);
grid on;
hold on;

for i = 1:length(t)
    % Rotation matrices
    R1 = [cos(theta1(i)), -sin(theta1(i)); sin(theta1(i)), cos(theta1(i))];
    R2 = [cos(theta2(i)), -sin(theta2(i)); sin(theta2(i)), cos(theta2(i))];
    R3 = [cos(theta3(i)), -sin(theta3(i)); sin(theta3(i)), cos(theta3(i))];

    % Rotate and translate bodies
    b1 = (R1 * body1_shape')' + pivot_1;
    b2 = (R2 * body2_shape')' + pivot_2;
    b3 = (R3 * body3_shape')' + pivot_3;

    % Clear previous frame
    cla;

    % Draw bodies
    plot(b1(:,1), b1(:,2), 'r-o', 'LineWidth', 2);
    fill(b2(:,1), b2(:,2), 'g', 'FaceAlpha', 0.4);
    plot(b3(:,1), b3(:,2), 'b-o', 'LineWidth', 2);

    % Optional: Draw a spring (from body1 end to body3 end)
    spring_start = b1(end, :);
    spring_end = b3(end, :);
    plot([spring_start(1), spring_end(1)], ...
         [spring_start(2), spring_end(2)], 'k--', 'LineWidth', 1.5);

    % Label points
    text(pivot_1(1), pivot_1(2)-0.2, '1 (Q)', 'HorizontalAlignment', 'center');
    text(pivot_2(1), pivot_2(2)-0.2, '2 (A)', 'HorizontalAlignment', 'center');
    text(pivot_3(1), pivot_3(2)+0.3, '3 (O)', 'HorizontalAlignment', 'center');

    drawnow;
    pause(0.01);
end
