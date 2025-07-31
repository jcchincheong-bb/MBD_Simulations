%% Meta
% Author: Justin Julius Chin Cheong 34140
% Description: This script is just plotting the values of the DoubleArm and
%               animating it
% Sources: [ChatGPT]
% Last Edit: 05.06.25
% Version: 1.0
% Dependencies: DoubleArm.m (current version)
%% Animation
% Requires that the Analysis Script is run first
%DoubleArmV2_2.m;

% Double Arm Data
x_1 = r_val{1}(1,:); y_1 = r_val{1}(2,:);
x_2 = r_val{2}(1,:); y_2 = r_val{2}(2,:);
x_3 = r_val{3}(1,:); y_3 = r_val{3}(2,:);

% Initialize plot
figure;
axis equal;
axis([min([x_1 x_2 x_3])-1, max([x_1 x_2 x_3])+1, ...
      min([y_1 y_2 y_3])-1, max([y_1 y_2 y_3])+1]);
grid on; hold on;

% Ground Point
plot(0,0,'ko','MarkerFaceColor', 'k');
text(0.1,0.1,'0','FontSize', 12, 'Color', 'k');

% Create points
h_1 = plot(x_1(1), y_1(1), 'ro', 'MarkerFaceColor', 'r');
h_2 = plot(x_2(1), y_2(1), 'go', 'MarkerFaceColor', 'g');
h_3 = plot(x_3(1), y_3(1), 'bo', 'MarkerFaceColor', 'b');

% Create labels
label_1 = text(x_1(1)+0.1, y_1(1)+0.1, '1', 'FontSize', 12, 'Color', 'r');
label_2 = text(x_2(1)+0.1, y_2(1)+0.1, '2', 'FontSize', 12, 'Color', 'g');
label_3 = text(x_3(1)+0.1, y_3(1)+0.1, '3', 'FontSize', 12, 'Color', 'b');

% Animate
for i = 1:n_steps
    % Update point positions
    set(h_1, 'XData', x_1(i), 'YData', y_1(i));
    set(h_2, 'XData', x_2(i), 'YData', y_2(i));
    set(h_3, 'XData', x_3(i), 'YData', y_3(i));

    % Update label positions (slightly offset from point)
    set(label_1, 'Position', [x_1(i)+0.1, y_1(i)+0.1]);
    set(label_2, 'Position', [x_2(i)+0.1, y_2(i)+0.1]);
    set(label_3, 'Position', [x_3(i)+0.1, y_3(i)+0.1]);


    drawnow;
    pause(t_step);
end
