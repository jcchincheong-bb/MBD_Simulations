% Time setup
dt = 0.01;
T = 1;
time = 0:dt:T;

% Initial conditions
rA0 = [4.2857; -0.050432];
vA0 = [2.2504; -0.21427];
aA = [5.0288; 7.5541];

rB0 = [1.7712; 3.2973];
vB0 = [-1.0973; -2.7288];

% Initial distance
L = norm(rB0 - rA0);

% Function handle: position of A as a function of time
rA_fn = @(t) rA0 + vA0 * t + 0.5 * aA * t^2;
vA_fn = @(t) vA0 + aA * t;

% Function handle: constraint-compliant acceleration of B
aB_fn = @(rA, vA, rB, vB) aA + ...
    ((-dot(vB - vA, vB - vA) - dot(rB - rA, -aA)) / dot(rB - rA, rB - rA)) * (rB - rA);

% Initialize arrays
rA_hist = zeros(2, length(time));
rB_hist = zeros(2, length(time));

% Initial states
rA = rA0;
vA = vA0;
rB = rB0;
vB = vB0;

% Plot setup
figure;
axis equal; grid on;
axis([-12 12 -12 12]);
hold on;

hA = plot(rA(1), rA(2), 'ro', 'MarkerFaceColor', 'r');
hB = plot(rB(1), rB(2), 'bo', 'MarkerFaceColor', 'b');
hLine = plot([rA(1), rB(1)], [rA(2), rB(2)], 'k-', 'LineWidth', 2);

% Simulation loop
for i = 1:length(time)
    t = time(i);

    % Update A's motion from function handle
    rA = rA_fn(t);
    vA = vA_fn(t);

    % Compute constraint-compliant acceleration of B
    aB = aB_fn(rA, vA, rB, vB);

    % Integrate B's motion
    rB = rB + vB * dt + 0.5 * aB * dt^2;
    vB = vB + aB * dt;

    % Store history
    rA_hist(:, i) = rA;
    rB_hist(:, i) = rB;

    % Update animation
    set(hA, 'XData', rA(1), 'YData', rA(2));
    set(hB, 'XData', rB(1), 'YData', rB(2));
    set(hLine, 'XData', [rA(1), rB(1)], 'YData', [rA(2), rB(2)]);
    drawnow;
    pause(dt);
end
