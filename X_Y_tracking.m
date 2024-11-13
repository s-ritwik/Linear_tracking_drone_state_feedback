clc;
clear;

% Constants
g = 9.8;
Iy = 0.87; % Moment of inertia around y-axis
Ix = 0.87; % Moment of inertia around x-axis (assuming symmetry for simplicity)

% State-space matrices for the extended 8-state model
a = [0, 1, 0, 0, 0, 0, 0, 0; 
     0, 0, -g, 0, 0, 0, 0, 0; 
     0, 0, 0, 1, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 0, 0, 0;
     0, 0, 0, 0, 0, 1, 0, 0; 
     0, 0, 0, 0, 0, 0, -g, 0; 
     0, 0, 0, 0, 0, 0, 0, 1; 
     0, 0, 0, 0, 0, 0, 0, 0];

b = [0, 0;
     0, 0;
     0, 0;
     1/Iy, 0;
     0, 0;
     0, 0;
     0, 0;
     0, 1/Ix];

% Output matrix to track X and Y positions
c = [1, 0, 0, 0, 0, 0, 0, 0; 
     0, 0, 0, 0, 1, 0, 0, 0];
d = zeros(2, 2);

% Time vector for simulation
t = 0:0.01:10;

% Circular reference trajectory in the X-Y plane
radius = 5;
frequency = 0.1; % Adjust frequency for desired oscillations
x_ref = radius * cos(2 * pi * frequency * t); % Desired x position trajectory
y_ref = radius * sin(2 * pi * frequency * t); % Desired y position trajectory

% Reference trajectory matrix in state form (x, x_dot, theta, theta_dot, y, y_dot, phi, phi_dot)
% Only X and Y are set to track; the rest are zeroed out
ref_matrix = [x_ref', zeros(size(t')), zeros(size(t')), zeros(size(t')), ...
              y_ref', zeros(size(t')), zeros(size(t')), zeros(size(t'))];

% Tracking control gain design
k_tracking = place(a, b, [-30, -35, -45, -40, -30.5, -35.5, -45.5, -40.5]);
l_tracking = place(a', c', [-105, -100, -90, -95, -105.5, -100.5, -90.5, -95.5])';

% Augmented system matrices for tracking controller
at = [a - b * k_tracking, b * k_tracking; zeros(size(a)), a - l_tracking * c];
bt = [b; zeros(size(b))];
ct = [c, zeros(size(c))];
dt = zeros(2, 2);

% Closed-loop system with tracking controller
sys_tracking = ss(at, bt, ct, dt);

% Gain matrix for tracking input reference (computed separately for x and y)
nx = -inv(c(1,:) * inv(a - b * k_tracking) * b(:,1));
ny = -inv(c(2,:) * inv(a - b * k_tracking) * b(:,2));

% Define input reference trajectory for system
u_ref_x = nx * x_ref';
u_ref_y = ny * y_ref';

% Combine x and y references into a single input for the simulation
u_ref = [u_ref_x, u_ref_y];

% Initial conditions for the augmented system
x0 = zeros(16, 1); % Sixteen states: eight for the system and eight for the observer

% Simulate the tracking response
[y_tracking, t_tracking, x_tracking] = lsim(sys_tracking, u_ref, t, x0);

% Plot tracking performance in X-Y plane
figure;
plot(y_tracking(:, 1), y_tracking(:, 2), '-r', 'DisplayName', 'Tracked Position');
hold on;
plot(x_ref, y_ref, '--k', 'DisplayName', 'Reference Circle');
legend;
title('Tracking Performance for Circular Trajectory in X-Y Plane');
xlabel('X Position');
ylabel('Y Position');
axis equal;
hold off;
