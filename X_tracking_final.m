clc
clear

% Constants
g = 9.8;
Iy = 0.87;

% State-space matrices for the linearized model
a = [0, 1, 0, 0; 
     0, 0, -g, 0; 
     0, 0, 0, 1; 
     0, 0, 0, 0];
b = [0; 0; 0; 1/Iy];

% Output matrix to track only X
c = [1, 0, 0, 0];
d = 0;

% Time vector for simulation
t = 0:0.01:10; % Adjusted to a longer time for better observation

% Define sinusoidal reference trajectory in X (position)
amplitude = 5;
frequency = 0.5; % Adjust frequency for desired oscillations
x_ref = amplitude * sin(2 * pi * frequency * t); % Desired x position trajectory

% Reference trajectory matrix in state form (x, x_dot, theta, theta_dot)
% Only X is set to track; the rest are zeroed out
x_ref_matrix = [x_ref', zeros(size(t')), zeros(size(t')), zeros(size(t'))];

% Tracking control gain design
k_tracking = place(a, b, [-30, -35, -45, -40]);
l_tracking = place(a', c', [-105, -100, -90, -95])';

% Augmented system matrices for tracking controller
at = [a - b * k_tracking, b * k_tracking; zeros(size(a)), a - l_tracking * c];
bt = [b; zeros(size(b))];
ct = [c, zeros(size(c))];
dt = 0;

% Closed-loop system with tracking controller
sys_tracking = ss(at, bt, ct, dt);

% Gain matrix for tracking input reference
n = -inv(c * inv(a - b * k_tracking) * b);

% Define input reference trajectory for system
u_ref = (n * x_ref_matrix')';
u_ref = u_ref(:, 1); % Ensuring u_ref is single-column for the single-input system

% Initial conditions for the augmented system
x0 = zeros(8, 1); % Eight states: four for the system and four for the observer

% Simulate the tracking response
[y_tracking, t_tracking, x_tracking] = lsim(sys_tracking, u_ref, t, x0);

% Plot tracking performance in X
figure(1)
plot(t_tracking, y_tracking(:, 1), '-r', 'DisplayName', 'Tracked X Position');
hold on;
plot(t, x_ref, '--k', 'DisplayName', 'Reference X Position');
legend;
title('Tracking Performance for X Position');
xlabel('Time (s)');
ylabel('X Position');
hold off;
