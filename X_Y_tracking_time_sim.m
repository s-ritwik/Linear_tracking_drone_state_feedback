clc;
clear;
%% GROUP2 SUBMISSION EE650A
%% TOPIC Linear tracking of drone on a stochastically moving platform	
%% MODEL DEFINATIONS
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
%% CONTRABILITY AND OBSERVABILITY matrices
% Compute the controllability matrix
controllability_matrix = ctrb(a, b);

% Check the rank of the controllability matrix
rank_controllability = rank(controllability_matrix);

% Determine if the system is controllable
if rank_controllability == size(a, 1)
    disp('The system is controllable.');
else
    disp('The system is NOT controllable.');
end

% Compute the observability matrix
observability_matrix = obsv(a, c);

% Check the rank of the observability matrix
rank_observability = rank(observability_matrix);

% Determine if the system is observable
if rank_observability == size(a, 1)
    disp('The system is observable.');
else
    disp('The system is NOT observable.');
end
%% Trajectory setup(NOT RELATED TO CODE:__________SKIP____________)
% Time vector for simulation
t = 0:0.01:10; % Increased time to accommodate "THANK YOU" trajectory

% Prompt user for trajectory option
trajectory_option = input(['Choose trajectory option:\n' ...
    '1 - Circular\n' ...
    '2 - Sinusoidal Circle\n' ...
    '3 - Straight Lines with Random sharp turns\n' ...
    'Enter option (1, 2, or 3): ']);

% Define reference trajectory based on selected option
radius = 5;
frequency = 0.1;
if trajectory_option == 1
    % Circular trajectory
    x_ref = radius * cos(2 * pi * frequency * t);
    y_ref = radius * sin(2 * pi * frequency * t);
elseif trajectory_option == 2
    % Sinusoidal circular trajectory
    x_ref = (radius + 2 * sin(10 * t)) .* cos(2 * pi * frequency * t);
    y_ref = (radius + 2 * sin(10 * t)) .* sin(2 * pi * frequency * t);
elseif trajectory_option== 3
       % Initialize x_ref and y_ref arrays
    x_ref = zeros(1, size(t, 2));
    y_ref = zeros(1, size(t, 2));
    
    % Initial position and direction
    x_ref(1) = 0;
    y_ref(1) = 0;
    step_size = 0.05;  % Step size for linear movement
    turn_interval = 100;  % Number of steps to wait before a random turn (1 second if dt = 0.01)
    
    % Initial random direction
    angle = rand * 2 * pi;
    dx = step_size * cos(angle);
    dy = step_size * sin(angle);
    
    % Generate the path
    for i = 2:size(t, 2)
        if mod(i, turn_interval) == 0
            % Every 'turn_interval' steps, change direction randomly
            angle = rand * 2 * pi;  % New random angle in radians
            dx = step_size * cos(angle);
            dy = step_size * sin(angle);
        end
        
        % Update position linearly in the current direction
        x_ref(i) = x_ref(i - 1) + dx;
        y_ref(i) = y_ref(i - 1) + dy;
        
        % Keep the position within the 4x4 window
        x_ref(i) = min(max(x_ref(i), -2), 2);
        y_ref(i) = min(max(y_ref(i), -2), 2);
    end
else
    error('Invalid trajectory option selected.');
end

% Reference trajectory matrix in state form (x, x_dot, theta, theta_dot, y, y_dot, phi, phi_dot)
ref_matrix = [x_ref', zeros(size(t')), zeros(size(t')), zeros(size(t')), ...
              y_ref', zeros(size(t')), zeros(size(t')), zeros(size(t'))];
%% PLACING POLES FOR BOTH CONTROLLER AND OBSERVER
% Tracking control gain design
% By trial and error we got good values here,increasing furthur will lead
% to physical instability of the drone
k_tracking = place(a, b, [-30, -35, -45, -40, -30.5, -35.5, -45.5, -40.5]);
% k_tracking = place(a, b, [-3, -3.5, -4.5, -4.0, -3.05, -3.55, -4.55, -4.05]);
% HIGH negative poles for faster convergence to actual values
l_tracking = place(a', c', [-105, -100, -90, -95, -105.5, -100.5, -90.5, -95.5])';

%%  PLOTTING COMPARISION OF POLES
% plotting poles
% Compute poles of the open-loop system
open_loop_poles = eig(a);

% Compute poles of the closed-loop system with state feedback
closed_loop_poles = eig(a - b * k_tracking);

% Display the poles in the Command Window
disp('------------------------------');
disp('Open-loop System Poles:');
disp(open_loop_poles);
disp('------------------------------');
disp('Closed-loop System Poles:');
disp(closed_loop_poles);
disp('------------------------------');

% Plot the poles on the complex plane
figure;
hold on;
grid on;

% Plot Open-loop Poles
plot(real(open_loop_poles), imag(open_loop_poles), 'x', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Open-loop Poles');

% Plot Closed-loop Poles
plot(real(closed_loop_poles), imag(closed_loop_poles), 'o', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Closed-loop Poles');

% Enhancing the Plot
xlabel('Real Part');
ylabel('Imaginary Part');
title('Comparison of Open-loop and Closed-loop System Poles');
legend('Location', 'best');
axis equal; % Equal scaling for both axes
xlim([min(real([open_loop_poles; closed_loop_poles])) - 1, max(real([open_loop_poles; closed_loop_poles])) + 1]);
ylim([min(imag([open_loop_poles; closed_loop_poles])) - 1, max(imag([open_loop_poles; closed_loop_poles])) + 1]);
hold off;

%% Animation part


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
x0(1) = 5; x0(5) = 1;

% Simulate the tracking response
[y_tracking, t_tracking, x_tracking] = lsim(sys_tracking, u_ref, t, x0);

% Setup for animation
figure;
% Set figure size to avoid padding issues
fig = figure('Position', [100, 100, 800, 800]); % Even dimensions
hold on;
legend;
title('Simulation of Drone Tracking Trajectory');
xlabel('X Position');
ylabel('Y Position');
axis equal;

% Adjust axis limits based on trajectory
all_x = x_ref;
all_y = y_ref;
radius_extended = max([abs(all_x), abs(all_y)]) + 3;
axis([-radius_extended radius_extended -radius_extended radius_extended]);

% Initialize paths for reference and tracked path
past_ref_path = plot(NaN, NaN, '-r', 'DisplayName', 'Past Reference Path'); % Past reference path in red
tracked_path = plot(NaN, NaN, '--k', 'DisplayName', 'Path Covered'); % Path covered in black dashed
drone_marker = plot(y_tracking(1, 1), y_tracking(1, 2), 'bo', 'MarkerSize', 6, 'DisplayName', 'Drone Position');
ref_window_marker = plot(NaN, NaN, '--g', 'DisplayName', 'Reference Path (Next 10 Steps)'); % Next 10 steps of reference path

% Initialize video writer
video_filename = 'drone_trajectory_simulation_tnx.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 30;
open(v);

% Timer text
timer_text = text(-radius_extended, radius_extended - 1, 'Time: 0 s', 'FontSize', 10, 'Color', 'blue');

% Animation loop
num_steps = length(t_tracking);
window_size = 10; % Show the next 10 time steps

for k = 1:num_steps
    % Update drone position marker
    set(drone_marker, 'XData', y_tracking(k, 1), 'YData', y_tracking(k, 2));
    
    % Update path covered by drone
    set(tracked_path, 'XData', y_tracking(1:k, 1), 'YData', y_tracking(1:k, 2));
    
    % Update past reference path
    set(past_ref_path, 'XData', x_ref(1:k), 'YData', y_ref(1:k));
    
    % Update reference path for the next 10 time steps
    if k + window_size <= num_steps
        ref_x_window = x_ref(k:k+window_size);
        ref_y_window = y_ref(k:k+window_size);
    else
        ref_x_window = x_ref(k:end);
        ref_y_window = y_ref(k:end);
    end
    set(ref_window_marker, 'XData', ref_x_window, 'YData', ref_y_window);
    
    % Update timer
    set(timer_text, 'String', sprintf('Time: %.2f s', t_tracking(k)));
    
    drawnow;
    % Capture and write each frame to the video file
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% Closing the video file after the loop
close(v);
disp(['Video saved as ', video_filename]);
hold off;

% Final plot after the animation: Compare x_ref, x_observed, y_ref, y_observed
figure;
hold on;
plot(t, x_ref, '-b', 'DisplayName', 'X Reference');
plot(t, y_ref, '-r', 'DisplayName', 'Y Reference');
plot(t_tracking, y_tracking(:, 1), '--b', 'DisplayName', 'X Observed');
plot(t_tracking, y_tracking(:, 2), '--r', 'DisplayName', 'Y Observed');
legend;
title('Comparison of Reference and Observed Positions',LineWidth=3);
xlabel('Time (s)',LineWidth=3);
ylabel('Position',LineWidth=3);
hold off;
