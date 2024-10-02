%% STEP 5: PLOT RESULTS
% Plot the results from NeuralPARC to see if they reach the goal without
% hitting the obstacles.
%
% This script is written for the drift-parking example.
%
% REQUIRED VARIABLES:
% --- from init_*.m ---
% See init_*.m
% --- from *_traindata.mat ---
% See step_01_format_for_training.m
% --- from *_network.mat ---
% See step_02_train_traj_network.m
% --- from *_results.mat ---
% See step_04_main.m
% --- defined in this script ---
% xl: 1*n_p double; lower and upper limit of x-axis for plotting
% yl: 1*n_p double; lower and upper limit of y-axis for plotting
%
% Authors: Long Kiu Chung
% Created: 2024/08/19
% Updated: 2024/10/02

clear;
close all;
init_drifting;

%% Load data
load([demo_name, '_traindata.mat']); % From step 1
load([demo_name, '_network.mat']); % From step 2
load([demo_name, '_results.mat']); % From step 4

%% User-Defined Parameters
% For plotting
xl = [0, 40];
yl = [-20, 0];

%% Pass results through neural network
% Unnormalize
X_norm = transpose(diag(1./X_std)*(safe_samples((n_p + 1):end, :)' - ...
                   X_mean)');
Y_predict_norm = double(predict(net, X_norm));
Y_predict = transpose(diag(Y_std)*Y_predict_norm' + Y_mean');

%% Plotting setup
figure();
hold on;
axis equal;

% Colors
green_G = 0.01.*[4.3, 69.4, 63.9];
red_O = [225, 117, 111]./256;

%% Plot each safe trajectory
% Dimensions
n_safe = size(safe_samples, 2);

% Plot goal
plot(G.projection(1:2), 'color', green_G, 'alpha', 0.7);

% Loop over each safe sample found
for i = 1:n_safe
    p_0 = safe_samples(1:n_p, i);
    k = safe_samples((n_p + 1):end, i);

    % Predicted trajectory
    traj_predict = [zeros(n_p, 1), ...
                    reshape(Y_predict(i, 1:(end - n_q)), n_p, [])] + p_0;
    plot(traj_predict(1, :), traj_predict(2, :), '--', ...
         'LineWidth', 0.1, 'Color', [0, 0, 0, 0.5]);

    % Actual trajectory
    fprintf("Simulating trajectory %d/%d...\n", i, n_safe);
    [p_xs, p_ys, thetas] = simulate_drifting(p_0, k, ts_data);
    plot(p_xs, p_ys, '-', 'LineWidth', 0.1, 'color', [0, 0, 1, 0.5]);
end

%% Plot obstacles
% Obstacles with agent volume
O_B = O + B;
O_B.plot('color', red_O, 'alpha', 0.3, 'LineStyle', 'none');

% Actual obstacles
O.plot('color', red_O, 'alpha', 0.4, 'LineStyle', 'none');

%% Labels
% Limits
xlim(xl);
ylim(yl);

% Axes
xlabel("$p_x$", 'interpreter', 'latex', 'FontSize', 20); 
ylabel("$p_y$", 'interpreter', 'latex', 'FontSize', 20);
box on;
hold off;