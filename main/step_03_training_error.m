%% STEP 3: COMPUTE TRAINING ERROR BOUND
% Compute the modeling error bound of the neural network trajectory model.
%
% See Section IV.B of Chung, Long Kiu, et al. "Guaranteed Reach-Avoid for 
% Black-Box Systems through Narrow Gaps via Neural Network Reachability." 
% arXiv preprint arXiv:2409.13195 (2024).
%
% Change init_* file to try different examples.
%
% REQUIRED VARIABLES:
% --- from init_*.m ---
% See init_*.m
% --- from *_traindata.mat ---
% See step_01_format_for_training.m
% --- from *_network.mat ---
% See step_02_train_traj_network.m
% --- from *_rawdata.mat ---
% ks_data: n_k*n_sample double; trajectory parameters for each sample
% ps_data: n_t_data*n_p*n_sample double; workspace history for each sample
% qs_data: n_q*n_sample double; non-workspace goal states of interest for
%          each sample
% --- defined in this script ---
% vis: boolean; true if visualize results
%
% RETURNS:
% --- saved as *_error.mat ---
% e_tilde_ts: (n_t - 1)*n_p*n_bin double; maximum interval errors
% e_t_f: (n_p + n_q)*n_bin double; maximum final error
%
% Authors: Long Kiu Chung
% Created: 2024/07/25
% Updated: 2024/10/01

clear;
close all;
init_drifting; % Drifting example
% init_boatsim; % ASV example

fprintf("Computing modeling error bound...\n");

%% User-defined parameters
vis = true;

%% Load data
load([demo_name, '_rawdata.mat']);
load([demo_name, '_traindata.mat']); % From step 1
load([demo_name, '_network.mat']); % From step 2

%% Initialization
% Error bounds
e_tilde_ts = zeros(n_t - 1, n_p, n_bin); % Maximum interval errors
e_t_f = zeros(n_p + n_q, n_bin); % Maximum final error

% Dimensions and sizes
n_sample = size(X, 1);

%% Validation on training data set
% Unnormalize
X_norm = transpose(diag(1./X_std)*(X - X_mean)');
Y_predict_norm = double(predict(net, X_norm));
Y_predict = transpose(diag(Y_std)*Y_predict_norm' + Y_mean');

% For each training data
for i = 1:n_sample
    % Find out which bin
    bin_idx = find(K_bin.contains(X(i, :)', true), 1);
    if isempty(bin_idx) % Not in bin
        continue
    end

    % Final error
    if n_q > 0
        e_t_f_i = abs(Y_predict(i, (end - n_p - n_q + 1):end) ...
                      - [ps_data(end, :, i), qs_data(:, i)'])';
    else
        e_t_f_i = abs(Y_predict(i, (end - n_p - n_q + 1):end) ...
                      - ps_data(end, :, i))';
    end
    e_t_f(:, bin_idx) = max(e_t_f(:, bin_idx), e_t_f_i);

    % Trajectory for the episode
    traj_predict = [zeros(n_p, 1), ...
                    reshape(Y_predict(i, 1:(end - n_q)), n_p, [])];
    traj_actual = ps_data(:, :, i)';

    % Linear interpolation
    traj_predict_ts_data = match_trajectories(ts_data, ts, traj_predict);
    
    % Trajectory prediction error
    traj_error = abs(traj_actual - traj_predict_ts_data);

    % Interval error
    for j = 1:(n_t - 1)
        % Find start and end time of interval
        t_lb = ts(j);
        t_ub = ts(j + 1);

        % Find index
        lb_idx = find(ts_data >= t_lb, 1);
        ub_idx = find(ts_data <= t_ub, 1, 'last');

        % Error bound
        e_tilde_t_i = max(traj_error(:, lb_idx:ub_idx), [], 2)';
        
        % Assignment
        e_tilde_ts(j, :, bin_idx) = max(e_tilde_ts(j, :, bin_idx), ...
                                        e_tilde_t_i);
    end
end

%% Visualization
if vis
    for i = 1:n_bin % New figure for each bin
        figure();
        for j = 1:n_p % For each workspace dimension
            subplot(n_p, 1, j);
            hold on;
            plot(ts(2:end), e_tilde_ts(:, j, i), '-k');
            xlabel("$t$", 'interpreter', 'latex', 'FontSize', 20); 
            ylabel(['$\tilde{e}_{t, ', num2str(j), '}$'], ...
                   'interpreter', 'latex', 'FontSize', 20);
            hold off;
        end
        sgtitle(['Bin ', num2str(i)]);
    end
end

%% Save
save(fullfile(base_dir, 'data', [demo_name, '_error.mat']), ...
     'e_tilde_ts', 'e_t_f');