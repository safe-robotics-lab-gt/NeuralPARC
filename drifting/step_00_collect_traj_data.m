%% STEP 0: COLLECT TRAJECTORIES FOR TRAINING
% Interact with the black-box model to collect data for training and
% computing the modeling error bound.
%
% See Section IV.A of Chung, Long Kiu, et al. "Guaranteed Reach-Avoid for 
% Black-Box Systems through Narrow Gaps via Neural Network Reachability." 
% arXiv preprint arXiv:2409.13195 (2024).
%
% This script is written for the drift-parking example.
%
% REQUIRED VARIABLES:
% --- from init_*.m ---
% See init_*.m
% --- defined in this script ---
% n_sample: int; number of trajectories to collect
%
% RETURNS:
% --- saved as *_rawdata.mat ---
% ks_data: n_k*n_sample double; trajectory parameters for each sample
% ps_data: n_t_data*n_p*n_sample double; workspace history for each sample
% qs_data: n_q*n_sample double; non-workspace goal states of interest for
%          each sample
%
% Authors: Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/09/30

clear;
close all;
rng(0);
init_drifting;

%% User-Defined Parameters
% Change this to 10000 to reproduce results from the paper
n_sample = 4000; % Number of samples

%% Collect data
% Initialization
ks_data = zeros(n_k, n_sample);
ps_data = zeros(n_t_data, n_p, n_sample);
qs_data = zeros(n_q, n_sample);

% For each simulation
for i = 1:n_sample
    fprintf("Collecting data %d/%d...\n", i, n_sample);

    % Randomly sample trajectory parameter from K
    k = k_lo + rand(n_k, 1).*(k_hi - k_lo);

    % Rollout trajectory
    [p_xs, p_ys, thetas] = simulate_drifting([0; 0], k, ts_data);
    
    % Write data
    ks_data(:, i) = k; % v, theta_beta
    ps_data(:, :, i) = [p_xs', p_ys']; % p_x, p_y
    qs_data(:, i) = thetas(end); % theta_t_f
end

%% Save
save(fullfile(base_dir, 'data', [demo_name, '_rawdata.mat']), ...
     'ks_data', 'ps_data', 'qs_data');