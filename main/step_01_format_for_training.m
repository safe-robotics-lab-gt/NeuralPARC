%% STEP 1: FORMAT DATA FOR TRAINING
% Format raw data into the desired features and labels for training a
% neural network trajectory model.
%
% See Section IV.A of Chung, Long Kiu, et al. "Guaranteed Reach-Avoid for 
% Black-Box Systems through Narrow Gaps via Neural Network Reachability." 
% arXiv preprint arXiv:2409.13195 (2024).
%
% Change init_* file to try different examples.
%
% REQUIRED VARIABLES:
% --- from init_*.m ---
% See init_*.m
% --- from *_rawdata.mat ---
% ks_data: n_k*n_sample double; trajectory parameters for each sample
% ps_data: n_t_data*n_p*n_sample double; workspace history for each sample
% qs_data: n_q*n_sample double; non-workspace goal states of interest for
%          each sample
%
% RETURNS:
% --- saved as *_traindata.mat ---
% X: n_sample*n_k double; features for training
% Y: n_sample*(n_p.*(n_t - 1) + n_q) double; labels for training
% X_mean: 1*n_k double; mean of X
% X_std: 1*n_k double; standard deviation of X
% Y_mean: 1*(n_p.*(n_t - 1) + n_q) double; mean of Y
% Y_std: 1*(n_p.*(n_t - 1) + n_q) double; standard deviation of Y
%
% Authors: Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/09/30

clear;
close all;
init_drifting; % Drifting example
% init_boatsim; % ASV example

fprintf("Formatting data for training...\n");

%% Load data
load([demo_name, '_rawdata.mat']);

%% Features
X = ks_data';

%% Labels
% Initialization
n_sample = size(ps_data, 3);
Y = zeros(n_sample, n_p.*(n_t - 1) + n_q); % Direct transcription

% For each datum
for i = 1:n_sample
    p_ts_data = ps_data(:, :, i);

    % Downsample
    p_ts = transpose(match_trajectories(ts, ts_data, p_ts_data'));

    % Labels for p
    Y(i, 1:(end - n_q)) = reshape(p_ts(2:end, :)', 1, []);

    % Label for q
    if n_q > 0
        Y(i, (end - n_q + 1):end) = qs_data(i);
    end
end

%% Normalization
% Compute mean and std for normalizing the data
X_mean = mean(X, 1);
X_std = std(X, 1);
Y_mean = mean(Y, 1);
Y_std = std(Y, 1);

%% Save
save(fullfile(base_dir, 'data', [demo_name, '_traindata.mat']), ...
     'X', 'Y', 'X_mean', 'Y_mean', 'X_std', 'Y_std');