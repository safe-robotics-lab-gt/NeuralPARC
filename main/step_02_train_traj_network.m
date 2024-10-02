%% STEP 2: TRAIN NEURAL NETWORK TRAJECTORY MODEL
% Train a ReLU neural network to model the trajectory behaviour of the
% robot.
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
% --- from *_traindata.mat ---
% See step_01_format_for_training.m
%
% RETURNS:
% --- saved as *_network.mat ---
% net: dlnetwork; ReLU network trajectory model
%
% Authors: Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/10/01

clear;
close all;
rng(0);
init_drifting; % Drifting example
% init_boatsim; % ASV example

%% Load data
load([demo_name, '_traindata.mat']); % From step 1

%% Create neural network
% Get dimensions
m = size(Y, 2); % Output size
d = length(network_size) + 1; % Network depth

% Neural network design
net = dlnetwork;
layers = featureInputLayer(n_k); % Input layer
for i = 1:(d - 1)
    % Hidden layer
    layers = [layers; fullyConnectedLayer(network_size(i)); reluLayer];
end
layers = [layers; fullyConnectedLayer(m)]; % No ReLU at output layer
net = addLayers(net, layers);

%% Extract validation data
% Normalize data
X_norm = transpose(diag(1./X_std)*(X - X_mean)');
Y_norm = transpose(diag(1./Y_std)*(Y - Y_mean)');

% Separate training and validation data
n_sample = size(X, 1);
I_all = 1:n_sample; % All data indices
n_test = ceil(n_sample.*validation_ratio); % Number of test samples
n_train = n_sample - n_test; % Number of training samples

% Get indices of test data
I_test = randsample(n_sample, n_test);

% Get indices of training data
I_train = setdiff(I_all', I_test);

% Select train and test data
X_train = X_norm(I_train, :);
Y_train = Y_norm(I_train, :);
X_test = X_norm(I_test, :);
Y_test = Y_norm(I_test, :);

%% Train network
% Training options
if isempty(X_test) % No validation
    options = trainingOptions(solverName, ...
                              'ExecutionEnvironment', ...
                              ExecutionEnvironment, ...
                              'MaxEpochs', MaxEpochs, ...
                              'InitialLearnRate', InitialLearnRate, ...
                              'LearnRateSchedule', LearnRateSchedule, ...
                              'LearnRateDropPeriod', ...
                              LearnRateDropPeriod, ...
                              'LearnRateDropFactor', ...
                              LearnRateDropFactor, ...
                              'Plots', Plots);
else % Yes validation
    options = trainingOptions(solverName, ...
                              'ExecutionEnvironment', ...
                              ExecutionEnvironment, ...
                              'MaxEpochs', MaxEpochs, ...
                              'InitialLearnRate', InitialLearnRate, ...
                              'LearnRateSchedule', LearnRateSchedule, ...
                              'LearnRateDropPeriod', ...
                              LearnRateDropPeriod, ...
                              'LearnRateDropFactor', ...
                              LearnRateDropFactor, ...
                              'ValidationData', {X_test, Y_test}, ...
                              'ValidationFrequency', ...
                              ValidationFrequency, ...
                              'Plots', Plots);
end

% Begin training
fprintf("Training...\n");
net = trainnet(X_train, Y_train, net, "mse", options);

%% Save
save(fullfile(base_dir, 'data', [demo_name, '_network.mat']), 'net');