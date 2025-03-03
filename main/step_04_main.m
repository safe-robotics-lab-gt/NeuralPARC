%% STEP 4: NeuralPARC
% Find the backward reach avoid set (BRAS) using NeuralPARC.
%
% See Section IV.C and IV.D of Chung, Long Kiu, et al. "Guaranteed Reach-
% Avoid for Black-Box Systems through Narrow Gaps via Neural Network 
% Reachability." arXiv preprint arXiv:2409.13195 (2024).
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
% --- from *_error.mat ---
% See step_03_training_error.m
%
% RETURNS:
% --- saved as *_results.mat ---
% safe_samples: (n_p + n_k)*n_safe double; safe initial workspace positions
%               and trajectory parameters certified by NeuralPARC
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
load([demo_name, '_network.mat']); % From step 2
load([demo_name, '_error.mat']); % From step 3

%% Extract neural network parameters
% Initialization
Ws = cell(1, d); % Weights
ws = cell(1, d); % Biases

% Extract from dlnetwork
for i = 1:d
    Ws{i} = double(net.Layers(2.*i).Weights);
    ws{i} = double(net.Layers(2.*i).Bias);
end

% Unnormalize input layer
ws{1} = ws{1} - Ws{1}*diag(1./X_std)*X_mean';
Ws{1} = Ws{1}*diag(1./X_std);

% Unnormalize output layer
Ws{end} = diag(Y_std)*Ws{end};
ws{end} = diag(Y_std)*ws{end} + Y_mean';

%% Setup domain
% Trajectory parameter
K = Polyhedron('A', [eye(n_k); -eye(n_k)], 'b', [k_hi; -k_lo]);

% Intial workspace condition
if isempty(p_0_hi)
    P_0 = [];
else
    P_0 = Polyhedron('A', [eye(n_p); -eye(n_p)], 'b', [p_0_hi; -p_0_lo]);
end

%% === Online portion (when obstacles and goal are known) starts here ===
%% Setup environment
% Account for agent volume
O_B = O + B;

% Initialization
G_tilde(1, n_bin) = Polyhedron;
O_tilde(n_t - 1, n_O, n_bin) = Polyhedron;

% Construct error polyhedrons
fprintf("Constructing error polyhedrons...\n");

% Compute bounding boxes ahead of time
O_B.outerApprox;

% Note: this code assumes G and O as general polyhedrons. If they are
% hyperrectangles, computation can be sped up by replacing the Minkowski
% sum and Pontryagin difference operations with closed-form expressions.

if isParfor
    % Shrink the goal
    if isBinning
        parfor i = 1:n_bin
            E_t_f = Polyhedron('lb', -e_t_f(:, i), 'ub', e_t_f(:, i));
            G_tilde(i) = G - E_t_f;
        end
    else
        E_t_f = Polyhedron('lb', -e_t_f(:), 'ub', e_t_f(:));
        G_tilde = G - E_t_f;
    end

    % Set up index conversion
    ns_O_tilde = [n_t - 1, n_O, n_bin];
    n_O_tilde = prod(ns_O_tilde);
    O_tilde_lin(1, n_O_tilde) = Polyhedron;

    parfor i = 1:n_O_tilde
        % Find grid indices for linear index
        [idx, jdx, kdx] = ind2sub(ns_O_tilde, i);

        % Buffer the obstacle
        E_tilde_ts = Polyhedron('lb', -e_tilde_ts(idx, :, kdx)', ...
                                'ub', e_tilde_ts(idx, :, kdx)');
        O_tilde_lin(i) = O_B(jdx) + E_tilde_ts;
    end
    O_tilde = reshape(O_tilde_lin, ns_O_tilde);
else
    for kdx = 1:n_bin
        % Shrink the goal
        E_t_f = Polyhedron('lb', -e_t_f(:, kdx), 'ub', e_t_f(:, kdx));
        G_tilde(kdx) = G - E_t_f;
        for idx = 1:(n_t - 1)
            for jdx = 1:n_O
                % Buffer the obstacle
                E_tilde_ts = Polyhedron('lb', ...
                                        -e_tilde_ts(idx, :, kdx)', ...
                                        'ub', e_tilde_ts(idx, :, kdx)');
                O_tilde(idx, jdx, kdx) = O_B(jdx) + E_tilde_ts;
            end
        end
    end
end

%% NeuralPARC
% Random seed to start
k_seed = k_lo + rand(n_k, 1).*(k_hi - k_lo);

% Main algorithm
safe_samples = neuralparc('weights', Ws, ...
                          'biases', ws, ...
                          'anytime', isAnytime, ...
                          'parfor', isParfor, ...
                          'bins', K_bin, ...
                          'goal', G_tilde, ...
                          'obstacles', O_tilde, ...
                          'seed', k_seed, ...
                          'trajdomain', K, ...
                          'initwsdomain', P_0, ...
                          'trials', n_try);

%% Save
% save(fullfile(base_dir, 'data', [demo_name, '_results.mat']), ...
%      'safe_samples');