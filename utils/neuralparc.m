function safe_samples = neuralparc(varargin)
% Main NeuralPARC algorithm. Find initial workspace positions and
% trajectory parameters such that the robot is guaranteed to reach the goal
% while avoiding obstacles.
%
% See Section IV.C and IV.D of Chung, Long Kiu, et al. "Guaranteed Reach-
% Avoid for Black-Box Systems through Narrow Gaps via Neural Network 
% Reachability." arXiv preprint arXiv:2409.13195 (2024).
%
% REQUIRED INPUTS:
% weights: 1*d cell of 2-D double; weights of the neural network
% biases: 1*d cell of 1-D column double; biases of the neural network
% goal: 1*n_bin MPT3 Polyhedron; goal set shrinked with maximum final error
% obstacles: (n_t - 1)*n_O*n_bin MPT3 Polyhedron; obstacles buffered with
%            maximum interval errors
% trajdomain: MPT3 Polyhedron; trajectory parameter domain
%
% OPTIONAL INPUTS:
% anytime: boolean; true if terminate early when one safe sample is found;
%          defaults as true
% parfor: boolean; true if using parfor; defaults as true
% bins: 1*n_bin MPT3 Polyhedron; binned trajectory parameter space; no
%       binning if not provided
% seed: n_k*1 double; starting seed for the algorithm; randomly sample from
%       domain if not provided
% initwsdomain: MPT3 Polyhedron; initial workspace domain; defaults as
%               \R^{n_p}
% trials: int; number of samples to safety check in each PWA region;
%         defaults as 50
%
% OUTPUTS:
% safe_samples: (n_p + n_k)*n_safe double; safe initial workspace positions
%               and trajectory parameters certified by NeuralPARC
%
% Author:  Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/10/01

%% User-defined inputs
epsilon = 1e-6; % Tolerance for RPM

%% Parse inputs
ip = inputParser;
ip.addParameter('weights', {});
ip.addParameter('biases', {});
ip.addParameter('anytime', true);
ip.addParameter('parfor', true);
ip.addParameter('bins', []);
ip.addParameter('goal', []);
ip.addParameter('obstacles', []);
ip.addParameter('seed', []);
ip.addParameter('trajdomain', []);
ip.addParameter('initwsdomain', []);
ip.addParameter('trials', 50);
ip.parse(varargin{:});
p = ip.Results;

% Assign parse results
Ws = p.weights;
ws = p.biases;
isAnytime = p.anytime;
isParfor = p.parfor;
K_bin = p.bins;
G_tilde = p.goal;
O_tilde = p.obstacles;
k_seed = p.seed;
K = p.trajdomain;
P_0 = p.initwsdomain;
n_try = p.trials;

% Yell if required variables are not provided
if isempty(Ws) || isempty(ws) || isempty(G_tilde) || isempty(O_tilde) ...
   || isempty(K)
    error('Missing required arguments!');
end

% Randomly sample from domain if seed is not declared
if isempty(k_seed)
    result = K.chebyCenter;
    if result.exitflag == 1
        k_seed = result.x;
    else
        error('Fail to sample from domain. Please provide a seed!');
    end
end

% Don't bin if Ps_bin is not provided
if isempty(K_bin)
    isBinning = false;
else
    isBinning = numel(K_bin) > 1;
end

%% Initialization
% Get dimensions
n_p = O_tilde(1).Dim;
n_q = G_tilde(1).Dim - n_p;
n_k = size(Ws{1}, 2);
n_t = size(O_tilde, 1) + 1;

% Get network size
depth = numel(ws);
network_size = zeros(1, depth - 1);
for i = 1:(depth - 1)
    network_size(i) = size(ws{i}, 1);
end

% Initialize the activation pattern
ap = get_ap_from_input(Ws, ws, k_seed);

% Initialize variables
working_queue = {ap}; % queue of the activation pattern
working_queue_arr = [ap{:}]; % array version of working_queue
visited_ap = {}; % array of the activation patterns
visited_ap_arr = [];
ap_count = 0; % number of the explored activation patterns

safe_samples = [];

% Precompute matrices to speed up calculation
C_eye = [repmat(eye(n_p), n_t - 1, 1); zeros(n_q, n_p)];

%% Reachable Polyhedral Marching (RPM)
while ~isempty(working_queue)
    fprintf("===== %d-th ap explored, %d ap in queue\n", ap_count, ...
            numel(working_queue));
    ap_count = ap_count + 1;

    % Pop current ap from the queue
    ap_cur = working_queue{1}; 
    working_queue(1) = [];
    working_queue_arr = working_queue_arr(2:end, :);

    % Add current ap to visited ap
    visited_ap{end+1} = ap_cur;
    ap_cur_arr = [ap_cur{:}];
    visited_ap_arr = [visited_ap_arr; ap_cur_arr];

    % Get constraints from the current ap
    [A_raw, b_raw, C, d] = get_pwa_from_ap(Ws, ws, ap_cur);

    % Get essential consrtaints
    [K_reduce, mask_reduce] = remove_redundant_constraints(A_raw, ...
                                                           b_raw, K);
    
    % Essential constraints that do not include the domain constraints
    % Since we do not have to flip w.r.t. to the domain constraints
    A_essential = A_raw(mask_reduce, :);
    b_essential = b_raw(mask_reduce, :);

    % If PWA region is empty
    if isEmptySet(K_reduce)
        continue
    end
    
    % === NeuralPARC ===
    % Intersect bins with PWA region
    if isBinning
        K_pwa = K_bin.intersect(K_reduce);
        bin_idx = find(~isEmptySet(K_pwa));
    else
        K_pwa = K_reduce;
        bin_idx = 1;
    end

    % Get map with workspace using translational invariance
    C_ws = [eye(n_p), zeros(n_p, n_k); C_eye, C];
    d_ws = [zeros(n_p, 1); d];
    
    % BRAS
    fprintf("Computing reach set and avoid set...\n");
    [brs, bas, ...
     brsEmptyMap, basEmptyMap] = compute_bras(C_ws, d_ws, ...
                                              G_tilde(bin_idx), ...
                                              O_tilde(:, :, bin_idx), ...
                                              K_pwa(bin_idx), P_0, ...
                                              isParfor);
    brs = brs(~brsEmptyMap);
    bas = bas(:, :, ~brsEmptyMap);
    basEmptyMap = basEmptyMap(:, :, ~brsEmptyMap);
    
    if ~isempty(brs)
        fprintf("Finding safe samples...\n");
        safe_sample = find_safe_sample(brs, bas, basEmptyMap, n_try, ...
                                       isParfor);
        
        % Append if safe sample found
        if ~isempty(safe_sample)
            fprintf("Safe sample found!\n");
            safe_samples = [safe_samples, safe_sample];
            if isAnytime % Early termination
                return
            end
        end
    else
        fprintf("BRS is empty!\n");
    end
    % === end of NeuralPARC ===

    % Flipping
    n_essential = size(A_essential, 1);
    for j = 1:n_essential
        % Loop over essential constraints and add neighboring ap
        a_essential_j = A_essential(j, :);
        b_essential_j = b_essential(j);

        % Deep copy: j-th neighbor of i-th ap
        neighbor_ap_j = ap_cur;

        % Flip the activation if we have to
        for k = 1:size(A_raw, 1)
            % Loop over every neuron in the network 
            [l_k, j_k] = neuron_index(k, network_size);

            if (norm(A_raw(k, :)) < epsilon) && ...
               (norm(b_raw(k, :)) < epsilon)
                % RPM Algorithm 3, line 6-7
                neighbor_ap_j{l_k}(j_k) = 0;
            elseif norm(A_raw(k, :) - a_essential_j) <= epsilon && ...
                   norm(b_raw(k, :) - b_essential_j) <= epsilon
                % RPM Algorithm 3, line 4-5
                neighbor_ap_j{l_k}(j_k) = ~neighbor_ap_j{l_k}(j_k);
            end
        end

        % Add activation patterns if it is not in the queue or visited ap
        if ~ap_is_in(visited_ap_arr, neighbor_ap_j) &&  ...
           ~ap_is_in(working_queue_arr, neighbor_ap_j)
            neighbor_ap_j_arr = [neighbor_ap_j{:}];
            working_queue_arr = [working_queue_arr; neighbor_ap_j_arr];
            working_queue{end+1} = neighbor_ap_j;
        end
    end
end
end

