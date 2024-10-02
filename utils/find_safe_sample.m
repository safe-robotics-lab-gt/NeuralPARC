function safe_sample = find_safe_sample(brs, bas, basEmptyMap, n_try, isParfor)
% Sample from BRS but not BAS.
%
% See Section IV.D of Chung, Long Kiu, et al. "Guaranteed Reach-Avoid for 
% Black-Box Systems through Narrow Gaps via Neural Network Reachability." 
% arXiv preprint arXiv:2409.13195 (2024).
%
% INPUTS:
% brs: 1*n_bin MPT3 Polyhedron; backward reachable set
% bas: (n_t - 1)*n_O*n_bin MPT3 Polyhedron; backward avoid set
% basEmptyMap: (n_t - 1)*n_O*n_bin boolean; true if bas is empty
% n_try: int; number of samples to safety check in each PWA region; 
%        defaults as 50
% isParfor: boolean; true if using parfor; defaults as true
%
% OUTPUTS:
% safe_sample: (n_p + n_k)*n_safe double; successfully sampled points
%
% Author:  Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/10/01

%% Initialization
% Dimensions
n_p = bas(1).n;

% Initialize variables
safe_sample = [];
isSamplesSafe = true(n_try, 1);

%% Sample from BRS
% Get n_try samples from BRS
[samples, idx] = sample_from_sets(brs, n_try);
if isempty(samples)
    return
end

%% Do not sample from BAS
if isParfor
    parfor i = 1:n_try
        % Extract variables
        sample = samples(:, i);
        sample_ws = sample(1:n_p);

        % Apply empty mask
        bas_i = bas(:, :, idx(i));
        basEmptyMap_i = basEmptyMap(:, :, idx(i));
        bas_i = bas_i(~basEmptyMap_i);

        n_bas = numel(bas_i);

        for j = 1:n_bas % Check if sample is in any of the BAS
            bas_j = bas_i(j);
            if bas_j.contains(sample_ws)
                isSamplesSafe(i) = false;
                break
            end
        end
    end
else
    % Exact same thing but with for
    for i = 1:n_try
        % Extract variables
        sample = samples(:, i);
        sample_ws = sample(1:n_p);

        % Apply empty mask
        bas_i = bas(:, :, idx(i));
        basEmptyMap_i = basEmptyMap(:, :, idx(i));
        bas_i = bas_i(~basEmptyMap_i);

        n_bas = numel(bas_i);

        for j = 1:n_bas % Check if sample is in any of the BAS
            bas_j = bas_i(j);
            if bas_j.contains(sample_ws)
                isSamplesSafe(i) = false;
                break
            end
        end
    end    
end

% Find successful cases
safe_idx = find(isSamplesSafe);
if ~isempty(safe_idx)
    safe_sample = samples(:, safe_idx);
end
end

