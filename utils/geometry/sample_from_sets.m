function [samples, idx] = sample_from_sets(sets, n_sample)
% Find n_sample points in the union of multiple sets.
% 
% INPUTS:
% sets: 1*n_set MPT3 Polyhedron; sets to sample from
% n_sample: int; number of desired samples
%
% OUTPUTS:
% samples: n*n_sample double; n_sample points in sets
% idx: 1*n_sample int; corresponding indices the set containing the samples
%
% Authors: Long Kiu Chung
% Created: 2024/09/12
% Updated: 2024/10/01

% Compute bounding box
sets_U = PolyUnion(sets);
sets_box = sets_U.outerApprox;

sets_lb = sets_box.Internal.lb;
sets_ub = sets_box.Internal.ub;

% If bounding box has not been computed
if any(isinf(sets_ub)) || any(isinf(sets_lb))
    samples = [];
    idx = [];
    return
end
sets_diff = sets_ub - sets_lb;

% Initialization
n = sets_box.Dim;
samples = zeros(n, n_sample);
idx = zeros(1, n_sample);

% Sample until n_sample == n_success
n_success = 0;
while n_success < n_sample
    sample = sets_lb + rand(n, 1).*sets_diff;
    [isin, inwhich] = sets_U.contains(sample, true);
    if isin
        n_success = n_success + 1;
        samples(:, n_success) = sample;
        idx(n_success) = inwhich;
    end
end
end

