function P_bin = create_bins(lb, ub, ns_bin)
% Partition a hyperrectangle defined by lb and ub into smaller, equally-
% sized hyperrectangles.
%
% See Section III.C of Kousik, Shreyas, Patrick Holmes, and Ramanarayan 
% Vasudevan. "Technical report: Safe, aggressive quadrotor flight via 
% reachability-based trajectory design." arXiv preprint arXiv:1904.05728 
% (2019).
%
% INPUTS:
% lb: n*1 double; lower bound of the big hyperrectangle
% ub: n*1 double; lower bound of the big hyperrectangle
% ns_bin: 1*n double; number of bins in each dimension
%
% OUTPUTS:
% P_bin: 1*n_bin MPT3 Polyhedron; partitions of the big hyperrectangle
%
% Authors: Long Kiu Chung
% Created: 2024/09/12
% Updated: 2024/10/01

%% Initialization
n_bin = prod(ns_bin); % Total number of bins
n = length(ns_bin); % Number of dimensions
P_bin = Polyhedron(1, n_bin);

bin_width = (ub - lb)./ns_bin'; % Width of the bins
A_bin = [eye(n); -eye(n)]; % A matrix of bins

%% Begin binning
for i = 1:n_bin
    % Find grid index from linear indices
    bin_idxs = cell(1, n);
    [bin_idxs{:}] = ind2sub(ns_bin, i);
    bin_idxs = cell2mat(bin_idxs)';

    % Extract lower and upper bounds
    lb_bin = lb + (bin_idxs - 1).*bin_width;
    ub_bin = lb_bin + bin_width;

    % Create bin
    b_bin = [ub_bin; -lb_bin];
    P_bin(i) = Polyhedron('A', A_bin, 'b', b_bin);
end
end

