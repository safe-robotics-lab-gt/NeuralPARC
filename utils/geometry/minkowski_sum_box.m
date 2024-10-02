function P_out = minkowski_sum_box(P_in, box_l)
% P_out is the Minkowski sum of P_in with the hyperrectangle [-box_l(1),
% box_l(1)]*...*[-box_l(n), box_l(n)].
%
% See Lemma 1 of Chung, Long Kiu, et al. "Guaranteed Reach-Avoid for Black-
% Box Systems through Narrow Gaps via Neural Network Reachability." arXiv 
% preprint arXiv:2409.13195 (2024).
%
% INPUTS:
% P_in: MPT3 Polyhedron; polyhedron to be summed
% box_l: n*1 double; half-length of the hyperrectangle
%
% OUTPUTS:
% P_out: MPT3 Polyhedron; summed polyhedron
%
% Authors: Long Kiu Chung
% Created: 2024/09/15
% Updated: 2024/10/01

%% Get bounding box
P_bound = P_in.outerApprox;

%% Create redundancy in P_in
P_re = intersect(P_in, P_bound);
A = P_re.A;
b = P_re.b;

%% Output
P_out = Polyhedron('A', A, 'b', b + abs(A)*box_l);
end

