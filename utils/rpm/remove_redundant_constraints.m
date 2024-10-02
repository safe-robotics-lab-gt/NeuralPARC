function [P_reduce, mask_reduce] = remove_redundant_constraints(A_raw, ...
                                                                b_raw, ...
                                                                P_domain)
% Remove redundant constraints of the polytope {x | A_raw*x <= b_raw}
% subjected to the domain P_domain.
%
% See Algorithm 2 of Vincent, Joseph A., and Mac Schwager. "Reachable 
% polyhedral marching (rpm): A safety verification algorithm for robotic 
% systems with deep neural network components." 2021 IEEE International 
% Conference on Robotics and Automation (ICRA). IEEE, 2021.
%
% INPUTS:
% A_raw: n_h*n double; inequality constraint matrix of input polytope
% b_raw: 1*n_h double; inequality constraint vector of input polytope
% P_domain: MPT3 Polyhedron; domain constraints
%
% OUTPUTS:
% P_reduce: MPT3 Polyhedron; non-redundant polytope
% mask_reduce: 1*n_reduce boolean; mask of A_raw for non-redundancy
%
% Author: Long Kiu Chung, Wonsuhk Jung
% Created: 2024/03/29
% Updated: 2024/10/01

% Dimensions
n_h = size(A_raw, 1);

% Add domain constraints
P = Polyhedron('A', A_raw, 'b', b_raw);
P = P.intersect(P_domain);

% Use MPT3 to remove redundancy
[P_reduce, sol] = P.minHRep();
mask_reduce = ~sol.I(1:n_h);
end

