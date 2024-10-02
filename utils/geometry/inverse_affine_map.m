function [A_prev, b_prev] = inverse_affine_map(A_next, b_next, A, b, C, d)
% Compute the inverse affine map of P_next, defined as
% {x | A_prev*x <= b_prev} = {x | A_next(C*x + d) <= b_next, A*x <= b}
%
% See Lemma 1  of Chung, Long Kiu, et al. "Goal-Reaching Trajectory Design 
% Near Danger with Piecewise Affine Reach-Avoid Computation". Proceedings 
% of Robotics: Science and Systems, 2024.
%
% Input:
% A_next: l_next*n array
% b_next: l_next*1 array
% A: l*m array
% b: l*1 array
% C: n*m array
% d: n*1 array
%
% Output:
% P: MPT3 Polyhedron
%
% Author: Long Kiu Chung
% Created: 2024/03/24
% Updated: 2024/10/01

% Inverse affine map and intersection
A_prev = [A_next*C; A];
b_prev = [b_next - A_next*d; b];
end

