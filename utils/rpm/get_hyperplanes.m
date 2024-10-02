function [A, b] = get_hyperplanes(M, ap)
% Get the corresponding hyperplanes given map M and activation pattern ap.
%
% See Algorithm 1 of Vincent, Joseph A., and Mac Schwager. "Reachable 
% polyhedral marching (rpm): A safety verification algorithm for robotic 
% systems with deep neural network components." 2021 IEEE International 
% Conference on Robotics and Automation (ICRA). IEEE, 2021.
%
% INPUTS:
% M: m*n double; mapping matrix
% ap: 1*m binary; activation pattern
%
% OUTPUTS:
% A: n_h*(n-1) double; inequality constraint matrix
% b: n_h*1 double; inequality constraint vector
%
% Author: Long Kiu Chung, Wonsuhk Jung
% Created: 2024/03/29
% Updated: 2024/10/01

%% User-defined inputs
epsilon = 1e-6; % Tolerance for RPM

%% Get the constraints
% Transpose
ap = ap';

% Eq. (14) of RPM
A_bar = M(1:(end - 1), 1:(end - 1));
b_bar = -M(1:(end - 1), end);
A_bar_norm = vecnorm(A_bar, 2, 2);

mask = (A_bar_norm < epsilon); % Zero-norm mask

% Initialize
A = zeros(size(A_bar));
b = zeros(size(b_bar));

% Apply mask
A(~mask, :) = (1 - 2*ap(~mask)).*A_bar(~mask, :)./A_bar_norm(~mask);
b(~mask, :) = (1 - 2*ap(~mask)).*b_bar(~mask, :)./A_bar_norm(~mask);
end

