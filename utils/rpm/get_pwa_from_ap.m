function [A, b, C, d] = get_pwa_from_ap(Ws, ws, ap)
% Get the PWA region and the PWA map corresponding to the activation
% pattern ap.
%
% See Vincent, Joseph A., and Mac Schwager. "Reachable polyhedral marching 
% (rpm): A safety verification algorithm for robotic systems with deep 
% neural network components." 2021 IEEE International Conference on 
% Robotics and Automation (ICRA). IEEE, 2021.
%
% INPUTS:
% Ws: 1*d cell of 2-D double; weights of the neural network
% ws: 1*d cell of 1-D column double; biases of the neural network
% ap: 1*(d - 1) cell of 1-D row binary; activation pattern
%
% OUTPUTS:
% A: n_h*n double; inequality constraint matrix
% b: n_h*1 double; inequality constraint vector
% C: m*n double; multiplying matrix of the affine map
% d: m*1 double; addition vector of the affine map
%
% Author: Long Kiu Chung, Wonsuhk Jung
% Created: 2024/03/30
% Updated: 2024/10/01

%% Initialization
% Dimensions
in_dim = size(Ws{1}, 2);
d = numel(Ws);

% Intialize variables
M = eye(in_dim + 1); % Input map
A = []; 
b = [];

%% Forward pass
for i = 1:(d - 1)
    % Homogenify
    W_homo = [Ws{i}, ws{i}; zeros(1, size(Ws{i}, 2)), 1];

    M = W_homo*M; % Linear layer
    [A_i, b_i] = get_hyperplanes(M, ap{i});
    M = diag([ap{i}, 1])*M; % ReLU

    A = [A; A_i];
    b = [b; b_i];
end

%% Final layer
W_homo = [Ws{end}, ws{end}; zeros(1, size(Ws{end}, 2)), 1]; % Homogenify
M = W_homo*M; % Linear layer

% Extract affine map
C = M(1:(end - 1), 1:(end - 1));
d = M(1:(end - 1), end);
end

