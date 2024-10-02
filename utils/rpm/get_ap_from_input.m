function ap = get_ap_from_input(Ws, ws, x)
% Get the activation pattern corresponding to the input x.
%
% See Vincent, Joseph A., and Mac Schwager. "Reachable polyhedral marching 
% (rpm): A safety verification algorithm for robotic systems with deep 
% neural network components." 2021 IEEE International Conference on 
% Robotics and Automation (ICRA). IEEE, 2021.
%
% INPUTS:
% Ws: 1*d cell of 2-D double; weights of the neural network
% ws: 1*d cell of 1-D column double; biases of the neural network
% x: n*1 double; input to the neural network
%
% OUTPUTS:
% ap: 1*(d - 1) cell of 1-D row binary; activation pattern of x
%
% Author: Long Kiu Chung, Wonsuhk Jung
% Created: 2024/03/30
% Updated: 2024/10/01

%% Pass through the neural network
% Dimension
d = length(Ws);

% Initialization
ap = cell(1, d - 1);

% Loop for each layer
for i = 1:(d - 1)
    x_pre = Ws{i}*x + ws{i}; % Pre-activation value
    ap{i} = transpose(x_pre > 0);
    x = max(0, x_pre); % ReLU
end
end

