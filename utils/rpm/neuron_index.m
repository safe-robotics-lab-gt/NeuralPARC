function [l, j] = neuron_index(k, network_size)
% Get the index of the k-th neuron with activation functions.
%
% INPUTS:
% k: int; the order of the neuron in the network
% network_size: 1*(d - 1) int; hidden layer size of neural network
%
% OUTPUTS:
% l: int; the layer index
% j: int; the order of the neuron in the l-th layer
%
% Author: Long Kiu Chung, Wonsuhk Jung
% Created: 2024/03/29
% Updated: 2024/10/01

% Dimensions
d = length(network_size) + 1;

% Loop over the layers
for i = 1:(d - 1)
    if k <= network_size(i)
        l = i;
        j = k;
        break
    else
        k = k - network_size(i);
    end
end
end

