function P = polygon_sweep(l, w, n)
% Create a n-sided polygon to overapproximate the circular sweep volume of
% a rectangle with length l and width w.
%
% See Appendix I.4 of Chung, Long Kiu, et al. "Goal-Reaching Trajectory 
% Design Near Danger with Piecewise Affine Reach-Avoid Computation". 
% Proceedings of Robotics: Science and Systems, 2024.
%
% INPUTS:
% l: double; length of the rectangle
% w: double; width of the rectangle
% n: int; number of sides of the polygonal overapproximation
%
% OUTPUTS:
% P: MPT3 Polyhedron; n-sided polygonal approximation
%
% Authors: Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/10/01

%% Initialization
r = sqrt(l.^2 + w.^2)./2; % Polygon radius
A = zeros(n, 2);
theta = linspace(0, 2.*pi, n + 1);

%% Create halfplanes
for i = 1:n
    A(i, :) = [cos(theta(i)), sin(theta(i))];
end

%% Create polyhedron
P = Polyhedron('A', A, 'b', r.*ones(n, 1));
end

