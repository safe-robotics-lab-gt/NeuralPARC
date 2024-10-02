function [brs, bas, brsEmptyMap, basEmptyMap] = compute_bras(C, d, G, ...
                                                             O, K, P_0, ...
                                                             isParfor)
% Compute the backward reachable set (BRS) and the backward avoid set (BAS)
% of goal sets and obstacles through a PWA system.
%
% See Section IV.C and IV.D of Chung, Long Kiu, et al. "Guaranteed Reach-
% Avoid for Black-Box Systems through Narrow Gaps via Neural Network 
% Reachability." arXiv preprint arXiv:2409.13195 (2024).
%
% INPUTS:
% C: (n_t*n_p + n_q)*(n_p + n_k) double; multiplying matrix of the affine 
%    map
% d: (n_t*n_p + n_q)*1 double; addition vector of the affine map
% G: 1*n_bin MPT3 Polyhedron; goal set shrinked with maximum final error
% O: (n_t - 1)*n_O*n_bin MPT3 Polyhedron; obstacles buffered with maximum 
%    interval errors
% K: 1*n_bin MPT3 Polyhedron; binned PWA region
% P_0: MPT3 Polyhedron; initial workspace domain; defaults as \R^{n_p}
% isParfor: boolean; true if using parfor; defaults as true
%
% OUTPUTS:
% brs: 1*n_bin MPT3 Polyhedron; backward reachable set
% bas: (n_t - 1)*n_O*n_bin MPT3 Polyhedron; backward avoid set
% brsEmptyMap: 1*n_bin boolean; true if brs is empty
% basEmptyMap: (n_t - 1)*n_O*n_bin boolean; true if bas is empty
%
% Author:  Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/10/01

%% Set default parameters
if nargin < 7
    isParfor = true;
end

if nargin < 6
    P_0 = [];
end

%% Initialization
% Dimensions
[n_tm1, n_O, n_bin] = size(O);
n_p = O(1).Dim;
n_t = n_tm1 + 1;
n_q = size(C, 1) - n_t*n_p;
n_k = size(C, 2) - n_p;

% Initialize variables
brs(1, n_bin) = Polyhedron;
brs_ws(1, n_bin) = AHPolytope;
bas(n_tm1, n_O, n_bin) = AHPolytope;
P_0_K(1, n_bin) = Polyhedron;
basEmptyMap = false(n_tm1, n_O, n_bin);

%% Backward reachable set (BRS)
% Map to final state
C_t_f = C((end - n_p - n_q + 1):end, :);
d_t_f = d((end - n_p - n_q + 1):end, :);

% Usually, n_bin is not that high, so it's not worth it to do parfor
for i = 1:n_bin
    % Extend domain to workspace
    if isempty(P_0)
        P_0_K(i) = Polyhedron('A', [zeros(size(K(i).A, 1), n_p), ...
                              K(i).A], 'b', K(i).b);
    else
        P_0_K(i) = Polyhedron('A', blkdiag(P_0.A, K(i).A), ...
                              'b', [P_0.b; K(i).b]);
    end

    % Inverse affine map
    [A_brs, b_brs] = inverse_affine_map(G(i).A, G(i).b, P_0_K(i).A, ...
                                        P_0_K(i).b, C_t_f, d_t_f);
    brs(i) = Polyhedron('A', A_brs, 'b', b_brs);

    % Project BRS onto workspace
    brs_ws(i) = AHPolytope("A", brs(i).A, "b", brs(i).b, ...
                           "C", [eye(n_p), zeros(n_p, n_k)], ...
                           "d", zeros(n_p, 1)); % Projection
end

% Emptiness check with MPT3
brsEmptyMap = brs.isEmptySet();

%% Backward avoid set (BAS)
if isParfor
    parfor i = 1:n_tm1
        % Map for timestep t
        C_t = C((n_p.*(i - 1) + 1):(n_p.*i), :);
        d_t = d((n_p.*(i - 1) + 1):(n_p.*i), :);

        % Map for timestep t + 1
        C_tp1 = C((n_p.*i + 1):(n_p.*(i + 1)), :);
        d_tp1 = d((n_p.*i + 1):(n_p.*(i + 1)), :);

        % For each obstacle
        for j = 1:n_O
            % For each bin
            for k = 1:n_bin
                if brsEmptyMap(k) % Don't need to compute BAS if empty BRS
                    basEmptyMap(i, j, k) = true;
                else
                    O_i_j_k = O(i, j, k);

                    % t-times BRS
                    [A_t, b_t] = inverse_affine_map(O_i_j_k.A, ...
                                                    O_i_j_k.b, ...
                                                    P_0_K(k).A, ...
                                                    P_0_K(k).b, ...
                                                    C_t, d_t);
                    B_t = AHPolytope("A", A_t, "b", b_t, ...
                                     "C", [eye(n_p), zeros(n_p, n_k)], ...
                                     "d", zeros(n_p, 1)); % Projection

                    % t + 1-times BRS
                    [A_tp1, b_tp1] = inverse_affine_map(O_i_j_k.A, ...
                                                        O_i_j_k.b, ...
                                                        P_0_K(k).A, ...
                                                        P_0_K(k).b, ...
                                                        C_tp1, d_tp1);
                    B_tp1 = AHPolytope("A", A_tp1, "b", b_tp1, ...
                                       "C", [eye(n_p), ...
                                       zeros(n_p, n_k)], ...
                                       "d", zeros(n_p, 1)); % Projection
                    
                    % Convex hull
                    B_ch = B_t.convexHull(B_tp1);

                    % Intersection
                    Lambda = B_ch.intersect(brs_ws(k));
                    
                    % Emptiness check
                    bas(i, j, k) = Lambda;
                    basEmptyMap(i, j, k) = Lambda.isEmptySet;
                end
            end
        end
    end
else
    % Exactly the same thing, but with for
    for i = 1:n_tm1
        % Map for timestep t
        C_t = C((n_p.*(i - 1) + 1):(n_p.*i), :);
        d_t = d((n_p.*(i - 1) + 1):(n_p.*i), :);

        % Map for timestep t + 1
        C_tp1 = C((n_p.*i + 1):(n_p.*(i + 1)), :);
        d_tp1 = d((n_p.*i + 1):(n_p.*(i + 1)), :);

        % For each obstacle
        for j = 1:n_O
            % For each bin
            for k = 1:n_bin
                if brsEmptyMap(k) % Don't need to compute BAS if empty BRS
                    basEmptyMap(i, j, k) = true;
                else
                    O_i_j_k = O(i, j, k);

                    % t-times BRS
                    [A_t, b_t] = inverse_affine_map(O_i_j_k.A, ...
                                                    O_i_j_k.b, ...
                                                    P_0_K(k).A, ...
                                                    P_0_K(k).b, ...
                                                    C_t, d_t);
                    B_t = AHPolytope("A", A_t, "b", b_t, ...
                                     "C", [eye(n_p), zeros(n_p, n_k)], ...
                                     "d", zeros(n_p, 1)); % Projection

                    % t + 1-times BRS
                    [A_tp1, b_tp1] = inverse_affine_map(O_i_j_k.A, ...
                                                        O_i_j_k.b, ...
                                                        P_0_K(k).A, ...
                                                        P_0_K(k).b, ...
                                                        C_tp1, d_tp1);
                    B_tp1 = AHPolytope("A", A_tp1, "b", b_tp1, ...
                                       "C", [eye(n_p), ...
                                       zeros(n_p, n_k)], ...
                                       "d", zeros(n_p, 1)); % Projection
                    
                    % Convex hull
                    B_ch = B_t.convexHull(B_tp1);

                    % Intersection
                    Lambda = B_ch.intersect(brs_ws(k));
                    
                    % Emptiness check
                    bas(i, j, k) = Lambda;
                    basEmptyMap(i, j, k) = Lambda.isEmptySet;
                end
            end
        end
    end
end
end

