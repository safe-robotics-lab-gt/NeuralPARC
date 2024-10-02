function P_out = convexHull(obj, P_in)
    % Compute the convex hull between two AH-polytopes obj and P_in as the
    % AH-polytope P_out.
    %
    % Method adapted from Sadraddini, Sadra, and Russ Tedrake. "Linear 
    % encodings for polytope containment problems." 2019 IEEE 58th 
    % conference on decision and control (CDC). IEEE, 2019.
    %
    % Input:
    % obj: Custom AHPolytope object
    % P_in: Custom AHPolytope object; Must be the same dimension as obj
    %
    % Output:
    % P_out: Custom AHPolytope object; Same dimension as obj and P_in;
    %        Convex hull of obj and P_in
    %
    % Author: Long Kiu Chung
    % Created: 2024/03/20
    % Updated: 2024/10/01

    % Convert to AHPolytope if P_in is MPT3 Polyhedron
    if isa(P_in, 'Polyhedron')
        P_in = AHPolytope('P', P_in);
    end
    
    % Extract parameters
    A_1 = obj.A;
    b_1 = obj.b;
    C_1 = obj.C;
    d_1 = obj.d;
    A_2 = P_in.A;
    b_2 = P_in.b;
    C_2 = P_in.C;
    d_2 = P_in.d;

    % Dimensions
    l_1 = obj.l;
    m_1 = obj.m;
    l_2 = P_in.l;
    m_2 = P_in.m;

    % Convex hull
    C_out = [C_1, C_2, d_1 - d_2];
    d_out = d_2;
    H_1 = [A_1, zeros(l_1, m_2), -b_1];
    H_2 = [zeros(l_2, m_1), A_2, b_2];
    H_3 = zeros(2, m_1 + m_2 + 1);
    H_3(:, end) = [1; -1];
    A_out = [H_1; H_2; H_3];
    b_out = [zeros(l_1, 1); b_2; 1; 0];

    P_out = AHPolytope("A", A_out, "b", b_out, "C", C_out, "d", d_out);
end

