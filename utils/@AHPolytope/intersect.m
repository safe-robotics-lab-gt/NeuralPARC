function P_out = intersect(obj, P_in)
    % Compute the intersection between two AH-polytopes obj and P_in as 
    % the AH-polytope P_out.
    %
    % Method adapted from Sadraddini, Sadra, and Russ Tedrake. "Linear 
    % encodings for polytope containment problems." 2019 IEEE 58th 
    % conference on decision and control (CDC). IEEE, 2019.
    %
    % Input:
    % obj: Custom AHPolytope object
    % P_in: Custom AHPolytope object or MPT3 Polyhedron; Must be the same 
    %       dimension as obj
    %
    % Output:
    % P_out: Custom AHPolytope object; Same dimension as obj and P_in;
    %        intersection of obj and P_in
    %
    % Author: Long Kiu Chung
    % Created: 2024/03/20
    % Updated: 2024/03/26

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
    m_2 = P_in.m;
    n_2 = P_in.n;

    % Intersection
    C_out = [C_1, zeros(n_2, m_2)];
    d_out = d_1;
    H_1 = blkdiag(A_1, A_2);
    H_2 = [C_1, -C_2];
    A_out = [H_1; H_2; -H_2];
    b_out = [b_1; b_2; d_2 - d_1; d_1 - d_2];

    P_out = AHPolytope("A", A_out, "b", b_out, "C", C_out, "d", d_out);
end

