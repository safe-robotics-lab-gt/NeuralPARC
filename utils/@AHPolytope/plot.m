function h = plot(obj, varargin)
    % Plot an AH-polytope.
    %
    % Convert the AH-Polytope to MPT3 Polyhedron, then plot it.
    %
    % Input:
    % obj: Custom AHPolytope object
    % varargin: Optional plotting arguments
    %
    % Output:
    % h: Plot handle
    %
    % Author: Long Kiu Chung
    % Created: 2024/03/20
    % Updated: 2024/03/20

    % Convert AH-polytope to H-polytope
    P = obj.P.affineMap(obj.C) + obj.d;
    P.plot(varargin{:});
end

