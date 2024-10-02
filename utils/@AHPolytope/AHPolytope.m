classdef AHPolytope
    % AH-polytope defined by {C*x + d | x \in P} \subset \R^n, where P = {x
    % | A*x <= b} \subset \R^m
    %
    % Author: Long Kiu Chung
    % Created: 2024/03/20
    % Updated: 2024/10/01
    
    properties
        % Definition
        A % l*m array
        b % l*1 array
        C % n*m array
        d % n*1 array
        P % MPT3 Polyhedron; m-dimensional polyhedron

        % Dimensions
        n % int
        m % int
        l % int
    end
    
    methods
        function obj = AHPolytope(varargin)
            % Required input: Either P or both A and b
            % Optional input: C or both C and d
            %
            % Example:
            % AH = AHPolytope("A", [1; -1], "b", [1; 1], "C", 2, "d", 3);
            % creates the AH-polytope {2*x + 3 | -1 <= x <= 1}
            
            % Parse inputs
            if nargin == 0
                obj.P = Polyhedron;
                obj.A = [];
                obj.b = [];
                obj.C = [];
                obj.d = [];
                obj.n = 0;
                obj.m = 0;
                obj.l = 0;
                return
            end

            ip = inputParser;
            ip.addParameter('A', []);
			ip.addParameter('b', []);
            ip.addParameter('C', []);
			ip.addParameter('d', []);
            ip.addParameter('P', []);
            ip.parse(varargin{:});
            p = ip.Results;
            
            % Extract information from the polyhedron
            if ~isempty(p.P) % If P is specified
                obj.P = p.P;
                obj.A = obj.P.A;
                obj.b = obj.P.b;
            else % If both A and b are specified
                obj.A = p.A;
                obj.b = p.b;
                obj.P = Polyhedron('A', obj.A, 'b', obj.b);
            end
            
            [obj.l, obj.m] = size(obj.A);

            if ~isempty(p.C) % If C is specified
                obj.C = p.C;
                obj.n = size(obj.C, 1);
                isC = true;
            else % Assume C = I
                obj.C = eye(obj.m); 
                obj.n = obj.m;
                isC = false;
            end

            if ~isempty(p.d) % If d is specified
                obj.d = p.d;
                if ~isC % If d is specified but C isn't
                    error("Please provide the C matrix")
                end
            else % Assume d = 0
                obj.d = zeros(obj.n, 1);
            end
            
        end
    end
end

