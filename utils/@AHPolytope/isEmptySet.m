function isEmpty = isEmptySet(obj, method)
    % Checks if obj is an empty AH-polytope by an LP.
    %
    % Input:
    % obj: Custom AHPolytope object
    %
    % Output:
    % isEmpty: Boolean; True if obj is empty, false otherwise
    %
    % Author: Long Kiu Chung
    % Created: 2024/03/20
    % Updated: 2024/03/24
    
    % Extracts parameters
    A = obj.A;
    b = obj.b;
    m = obj.m;

    % Decide which empty set method to use
    if nargin < 2
        if m >= 10
            method = "linprog";
        else % mpt3 is better for low-dimensions
            method = "mpt3";
        end
    end
    
    % Emptiness check
    if method == "mpt3" % MPT3 method
        isEmpty = obj.P.isEmptySet();
    else % linprog method
        options = optimoptions('linprog', 'Display', 'None'); % Mute message
        [~, ~, exitflag] = linprog(zeros(1, m), A, b, [], [], [], [], options);
        if exitflag == 1
            isEmpty = false;
        else
            isEmpty = true;
        end
    end
end

