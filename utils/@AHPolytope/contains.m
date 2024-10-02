function isin = contains(obj, x)
    % Check if x \in {obj.C*y + obj.d | obj.A*y <= obj.b} is true or not.
    %
    % Input:
    % obj: Custom AHPolytope object
    % x: Column vector; Must be the same dimension as obj
    %
    % Output:
    % isin: Boolean; Returns true if x \in obj, false otherwise
    %
    % Author: Long Kiu Chung
    % Created: 2024/08/19
    % Updated: 2024/08/19

    % Extract parameters
    m = obj.m;
    A = obj.A;
    b = obj.b;
    C = obj.C;
    d = obj.d;
    
    % Feasibility LP
    options = optimoptions('linprog', 'Display', 'None'); % Mute message

    [~, ~, exitflag] = linprog(zeros(1, m), A, b, C, x - d, [], [], options);

    if exitflag == 1
        isin = true;
    else
        isin = false;
    end
end

