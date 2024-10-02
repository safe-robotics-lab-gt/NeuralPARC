function flag = ap_is_in(aps_arr, ap)
% Returns true if the ap is already in ap_arr, if not return false
%
% INPUT:
% aps_arr: n_ap*n_neuron boolean; array of activation patterns
% ap: 1*(d - 1) cell of 1-D row binary; activation pattern to check
%
% OUTPUT:
% flag: boolean; true if ap is a subset of aps_arr
%
% Author: Long Kiu Chung, Wonsuhk Jung
% Created: 2024/03/30
% Updated: 2024/10/01

if isempty(aps_arr)
    flag = false;
else
    ap_arr = [ap{:}]; % Convert ap from cell to array
    flag = any(all(aps_arr == ap_arr, 2));
end
end

