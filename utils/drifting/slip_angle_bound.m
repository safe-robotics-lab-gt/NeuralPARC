function [alpha_slip_f, alpha_slip_r] = ...
    slip_angle_bound(tire_f, Fxf,tire_r, Fxr)
% -------------------------------------------------------------------------
% This is a legacy code from Chung, Long Kiu, et al. "Goal-Reaching 
% Trajectory Design Near Danger with Piecewise Affine Reach-Avoid 
% Computation". Proceedings of Robotics: Science and Systems, 2024.
%
% Original code is written by Chuizheng Kong. Direct all questions for this
% code to him.
%
% Updated: 2024/09/27
% -------------------------------------------------------------------------

% calculate the boundary of slipping angle
zeta_f = sqrt((tire_f.mu*tire_f.Fz)^2 - Fxf^2) / (tire_f.mu*tire_f.Fz);
alpha_slip_f = atan2(3*zeta_f*tire_f.mu*tire_f.Fz, tire_f.Ca);

zeta_r = sqrt((tire_r.mu*tire_r.Fz)^2 - Fxr^2) / (tire_r.mu*tire_r.Fz);
alpha_slip_r = atan2(3*zeta_r*tire_r.mu*tire_r.Fz, tire_r.Ca);
end
