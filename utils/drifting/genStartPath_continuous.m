function S = genStartPath_continuous(V_drift,beta_des,dT, veh, tire_f, tire_r)%, x_start, y_start)
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
%
% generate reference path for parallel parking
%
% given a desired 
% using control limit a_max 5.1 and inverse alpha_f alpha_r to get r_max 
% before drifting, then create a trajectory to achieve that r_max at that
% speed 
% once drifting start as predicted, find the corresponding equilibrium
% (with front break --- solve the a trajectory that results in a 90 degree
% beta angle when beta_dot is zero

% set input bounds
delta_max = 38 * pi / 180; % [rad]
V_dot_max = 5.0; % m/s^2

% discrete dynamics for bike
f_bike = @bike_f_disc;

% desired drifting velocity
V_start = 0.1; %m/s
% V_drift = 7;

%% drift equilibrium trajectory
equil = getDriftState_w_V_beta(V_drift, beta_des, veh, tire_f, tire_r);
S.Fxr_opt = equil.Fxr;
S.delta_opt = equil.delta;
S.r_opt = equil.r;

% create a nonlinear r profile


% integration vars
% dT = 0.01;
t_s = [];
t_curr = 0;
x = [0,0,0,V_start]; %[x_m, y_m, yaw_rad, V_mps]
u = [V_dot_max, 0]; %[V_dot, delta]
V_curr = x(4);

% accelearate to the drifting speed
n_t_drift = ceil((11 - 2 - V_start)./(V_dot_max.*dT));
V_dot_curr = ((V_drift - 2 - V_start)./n_t_drift)./dT;
u = [V_dot_curr, 0];

for i = 1:n_t_drift
    t_s = [t_s; t_curr];
    x_curr = x(end,:)';
    u_curr = u(end,:)';
    x_next = f_bike(x_curr, u_curr, dT,[]);

    x = [x;x_next'];
    if i == n_t_drift
        u = [u;[V_dot_max, 0]];
    else
        u = [u;[V_dot_curr, 0]];
    end
    t_curr = t_curr + dT;
%     disp('percent speed reached '+string(round(V_curr/V_drift*100)))
    V_curr = x(end,4);
end

% isDriftSpeed = false;
% while ~isDriftSpeed
%     if V_curr + V_dot_max.*dT > V_drift - 2
%         dT_drift = (V_drift - 2 - V_curr)./V_dot_max;
%         isDriftSpeed = true;
%     else
%         dT_drift = dT;
%     end
%     t_s = [t_s; t_curr];
%     x_curr = x(end,:)';
%     u_curr = u(end,:)';
%     x_next = f_bike(x_curr, u_curr, dT_drift,[]);
% 
%     x = [x;x_next'];
%     u = [u;[V_dot_max, 0]];
%     t_curr = t_curr + dT_drift;
% 
%     V_curr = x(end,4);
% end

% perturb distablilize then turn into a drift
sim_duration = 4;
dist_duration = 0.05;
r_dist = -S.r_opt*3;

t_sim = sim_duration + t_curr;
t_start = t_curr;
r_goal = r_dist;

while t_curr <= t_sim
    t_s = [t_s; t_curr];
    x_curr = x(end,:)';
    u_curr = u(end,:)';
    x_next = f_bike(x_curr, u_curr, dT,[]);

    V_curr = x(end,4);
    if t_curr > t_start + dist_duration
        r_goal = S.r_opt;
    end

    delta = atan2(r_goal*veh.L, V_curr);
    delta = bound_values(delta, -delta_max, delta_max);
    x = [x;x_next'];
    u = [u;[V_dot_max, delta]];
    t_curr = t_curr + dT;
end

x = x(1:end-1,:);
u = u(1:end-1,:);

S.x = x(:,1);
S.y = x(:,2);
S.yaw = x(:,3);
S.V = x(:,4);
S.t_s = t_s;
S.V_dot = u(:,1);
S.delta = u(:,2);
% 
% figure;
% plot(S.x, S.y)
% xlabel('x (m)')
% ylabel('y (m)')
% title('Parallel Parking Trajectory')
% 
% figure;
% plot(t_s, S.V)
% xlabel('t (s)')
% ylabel('V (m/s)')
end