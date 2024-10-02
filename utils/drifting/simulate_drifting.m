function [p_xs, p_ys, thetas] = simulate_drifting(p_0, k, ts)
% Simulate a drifting trajectory. 
%
% See Appendix I of Chung, Long Kiu, et al. "Goal-Reaching Trajectory 
% Design Near Danger with Piecewise Affine Reach-Avoid Computation". 
% Proceedings of Robotics: Science and Systems, 2024.
%
% Much of this code is based on collectDriftParkingTraj.m, a legacy code
% written by Chuizheng Kong. Direct all questions for this code to him.
%
% INPUTS:
% p_0: 2*1 double; (p_0(1), p_0(2)) is the x- and y-coordinates of the
%      car's initial position
% k: 2*1 double; k(1) is v, desired velocity to perform drifting; k(2) is 
%    theta_beta, desired angle to perform braking
% ts: 1*n_t double; desired timestamps of the collected trajectory
%
% OUTPUTS:
% p_xs: 1*n_t double; x-coordinate of the trajectory at ts
% p_ys: 1*n_t double; y-coordinate of the trajectory at ts
% thetas: 1*n_t double; heading angle of the trajectory at ts
%
% Authors: Long Kiu Chung
% Created: 2024/09/27
% Updated: 2024/09/27

%% Drifting hyperparameters
% Trajectory options
theta_beta_traj = deg2rad(15); % rad
dt_drift = 0.1; % s (Not to be confused with dt_data)
t_final = 4; % s (Not to be confused with t_f)

% Vehicle options
veh.Caf = 60000; % N/rad
veh.Car = 160000; % N/rad
veh.m = 1450; % kg
veh.Iz = 2300; % m^2
veh.L = 2.4; % m
veh.a = 0.67*veh.L; % m
veh.b = veh.L-veh.a; % m
veh.mu = 1.1; % dimensionless
veh.body_L = 4.267; % m
veh.body_W = 1.988; % m

% Front tire options
tire_f.Ca = veh.Caf; % N/rad
tire_f.mu = 1.1; % dimensionless

% Rear tire options
tire_r.Ca = veh.Car; % n/rad
tire_r.mu = 1.1; % dimensionless

% Static normal load
g = 9.81; % m/s^2
tire_f.Fz = veh.m.*g.*veh.b./veh.L; % N
tire_r.Fz = veh.m.*g.*veh.a./veh.L; % N

% Input bound options
delta_max = deg2rad(38); % rad
Fxf_max = min(tire_f.mu.*tire_f.Fz-1, 7684); % N*6684
Fxr_max = min(tire_r.mu.*tire_r.Fz-1, 7684); % N
bounds = [Fxr_max, delta_max, Fxf_max];

% MPC options
% Set the mpc horizon
param.t_horizon = 0.5; % seconds

% Set max numebr of iterations
param.n_iterations = 50;

% Define weighting matrices
param.Q_k = 70*eye(4);
param.R_k = 30*eye(2);
param.Q_T = 100*eye(4);

% There are no physical parameters to adjust
param.parameters = [];

%% Extract variables
v = k(1); % desired velocity to perform drifting
theta_beta = k(2); % desired angle to perform braking

%% Define agents
% Before drifting
A_predrift = bicycle_agent_predrifting(veh, tire_f, tire_r, bounds, ...
                                       dt_drift);
A_predrift.visual = false;

% During drifting
A_drift = bicycle_agent_drifting(veh, tire_f, tire_r, bounds, dt_drift);
A_drift.visual = false;

%% Getting to the drift states
S = genStartPath_continuous(v, theta_beta_traj, dt_drift, veh, tire_f, ...
                            tire_r);

t_p = S.t_s;
    
init_states = [S.x(1); S.y(1); S.yaw(1); S.V(1)]; % Define the initial state
target_states = [S.x, S.y, S.yaw, S.V];% [N,4] tall matrix

% Set initial guess for input
initial_guess = [S.V_dot, S.delta];

% Simulate predrifting trajectory
A_predrift.simulate(t_p, init_states,initial_guess, target_states, param);

% Detect if desired drifting slide slip angle beta was achieved
[point_found, t_span, ...
 states, states_dot, ...
 inputs, fiala_tire_vars] = find_switching_point(A_predrift, theta_beta);

if ~point_found
    error("k(2) did not matched the desired drifting point!")
end

%% Begin drifting
%Control parameters using equilibrium method
V_des = states(end, 5);
beta_des = states(end, 6);

equil = getDriftState_w_V_beta(V_des, beta_des, veh, tire_f, tire_r);

Fxr_ff = equil.Fxr;
delta_ff = equil.delta;
r_des = equil.r;

Ux_des = V_des.*cos(beta_des); %8
Uy_des = V_des.*sin(beta_des); % -4.34

% initial path condition same as the reference
r_radps = states(end,4); % 1.19
uy_mps = Uy_des; % -4.34
ux_mps = Ux_des; % 8
beta_rad = beta_des;
V_mps = V_des;
x_m = states(end,1);
y_m = states(end,2);
yaw_rad = states(end,3);

init_states = [x_m(1); y_m(1); yaw_rad(1);
               r_radps(1); V_mps(1); beta_rad(1);
               ux_mps(1); uy_mps(1)];
input_guess = [Fxr_ff; delta_ff];
des_rVb_states = [r_des; V_des; beta_des];
prev_inputs = inputs(end,:);

% simulate drifting and parking trajectory
A_drift.simulate(t_final,init_states, input_guess, des_rVb_states, ...
                 prev_inputs);

%% Data logging
t_actual = [t_span(1:(end-1)); A_drift.t_s + t_span(end)];
states_actual = [states(1:(end-1),1:3);A_drift.states(:,1:3)];

if t_actual(end) > ts(end)
    error("Set t_f to a higher number!")
end

% Use linear interpolation to match trajectories to desired timesteps
states = transpose(match_trajectories(ts, t_actual', states_actual'));

%% Return results
% Shift workspace states by translation invariance
p_xs = states(:, 1)' + p_0(1);
p_ys = states(:, 2)' + p_0(2);
thetas = states(:, 3)';
end

