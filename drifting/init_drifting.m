%% INITIALIZATION SCRIPT FOR DRIFTING
% Modify all user-defined hyperparameters for NeuralPARC in this script.
%
% This script is written for the drift-parking example.
%
% REQUIRED VARIABLES:
% --- directory options ---
% demo_name: char; beginning of the file name for this example
% --- timing options ---
% dt: double; timestep for NeuralPARC
% t_f: double; final time for NeuralPARC
% dt_data: double; timestep for collected data
% --- domain options ---
% p_0_lo: n_p*1 double; lower bound of workspace positions;
%         empty if P_0 = \R^{n_p}
% p_0_hi: n_p*1 double; upper bound of workspace positions;
%         empty if P_0 = \R^{n_p}
% k_lo: n_k*1 double; lower bound of trajectory parameters
% k_hi: n_k*1 double; upper bound of trajectory parameters
% --- training options ---
% validation_ratio: int; ratio of data to be used for validation
% network_size: 1*(d - 1) int; hidden layer size of neural network
% solverName: see MATLAB's documentation for trainingOptions
% ExecutionEnvironment: see MATLAB's documentation for trainingOptions
% MaxEpochs: see MATLAB's documentation for trainingOptions
% InitialLearnRate: see MATLAB's documentation for trainingOptions
% LearnRateSchedule: see MATLAB's documentation for trainingOptions
% LearnRateDropPeriod: see MATLAB's documentation for trainingOptions
% LearnRateDropFactor: see MATLAB's documentation for trainingOptions
% ValidationFrequency: see MATLAB's documentation for trainingOptions
% Plots: see MATLAB's documentation for trainingOptions
% --- binning options ---
% ns_bin: 1*n_k double; number of bins for each trajectory parameter
% --- environment options ---
% G: MPT3 Polyhedron; (n_p + n_q)-dimensional goal set
% O: n_O*1 MPT3 Polyhedron; n_p-dimsneional obstacles
% B: MPT3 Polyhedron; Overapproximation of the circular sweep of the agent
% --- NeuralPARC options ---
% isParfor: boolean; true if using parfor
% isAnytime: boolean; true if terminate early when one safe sample is found
% n_try: int; number of samples to safety check in each PWA region
%
% RETURNS:
% All required variables: see above
% --- directory options ---
% base_dir: char; directory of this file
% --- timing options ---
% ts: 1*n_t double; time series for NeuralPARC
% ts_data: 1*n_t_data double; time series for collected data
% n_t: int; number of timesteps for NeuralPARC
% n_t_data: int; number of timesteps for collected data
% --- domain options ---
% n_k: int; number of trajectory parameters
% --- training options ---
% d: int; depth of neural network
% --- binning options ---
% n_bin: int; total number of bins
% isBinning: boolean; true if binning is enabled
% K_bin: 1*n_bin MPT3 Polyhedron; binned trajectory parameter space
% --- environment options ---
% n_O: int; number of obstacles
% n_p: int; number of workspace dimensions
% n_q: int; number of non-workspace goal states of interest
%
% Author:  Long Kiu Chung
% Created: 2024/07/24
% Updated: 2024/10/01

fprintf("Initializing...\n");

%% Directory Options
% User-defined parameters: demo_name

% For saving
demo_name = 'drifting';
base_dir = fileparts(mfilename('fullpath')); % Get the directory of init

%% Timing Options
% User-defined parameters: dt, t_f, dt_data

% For set computation
dt = 0.1; % Timestep
t_f = 7.8; % Final time

% For error bound
dt_data = 0.01; % Actual timestep

% Time series
ts = 0:dt:t_f;
ts_data = 0:dt_data:t_f;

% Dimensions
n_t = length(ts);
n_t_data = length(ts_data);

%% Domain Options
% User-defined parameters: p_0_lo, p_0_hi, k_lo, k_hi

% Workspace domains (set as [] if P_0 = \R^{n_p})
p_0_lo = [];
p_0_hi = [];

% Parameter domains
% For drifting, k(1) is v, desired velocity to perform drifting
%               k(2) is theta_beta, desired angle to perform braking
% Lower bound
k_lo = [9; pi./6];

% Upper bound
k_hi = [11; (2./9).*pi];

% Dimensions
n_k = size(k_lo, 1);

%% Training Options
% User-defined parameters: validation_ratio, network_size, solverName, 
%                          ExecutionEnvironment, MaxEpochs, 
%                          InitialLearnRate, LearnRateSchedule,
%                          LearnRateDropPeriod, LearnRateDropFactor,
%                          ValidationFrequency, Plots

% For validation
validation_ratio = 0.1;

% Hidden layer size
network_size = [8, 8, 8, 8];
d = length(network_size) + 1; % depth

% MATLAB trainingOptions
% See MATLAB's documentation for trainingOptions for details
solverName = 'adam';
ExecutionEnvironment = 'gpu';
MaxEpochs = 500;
InitialLearnRate = 0.01;
LearnRateSchedule = 'piecewise';
LearnRateDropPeriod = 100;
LearnRateDropFactor = 0.2;
ValidationFrequency = 50;
Plots = 'training-progress';

%% Binning Options
% User-defined parameters: ns_bin

% Number of bins for each trajectory parameter
ns_bin = [1, 1]; 

% Determine if binning is necessary
n_bin = prod(ns_bin); % Total number of bins
isBinning = n_bin > 1;

% Create the bins
K_bin = create_bins(k_lo, k_hi, ns_bin);

%% Environment Options
% User-defined parameters: G, O, B

% Goal set
G = Polyhedron('A', [eye(3); -eye(3)], ...
               'b', [31.3; -15.2; -(5./6).*pi; -29.7; 16.8; (7./6).*pi]);

% Obstacles
O = [Polyhedron('A', [eye(2); -eye(2)], ...
                'b', [27.2335; -15.006; -22.9665; 16.994]), ...
     Polyhedron('A', [eye(2); -eye(2)], ...
                'b', [38.0335; -15.006; -33.7665; 16.994])];

% Agent volume
B = polygon_sweep(4.267, 1.988, 10);

% Dimensions
n_O = numel(O);
n_p = O(1).Dim;
n_q = G.Dim - n_p;

%% NeuralPARC Options
% User-defined parameters: isParfor, isAnytime

isParfor = true; % Requires Parallel Computing Toolbox
isAnytime = false; % Terminate early if safe sample found
n_try = 50; % Number of samples to check in each PWA region

%% Sanity Check
% Timing options
assert(mod(t_f, dt) == 0, 't_f not divisible by dt!');
assert(mod(t_f, dt_data) == 0, 't_f not divisible by dt_data!');

% Domain options
assert(all(size(p_0_lo) == size(p_0_hi)), ...
       'Size of p_0_lo different from p_0_hi!');
assert(all(size(k_lo) == size(k_hi)), 'Size of k_lo different from k_hi!');

% Binning options
assert(length(ns_bin) == n_k, 'Bins not set up correctly!');

% Environment Options
assert(n_q >= 0, 'Dimension of G is not right!');
if ~isempty(p_0_lo)
    assert(all(size(p_0_lo, 1) == [O.Dim]), ...
           'Dimension of O is not right!');
end
assert(B.Dim == n_p, 'Dimension of B is not right');