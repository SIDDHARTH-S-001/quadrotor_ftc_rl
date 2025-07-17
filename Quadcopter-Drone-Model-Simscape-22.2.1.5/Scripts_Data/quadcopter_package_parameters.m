% Parameters for quadcopter_package_delivery with RL Fault-Tolerant Control
% Copyright 2021-2022 The MathWorks, Inc.
% Modified 2023 for Reinforcement Learning Integration

%% Environment Parameters (Unchanged)
planex = 12.5;           % m
planey = 8.5;            % m
planedepth = 0.2;        % m
battery_capacity = 7.6*3; % Ah

%% Physical Properties (Unchanged)
rho_pla = 1.25;          % g/cm^3
drone_mass = 1.2726;     % kg
pkgSize = [1 1 1]*0.14;  % m
pkgDensity = 1/prod(pkgSize); % kg/m^3

%% Contact Parameters (Unchanged)
pkgGrndStiff = 1000;
pkgGrndDamp = 300;
pkgGrndTransW = 1e-3;

%% Propulsion System (Added fault parameters)
propeller = struct(...
    'diameter', 0.254, ...    % m
    'Kthrust', 0.1072, ...
    'Kdrag', 0.01, ...
    'fault_thrust_loss', 0.5); % Max thrust loss during faults (NEW)

qc_motor = struct(...
    'max_torque', 0.8, ...    % N*m
    'max_power', 160, ...     % W
    'time_const', 0.02, ...   % sec
    'efficiency', 25/30*100, ... % %
    'efficiency_spd', 5000, ...  % rpm
    'efficiency_trq', 0.05, ...  % N*m
    'rotor_damping', 1e-7, ...   % N*m/(rad/s)
    'fault_efficiency_range', [0.3, 0.8]); % Efficiency during faults (NEW)

qc_max_power = qc_motor.max_power;

%% Aerodynamics (Unchanged)
air_rho = 1.225;         % kg/m^3
air_temperature = 273+25; % K
wind_speed = 0;          % m/s

%% RL-Tunable Controller Parameters (MODIFIED)
% Base PID values remain unchanged for compatibility
% Added ranges and scaling factors for RL tuning

% Position Controller 
pos_ctrl = struct(...
    'base_kp', 8, ...        % Original value
    'base_ki', 0.04, ...
    'base_kd', 3.2, ...
    'kp_range', [4, 12], ... % RL tuning range (NEW)
    'ki_range', [0.01, 0.08], ...
    'kd_range', [1.6, 6.4], ...
    'filtM', 0.005, ...
    'filtD', 100, ...
    'pos2attitude', 2.4);

% Attitude Controller
att_ctrl = struct(...
    'base_kp', 128.505, ...
    'base_ki', 5.9203, ...
    'base_kd', 156.4, ... % 78.2000*2
    'kp_range', [64, 256], ...
    'ki_range', [2, 10], ...
    'kd_range', [80, 320], ...
    'filtM', 0.01, ...
    'filtD', 1000, ...
    'limit', 800);

% Yaw Controller
yaw_ctrl = struct(...
    'base_kp', 205.608, ... % 25.7010*4*2
    'base_ki', 0.059203, ...
    'base_kd', 0.782, ...
    'kp_range', [100, 400], ...
    'ki_range', [0.01, 0.1], ...
    'kd_range', [0.1, 1.5], ...
    'filtM', 0.01, ...
    'filtD', 100, ...
    'limit', 20);

% Altitude Controller
alt_ctrl = struct(...
    'base_kp', 0.27, ...
    'base_ki', 0.07, ...
    'base_kd', 0.35, ...
    'kp_range', [0.1, 0.5], ...
    'ki_range', [0.02, 0.15], ...
    'kd_range', [0.1, 0.7], ...
    'filtM', 0.05, ...
    'filtD', 10000, ...
    'limit', 10);

% Motor Controller
motor_ctrl = struct(...
    'base_kp', 0.00375, ...
    'base_ki', 4.5e-4, ...
    'base_kd', 0, ...
    'kp_range', [0.001, 0.007], ...
    'ki_range', [1e-4, 1e-3], ...
    'kd_range', [0, 0.001], ... % Disabled
    'filtD', 10000, ...
    'filtSpd', 0.001, ...
    'limit', 0.25);

%% RL Agent Configuration (NEW)
rl_params = struct(...
    'action_scale', [0.8, 1.2], ... % Multiplicative gain adjustment range
    'update_rate', 0.1, ...         % Seconds between updates
    'max_delta_k', 0.05, ...        % Max gain change per step
    'observation_noise', 0.01);      % Measurement noise std dev

%% Drag Coefficients (Unchanged)
qd_drag.Cd_X = 0.35;
qd_drag.Cd_Y = 0.35;
qd_drag.Cd_Z = 0.6;
qd_drag.Roll = 0.2;
qd_drag.Pitch = 0.2;
qd_drag.Yaw = 0.2;

%% Structural Parameters (Unchanged)
drone_leg.Extr_Data = flipud([...
    0     0;
    0.5   0;
    1    -1;
    0.98 -1;
    0.5  -0.02;
   -0.5  -0.02;
   -0.98 -1;
   -1    -1;
   -0.5   0].*[1 1]*0.15);
drone_leg.width = 0.01;

%% Legacy Parameter Mapping (For Backward Compatibility)
% Ensures existing Simulink models continue working
filtM_position = pos_ctrl.filtM;
kp_position = pos_ctrl.base_kp; 
ki_position = pos_ctrl.base_ki;
kd_position = pos_ctrl.base_kd;
filtD_position = pos_ctrl.filtD;
pos2attitude = pos_ctrl.pos2attitude;

filtM_attitude = att_ctrl.filtM;
kp_attitude = att_ctrl.base_kp;
ki_attitude = att_ctrl.base_ki;
kd_attitude = att_ctrl.base_kd;
filtD_attitude = att_ctrl.filtD;
limit_attitude = att_ctrl.limit;

filtM_yaw = yaw_ctrl.filtM;
kp_yaw = yaw_ctrl.base_kp;
ki_yaw = yaw_ctrl.base_ki;
kd_yaw = yaw_ctrl.base_kd;
filtD_yaw = yaw_ctrl.filtD;
limit_yaw = yaw_ctrl.limit;

filtM_altitude = alt_ctrl.filtM;
kp_altitude = alt_ctrl.base_kp;
ki_altitude = alt_ctrl.base_ki;
kd_altitude = alt_ctrl.base_kd;
filtD_altitude = alt_ctrl.filtD;
limit_altitude = alt_ctrl.limit;

kp_motor = motor_ctrl.base_kp;
ki_motor = motor_ctrl.base_ki;
kd_motor = motor_ctrl.base_kd;
filtD_motor = motor_ctrl.filtD;
filtSpd_motor = motor_ctrl.filtSpd;
limit_motor = motor_ctrl.limit;