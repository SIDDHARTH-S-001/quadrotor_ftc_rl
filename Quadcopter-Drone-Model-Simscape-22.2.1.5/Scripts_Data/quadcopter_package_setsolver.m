function solverBlock_pth = quadcopter_package_setsolver(mdl, deskreal, varargin)
%QUADCOPTER_PACKAGE_SETSOLVER Configure solvers for quadcopter simulation
%   solverBlock_pth = quadcopter_package_setsolver(mdl, deskreal)
%   solverBlock_pth = quadcopter_package_setsolver(mdl, deskreal, rl_mode)
%
%   Inputs:
%       mdl      - Model name
%       deskreal - 'desktop' or 'realtime'
%       rl_mode  - (Optional) true for RL tuning mode
%
%   Outputs:
%       solverBlock_pth - Path to solver blocks
%
%   RL Mode:
%       - Uses fixed-step solver regardless of deskreal setting
%       - Optimized for real-time PID tuning
%       - Reduced solver overhead for faster RL training

% Copyright 2011-2022 The MathWorks, Inc.
% Modified 2023 for Reinforcement Learning Integration

% Default solver configuration
desktop_solver = 'ode23t';
rl_solver = 'ode8';  % Higher order fixed-step for RL (NEW)

% Real-time configuration
realtime_nonlinIter = '2';
realtime_stepSize = '0.01';
rl_stepSize = '0.005';  % Smaller step for RL (NEW)
realtime_localSolver = 'NE_BACKWARD_EULER_ADVANCER';
realtime_globalSolver = 'ode14x';

% Handle optional RL mode
if nargin > 2 && varargin{1}
    rl_mode = true;
    fprintf('RL mode: Using fixed-step solver (ode8) with 5ms step size\n');
else
    rl_mode = false;
end

% Find all solver blocks
f = Simulink.FindOptions('FollowLinks', 1, 'LookUnderMasks', 'all');
solverBlock_pth = Simulink.findBlocks(bdroot, 'SubClassName', 'solver', f);

if rl_mode
    % RL-optimized configuration (NEW)
    set_param(mdl, ...
        'Solver', rl_solver, ...
        'FixedStep', rl_stepSize, ...
        'ReturnWorkspaceOutputs', 'on', ...
        'SignalLogging', 'on');
    
    for svb_i = 1:size(solverBlock_pth, 1)
        set_param(solverBlock_pth(svb_i), ...
            'UseLocalSolver', 'on', ...
            'DoFixedCost', 'on', ...
            'MaxNonlinIter', '1', ...  % Reduced for speed
            'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
            'LocalSolverSampleTime', rl_stepSize);
    end
    
    % Enable continuous logging for RL
    set_param([mdl '/Quadcopter/Load/Disengage Logic'], 'checkbox_stop_release', 'off');
    
elseif strcmpi(deskreal, 'desktop')
    % Original desktop configuration
    set_param(mdl, 'Solver', desktop_solver);
    for svb_i = 1:size(solverBlock_pth, 1)
        set_param(solverBlock_pth(svb_i), ...
            'UseLocalSolver', 'off', ...
            'DoFixedCost', 'off');
    end
    set_param([mdl '/Quadcopter/Load/Disengage Logic'], 'checkbox_stop_release', 'off');
    
else
    % Original real-time configuration
    set_param(mdl, ...
        'Solver', realtime_globalSolver, ...
        'FixedStep', realtime_stepSize);
    
    for svb_i = 1:size(solverBlock_pth, 1)
        set_param(solverBlock_pth(svb_i), ...
            'UseLocalSolver', 'on', ...
            'DoFixedCost', 'on', ...
            'MaxNonlinIter', realtime_nonlinIter, ...
            'LocalSolverChoice', realtime_localSolver, ...
            'LocalSolverSampleTime', realtime_stepSize);
    end
    set_param([mdl '/Quadcopter/Load/Disengage Logic'], 'checkbox_stop_release', 'on');
end