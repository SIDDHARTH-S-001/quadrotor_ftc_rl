function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis, fault_params] = quadcopter_package_select_trajectory(path_number, varargin)
%QUADCOPTER_SELECT_TRAJECTORY Obtain parameters for selected quadcopter trajectory with fault injection
%   [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis, fault_params] = 
%       quadcopter_package_select_trajectory(path_number, roundtrip)
%
%   Inputs:
%       path_number - ID of trajectory (1-6)
%       roundtrip   - (Optional) Boolean for round trip trajectory
%
%   Outputs:
%       waypoints      - Key x-y-z locations
%       timespot_spl   - Time points along spline
%       spline_data    - Spline interpolation points  
%       spline_yaw     - Yaw angles at spline points
%       wayp_path_vis  - Visualization points
%       fault_params   - Fault configuration struct (NEW)
%
%   Fault Parameters:
%       probability    - Chance of motor fault (0-1)
%       severity_range - [min,max] speed reduction during fault
%       active         - Enable/disable faults globally

% Copyright 2021-2022 The MathWorks, Inc.
% Modified 2023 for Fault-Tolerant RL Control

% Handle optional roundtrip parameter
if nargin == 2
    roundtrip = varargin{1};
else
    roundtrip = false;
end

% Define fault parameters (NEW)
fault_params = struct(...
    'probability', 0.1, ...       % 10% chance of fault
    'severity_range', [0.3, 0.8], ... % 30-80% speed during fault
    'active', true, ...           % Global fault toggle
    'fault_zones', []);           % Reserved for future use

% Select trajectory profile
switch path_number
    case 1 % Basic delivery with straight approach
        waypoints = [ 
            -2    -2  0  2  5;
            -2    -2  0  0  0;
            0.14  6   6  6  0.14];
        max_speed = 1;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;
        
        % Mark approach zone as high-probability fault area (NEW)
        fault_params.fault_zones = [3, 4; % Waypoint indices
                                0.3];   % Increased fault probability

    case 2 % Square pattern with sharp turns
        waypoints = [
            -2    -2  -2  -2  -2  2  2;
            -2    -2  -2   2   2  2  2;
            0.15  6    6   6   6  6  0.15];
        max_speed = 1;
        min_speed = 0.1;
        xApproach = [2 0.5];
        vApproach = 0.1;
        
        % Higher fault probability during turns (NEW)
        fault_params.fault_zones = [2, 3; 4, 5;
                                0.25]; 

    case 3 % Vertical profile (predefined)
        waypoints = [
            -2   -2    -2     -2  -2  -2 -2  -2  -2 -2 -2 -2 -2 -2;
            -2   -2    -2     -2  -2  -2 -2  -2  -2 -2 -2 -2  0  0;
            0.15 0.15  0.15   4   4   4  4   4   4  4  4  4  4  0.14];
        spline_data = waypoints';
        timespot_spl = [0:4:11*4 11*4+6 11*4+6+6]';
        spline_yaw = [0 0 0 pi/4 pi/4 pi/4 0 0 0 -pi/4 -pi/4 -pi/4 -pi/4 -pi/4];
        
        % No automatic fault zones for predefined trajectories

    case 4 % Complex 3D path
        waypoints = [
            -3.0000  0.5633  4.5492  7.7662  9.0011  7.3491  3.7145  -0.0156  2.2687  5.0000;
            -5.0000 -4.4724 -4.5758 -2.3910  1.5272  5.3013  6.5986  6.8774  9.5797  8.0000;
            0.1500  6.0000  6.0000  6.0000  6.0000  6.0000  6.0000  6.0000  6.0000  0.15];
        max_speed = 1;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;

    case 5 % Long horizontal path
        waypoints = [ 
            0    0  50   50  100   100  150  150  150;
            0    0   0   50   50   100  100  150  150;
            0.15 6   6    6    6     6    6    6  0.14];
        max_speed = 2;
        min_speed = 0.1;
        xApproach = [4 1];
        vApproach = 0.1;

    case 6 % Diagonal shortcut
        waypoints = [ 
            0    0  150  150  150;
            0    0    0  150  150;
            0.15 6    6    6  0.14];
        max_speed = 2;
        min_speed = 0.1;
        xApproach = [4 1];
        vApproach = 0.1;

    otherwise
        error('Invalid path_number. Must be 1-6.');
end

% Generate trajectory if not predefined (case 3)
if exist("xApproach", "var")
    if roundtrip
        [timespot_spl_re, spline_data_re, spline_yaw_re, ~] = ...
            quadcopter_waypoints_to_trajectory(...
            fliplr(waypoints), max_speed, min_speed, xApproach, vApproach);

        [timespot_spl_to, spline_data_to, spline_yaw_to, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints, max_speed, min_speed, xApproach, vApproach);
        
        pause_at_target = 5; % sec
        timespot_spl = [timespot_spl_to; timespot_spl_re+timespot_spl_to(end)+pause_at_target];
        spline_data = [spline_data_to; spline_data_re];
        spline_yaw = unwrap([spline_yaw_to spline_yaw_re], 1.5*pi);
    else
        [timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints, max_speed, min_speed, xApproach, vApproach);
    end
else
    % For predefined trajectories (case 3)
    wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
    if roundtrip
        spline_data = [spline_data; flipud(spline_data)];
        timespot_spl = [timespot_spl; timespot_spl(end)+5; ...
            timespot_spl(end)+5+cumsum(flipud(diff(timespot_spl)))];
        spline_yaw = unwrap([spline_yaw flipud(spline_yaw)+pi], 1.5*pi);
    end
end
end