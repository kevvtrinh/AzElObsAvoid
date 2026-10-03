function [result, resultFullRange] = exampleGroundKeepOutAreaSlew(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   [result, resultFullRange] = exampleGroundKeepOutAreaSlew()
%   [result, resultFullRange] = exampleGroundKeepOutAreaSlew(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Slew an az/el camera between two area scans around a ground keep-out
%     that moves through the mounted-sensor frame as the satellite passes.
%   - Taken from RegressionTest_singleSatelliteAreaSlewAvoidance (pass over
%     the central US, 1020 km, 45 deg inclination) with no STK required.
%   - With default controls, workspaces give different motions; route corner offset =
%     largest workspace side / 64 (1.26 deg local, 5.47 deg full). BMTP also gets
%     the workspace. Full-range motion dips below the local elevation bound;
%     neither plan reaches its own workspace limits.
%   - Requires Aerospace Toolbox (lla2ecef and angle2dcm).
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%       FigureVisible = "off" allows plotting in a batch run.
%**************************************************************************
% OUTPUTS
%   - result, resultFullRange (scalar structs)
%       Unmodified public planner results for the local workspace and full
%       gimbal range. Ordinary planning failure returns Success = false;
%       invalid input throws an error.
%**************************************************************************
% UNITS
%   - Position is [azimuth elevation] in degrees; time is seconds after the
%     first scan ends (scenario epoch 8415 s).
%   - Derivative limits use deg/s, deg/s^2, and deg/s^3. Satellite Earth-fixed
%     positions are kilometers; latitude, longitude, and attitude are degrees.
%**************************************************************************

%% Section 1: Resolve Example Controls
if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
scenarioDefaults = struct( ...
    "GoalTimeMode", "fixedArrival", ...
    "SampleTime_s", 1, ...
    "Title",        "Ground keep-out area slew (STK-free)");
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, scenarioDefaults, [1e6 1e6]);

%% Section 2: Create Obstacles
missionEndTime_s   = 120;
safetyMargin_units = 1;

% Keep-out ground boundary [latitude longitude], a 5 x 5 deg box.
keepOutLatLon_deg = [42.7 -90.5; 42.7 -85.5; 47.7 -85.5; 47.7 -90.5];

% Satellite Earth-fixed position (km) and Earth-fixed -> body Euler 321
% attitude (deg) from STK, every 10 s. The camera frame is the body frame.
% Columns: time, x, y, z, yaw, pitch, roll.
satellitePose = [ ...
     -10    -936.0067  -5725.0293   4591.3355  1048.66774   25.37336  586.61749; ...
       0    -877.7903  -5754.0662   4566.4688  1049.07664   25.76367  586.73483; ...
      10    -819.5302  -5782.6243   4541.1516  1049.48733   26.15369  586.85567; ...
      20    -761.2315  -5810.7009   4515.3863  1049.89988   26.54339  586.98004; ...
      30    -702.8992  -5838.2933   4489.1754  1050.31434   26.93275  587.10798; ...
      40    -644.5382  -5865.3989   4462.5215  1050.73076   27.32176  587.23950; ...
      50    -586.1536  -5892.0150   4435.4272  1051.14921   27.71038  587.37465; ...
      60    -527.7503  -5918.1390   4407.8953  1051.56974   28.09859  587.51344; ...
      70    -469.3335  -5943.7685   4379.9284  1051.99241   28.48637  587.65592; ...
      80    -410.9081  -5968.9009   4351.5294  1052.41728   28.87369  587.80212; ...
      90    -352.4790  -5993.5337   4322.7009  1052.84441   29.26054  587.95206; ...
     100    -294.0514  -6017.6646   4293.4459  1053.27386   29.64689  588.10579; ...
     110    -235.6301  -6041.2912   4263.7672  1053.70569   30.03272  588.26334; ...
     120    -177.2201  -6064.4111   4233.6677  1054.13997   30.41800  588.42474; ...
     130    -118.8265  -6087.0220   4203.1506  1054.57674   30.80271  588.59004];

% Project the ground box into camera az/el, then apply the margin once.
obstacleTime_s = satellitePose(:, 1);
angles_rad     = deg2rad(satellitePose(:, 5:7));
CfixedToCamera = angle2dcm(angles_rad(:, 1), angles_rad(:, 2), angles_rad(:, 3), 'ZYX');
keepOutAzEl    = calculateAreaTargetAzEl("Ground keep-out", keepOutLatLon_deg, ...
    obstacleTime_s, satellitePose(:, 2:4), CfixedToCamera, 0.5);
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    keepOutAzEl, safetyMargin_units);

%% Section 3: Create Planner Inputs
% Endpoint motion is the scan pattern's: the first scan ends moving down
% and left, so the slew starts with downward momentum.
initialState = struct( ...
    "time_s",                0, ...
    "position_units",        [-120.34632 48.151971], ...
    "velocity_units_s",      [-0.684975 -0.878125], ...
    "acceleration_units_s2", [-1e-06 -0.3]);
goalState = struct( ...
    "time_s",                missionEndTime_s, ...
    "position_units",        [-77.652454 40.426115], ...
    "velocity_units_s",      [-0.14632 0.100051], ...
    "acceleration_units_s2", [-0.277712 0.173434]);
fullRangeLimits = struct( ...
    "xInterval_units",          [-175 175], ...
    "yInterval_units",          [0 90], ...
    "maxVelocity_units_s",      [3 3], ...
    "maxAcceleration_units_s2", [0.3 0.3], ...
    "maxJerk_units_s3",         displayOptions.MaxJerk_units_s3);

% SmartSensor's workspace: endpoints and PROTECTED keep-out samples, padded
% 5 deg. Use x_units and y_units, which already include the 1 deg margin.
boxPad_units = 5;
boxPoints_units = [initialState.position_units; goalState.position_units; ...
    vertcat(obstacles.x_units{:}), vertcat(obstacles.y_units{:})];
localLimits = fullRangeLimits;
localLimits.xInterval_units = ...
    [min(boxPoints_units(:, 1)), max(boxPoints_units(:, 1))] + ...
    [-boxPad_units boxPad_units];
localLimits.yInterval_units = ...
    [min(boxPoints_units(:, 2)), max(boxPoints_units(:, 2))] + ...
    [-boxPad_units boxPad_units];

%% Section 4: Run Planner
% Compare both workspaces using the same obstacle and endpoint motion inputs.
result          = planner(obstacles, initialState, goalState, localLimits, options);
resultFullRange = planner(obstacles, initialState, goalState, fullRangeLimits, options);
if displayOptions.Verbose
    workspaceResults = {result, resultFullRange};
    workspaceLabels  = ["Local workspace", "Full gimbal range"];
    for workspaceIndex = 1:numel(workspaceResults)
        workspaceResult = workspaceResults{workspaceIndex};
        if workspaceResult.Success && ~isempty(workspaceResult.position_units)
            fprintf(['%s: Success=%d, lowest elevation %.2f deg, ' ...
                'motion length %.2f deg.\n'], workspaceLabels(workspaceIndex), ...
                workspaceResult.Success, min(workspaceResult.position_units(:, 2)), ...
                workspaceResult.MotionLength_units);
        else
            fprintf('%s: Success=%d, TerminationReason=%s.\n', ...
                workspaceLabels(workspaceIndex), workspaceResult.Success, ...
                workspaceResult.TerminationReason);
        end
    end
end

%% Section 5: Validate Result
localValidation     = obstacleAvoidance.validateTrajectory(result);
fullRangeValidation = obstacleAvoidance.validateTrajectory(resultFullRange);
if ~localValidation.Passed
    warning("exampleGroundKeepOutAreaSlew:ValidationFailed", ...
        "Local workspace: %s", localValidation.Message);
end
if ~fullRangeValidation.Passed
    warning("exampleGroundKeepOutAreaSlew:ValidationFailed", ...
        "Full gimbal range: %s", fullRangeValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion
if displayOptions.PlotOutputs
    [animationGifFolder, animationGifName, animationGifExtension] = ...
        fileparts(displayOptions.AnimationGifFile);
    plotOptions                  = displayOptions.PlotOptions;
    plotOptions.Title            = displayOptions.Title + ": local workspace";
    plotOptions.AnimationGifFile = fullfile(animationGifFolder, ...
        animationGifName + "_localWorkspace" + animationGifExtension);
    obstacleAvoidance.plotting.plotTrajectory(result, plotOptions);

    plotOptions.Title            = displayOptions.Title + ": full gimbal range";
    plotOptions.AnimationGifFile = fullfile(animationGifFolder, ...
        animationGifName + "_fullGimbalRange" + animationGifExtension);
    obstacleAvoidance.plotting.plotTrajectory(resultFullRange, plotOptions);
end
end
