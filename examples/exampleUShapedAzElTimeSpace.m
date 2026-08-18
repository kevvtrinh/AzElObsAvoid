function result = exampleUShapedAzElTimeSpace(options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleUShapedAzElTimeSpace()
%   result = exampleUShapedAzElTimeSpace(options)
%**************************************************************************
% PURPOSE
%   - Construct one protected U-shaped obstacle and run the maintained
%     planner without waypoints or a directed route.
%**************************************************************************
% INPUTS
%   - options (scalar struct, optional)
%       Planner option overrides plus the finite MaxJerk_deg_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and scenario geometry.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(options)
    options = struct();
end
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    options, struct( ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", "U-shaped az/el obstacle"), [2.5 2.5]);

%% Section 2: Create Obstacles

% Keep the physical request independent of solver initialization choices.
missionEndTime_s = 120;
safetyMargin_deg = 0.20;
uBoundary_deg = [ ...
    -8,  7; -5,  7; -5, -4; 5, -4; ...
     5,  7;  8,  7;  8, -7; -8, -7];
obstacle = makeAzElObstacleData( ...
    "Static U-shaped obstacle", [0; missionEndTime_s], ...
    uBoundary_deg(:, 1), uBoundary_deg(:, 2), safetyMargin_deg);

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, "position_deg", [0 -10]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

[result, diagnostics] = planAzElMotion( ...
    obstacle, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( ...
    result, diagnostics, "single U", ...
    struct("RequireDirectBlocked", true));

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, diagnostics, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleValidation = exampleValidation;
result.uBoundary_deg = uBoundary_deg;
result.ExampleConfiguration = jerkConfiguration;
end
