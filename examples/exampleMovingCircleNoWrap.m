function [result, diagnosis] = exampleMovingCircleNoWrap(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingCircleNoWrap()
%   result = exampleMovingCircleNoWrap(exampleOverrides)
%
% PURPOSE
%   - Demonstrate an immediate non-wrapping detour around a rising circle.
%
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%   - diagnosis (optional second output): search attempts and solver details.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s, units/s^2,
%     and units/s^3.
%

%% Section 1: Resolve Example Controls

% Disable x wrapping. The route must remain inside the stated interval.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions(exampleOverrides, struct("GoalTimeMode", "earliestArrival", "WrapX", false), [2 2]);

%% Section 2: Create Obstacles

% A circle rises across the direct path. The planner must start an immediate
% detour because waiting or wrapping around the x boundary is not allowed.

obstacleTime_s            = [0; 15];
circleCenterY_units = [0; 3];
circleAngle_rad           = (0:23).' * (2 * pi / 24);
circleRadius_units          = 1.5;
xBySlice_units        = cell(2, 1);
yBySlice_units      = cell(2, 1);

% Create the circle at both sampled ys. Keep its x outline fixed.
for sampleIndex = 1:2
    xBySlice_units{sampleIndex} = circleRadius_units * cos(circleAngle_rad);
    yBySlice_units{sampleIndex} = circleCenterY_units(sampleIndex) + circleRadius_units * sin(circleAngle_rad);
end
safetyMargin_units = 0.1;
obstacles        = obstacleAvoidance.obstacles.createObstacle("rising circle", obstacleTime_s, xBySlice_units, yBySlice_units, safetyMargin_units);

%% Section 3: Create Planner Inputs

% Place the endpoints on opposite sides of the moving circle. Workspace bounds
% make the non-wrapping requirement explicit.

initialState = struct();
initialState.time_s       = 0;
initialState.position_units = [-6 0];
goalState = struct();
goalState.time_s       = 15;
goalState.position_units = [6 0];
limits = struct("maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [1 1], "maxJerk_units_s3", displayOptions.MaxJerk_units_s3);

%% Section 4: Run Planner

% Run the same planner used by the other dynamic-obstacle examples.

[result, diagnosis] = planner(obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Confirm collision freedom and confirm that all x samples stay in bounds.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleMovingCircleNoWrap:ValidationFailed", "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% The animation shows the detour and the vertical circle motion together.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory(result, displayOptions.PlotOptions, diagnosis);
end

end
