function result = exampleMovingCircleNoAzimuthWrap(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleMovingCircleNoAzimuthWrap()
%   result = exampleMovingCircleNoAzimuthWrap(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Demonstrate an immediate non-wrapping detour around a rising circle.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Unmodified public planner result.
%**************************************************************************
% UNITS
%   - Position is degrees; time is seconds; derivatives use deg/s, deg/s^2,
%     and deg/s^3.
%**************************************************************************

%% Section 1: Resolve Example Controls

% Disable azimuth wrapping. The route must remain inside the stated interval.

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
[options, displayOptions] = resolveExampleOptions( ...
    exampleOverrides, struct( ...
    "GoalTimeMode", "earliestArrival", ...
    "AllowAzimuthWrapping", false), [2 2]);

%% Section 2: Create Obstacles

% A circle rises across the direct path. The planner must start an immediate
% detour because waiting or wrapping around the azimuth boundary is not allowed.

obstacleTime_s = [0; 15];
circleCenterElevation_deg = [0; 3];
circleAngle_rad = (0:23).' * (2 * pi / 24);
circleRadius_deg = 1.5;
azimuthBySlice_deg = cell(2, 1);
elevationBySlice_deg = cell(2, 1);

% Create the circle at both sampled elevations. Keep its azimuth outline fixed.
for sampleIndex = 1:2
    azimuthBySlice_deg{sampleIndex} = circleRadius_deg * cos(circleAngle_rad);
    elevationBySlice_deg{sampleIndex} = circleCenterElevation_deg(sampleIndex) + ...
        circleRadius_deg * sin(circleAngle_rad);
end
safetyMargin_deg = 0.1;
obstacles = obstacleAvoidance.obstacles.createObstacle( ...
    "rising circle", obstacleTime_s, azimuthBySlice_deg, elevationBySlice_deg, safetyMargin_deg);

%% Section 3: Create Planner Inputs

% Place the endpoints on opposite sides of the moving circle. Workspace bounds
% make the non-wrapping requirement explicit.

initialState = struct("time_s", 0, "position_deg", [-6 0]);
goalState = struct("time_s", 15, "position_deg", [6 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], "maxAcceleration_deg_s2", [1 1], "maxJerk_deg_s3", displayOptions.MaxJerk_deg_s3);

%% Section 4: Run Planner

% Run the same planner used by the other dynamic-obstacle examples.

result = obstacleAvoidance.planTrajectory( obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

% Confirm collision freedom and confirm that all azimuth samples stay in bounds.

exampleValidation = obstacleAvoidance.validateTrajectory(result);
if ~exampleValidation.Passed
    warning("exampleMovingCircleNoAzimuthWrap:ValidationFailed", ...
        "%s", exampleValidation.Message);
end

%% Section 6: Plot Diagnostics And Motion

% The animation shows the detour and the vertical circle motion together.

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectory( ...
        result, displayOptions.PlotOptions);
end

end
