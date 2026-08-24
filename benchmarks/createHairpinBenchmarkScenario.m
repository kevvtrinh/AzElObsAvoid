function [obstacles, initialState, goalState, limits] = ...
        createHairpinBenchmarkScenario(hairpinCount)
%% Section 0: Header & Readme
% SYNTAX
%   [obstacles, initialState, goalState, limits] = ...
%       createHairpinBenchmarkScenario()
%   [obstacles, initialState, goalState, limits] = ...
%       createHairpinBenchmarkScenario(hairpinCount)
%**************************************************************************
% PURPOSE
%   - Build the deterministic alternating-end maze shared by compact and
%     standalone HS3 benchmarks.
%**************************************************************************
% INPUTS
%   - hairpinCount (positive integer scalar, optional; default 12)
%       Number of alternating horizontal walls.
%**************************************************************************
% OUTPUTS
%   - obstacles (canonical protected obstacle struct array)
%   - initialState (scalar rest initial state)
%   - goalState (scalar rest latest-arrival goal state)
%   - limits (scalar workspace and physical-limit struct)
%**************************************************************************
% UNITS
%   - Geometry and clearance are degrees; time is seconds; derivatives use
%     deg/s powers. Histories use [azimuth elevation].
%**************************************************************************

%% Section 1: Resolve Scenario Scale

if nargin < 1 || isempty(hairpinCount)
    hairpinCount = 12;
end
validateattributes(hairpinCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});

%% Section 2: Construct Alternating Protected Geometry

wallSpacing_deg = 4;
wallHalfThickness_deg = 0.35;
workspaceHalfWidth_deg = 10;
openingInnerEdgeMagnitude_deg = 6.5;
safetyMargin_deg = 0.15;
bottomElevation_deg = 0;
topElevation_deg = wallSpacing_deg * (hairpinCount + 1);
obstacleCells = cell(hairpinCount, 1);

% Alternating openings force each input-derived route to reverse laterally.
for wallIndex = 1:hairpinCount
    wallElevation_deg = wallSpacing_deg * wallIndex;
    if mod(wallIndex, 2) == 1
        wallAzimuthInterval_deg = [ ...
            -workspaceHalfWidth_deg, openingInnerEdgeMagnitude_deg];
    else
        wallAzimuthInterval_deg = [ ...
            -openingInnerEdgeMagnitude_deg, workspaceHalfWidth_deg];
    end
    wallAzimuth_deg = wallAzimuthInterval_deg([1 2 2 1]).';
    wallElevationBoundary_deg = wallElevation_deg + ...
        wallHalfThickness_deg * [-1 -1 1 1].';
    obstacleCells{wallIndex} = makeAzElObstacleData( ...
        "alternating wall " + wallIndex, 0, wallAzimuth_deg, ...
        wallElevationBoundary_deg, safetyMargin_deg);
end
obstacles = [obstacleCells{:}].';
obstacles = azElInternal.obstacles.prepareDynamic(obstacles);

%% Section 3: Assemble Planner-Role Inputs

initialState = struct( ...
    "time_s", 0, ...
    "position_deg", [0, 0.5 * wallSpacing_deg], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 120 * (hairpinCount + 1), ...
    "position_deg", [0, topElevation_deg - 0.5 * wallSpacing_deg], ...
    "velocity_deg_s", [0 0], ...
    "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2], ...
    "azimuthInterval_deg", ...
    [-workspaceHalfWidth_deg, workspaceHalfWidth_deg], ...
    "elevationInterval_deg", [bottomElevation_deg, topElevation_deg]);
end
