function scene = preparePlanningScene(obstacles, initialState, goalState)
%% Section 0: Header & Readme
% SYNTAX
%   scene = obstacleAvoidance.obstacles.preparePlanningScene( ...
%       obstacles, initialState, goalState)
%
% PURPOSE
%   - Prepare obstacle histories once for repeated planning queries.
%   - Record the planning interval and whether all obstacles remain stationary.
%
% INPUTS
%   - obstacles: canonical obstacle histories.
%   - initialState, goalState: start and end of the planning interval.
%
% OUTPUTS
%   - scene (scalar struct)
%       Prepared obstacles, start/end times, and the stationary-scene flag.
%
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%

%% Section 1: Read The Planning Horizon

startTime_s = initialState.time_s;
endTime_s   = goalState.time_s;

%% Section 2: Prepare Complete Obstacle Histories

% Prepare shared obstacle geometry once for search and validation.

preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles);

%% Section 3: Check The Request Horizon

% Use static BMTP only if every obstacle is unchanged over the full horizon.

obstaclesRemainStatic = obstacleAvoidance.obstacles.queryStaticHorizon(preparedObstacles, startTime_s, endTime_s);

%% Section 4: Return The Shared Planning Scene
scene = struct("preparedObstacles", preparedObstacles, ...
    "startTime_s", startTime_s, "endTime_s", endTime_s, ...
    "obstaclesRemainStatic", obstaclesRemainStatic);
end
