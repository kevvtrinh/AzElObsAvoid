function obstaclePlanningData = createObstaclePlanningData(obstacles, initialState, goalState)
%% Section 0: Header & Readme
% SYNTAX
%   obstaclePlanningData = obstacleAvoidance.obstacles.createObstaclePlanningData( ...
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
%   - obstaclePlanningData (scalar struct)
%       The single request-wide record shared by route search and motion
%       planning. It owns prepared obstacles, the planning interval, the
%       stationary-scene flag, and later the derived route-search geometry.
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

%% Section 4: Return The Shared Obstacle-Planning Data

% Keep request-wide inputs and derived geometry in one record. The empty
% routeSearchGeometry field is filled only when graph search is required.
obstaclePlanningData = struct("preparedObstacles", preparedObstacles, ...
    "startTime_s", startTime_s, "endTime_s", endTime_s, ...
    "obstaclesRemainStatic", obstaclesRemainStatic, ...
    "routeSearchGeometry", struct());
end
