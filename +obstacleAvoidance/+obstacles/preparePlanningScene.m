function scene = preparePlanningScene(request)
%% Section 0: Header & Readme
% SYNTAX
%   scene = obstacleAvoidance.obstacles.preparePlanningScene(request)
%**************************************************************************
% PURPOSE
%   - Prepare obstacle histories once for repeated planning queries.
%   - Describe the request horizon and reusable obstacle preparation details.
%**************************************************************************
% INPUTS
%   - request (scalar planning-request struct)
%       Must contain normalized obstacles, initialState, and goalState.
%**************************************************************************
% OUTPUTS
%   - scene (scalar struct)
%       Prepared obstacles, request horizon, static-horizon decision, and
%       per-obstacle preparation details. This data supports planning but
%       cannot approve a completed trajectory.
%**************************************************************************
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%**************************************************************************

%% Section 1: Check The Request Record

% This stage is called after public input normalization. Check its small
% internal interface here so a broken stage handoff fails at its source.

requiredFields = {'obstacles', 'initialState', 'goalState'};
if ~isstruct(request) || ~isscalar(request) || ...
        ~all(isfield(request, requiredFields))
    error("preparePlanningScene:InvalidRequest", ...
        "request must be a scalar struct with obstacles, initialState, and goalState.");
end
startTime_s = request.initialState.time_s;
endTime_s = request.goalState.time_s;

%% Section 2: Prepare Complete Obstacle Histories

% Graph construction, motion solving, and the final motion check repeatedly
% query the same histories. Prepare shapes, bounds, edges, and interpolation
% data once so those stages cannot rebuild different geometry independently.

preparedObstacles = obstacleAvoidance.obstacles.prepareDynamic( ...
    request.obstacles);

%% Section 3: Check The Request Horizon

% Static BMTP is valid only when every obstacle is unchanged over the complete
% physical request interval. Keep this decision beside the prepared histories
% that support it so later solver routing uses one shared result.

isStaticHorizon = obstacleAvoidance.obstacles.queryStaticHorizon( ...
    preparedObstacles, startTime_s, endTime_s);

%% Section 4: Create Inspectable Preparation Details

% These details explain which source histories and interval models planning
% will query. They are diagnostic data and do not replace final motion checks.

obstacleCount = numel(preparedObstacles);
detailTemplate = struct( ...
    "ObstacleIndex", 0, ...
    "Name", "", ...
    "SampleCount", 0, ...
    "IntervalCount", 0, ...
    "HistoryBounds_deg", [NaN NaN NaN NaN], ...
    "IntervalGeometryMethod", strings(0, 1), ...
    "IntervalSpeedBound_deg_s", zeros(0, 1), ...
    "IsTimeInvariant", false);
obstacleDetails = repmat(detailTemplate, obstacleCount, 1);
for obstacleIndex = 1:obstacleCount
    obstacle = preparedObstacles(obstacleIndex);
    preparation = obstacle.InternalPreparation;
    obstacleDetails(obstacleIndex) = struct( ...
        "ObstacleIndex", obstacleIndex, ...
        "Name", string(obstacle.targetName), ...
        "SampleCount", numel(obstacle.time_s), ...
        "IntervalCount", max(0, numel(obstacle.time_s) - 1), ...
        "HistoryBounds_deg", preparation.HistoryBounds_deg, ...
        "IntervalGeometryMethod", preparation.IntervalGeometryModel, ...
        "IntervalSpeedBound_deg_s", ...
        preparation.IntervalSpeedBound_deg_s, ...
        "IsTimeInvariant", preparation.IsTimeInvariant);
end

%% Section 5: Assemble The Scene

% The scene keeps prepared geometry separate from normalized public inputs.
% Proposal, search, and solver stages can inspect it without changing request.

scene = struct( ...
    "preparedObstacles", preparedObstacles, ...
    "startTime_s", startTime_s, ...
    "endTime_s", endTime_s, ...
    "isStaticHorizon", isStaticHorizon, ...
    "obstacleDetails", obstacleDetails);
end
