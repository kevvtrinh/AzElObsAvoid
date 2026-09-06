function obstacles = prepareObstacles(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles)
%
% PURPOSE
%   - Reuse current obstacle-history preparation for a complete collection.
%   - Rebuild stale preparation through one per-obstacle stage.
%
% INPUTS
%   - obstacles (canonical obstacle struct array)
%       Protected histories remain unchanged and authoritative.
%
% OUTPUTS
%   - obstacles (prepared obstacle struct array)
%       Each record contains source-checked reusable geometry data.
%
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%

%% Section 1: Reuse Only Current Complete Preparation

preparationIsCurrent = false(numel(obstacles), 1);

% Reuse cached geometry only when its layout and source data match.

if isempty(obstacles)
    return;
end
preparationVersion = 1;
if isfield(obstacles, "InternalPreparation")
    preparationIsCurrent = true(numel(obstacles), 1);
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        preparation      = obstacles(obstacleIndex).InternalPreparation;
        hasCurrentLayout = isstruct(preparation) && isscalar(preparation) && isfield(preparation, "PreparationVersion") && isequal(preparation.PreparationVersion, preparationVersion) && isfield(preparation, "SourceSnapshot");
        if hasCurrentLayout
            sourceSnapshot = createSourceSnapshot(obstacles(obstacleIndex));
            preparationIsCurrent(obstacleIndex) = isequaln(preparation.SourceSnapshot, sourceSnapshot);
        else
            preparationIsCurrent(obstacleIndex) = false;
        end
    end
    if all(preparationIsCurrent)
        return;
    end
end

%% Section 2: Prepare Each Complete History

% Prepare each obstacle separately.

for obstacleIndex = 1:numel(obstacles)
    if preparationIsCurrent(obstacleIndex), continue; end
    preparedObstacle = obstacleAvoidance.obstacles.prepareOneObstacle(obstacles(obstacleIndex), preparationVersion, createSourceSnapshot(obstacles(obstacleIndex)));
    obstacles(obstacleIndex).InternalPreparation = preparedObstacle.InternalPreparation;
end
end

%% Section 3: Local Functions

function snapshot = createSourceSnapshot(obstacle)
    % Store the source fields for cache checks.
    snapshot = struct("targetName", obstacle.targetName, ...
        "time_s", obstacle.time_s, ...
        "az_deg", {obstacle.az_deg}, ...
        "el_deg", {obstacle.el_deg}, ...
        "originalAz_deg", {obstacle.originalAz_deg}, ...
        "originalEl_deg", {obstacle.originalEl_deg}, ...
        "safetyMargin_deg", obstacle.safetyMargin_deg, ...
        "status", obstacle.status);
end
