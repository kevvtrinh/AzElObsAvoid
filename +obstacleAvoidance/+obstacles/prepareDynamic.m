function obstacles = prepareDynamic(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles)
%**************************************************************************
% PURPOSE
%   - Reuse current obstacle-history preparation for a complete collection.
%   - Rebuild stale preparation through one per-obstacle stage.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle struct array)
%       Protected histories remain unchanged and authoritative.
%**************************************************************************
% OUTPUTS
%   - obstacles (prepared obstacle struct array)
%       Each record contains source-checked reusable geometry data.
%**************************************************************************
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%**************************************************************************

%% Section 1: Reuse Only Current Complete Preparation

% Prepared geometry is queried throughout search and validation. Reuse it only
% when every record has the current layout and an exact matching source copy;
% otherwise rebuild the collection so callers never mix cache generations.

if isempty(obstacles)
    return;
end
preparationVersion = 1;
if isfield(obstacles, "InternalPreparation")
    preparationIsCurrent = true(numel(obstacles), 1);
    for obstacleIndex = 1:numel(obstacles)
        preparation = obstacles(obstacleIndex).InternalPreparation;
        hasCurrentLayout = isstruct(preparation) && ...
            isscalar(preparation) && ...
            isfield(preparation, "PreparationVersion") && ...
            isequal(preparation.PreparationVersion, preparationVersion) && ...
            isfield(preparation, "SourceSnapshot");
        if hasCurrentLayout
            sourceSnapshot = createSourceSnapshot(obstacles(obstacleIndex));
            preparationIsCurrent(obstacleIndex) = isequaln( ...
                preparation.SourceSnapshot, sourceSnapshot);
        else
            preparationIsCurrent(obstacleIndex) = false;
        end
    end
    if all(preparationIsCurrent)
        return;
    end
    obstacles = rmfield(obstacles, "InternalPreparation");
end

%% Section 2: Prepare Each Complete History

% Each obstacle needs the same ordered sample and interval work. Delegate one
% complete record at a time so its method choices and derived values can be
% inspected without collection-level cache logic obscuring them.

for obstacleIndex = 1:numel(obstacles)
    preparedObstacle = ...
        obstacleAvoidance.obstacles.prepareOneObstacle( ...
        obstacles(obstacleIndex), preparationVersion);
    obstacles(obstacleIndex).InternalPreparation = ...
        preparedObstacle.InternalPreparation;
end
end

%% Section 3: Local Functions

function snapshot = createSourceSnapshot(obstacle)
% Retain an exact immutable copy of every canonical public source field.
snapshot = struct( ...
    "targetName", obstacle.targetName, ...
    "time_s", obstacle.time_s, ...
    "az_deg", {obstacle.az_deg}, ...
    "el_deg", {obstacle.el_deg}, ...
    "originalAz_deg", {obstacle.originalAz_deg}, ...
    "originalEl_deg", {obstacle.originalEl_deg}, ...
    "safetyMargin_deg", obstacle.safetyMargin_deg, ...
    "status", obstacle.status);
end
