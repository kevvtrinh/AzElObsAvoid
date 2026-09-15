function obstacles = prepareObstacles(obstacles, timeRange_s, stopAtUnsupported)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles)
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles, timeRange_s)
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
%       obstacles, timeRange_s, stopAtUnsupported)
%**************************************************************************
% PURPOSE
%   - Prepare requested source intervals and reuse source-checked cache data.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle array)
%       Obstacle histories to prepare; empty input remains empty.
%   - timeRange_s (finite 1-by-2 row, optional; default full history)
%       Nondecreasing time interval whose samples and spans are needed.
%   - stopAtUnsupported (logical scalar, optional; default false)
%       Whether preparation stops at the first unsupported touched interval.
%**************************************************************************
% OUTPUTS
%   - obstacles (prepared obstacle array)
%       Each record contains reusable source-derived geometry. Invalid input
%       throws an error.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Reuse Only Source-Checked Preparation

if nargin < 3
    stopAtUnsupported = false;
end
validateattributes(stopAtUnsupported, {'logical'}, {'scalar'});
if nargin < 2
    timeRange_s = [-Inf, Inf];
else
    validateattributes(timeRange_s, {'numeric'}, {'real', 'finite', 'size', [1, 2]});
    assert(timeRange_s(1) <= timeRange_s(2), 'prepareObstacles:InvalidTimeRange', ...
        'The requested time interval must be nondecreasing.');
end

if iscell(obstacles)
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
if ~isempty(obstacles) && isstruct(obstacles) && isfield(obstacles, 'Vertices_units')
    canonicalObstacles = cell(numel(obstacles), 1);
    for obstacleIndex = 1:numel(obstacles)
        vertices_units = obstacles(obstacleIndex).Vertices_units;
        validateattributes(vertices_units, {'numeric'}, {'2d', 'ncols', 2, 'real', 'finite'});
        obstacleName       = "obstacle " + obstacleIndex;
        safetyMargin_units = 0;
        if isfield(obstacles, 'Name')
            obstacleName = obstacles(obstacleIndex).Name;
        end
        if isfield(obstacles, 'SafetyMargin_units')
            safetyMargin_units = obstacles(obstacleIndex).SafetyMargin_units;
        end
        canonicalObstacles{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
            obstacleName, 0, vertices_units(:, 1), vertices_units(:, 2), safetyMargin_units);
    end
    obstacles = vertcat(canonicalObstacles{:});
end

if isempty(obstacles)
    return;
end
preparationVersion = 10;

%% Section 2: Extend Only The Requested Entries

% Prepare each obstacle separately.
for obstacleIndex = 1:numel(obstacles)
    previous = [];
    preparationIsCurrent = false;
    if isfield(obstacles, 'InternalPreparation')
        preparation = obstacles(obstacleIndex).InternalPreparation;
        preparationIsCurrent = isstruct(preparation) && isscalar(preparation) && ...
            all(isfield(preparation, {'PreparationVersion', 'SourceSnapshot'})) && ...
            isequal(preparation.PreparationVersion, preparationVersion) && ...
            isequaln(preparation.SourceSnapshot, createSourceSnapshot(obstacles(obstacleIndex)));
    end
    if preparationIsCurrent
        normalized = obstacles(obstacleIndex);
        previous   = normalized.InternalPreparation;
        % Source equality was checked above. Complete preparation needs no
        % extension or reconstruction of its unchanged derived fields.
        if all(previous.SamplePrepared) && all(previous.IntervalPrepared)
            continue;
        end
    else
        normalized = obstacleAvoidance.obstacles.createObstacle(obstacles(obstacleIndex));
    end
    sourceSnapshot = createSourceSnapshot(normalized);
    preparedObstacle = obstacleAvoidance.obstacles.prepareOneObstacle( ...
        normalized, preparationVersion, sourceSnapshot, timeRange_s, previous, stopAtUnsupported);
    for fieldName = reshape(string(fieldnames(normalized)), 1, [])
        obstacles(obstacleIndex).(fieldName) = normalized.(fieldName);
    end
    obstacles(obstacleIndex).InternalPreparation = preparedObstacle.InternalPreparation;
end
end

%% Section 3: Local Functions

function snapshot = createSourceSnapshot(obstacle)
    % Store the source fields for cache checks.
    snapshot = struct( ...
        "targetName",           obstacle.targetName, ...
        "time_s",               obstacle.time_s, ...
        "x_units",              {obstacle.x_units}, ...
        "y_units",              {obstacle.y_units}, ...
        "originalX_units",      {obstacle.originalX_units}, ...
        "originalY_units",      {obstacle.originalY_units}, ...
        "safetyMargin_units",   obstacle.safetyMargin_units, ...
        "status",               obstacle.status, ...
        "vertexCorrespondence", string(obstacle.vertexCorrespondence));
end
