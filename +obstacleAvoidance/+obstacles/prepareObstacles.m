function obstacles = prepareObstacles(obstacles, timeRange_s, stopAtUnsupported)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles)
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles, timeRange_s)
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles( ...
%       obstacles, timeRange_s, stopAtUnsupported)
%**************************************************************************
% PURPOSE
%   - Prepare obstacle shapes and motion models for the requested time
%     range. Reuse saved preparation only when its original inputs still match.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle array)
%       Obstacle histories to prepare; empty input remains empty.
%   - timeRange_s (finite 1-by-2 row, optional; default full history)
%       [start end] times to prepare, with start <= end.
%   - stopAtUnsupported (logical scalar, optional; default false)
%       Whether to stop preparing each obstacle at the first requested
%       interval whose motion model cannot be verified.
%**************************************************************************
% OUTPUTS
%   - obstacles (prepared obstacle array)
%       Each record includes reusable shapes and interval motion models.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Check Controls And Standardize The Obstacle Records

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

obstacles = obstacleAvoidance.obstacles.canonicalizeObstacles(obstacles);

if isempty(obstacles)
    return;
end

% A version mismatch means the saved preparation uses a different format or
% calculation. Rebuild it even if the obstacle inputs are unchanged.
preparationVersion = 13;

%% Section 2: Reuse Or Prepare Each Obstacle For The Requested Times

for obstacleIndex = 1:numel(obstacles)
    previousPreparation  = [];
    preparationIsCurrent = false;

    % Compare every input used by preparation, including protected and
    % original vertices. Even a small coordinate change requires rebuilding.
    if isfield(obstacles, 'InternalPreparation')
        savedPreparation = obstacles(obstacleIndex).InternalPreparation;
        preparationIsCurrent = isstruct(savedPreparation) && isscalar(savedPreparation) && ...
            all(isfield(savedPreparation, {'PreparationVersion', 'SourceSnapshot'})) && ...
            isequal(savedPreparation.PreparationVersion, preparationVersion) && ...
            isequaln(savedPreparation.SourceSnapshot, createSourceSnapshot(obstacles(obstacleIndex)));
    end
    if preparationIsCurrent
        normalizedObstacle  = obstacles(obstacleIndex);
        previousPreparation = normalizedObstacle.InternalPreparation;
        % When every sample and interval is already prepared, no further
        % work is needed for this unchanged obstacle.
        if all(previousPreparation.SamplePrepared) && all(previousPreparation.IntervalPrepared)
            continue;
        end
    else
        normalizedObstacle = obstacleAvoidance.obstacles.createObstacle(obstacles(obstacleIndex));
    end

    % Keep valid partial results and prepare the missing times. If the
    % inputs changed, previous preparation is empty and the work starts anew.
    sourceSnapshot   = createSourceSnapshot(normalizedObstacle);
    preparedObstacle = obstacleAvoidance.obstacles.prepareOneObstacle( ...
        normalizedObstacle, preparationVersion, sourceSnapshot, timeRange_s, ...
        previousPreparation, stopAtUnsupported);

    % Return the normalized input fields together with the prepared geometry.
    for fieldName = reshape(string(fieldnames(normalizedObstacle)), 1, [])
        obstacles(obstacleIndex).(fieldName) = normalizedObstacle.(fieldName);
    end
    obstacles(obstacleIndex).InternalPreparation = preparedObstacle.InternalPreparation;
end
end

%% Section 3: Local Functions

function sourceSnapshot = createSourceSnapshot(obstacle)
    % Save the inputs that determine geometry and vertex correspondence.
    % Later calls compare them before reusing any prepared shapes or models.
    sourceSnapshot = struct( ...
        "targetName",         obstacle.targetName, ...
        "time_s",             obstacle.time_s, ...
        "x_units",            {obstacle.x_units}, ...
        "y_units",            {obstacle.y_units}, ...
        "originalX_units",    {obstacle.originalX_units}, ...
        "originalY_units",    {obstacle.originalY_units}, ...
        "safetyMargin_units", obstacle.safetyMargin_units, ...
        "status",             obstacle.status, ...
        "UsesSourceIndex",    obstacle.UsesSourceIndex);
end
