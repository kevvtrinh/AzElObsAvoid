function obstacles = prepareObstacles(obstacles, timeRange_s)
%% Section 0: Header & Readme
% SYNTAX: obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles)
%   obstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles,[t0,t1])
% PURPOSE: Prepare requested source intervals and reuse overlapping cached entries.
%   Rebuild stale preparation through one per-obstacle stage.
% INPUTS: obstacles (canonical obstacle struct array) Normalize expected boundary fragments before
%   preparing authoritative protected geometry; retain original geometry and absolute margin.
%   timeRange_s: finite nondecreasing 1-by-2 interval; omit for full history.
% OUTPUTS: obstacles (prepared obstacle struct array) Each record contains source-checked reusable
%   geometry data.
% UNITS: Geometry is coordinate units, time is seconds, and speed is coordinate units per second.

%% Section 1: Reuse Only Source-Checked Preparation
if nargin<2
    timeRange_s=[-Inf,Inf];
else
    validateattributes(timeRange_s,{'numeric'},{'real','finite','size',[1,2]});
    assert(timeRange_s(1)<=timeRange_s(2),'prepareObstacles:InvalidTimeRange','The requested time interval must be nondecreasing.');
end

if iscell(obstacles)
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
if ~isempty(obstacles) && isstruct(obstacles) && isfield(obstacles, 'Vertices_units')
    canonical = cell(numel(obstacles), 1);
    for k = 1:numel(obstacles)
        vertices_units = obstacles(k).Vertices_units;
        validateattributes(vertices_units, {'numeric'}, {'2d','ncols',2,'real','finite'});
        name = "obstacle " + k; margin_units = 0;
        if isfield(obstacles, 'Name'), name = obstacles(k).Name; end
        if isfield(obstacles, 'SafetyMargin_units'), margin_units = obstacles(k).SafetyMargin_units; end
        canonical{k} = obstacleAvoidance.obstacles.createObstacle(name, 0, vertices_units(:,1), vertices_units(:,2), margin_units);
    end
    obstacles = vertcat(canonical{:});
end

% Reuse cached geometry only when its layout and source data match.

if isempty(obstacles)
    return;
end
preparationVersion = 7;
%% Section 2: Extend Only The Requested Entries
% Prepare each obstacle separately.

for obstacleIndex = 1:numel(obstacles)
    previous=[];
    preparationIsCurrent = false;
    if isfield(obstacles,'InternalPreparation')
        preparation = obstacles(obstacleIndex).InternalPreparation;
        preparationIsCurrent = isstruct(preparation) && isscalar(preparation) && ...
            all(isfield(preparation,{'PreparationVersion','SourceSnapshot'})) && ...
            isequal(preparation.PreparationVersion,preparationVersion) && ...
            isequaln(preparation.SourceSnapshot,createSourceSnapshot(obstacles(obstacleIndex)));
    end
    if preparationIsCurrent
        normalized=obstacles(obstacleIndex);
        previous=normalized.InternalPreparation;
        % Source equality was checked above. Complete preparation needs no
        % extension or reconstruction of its unchanged derived fields.
        if all(previous.SamplePrepared) && all(previous.IntervalPrepared)
            continue;
        end
    else
        normalized=obstacleAvoidance.obstacles.createObstacle(obstacles(obstacleIndex));
    end
    preparedObstacle = obstacleAvoidance.obstacles.prepareOneObstacle(normalized, preparationVersion, createSourceSnapshot(normalized),timeRange_s,previous);
    for name = reshape(string(fieldnames(normalized)),1,[])
        obstacles(obstacleIndex).(name) = normalized.(name);
    end
    obstacles(obstacleIndex).InternalPreparation = preparedObstacle.InternalPreparation;
end
end

%% Section 3: Local Functions
function snapshot = createSourceSnapshot(obstacle)
    % Store the source fields for cache checks.
    snapshot = struct("targetName", obstacle.targetName, ...
        "time_s", obstacle.time_s, ...
        "x_units", {obstacle.x_units}, ...
        "y_units", {obstacle.y_units}, ...
        "originalX_units", {obstacle.originalX_units}, ...
        "originalY_units", {obstacle.originalY_units}, ...
        "safetyMargin_units", obstacle.safetyMargin_units, ...
        "status", obstacle.status);
end
