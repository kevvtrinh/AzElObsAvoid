function [isOccupied, blockingObstacleIndex, queryDetails] = queryTimeObstacle( ...
        obstacles, azimuth_deg, elevation_deg, ...
        queryTime, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = azElPlannerMethods.hs3.queryTimeObstacle()
%   isOccupied = azElPlannerMethods.hs3.queryTimeObstacle( ...
%       obstacles, azimuth_deg, elevation_deg, queryTime)
%   [isOccupied, blockingObstacleIndex, queryDetails] = azElPlannerMethods.hs3.queryTimeObstacle( ...
%       obstacles, azimuth_deg, elevation_deg, ...
%       queryTime, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Query canonical protected polygon geometry at explicit times.
%   - Return signed-clearance diagnostics from the same geometry queried.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle array, nested cells, or [])
%       Safety margins must already be present in protected geometry.
%   - azimuth_deg, elevation_deg (numeric arrays or scalars)
%       Non-scalar inputs must have equal element counts.
%   - queryTime (numeric seconds or datetime array)
%       Numeric values use the supplied ReferenceTime only for provenance.
%   - optionOverrides (scalar struct, optional; default struct())
%       BoundaryIsOccupied is logical (default true).
%       ClearanceTolerance_deg is nonnegative (default 1e-10).
%       ReferenceTime is a datetime scalar (default Unix epoch).
%**************************************************************************
% OUTPUTS
%   - isOccupied (logical array)
%       True for protected-interior points and, by default, boundary points.
%   - blockingObstacleIndex (uint32 array)
%       First blocking obstacle index, or zero when clear.
%   - queryDetails (scalar struct)
%       Signed minimum clearance, blocker names, times, margins, and options.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Numeric time is seconds.
%**************************************************************************

%% Section 1: Resolve Options And Obstacles

defaults = struct( ...
    "BoundaryIsOccupied", true, ...
    "ClearanceTolerance_deg", 1e-10, ...
    "ReferenceTime", datetime(1970, 1, 1, 0, 0, 0, ...
    "TimeZone", "UTC"));
if nargin == 0
    isOccupied = defaults;
    blockingObstacleIndex = [];
    queryDetails = struct();
    return;
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("queryAzElTimeObstacle:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
[options, unknownNames] = azElPlannerMethods.hs3.internal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("queryAzElTimeObstacle:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.BoundaryIsOccupied = azElPlannerMethods.hs3.internal.normalizeLogicalScalar( ...
    options.BoundaryIsOccupied, "BoundaryIsOccupied", ...
    "queryAzElTimeObstacle:InvalidBoundaryPolicy");
validateattributes(options.ClearanceTolerance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
if ~isdatetime(options.ReferenceTime) || ...
        ~isscalar(options.ReferenceTime) || isnat(options.ReferenceTime)
    error("queryAzElTimeObstacle:InvalidReferenceTime", ...
        "ReferenceTime must be one finite datetime scalar.");
end
options.ReferenceTime.TimeZone = "UTC";
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = azElPlannerMethods.hs3.combineObstacles(obstacles);
    obstacles = azElPlannerMethods.hs3.internal.obstacles.prepareDynamic(obstacles);
end

%% Section 2: Broadcast Query Arrays

if isdatetime(queryTime)
    queryTime.TimeZone = "UTC";
    queryTime_s = seconds(queryTime - options.ReferenceTime);
elseif isnumeric(queryTime)
    queryTime_s = double(queryTime);
else
    error("queryAzElTimeObstacle:InvalidTime", ...
        "queryTime must be numeric seconds or a datetime array.");
end
[queryAzimuth_deg, queryElevation_deg, queryTime_s, outputSize] = broadcastQueries( ...
    azimuth_deg, elevation_deg, queryTime_s);
queryCount = numel(queryTime_s);
isOccupied = false(queryCount, 1);
blockingObstacleIndex = zeros(queryCount, 1, "uint32");
if nargout < 2
    isOccupied = occupancyOnly( ...
        obstacles, queryAzimuth_deg, queryElevation_deg, queryTime_s, ...
        options);
    isOccupied = reshape(isOccupied, outputSize);
    return;
end
minimumClearance_deg = Inf(queryCount, 1);
nearestObstacleIndex = zeros(queryCount, 1, "uint32");

%% Section 3: Query Interpolated Protected Geometry

clearanceTolerance_deg = double(options.ClearanceTolerance_deg);

% Evaluate every broadcast query independently so invalid samples can return
% NaN diagnostics without affecting valid samples.
for queryIndex = 1:queryCount
    if ~all(isfinite([queryAzimuth_deg(queryIndex), ...
            queryElevation_deg(queryIndex), queryTime_s(queryIndex)]))
        minimumClearance_deg(queryIndex) = NaN;
        continue;
    end
    point_deg = [queryAzimuth_deg(queryIndex), ...
        queryElevation_deg(queryIndex)];

    % Compare this point with every active obstacle and retain both its nearest
    % geometry and its first blocker.
    for obstacleIndex = 1:numel(obstacles)
        shape = azElPlannerMethods.hs3.internal.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), queryTime_s(queryIndex));
        clearance_deg = azElPlannerMethods.hs3.internal.geometry.pointPolygonClearance( ...
            shape, point_deg);
        if clearance_deg < minimumClearance_deg(queryIndex)
            minimumClearance_deg(queryIndex) = clearance_deg;
            nearestObstacleIndex(queryIndex) = uint32(obstacleIndex);
        end
        isBoundaryBlocked = options.BoundaryIsOccupied && ...
            clearance_deg <= clearanceTolerance_deg;
        isInteriorBlocked = clearance_deg < -clearanceTolerance_deg;
        if blockingObstacleIndex(queryIndex) == 0 && ...
                (isBoundaryBlocked || isInteriorBlocked)
            isOccupied(queryIndex) = true;
            blockingObstacleIndex(queryIndex) = uint32(obstacleIndex);
        end
    end
end

%% Section 4: Assemble Outputs

isOccupied = reshape(isOccupied, outputSize);
blockingObstacleIndex = reshape(blockingObstacleIndex, outputSize);
minimumClearance_deg = reshape(minimumClearance_deg, outputSize);
nearestObstacleIndex = reshape(nearestObstacleIndex, outputSize);
obstacleNames = strings(outputSize);

% Translate each retained nearest-obstacle index into its public target name.
for obstacleIndex = 1:numel(obstacles)
    obstacleNames(nearestObstacleIndex == obstacleIndex) = obstacles( ...
        obstacleIndex).targetName;
end
queryDetails = struct( ...
    "MinimumClearance_deg", minimumClearance_deg, ...
    "NearestObstacleIndex", nearestObstacleIndex, ...
    "NearestObstacleName", obstacleNames, ...
    "QueryTime_s", reshape(queryTime_s, outputSize), ...
    "ObstacleSafetyMargins_deg", reshape( ...
    [obstacles.safetyMargin_deg], [], 1), ...
    "Options", options);
end


function [azimuth_deg, elevation_deg, time_s, outputSize] = broadcastQueries( ...
        azimuth_deg, elevation_deg, time_s)
% Apply scalar expansion while preserving one caller-provided shape.
values = {double(azimuth_deg), double(elevation_deg), double(time_s)};
elementCounts = zeros(1, 3);

% Record each query input's element count before choosing the shared output
% shape and applying scalar expansion.
for valueIndex = 1:3
    elementCounts(valueIndex) = numel(values{valueIndex});
end
nonScalarCounts = elementCounts(elementCounts > 1);
if isempty(nonScalarCounts)
    outputSize = size(values{1});
    queryCount = 1;
elseif any(nonScalarCounts ~= nonScalarCounts(1))
    error("queryAzElTimeObstacle:SizeMismatch", ...
        "Non-scalar azimuth, elevation, and time inputs must have " + ...
        "equal element counts.");
else
    queryCount = nonScalarCounts(1);
    shapeSourceIndex = find(elementCounts == queryCount, 1, "first");
    outputSize = size(values{shapeSourceIndex});
end

% Expand scalar inputs and columnize all three arrays in the same fixed order.
for valueIndex = 1:3
    if elementCounts(valueIndex) == 1
        values{valueIndex} = repmat(values{valueIndex}, queryCount, 1);
    end
    values{valueIndex} = values{valueIndex}(:);
end
azimuth_deg = values{1};
elevation_deg = values{2};
time_s = values{3};
end

function isOccupied = occupancyOnly( ...
        obstacles, azimuth_deg, elevation_deg, time_s, options)
% Test occupancy without constructing clearance data that is not used.
queryCount = numel(time_s);
isOccupied = false(queryCount, 1);
finiteQuery = isfinite(azimuth_deg) & isfinite(elevation_deg) & ...
    isfinite(time_s);
uniqueTime_s = unique(time_s(finiteQuery));
if isempty(obstacles)
    obstacleBounds_deg = zeros(0, 4);
else
    preparation = [obstacles.InternalPreparation];
    obstacleBounds_deg = vertcat(preparation.HistoryBounds_deg);
end

% Reuse one obstacle geometry evaluation for every query sharing the same time.
for timeIndex = 1:numel(uniqueTime_s)
    queryIndices = find(finiteQuery & time_s == uniqueTime_s(timeIndex));

    % Test each obstacle only against points not already blocked at this time.
    for obstacleIndex = 1:numel(obstacles)
        remainingIndices = queryIndices(~isOccupied(queryIndices));
        if isempty(remainingIndices)
            break;
        end
        bound_deg = obstacleBounds_deg(obstacleIndex, :);
        tolerance_deg = options.ClearanceTolerance_deg;
        canIntersect = azimuth_deg(remainingIndices) >= ...
            bound_deg(1) - tolerance_deg & ...
            azimuth_deg(remainingIndices) <= bound_deg(2) + tolerance_deg & ...
            elevation_deg(remainingIndices) >= bound_deg(3) - tolerance_deg & ...
            elevation_deg(remainingIndices) <= bound_deg(4) + tolerance_deg;
        candidateIndices = remainingIndices(canIntersect);
        if isempty(candidateIndices)
            continue;
        end
        points_deg = [azimuth_deg(candidateIndices), ...
            elevation_deg(candidateIndices)];
        [shape, geometry] = azElPlannerMethods.hs3.internal.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), uniqueTime_s(timeIndex), true);
        if ~geometry.Active
            continue;
        end
        finiteBoundary = isfinite(geometry.azimuth_deg) & ...
            isfinite(geometry.elevation_deg);
        hasOneRing = all(finiteBoundary);
        if hasOneRing
            [inside, onBoundary] = inpolygon( ...
                points_deg(:, 1), points_deg(:, 2), ...
                geometry.azimuth_deg, geometry.elevation_deg);
            blocked = inside;
            if ~options.BoundaryIsOccupied
                blocked(onBoundary) = false;
            end
        else
            if isempty(shape)
                shape = azElPlannerMethods.hs3.internal.obstacles.shapeAtTime( ...
                    obstacles(obstacleIndex), uniqueTime_s(timeIndex));
            end
            blocked = complexShapeOccupancy(shape, points_deg, options);
        end
        isOccupied(candidateIndices(blocked)) = true;
    end
end
end

function blocked = complexShapeOccupancy(shape, points_deg, options)
% Preserve the detailed boundary policy for multi-ring geometry.
pointCount = size(points_deg, 1);
blocked = false(pointCount, 1);

% Compute signed clearance for every point because inpolygon cannot preserve
% the required boundary policy across multiple rings and holes.
for pointIndex = 1:pointCount
    clearance_deg = azElPlannerMethods.hs3.internal.geometry.pointPolygonClearance( ...
        shape, points_deg(pointIndex, :));
    blocked(pointIndex) = clearance_deg < ...
        -options.ClearanceTolerance_deg || ...
        (options.BoundaryIsOccupied && clearance_deg <= ...
        options.ClearanceTolerance_deg);
end
end
