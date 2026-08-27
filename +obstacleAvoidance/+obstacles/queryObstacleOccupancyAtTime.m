function [isOccupied, blockingObstacleIndex, queryDetails] = queryObstacleOccupancyAtTime(obstacles, azimuth_deg, elevation_deg, ...
        queryTime, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime()
%   isOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, azimuth_deg, elevation_deg, queryTime)
%   [isOccupied, blockingObstacleIndex, queryDetails] = ...
%       obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
%       obstacles, azimuth_deg, elevation_deg, ...
%       queryTime, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Query canonical protected polygon geometry at explicit times.
%   - Return signed-clearance diagnostics from one shared implementation.
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

% Resolve query options and normalize obstacle inputs first. Use protected
% geometry by default because planner collision checks must include the safety
% margin. An original-geometry query is for reporting or independent inspection.

% Boundary policy is explicit because a zero-clearance point can be treated as
% blocked for conservative planning or clear for specialized diagnostics. The
% tiny default tolerance absorbs polygon arithmetic roundoff in degree units.
defaults = struct( ...
    "BoundaryIsOccupied", true, ...
    "ClearanceTolerance_deg", 1e-10, "ReferenceTime", datetime(1970, 1, 1, 0, 0, 0, "TimeZone", "UTC"));
if nargin == 0
    isOccupied = defaults;
    blockingObstacleIndex = [];
    queryDetails = struct();
    return;
end
if nargin ~= 4 && nargin ~= 5
    error("queryObstacleOccupancyAtTime:InvalidCall", ...
        "Use zero inputs, four query inputs, or four query inputs plus options.");
end
if nargin < 5 || isempty(optionOverrides)
    optionOverrides = struct();
end
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("queryObstacleOccupancyAtTime:InvalidOptions", "optionOverrides must be a scalar struct.");
end
[options, unknownNames] = obstacleAvoidance.input.resolveOptions(defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("queryObstacleOccupancyAtTime:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", strjoin(unknownNames, ", "));
end
options.BoundaryIsOccupied = obstacleAvoidance.input.normalizeLogicalScalar( ...
    options.BoundaryIsOccupied, "BoundaryIsOccupied", "queryObstacleOccupancyAtTime:InvalidBoundaryPolicy");
validateattributes(options.ClearanceTolerance_deg, {'numeric'}, {'real', 'finite', 'scalar', 'nonnegative'});
if ~isdatetime(options.ReferenceTime) || ~isscalar(options.ReferenceTime) || isnat(options.ReferenceTime)
    error("queryObstacleOccupancyAtTime:InvalidReferenceTime", "ReferenceTime must be one finite datetime scalar.");
end
options.ReferenceTime.TimeZone = "UTC";
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    % Normalize arbitrary supported containers, then cache the dynamic geometry
    % needed by repeated time queries. Already prepared arrays are reused.
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
    obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
end

%% Section 2: Broadcast Query Arrays

% Allow scalar coordinates or times to apply to every query. Expand them to a
% common size. A size error here means azimuth, elevation, and time cannot form
% matching query points.

if isdatetime(queryTime)
    % Convert absolute datetimes to seconds relative to one UTC reference so
    % geometry interpolation uses the same numeric time representation as the
    % rest of the planner.
    queryTime.TimeZone = "UTC";
    queryTime_s = seconds(queryTime - options.ReferenceTime);
elseif isnumeric(queryTime)
    queryTime_s = double(queryTime);
else
    error("queryObstacleOccupancyAtTime:InvalidTime", "queryTime must be numeric seconds or a datetime array.");
end
[queryAzimuth_deg, queryElevation_deg, queryTime_s, outputSize] = broadcastQueries(azimuth_deg, elevation_deg, queryTime_s);
queryCount = numel(queryTime_s);
isOccupied = false(queryCount, 1);
blockingObstacleIndex = zeros(queryCount, 1, "uint32");
if nargout < 2
    % Occupancy-only callers do not need nearest points or obstacle names. The
    % specialized path groups equal times and uses broad bounds for less work.
    isOccupied = occupancyOnly( obstacles, queryAzimuth_deg, queryElevation_deg, queryTime_s, options);
    isOccupied = reshape(isOccupied, outputSize);
    return;
end
minimumClearance_deg = Inf(queryCount, 1);
nearestObstacleIndex = zeros(queryCount, 1, "uint32");

%% Section 3: Query Interpolated Protected Geometry

% For each obstacle and query time, get the active shape and test whether the
% point is inside or on its boundary. Stop early in occupancy-only mode after
% any obstacle contains the point. Detailed mode keeps per-obstacle evidence.

clearanceTolerance_deg = double(options.ClearanceTolerance_deg);

% Evaluate each caller query independently so invalid samples retain stable diagnostics.
for queryIndex = 1:queryCount
    if ~all(isfinite([queryAzimuth_deg(queryIndex), queryElevation_deg(queryIndex), queryTime_s(queryIndex)]))
        % Nonfinite samples are not considered occupied, but NaN clearance makes
        % their invalidity visible in the detailed output.
        minimumClearance_deg(queryIndex) = NaN;
        continue;
    end
    point_deg = [queryAzimuth_deg(queryIndex), queryElevation_deg(queryIndex)];

    % Compare this query point with every obstacle to find its nearest blocker.
    for obstacleIndex = 1:numel(obstacles)
        shape = obstacleAvoidance.obstacles.shapeAtTime( obstacles(obstacleIndex), queryTime_s(queryIndex));
        clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance( shape, point_deg);
        if clearance_deg < minimumClearance_deg(queryIndex)
            % Signed clearance is smallest inside an obstacle and otherwise
            % selects the closest exterior boundary.
            minimumClearance_deg(queryIndex) = clearance_deg;
            nearestObstacleIndex(queryIndex) = uint32(obstacleIndex);
        end
        isBoundaryBlocked = options.BoundaryIsOccupied && clearance_deg <= clearanceTolerance_deg;
        isInteriorBlocked = clearance_deg < -clearanceTolerance_deg;
        if blockingObstacleIndex(queryIndex) == 0 && (isBoundaryBlocked || isInteriorBlocked)
            % Preserve the first blocker in caller obstacle order for a stable,
            % reproducible result even if several obstacles overlap the point.
            isOccupied(queryIndex) = true;
            blockingObstacleIndex(queryIndex) = uint32(obstacleIndex);
        end
    end
end

%% Section 4: Assemble Outputs

% Combine per-obstacle occupancy with logical OR. Preserve detailed clearance
% and obstacle indices when requested. If occupancy and details disagree,
% inspect this reduction before inspecting shape interpolation.

isOccupied = reshape(isOccupied, outputSize);
blockingObstacleIndex = reshape(blockingObstacleIndex, outputSize);
minimumClearance_deg = reshape(minimumClearance_deg, outputSize);
nearestObstacleIndex = reshape(nearestObstacleIndex, outputSize);
obstacleNames = strings(outputSize);

% Map every nearest-obstacle index back to its public target name.
for obstacleIndex = 1:numel(obstacles)
    obstacleNames(nearestObstacleIndex == obstacleIndex) = obstacles(obstacleIndex).targetName;
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


function [azimuth_deg, elevation_deg, time_s, outputSize] = broadcastQueries(azimuth_deg, elevation_deg, time_s)
% Apply scalar expansion while preserving one caller-provided shape. This lets
% a scalar time query an entire position grid or one position query many times.
values = {double(azimuth_deg), double(elevation_deg), double(time_s)};
elementCounts = zeros(1, 3);

% Record each coordinate input size before applying scalar expansion.
for valueIndex = 1:3
    elementCounts(valueIndex) = numel(values{valueIndex});
end
nonScalarCounts = elementCounts(elementCounts > 1);
if isempty(nonScalarCounts)
    outputSize = size(values{1});
    queryCount = 1;
elseif any(nonScalarCounts ~= nonScalarCounts(1))
    % Equal element counts are sufficient; inputs need not begin with identical
    % row/column shapes because the first non-scalar input defines output shape.
    error("queryObstacleOccupancyAtTime:SizeMismatch", ...
        "Non-scalar azimuth, elevation, and time inputs must have " + "equal element counts.");
else
    queryCount = nonScalarCounts(1);
    shapeSourceIndex = find(elementCounts == queryCount, 1, "first");
    outputSize = size(values{shapeSourceIndex});
end

% Expand scalar inputs and reshape all three coordinates into aligned columns.
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

function isOccupied = occupancyOnly( obstacles, azimuth_deg, elevation_deg, time_s, options)
% Test occupancy without constructing clearance data that is not used.
queryCount = numel(time_s);
isOccupied = false(queryCount, 1);
finiteQuery = isfinite(azimuth_deg) & isfinite(elevation_deg) & isfinite(time_s);
uniqueTime_s = unique(time_s(finiteQuery));
% Grouping by time evaluates each obstacle shape once per distinct instant
% instead of once per point, which is important for dense collision grids.
if isempty(obstacles)
    obstacleBounds_deg = zeros(0, 4);
else
    preparation = [obstacles.InternalPreparation];
    obstacleBounds_deg = vertcat(preparation.HistoryBounds_deg);
end

% Reuse one obstacle geometry evaluation for all queries at each unique time.
for timeIndex = 1:numel(uniqueTime_s)
    queryIndices = find(finiteQuery & time_s == uniqueTime_s(timeIndex));

    % Test every obstacle against the still-unblocked queries at this time.
    for obstacleIndex = 1:numel(obstacles)
        remainingIndices = queryIndices(~isOccupied(queryIndices));
        if isempty(remainingIndices)
            break;
        end
        bound_deg = obstacleBounds_deg(obstacleIndex, :);
        tolerance_deg = options.ClearanceTolerance_deg;
        canIntersect = azimuth_deg(remainingIndices) >= bound_deg(1) - tolerance_deg & ...
            azimuth_deg(remainingIndices) <= bound_deg(2) + tolerance_deg & ...
            elevation_deg(remainingIndices) >= bound_deg(3) - tolerance_deg & ...
            elevation_deg(remainingIndices) <= bound_deg(4) + tolerance_deg;
        candidateIndices = remainingIndices(canIntersect);
        % The whole-history axis-aligned bound is a cheap conservative filter.
        % Passing it does not imply occupancy; failing it proves the point cannot
        % touch this obstacle at any stored time.
        if isempty(candidateIndices)
            continue;
        end
        points_deg = [azimuth_deg(candidateIndices), elevation_deg(candidateIndices)];
        [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), uniqueTime_s(timeIndex), true);
        if ~geometry.Active
            continue;
        end
        finiteBoundary = isfinite(geometry.azimuth_deg) & isfinite(geometry.elevation_deg);
        hasOneRing = all(finiteBoundary);
        if hasOneRing
            % inpolygon handles a single ordered ring in vectorized form and
            % separately reports points exactly on its boundary.
            [inside, onBoundary] = inpolygon( ...
                points_deg(:, 1), points_deg(:, 2), geometry.azimuth_deg, geometry.elevation_deg);
            blocked = inside;
            if ~options.BoundaryIsOccupied
                blocked(onBoundary) = false;
            end
        else
            % Multi-ring shapes need signed polygon clearance to preserve holes
            % and apply the requested boundary rule consistently.
            if isempty(shape)
                shape = obstacleAvoidance.obstacles.shapeAtTime( obstacles(obstacleIndex), uniqueTime_s(timeIndex));
            end
            blocked = complexShapeOccupancy(shape, points_deg, options);
        end
        isOccupied(candidateIndices(blocked)) = true;
    end
end
end
function blocked = complexShapeOccupancy(shape, points_deg, options)
% Preserve the detailed boundary policy for multi-ring geometry.
% The clearance helper already batches points while reusing one edge traversal.
% Passing the complete block avoids rediscovering identical ring edges for
% every point without changing the signed-distance or boundary policy.
clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance( ...
    shape, points_deg);
blocked = clearance_deg < -options.ClearanceTolerance_deg | ...
    (options.BoundaryIsOccupied & ...
    clearance_deg <= options.ClearanceTolerance_deg);
end
