function [shape, geometry] = obstacleShapeAtTime( ...
        obstacle, queryTime_s, geometryOnly)
%% Section 0: Header & Readme
% SYNTAX
%   [shape, geometry] = azElInternal.obstacleShapeAtTime( ...
%       obstacle, queryTime_s)
%   [shape, geometry] = azElInternal.obstacleShapeAtTime( ...
%       obstacle, queryTime_s, geometryOnly)
%**************************************************************************
% PURPOSE
%   - Interpolate one canonical protected obstacle at one query time.
%   - Use a conservative union when adjacent slice topology differs.
%**************************************************************************
% INPUTS
%   - obstacle (scalar canonical obstacle struct)
%       Protected az_deg and el_deg histories from makeAzElObstacleData.
%   - queryTime_s (finite scalar)
%       Absolute query time.
%   - geometryOnly (logical scalar, optional; default false)
%       If true, skip polyshape construction when topology is unchanged.
%**************************************************************************
% OUTPUTS
%   - shape (scalar polyshape or [])
%       Protected occupied geometry. It is [] in geometry-only mode when
%       topology is unchanged, and empty outside the active span.
%   - geometry (scalar struct)
%       Interpolated boundary, vertex-speed bound, and provenance flags.
%**************************************************************************
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%**************************************************************************

%% Section 1: Validate The Query

if ~isstruct(obstacle) || ~isscalar(obstacle) || ...
        ~all(isfield(obstacle, {'time_s', 'az_deg', 'el_deg'}))
    error("azElInternal:obstacleShapeAtTime:InvalidObstacle", ...
        "obstacle must be one canonical obstacle record.");
end
validateattributes(queryTime_s, {'numeric'}, ...
    {'real', 'finite', 'scalar'});
if nargin < 3 || isempty(geometryOnly)
    geometryOnly = false;
end
validateattributes(geometryOnly, {'logical', 'numeric'}, ...
    {'real', 'finite', 'scalar'});
if isnumeric(geometryOnly) && ~ismember(geometryOnly, [0 1])
    error("azElInternal:obstacleShapeAtTime:InvalidGeometryOnly", ...
        "geometryOnly must be a scalar logical or binary numeric value.");
end
geometryOnly = logical(geometryOnly);
queryTime_s = double(queryTime_s);
time_s = double(obstacle.time_s(:));
geometry = emptyGeometry();
if geometryOnly
    shape = [];
else
    shape = polyshape();
end
if isempty(time_s)
    return;
end
%% Section 2: Select Or Interpolate A Slice

if isscalar(time_s)
    lowerIndex = 1;
    upperIndex = 1;
    fraction = 0;
elseif queryTime_s < time_s(1) || queryTime_s > time_s(end)
    return;
else
    upperIndex = find(time_s >= queryTime_s, 1, "first");
    if time_s(upperIndex) == queryTime_s || upperIndex == 1
        lowerIndex = upperIndex;
        fraction = 0;
    else
        lowerIndex = upperIndex - 1;
        intervalDuration_s = time_s(upperIndex) - time_s(lowerIndex);
        fraction = (queryTime_s - time_s(lowerIndex)) / ...
            intervalDuration_s;
    end
end

lowerAzimuth_deg = double(obstacle.az_deg{lowerIndex}(:));
lowerElevation_deg = double(obstacle.el_deg{lowerIndex}(:));
if lowerIndex == upperIndex
    azimuth_deg = lowerAzimuth_deg;
    elevation_deg = lowerElevation_deg;
    vertexSpeedBound_deg_s = adjacentVertexSpeedBound( ...
        obstacle, lowerIndex);
    topologyIsInterpolated = true;
else
    upperAzimuth_deg = double(obstacle.az_deg{upperIndex}(:));
    upperElevation_deg = double(obstacle.el_deg{upperIndex}(:));
    matchingTopology = ...
        numel(lowerAzimuth_deg) == numel(upperAzimuth_deg) && ...
        isequal(isfinite(lowerAzimuth_deg), isfinite(upperAzimuth_deg)) && ...
        isequal(isfinite(lowerElevation_deg), ...
        isfinite(upperElevation_deg));
    if matchingTopology
        azimuth_deg = lowerAzimuth_deg + fraction * ...
            (upperAzimuth_deg - lowerAzimuth_deg);
        elevation_deg = lowerElevation_deg + fraction * ...
            (upperElevation_deg - lowerElevation_deg);
        intervalDuration_s = time_s(upperIndex) - time_s(lowerIndex);
        finiteRows = isfinite(lowerAzimuth_deg) & ...
            isfinite(lowerElevation_deg);
        vertexSpeed_deg_s = hypot( ...
            upperAzimuth_deg(finiteRows) - lowerAzimuth_deg(finiteRows), ...
            upperElevation_deg(finiteRows) - ...
            lowerElevation_deg(finiteRows)) / intervalDuration_s;
        vertexSpeedBound_deg_s = maximumOrZero(vertexSpeed_deg_s);
        topologyIsInterpolated = true;
    else
        lowerShape = boundaryShape( ...
            lowerAzimuth_deg, lowerElevation_deg);
        upperShape = boundaryShape( ...
            upperAzimuth_deg, upperElevation_deg);
        shape = union(lowerShape, upperShape);
        [azimuth_deg, elevation_deg] = boundary(shape);
        % The conservative union is constant inside this source interval.
        % Source-time splits contain its discontinuous endpoint change.
        vertexSpeedBound_deg_s = 0;
        topologyIsInterpolated = false;
    end
end

%% Section 3: Assemble The Geometry Record

azimuth_deg(~isfinite(azimuth_deg)) = NaN;
elevation_deg(~isfinite(elevation_deg)) = NaN;
if ~geometryOnly && isempty(shape.Vertices)
    shape = boundaryShape(azimuth_deg, elevation_deg);
end
active = nnz(isfinite(azimuth_deg) & isfinite(elevation_deg)) >= 3;
geometry = struct( ...
    "Active", active, ...
    "azimuth_deg", double(azimuth_deg(:)), ...
    "elevation_deg", double(elevation_deg(:)), ...
    "VertexSpeedBound_deg_s", vertexSpeedBound_deg_s, ...
    "TopologyIsInterpolated", topologyIsInterpolated, ...
    "LowerSampleIndex", lowerIndex, ...
    "UpperSampleIndex", upperIndex);
end

%% Section 4: Local Functions

function shape = boundaryShape(azimuth_deg, elevation_deg)
% PURPOSE
%   - Construct one polyshape without changing canonical ring membership.
finiteRows = isfinite(azimuth_deg) & isfinite(elevation_deg);
if nnz(finiteRows) < 3
    shape = polyshape();
    return;
end
shape = polyshape(azimuth_deg, elevation_deg, ...
    "Simplify", false, "KeepCollinearPoints", true);
end

function speedBound_deg_s = adjacentVertexSpeedBound(obstacle, sampleIndex)
% PURPOSE
%   - Bound vertex motion next to an exact source sample when topology agrees.
time_s = double(obstacle.time_s(:));
speedBound_deg_s = 0;
neighborIndices = unique([sampleIndex - 1, sampleIndex + 1]);
neighborIndices = neighborIndices( ...
    neighborIndices >= 1 & neighborIndices <= numel(time_s));
for neighborIndex = reshape(neighborIndices, 1, [])
    firstAzimuth_deg = double(obstacle.az_deg{sampleIndex}(:));
    firstElevation_deg = double(obstacle.el_deg{sampleIndex}(:));
    secondAzimuth_deg = double(obstacle.az_deg{neighborIndex}(:));
    secondElevation_deg = double(obstacle.el_deg{neighborIndex}(:));
    matchingTopology = ...
        numel(firstAzimuth_deg) == numel(secondAzimuth_deg) && ...
        isequal(isfinite(firstAzimuth_deg), ...
        isfinite(secondAzimuth_deg));
    if ~matchingTopology
        speedBound_deg_s = Inf;
        return;
    end
    finiteRows = isfinite(firstAzimuth_deg) & ...
        isfinite(firstElevation_deg);
    duration_s = abs(time_s(neighborIndex) - time_s(sampleIndex));
    speed_deg_s = hypot( ...
        secondAzimuth_deg(finiteRows) - firstAzimuth_deg(finiteRows), ...
        secondElevation_deg(finiteRows) - ...
        firstElevation_deg(finiteRows)) / duration_s;
    speedBound_deg_s = max(speedBound_deg_s, maximumOrZero(speed_deg_s));
end
end

function value = maximumOrZero(values)
% PURPOSE
%   - Return a stable zero for an empty finite-vertex set.
if isempty(values)
    value = 0;
else
    value = max(values);
end
end

function geometry = emptyGeometry()
% PURPOSE
%   - Define the stable inactive-geometry schema.
geometry = struct( ...
    "Active", false, ...
    "azimuth_deg", zeros(0, 1), ...
    "elevation_deg", zeros(0, 1), ...
    "VertexSpeedBound_deg_s", 0, ...
    "TopologyIsInterpolated", true, ...
    "LowerSampleIndex", 0, ...
    "UpperSampleIndex", 0);
end
