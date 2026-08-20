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
obstacle = azElInternal.prepareDynamicObstacles(obstacle);
preparation = obstacle.InternalPreparation;
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
    vertexSpeedBound_deg_s = ...
        preparation.SampleSpeedBound_deg_s(lowerIndex);
    if ~geometryOnly
        shape = preparation.SampleShapes{lowerIndex};
    end
    topologyIsInterpolated = true;
else
    matchingTopology = preparation.MatchingTopology(lowerIndex);
    if matchingTopology
        azimuthDelta_deg = preparation.DeltaAzimuth_deg{lowerIndex};
        elevationDelta_deg = preparation.DeltaElevation_deg{lowerIndex};
        azimuth_deg = lowerAzimuth_deg + fraction * ...
            azimuthDelta_deg;
        elevation_deg = lowerElevation_deg + fraction * ...
            elevationDelta_deg;
        vertexSpeedBound_deg_s = ...
            preparation.IntervalSpeedBound_deg_s(lowerIndex);
        topologyIsInterpolated = true;
    else
        shape = preparation.IntervalUnionShapes{lowerIndex};
        azimuth_deg = preparation.IntervalUnionAzimuth_deg{lowerIndex};
        elevation_deg = preparation.IntervalUnionElevation_deg{lowerIndex};
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
