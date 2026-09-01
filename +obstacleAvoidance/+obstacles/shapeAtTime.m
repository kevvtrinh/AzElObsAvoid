function [shape, geometry] = shapeAtTime(obstacle, queryTime_s, geometryOnly)
%% Section 0: Header & Readme
% SYNTAX
%   [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
%       obstacle, queryTime_s)
%   [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
%       obstacle, queryTime_s, geometryOnly)
%**************************************************************************
% PURPOSE
%   - Return protected obstacle geometry active at one physical time.
%   - Interpolate verified corresponding vertices and otherwise return a
%     conservative swept enclosure for the complete source interval.
%**************************************************************************
% INPUTS
%   - obstacle (scalar canonical or prepared struct)
%   - queryTime_s (finite numeric scalar)
%   - geometryOnly (logical scalar, optional; default false)
%**************************************************************************
% OUTPUTS
%   - shape (scalar polyshape or [])
%   - geometry (scalar struct)
%       Boundary, speed bound, topology, and source-slice metadata.
%       status is metadata and never deactivates supplied geometry.
%**************************************************************************
% UNITS
%   - Geometry is degrees; time is seconds; speed is degrees per second.
%   - See obstacle_history_contract.md for the complete history model.
%**************************************************************************

%% Section 1: Validate And Select The Source Interval

if ~isstruct(obstacle) || ~isscalar(obstacle) || ...
        ~all(isfield(obstacle, {'time_s', 'az_deg', 'el_deg'}))
    error("shapeAtTime:InvalidObstacle", "obstacle must be one canonical record.");
end
validateattributes(queryTime_s, {'numeric'}, {'real', 'finite', 'scalar'});
if nargin < 3 || isempty(geometryOnly)
    geometryOnly = false;
end
geometryOnly = obstacleAvoidance.input.normalizeLogicalScalar( ...
    geometryOnly, "geometryOnly", "shapeAtTime:InvalidGeometryOnly");
queryTime_s = double(queryTime_s);
obstacle = obstacleAvoidance.obstacles.prepareDynamic(obstacle);
preparation = obstacle.InternalPreparation;
time_s = double(obstacle.time_s(:));
shape = [];
if isempty(time_s) || (numel(time_s) > 1 && (queryTime_s < time_s(1) || queryTime_s > time_s(end)))
    geometry = boundaryGeometry( ...
        zeros(0, 1), zeros(0, 1), 0, false, 0, 0, "inactive");
    if ~geometryOnly
        shape = polyshape();
    end
    return;
end
lowerIndex = find(time_s <= queryTime_s, 1, "last");
upperIndex = find(time_s >= queryTime_s, 1, "first");
if isscalar(time_s)
    lowerIndex = 1;
    upperIndex = 1;
end
fraction = 0;
if lowerIndex ~= upperIndex
    fraction = (queryTime_s - time_s(lowerIndex)) / (time_s(upperIndex) - time_s(lowerIndex));
end
%% Section 2: Evaluate The Protected Boundary

azimuth_deg = double(obstacle.az_deg{lowerIndex}(:));
elevation_deg = double(obstacle.el_deg{lowerIndex}(:));
topologyIsInterpolated = true;
if lowerIndex == upperIndex
    speed_deg_s = preparation.SampleSpeedBound_deg_s(lowerIndex);
    geometryModel = "authoritativeSample";
    if ~geometryOnly
        shape = preparation.SampleShapes{lowerIndex};
    end
elseif preparation.MatchingTopology(lowerIndex)
    azimuth_deg = azimuth_deg + fraction * preparation.DeltaAzimuth_deg{lowerIndex};
    elevation_deg = elevation_deg + fraction * preparation.DeltaElevation_deg{lowerIndex};
    speed_deg_s = preparation.IntervalSpeedBound_deg_s(lowerIndex);
    geometryModel = preparation.IntervalGeometryModel(lowerIndex);
    if ~geometryOnly && speed_deg_s == 0
        shape = preparation.SampleShapes{lowerIndex};
    end
else
    shape = preparation.IntervalUnionShapes{lowerIndex};
    [azimuth_deg, elevation_deg] = boundary(shape);
    speed_deg_s = 0;
    topologyIsInterpolated = false;
    geometryModel = preparation.IntervalGeometryModel(lowerIndex);
end
azimuth_deg(~isfinite(azimuth_deg)) = NaN;
elevation_deg(~isfinite(elevation_deg)) = NaN;
if ~geometryOnly && (isempty(shape) || isempty(shape.Vertices))
    shape = obstacleAvoidance.geometry.boundaryToShape(azimuth_deg, elevation_deg);
end
geometry = boundaryGeometry(azimuth_deg, elevation_deg, speed_deg_s, ...
    topologyIsInterpolated, lowerIndex, upperIndex, geometryModel);
end

function geometry = boundaryGeometry(azimuth_deg, elevation_deg, ...
        speed_deg_s, topologyIsInterpolated, lowerIndex, upperIndex, ...
        geometryModel)
% Classify one ordered boundary without changing its vertices or ring order.
finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
active = nnz(finiteVertex) >= 3;
hasOneRing = active && all(finiteVertex);
isConvex = false;
outwardSign = NaN;
if hasOneRing
    vertices_deg = [azimuth_deg(:), elevation_deg(:)];
    nextVertices_deg = circshift(vertices_deg, -1, 1);
    areaTerms_deg2 = vertices_deg(:, 1) .* nextVertices_deg(:, 2) - ...
        vertices_deg(:, 2) .* nextVertices_deg(:, 1);
    signedDoubleArea_deg2 = sum(areaTerms_deg2);
    areaTolerance_deg2 = 64 * eps * max(1, sum(abs(areaTerms_deg2)));
    hasOneRing = abs(signedDoubleArea_deg2) > areaTolerance_deg2;
    if hasOneRing
        edges_deg = nextVertices_deg - vertices_deg;
        nextEdges_deg = circshift(edges_deg, -1, 1);
        turns_deg2 = edges_deg(:, 1) .* nextEdges_deg(:, 2) - ...
            edges_deg(:, 2) .* nextEdges_deg(:, 1);
        turnTolerance_deg2 = 64 * eps * max(1, max(abs(turns_deg2)));
        isConvex = all(turns_deg2 >= -turnTolerance_deg2) || all(turns_deg2 <= turnTolerance_deg2);
        outwardSign = -sign(signedDoubleArea_deg2);
    end
end
geometry = struct("Active", active, "azimuth_deg", double(azimuth_deg(:)), ...
    "elevation_deg", double(elevation_deg(:)), "VertexSpeedBound_deg_s", speed_deg_s, ...
    "HasOrderedSingleRegion", hasOneRing, "IsConvex", isConvex, "OutwardSign", outwardSign, ...
    "TopologyIsInterpolated", topologyIsInterpolated, ...
    "GeometryModel", string(geometryModel), ...
    "LowerSampleIndex", lowerIndex, "UpperSampleIndex", upperIndex);
end
