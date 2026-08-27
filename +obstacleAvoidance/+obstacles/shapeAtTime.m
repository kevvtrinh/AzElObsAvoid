function [shape, geometry] = shapeAtTime(obstacle, queryTime_s, geometryOnly)
%% Section 0: Header & Readme
% SYNTAX
%   [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
%       obstacle, queryTime_s, geometryOnly)
%**************************************************************************
% PURPOSE
%   - Return the protected obstacle geometry active at one physical time.
%     Matching topology is interpolated vertex-by-vertex; topology-changing
%     intervals return their prepared conservative union.
%**************************************************************************
% INPUTS
%   - obstacle (scalar canonical or prepared struct), one obstacle history.
%   - queryTime_s (finite scalar), absolute geometry query time.
%   - geometryOnly (logical scalar, optional; default false).
%**************************************************************************
% OUTPUTS
%   - shape (scalar polyshape or []), protected query-time geometry.
%   - geometry (scalar struct), boundary, speed, and source-slice metadata.
%**************************************************************************
% UNITS
%   - Geometry is degrees; time is seconds; speed is degrees per second.
%**************************************************************************

%% Section 1: Validate The Query

% Check one normalized obstacle and one finite absolute time. Determine whether
% the obstacle is active before any interpolation. Inactive obstacles return an
% empty shape and do not block motion.

% geometryOnly skips polyshape construction when a caller needs boundary
% coordinates and motion information for a faster specialized calculation.
if ~isstruct(obstacle) || ~isscalar(obstacle) || ~all(isfield(obstacle, {'time_s', 'az_deg', 'el_deg'}))
    error("shapeAtTime:InvalidObstacle", ...
        "obstacle must be one canonical obstacle record.");
end
validateattributes(queryTime_s, {'numeric'}, {'real', 'finite', 'scalar'});
if nargin < 3 || isempty(geometryOnly)
    geometryOnly = false;
end
validateattributes(geometryOnly, {'logical', 'numeric'}, {'real', 'finite', 'scalar'});
if isnumeric(geometryOnly) && ~ismember(geometryOnly, [0 1])
    error("shapeAtTime:InvalidGeometryOnly", ...
        "geometryOnly must be a scalar logical or binary numeric value.");
end
geometryOnly = logical(geometryOnly);
queryTime_s = double(queryTime_s);
obstacle = obstacleAvoidance.obstacles.prepareDynamic(obstacle);
preparation = obstacle.InternalPreparation;
time_s = double(obstacle.time_s(:));
shape = [];
if isempty(time_s)
    geometry = createEmptyGeometry();
    if ~geometryOnly
        shape = polyshape();
    end
    return;
end

%% Section 2: Select Or Interpolate A Slice

% Use an exact stored slice at a sample time. Between samples, interpolate
% matching ordered vertices. When topology differs, use the prepared
% conservative interval shape instead of pairing unrelated vertices.

% Outside the stored history the obstacle is inactive. At an exact source time
% the original prepared slice is reused without rebuilding a polyshape.
if isscalar(time_s)
    % A one-slice obstacle is treated as a static obstacle at all query times.
    % Multi-slice histories, in contrast, are active only over their stored
    % time span.
    lowerIndex = 1;
    upperIndex = 1;
    fraction = 0;
elseif queryTime_s < time_s(1) || queryTime_s > time_s(end)
    % Only inactive queries need the empty record. Active queries overwrite
    % every geometry field below, so constructing it eagerly adds repeated
    % allocation to dense collision and visibility checks.
    geometry = createEmptyGeometry();
    if ~geometryOnly
        shape = polyshape();
    end
    return;
else
    upperIndex = find(time_s >= queryTime_s, 1, "first");
    % The first source sample at or after the query brackets the interval from
    % above. Exact source times use the stored slice and avoid interpolation.
    if time_s(upperIndex) == queryTime_s || upperIndex == 1
        lowerIndex = upperIndex;
        fraction = 0;
    else
        lowerIndex = upperIndex - 1;
        intervalDuration_s = time_s(upperIndex) - time_s(lowerIndex);
        fraction = (queryTime_s - time_s(lowerIndex)) / intervalDuration_s;
        % fraction is in [0,1]: zero selects the lower sample and one selects
        % the upper sample.
    end
end
if preparation.IsTimeInvariant
    geometry = preparation.StaticGeometry;
    geometry.LowerSampleIndex = lowerIndex;
    geometry.UpperSampleIndex = upperIndex;
    if ~geometryOnly
        shape = preparation.StaticShape;
    end
    return;
end
lowerAzimuth_deg = double(obstacle.az_deg{lowerIndex}(:));
lowerElevation_deg = double(obstacle.el_deg{lowerIndex}(:));
if lowerIndex == upperIndex
    azimuth_deg = lowerAzimuth_deg;
    elevation_deg = lowerElevation_deg;
    vertexSpeedBound_deg_s = preparation.SampleSpeedBound_deg_s(lowerIndex);
    if ~geometryOnly
        shape = preparation.SampleShapes{lowerIndex};
    end
    topologyIsInterpolated = true;
    % "Interpolated" here means vertex correspondence remains valid; at an
    % exact sample the interpolation fraction is simply zero.
else
    matchingTopology = preparation.MatchingTopology(lowerIndex);
    if matchingTopology
        % Matching NaN separators and vertex counts establish correspondence.
        azimuthDelta_deg = preparation.DeltaAzimuth_deg{lowerIndex};
        elevationDelta_deg = preparation.DeltaElevation_deg{lowerIndex};
        azimuth_deg = lowerAzimuth_deg + fraction * azimuthDelta_deg;
        elevation_deg = lowerElevation_deg + fraction * elevationDelta_deg;
        if ~geometryOnly && preparation.IntervalSpeedBound_deg_s(lowerIndex) == 0
            % A zero speed bound means every corresponding vertex is unchanged,
            % so the cached lower shape is exactly the query-time shape.
            shape = preparation.SampleShapes{lowerIndex};
        end
        vertexSpeedBound_deg_s = preparation.IntervalSpeedBound_deg_s(lowerIndex);
        topologyIsInterpolated = true;
    else
        % Never guess correspondence across a topology change. A conservative
        % union may reject feasible motion, but it cannot hide occupied space.
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

% Return the shape and boundary arrays used for the query. Include active state
% and interpolation details so collision diagnostics can explain which geometry
% blocked a point.

azimuth_deg(~isfinite(azimuth_deg)) = NaN;
elevation_deg(~isfinite(elevation_deg)) = NaN;
if ~geometryOnly && (isempty(shape) || isempty(shape.Vertices))
    shape = obstacleAvoidance.geometry.boundaryToShape(azimuth_deg, elevation_deg);
end
active = nnz(isfinite(azimuth_deg) & isfinite(elevation_deg)) >= 3;
% Three finite paired vertices are the minimum needed to enclose area. Ring
% separators are excluded from this count.
finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
hasOrderedSingleRegion = active && all(finiteVertex);
isConvex = false;
outwardSign = NaN;
if hasOrderedSingleRegion
    % The shoelace sum is twice the signed polygon area. Its sign gives winding
    % direction, while a near-zero magnitude identifies a degenerate ring whose
    % outward normal cannot be defined reliably.
    vertices_deg = [azimuth_deg, elevation_deg];
    nextVertices_deg = circshift(vertices_deg, -1, 1);
    signedAreaTerms_deg2 = vertices_deg(:, 1) .* ...
        nextVertices_deg(:, 2) - vertices_deg(:, 2) .* ...
        nextVertices_deg(:, 1);
    orientationTolerance_deg2 = 64 * eps * max( ...
        1, sum(abs(signedAreaTerms_deg2)));
    signedDoubleArea_deg2 = sum(signedAreaTerms_deg2);
    hasOrderedSingleRegion = abs(signedDoubleArea_deg2) > ...
        orientationTolerance_deg2;
    if hasOrderedSingleRegion
        % Consecutive edge cross products all share a sign for a convex polygon.
        % The scaled tolerance allows nearly collinear edges without classifying
        % harmless floating-point noise as a change in turn direction.
        edgeDelta_deg = nextVertices_deg - vertices_deg;
        nextEdgeDelta_deg = circshift(edgeDelta_deg, -1, 1);
        turn_deg2 = edgeDelta_deg(:, 1) .* nextEdgeDelta_deg(:, 2) - ...
            edgeDelta_deg(:, 2) .* nextEdgeDelta_deg(:, 1);
        turnTolerance_deg2 = 64 * eps * max(1, max(abs(turn_deg2)));
        isConvex = all(turn_deg2 >= -turnTolerance_deg2) || ...
            all(turn_deg2 <= turnTolerance_deg2);
        outwardSign = -sign(signedDoubleArea_deg2);
    end
end
geometry = struct( ...
    "Active", active, ...
    "azimuth_deg", double(azimuth_deg(:)), ...
    "elevation_deg", double(elevation_deg(:)), ...
    "VertexSpeedBound_deg_s", vertexSpeedBound_deg_s, ...
    "HasOrderedSingleRegion", hasOrderedSingleRegion, ...
    "IsConvex", isConvex, "OutwardSign", outwardSign, ...
    "TopologyIsInterpolated", topologyIsInterpolated, "LowerSampleIndex", lowerIndex, "UpperSampleIndex", upperIndex);
end

function geometry = createEmptyGeometry()
% Define the stable inactive-geometry record. Zero indices show that no source
% samples were selected, while empty coordinate columns preserve their meaning.
geometry = struct( ...
    "Active", false, ...
    "azimuth_deg", zeros(0, 1), ...
    "elevation_deg", zeros(0, 1), ...
    "VertexSpeedBound_deg_s", 0, ...
    "HasOrderedSingleRegion", false, ...
    "IsConvex", false, "OutwardSign", NaN, ...
    "TopologyIsInterpolated", true, ...
    "LowerSampleIndex", 0, "UpperSampleIndex", 0);
end
