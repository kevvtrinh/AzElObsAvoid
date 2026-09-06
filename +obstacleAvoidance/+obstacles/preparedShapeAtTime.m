function [shape, geometry] = preparedShapeAtTime(obstacle, queryTime_s, geometryOnly)
%% Section 0: Header & Readme
% SYNTAX
%   [shape, geometry] = preparedShapeAtTime(obstacle, queryTime_s, geometryOnly)
% PURPOSE
%   Evaluate one prepared obstacle at a physical time.
% INPUTS
%   obstacle: prepared history; queryTime_s: scalar seconds; geometryOnly: optional flag.
% OUTPUTS
%   shape and geometry: the interpolated protected boundary and its cached interval model.
% UNITS
%   Positions are degrees; time is seconds; derivatives retain physical units.

%% Section 1: Evaluate Prepared Inputs
if nargin < 3, geometryOnly = false; end
preparation = obstacle.InternalPreparation;
time_s      = double(obstacle.time_s(:));
shape       = [];
if isempty(time_s) || (numel(time_s) > 1 && (queryTime_s < time_s(1) || queryTime_s > time_s(end)))
    geometry = boundaryGeometry(zeros(0, 1), zeros(0, 1), 0, false, 0, 0, "inactive");
    geometry.EdgeStart_deg = zeros(0, 2);
    geometry.EdgeEnd_deg   = zeros(0, 2);
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

azimuth_deg            = double(obstacle.az_deg{lowerIndex}(:));
elevation_deg          = double(obstacle.el_deg{lowerIndex}(:));
topologyIsInterpolated = true;
if lowerIndex == upperIndex
    speed_deg_s   = preparation.SampleSpeedBound_deg_s(lowerIndex);
    geometryModel = "authoritativeSample";
    if ~geometryOnly
        shape = preparation.SampleShapes{lowerIndex};
    end
    edgeStart_deg = preparation.SampleEdgeStart_deg{lowerIndex};
    edgeEnd_deg   = preparation.SampleEdgeEnd_deg{lowerIndex};
elseif preparation.MatchingTopology(lowerIndex)
    azimuth_deg   = azimuth_deg + fraction * preparation.DeltaAzimuth_deg{lowerIndex};
    elevation_deg = elevation_deg + fraction * preparation.DeltaElevation_deg{lowerIndex};
    speed_deg_s   = preparation.IntervalSpeedBound_deg_s(lowerIndex);
    geometryModel = preparation.IntervalGeometryModel(lowerIndex);
    if ~geometryOnly && speed_deg_s == 0
        shape = preparation.SampleShapes{lowerIndex};
    end
    edgeStart_deg = [azimuth_deg, elevation_deg];
    edgeEnd_deg   = circshift(edgeStart_deg, -1, 1);
else
    shape = preparation.IntervalUnionShapes{lowerIndex};
    [azimuth_deg, elevation_deg] = boundary(shape);
    speed_deg_s            = 0;
    topologyIsInterpolated = false;
    geometryModel          = preparation.IntervalGeometryModel(lowerIndex);
    edgeStart_deg          = preparation.IntervalUnionEdgeStart_deg{lowerIndex};
    edgeEnd_deg            = preparation.IntervalUnionEdgeEnd_deg{lowerIndex};
end
azimuth_deg(~isfinite(azimuth_deg)) = NaN;
elevation_deg(~isfinite(elevation_deg)) = NaN;
if ~geometryOnly && (isempty(shape) || isempty(shape.Vertices))
    shape = obstacleAvoidance.geometry.boundaryToShape(azimuth_deg, elevation_deg);
end
geometry = boundaryGeometry(azimuth_deg, elevation_deg, speed_deg_s, topologyIsInterpolated, lowerIndex, upperIndex, geometryModel);
geometry.EdgeStart_deg = edgeStart_deg;
geometry.EdgeEnd_deg   = edgeEnd_deg;
end

function geometry = boundaryGeometry(azimuth_deg, elevation_deg, speed_deg_s, topologyIsInterpolated, lowerIndex, upperIndex, geometryModel)
    % Classify one ordered boundary without changing its vertices or ring order.
    finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
    active       = nnz(finiteVertex) >= 3;
    hasOneRing   = active && all(finiteVertex);
    isConvex     = false;
    outwardSign  = NaN;
    if hasOneRing
        vertices_deg          = [azimuth_deg(:), elevation_deg(:)];
        nextVertices_deg      = circshift(vertices_deg, -1, 1);
        areaTerms_deg2        = vertices_deg(:, 1) .* nextVertices_deg(:, 2) - vertices_deg(:, 2) .* nextVertices_deg(:, 1);
        signedDoubleArea_deg2 = sum(areaTerms_deg2);
        areaTolerance_deg2    = 64 * eps * max(1, sum(abs(areaTerms_deg2)));
        hasOneRing            = abs(signedDoubleArea_deg2) > areaTolerance_deg2;
        if hasOneRing
            edges_deg          = nextVertices_deg - vertices_deg;
            nextEdges_deg      = circshift(edges_deg, -1, 1);
            turns_deg2         = edges_deg(:, 1) .* nextEdges_deg(:, 2) - edges_deg(:, 2) .* nextEdges_deg(:, 1);
            turnTolerance_deg2 = 64 * eps * max(1, max(abs(turns_deg2)));
            isConvex           = all(turns_deg2 >= -turnTolerance_deg2) || all(turns_deg2 <= turnTolerance_deg2);
            outwardSign        = -sign(signedDoubleArea_deg2);
        end
    end
    geometry = struct("Active", active, "azimuth_deg", double(azimuth_deg(:)), ...
        "elevation_deg", double(elevation_deg(:)), "VertexSpeedBound_deg_s", speed_deg_s, ...
        "HasOrderedSingleRegion", hasOneRing, "IsConvex", isConvex, "OutwardSign", outwardSign, ...
        "TopologyIsInterpolated", topologyIsInterpolated, ...
        "GeometryModel", string(geometryModel), ...
        "LowerSampleIndex", lowerIndex, "UpperSampleIndex", upperIndex);
end
