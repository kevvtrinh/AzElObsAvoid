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
%   Positions are coordinate units; time is seconds; derivatives retain physical units.

%% Section 1: Evaluate Prepared Inputs
if nargin < 3, geometryOnly = false; end
preparation = obstacle.InternalPreparation;
time_s      = double(obstacle.time_s(:));
shape       = [];
if isempty(time_s) || (numel(time_s) > 1 && (queryTime_s < time_s(1) || queryTime_s > time_s(end)))
    geometry = boundaryGeometry(zeros(0, 1), zeros(0, 1), 0, false, 0, 0, "inactive");
    geometry.EdgeStart_units = zeros(0, 2);
    geometry.EdgeEnd_units   = zeros(0, 2);
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

x_units            = double(obstacle.x_units{lowerIndex}(:));
y_units          = double(obstacle.y_units{lowerIndex}(:));
topologyIsInterpolated = true;
if lowerIndex == upperIndex
    speed_units_s   = preparation.SampleSpeedBound_units_s(lowerIndex);
    geometryModel = "authoritativeSample";
    if ~geometryOnly
        shape = preparation.SampleShapes{lowerIndex};
    end
    edgeStart_units = preparation.SampleEdgeStart_units{lowerIndex};
    edgeEnd_units   = preparation.SampleEdgeEnd_units{lowerIndex};
elseif preparation.MatchingTopology(lowerIndex)
    x_units   = x_units + fraction * preparation.DeltaX_units{lowerIndex};
    y_units = y_units + fraction * preparation.DeltaY_units{lowerIndex};
    speed_units_s   = preparation.IntervalSpeedBound_units_s(lowerIndex);
    geometryModel = preparation.IntervalGeometryModel(lowerIndex);
    if ~geometryOnly && speed_units_s == 0
        shape = preparation.SampleShapes{lowerIndex};
    end
    edgeStart_units = [x_units, y_units];
    edgeEnd_units   = circshift(edgeStart_units, -1, 1);
else
    shape = preparation.IntervalUnionShapes{lowerIndex};
    [x_units, y_units] = boundary(shape);
    speed_units_s            = 0;
    topologyIsInterpolated = false;
    geometryModel          = preparation.IntervalGeometryModel(lowerIndex);
    edgeStart_units          = preparation.IntervalUnionEdgeStart_units{lowerIndex};
    edgeEnd_units            = preparation.IntervalUnionEdgeEnd_units{lowerIndex};
end
x_units(~isfinite(x_units)) = NaN;
y_units(~isfinite(y_units)) = NaN;
if ~geometryOnly && (isempty(shape) || isempty(shape.Vertices))
    shape = obstacleAvoidance.geometry.boundaryToShape(x_units, y_units);
end
geometry = boundaryGeometry(x_units, y_units, speed_units_s, topologyIsInterpolated, lowerIndex, upperIndex, geometryModel);
geometry.EdgeStart_units = edgeStart_units;
geometry.EdgeEnd_units   = edgeEnd_units;
end

function geometry = boundaryGeometry(x_units, y_units, speed_units_s, topologyIsInterpolated, lowerIndex, upperIndex, geometryModel)
    % Classify one ordered boundary without changing its vertices or ring order.
    finiteVertex = isfinite(x_units) & isfinite(y_units);
    active       = nnz(finiteVertex) >= 3;
    hasOneRing   = active && all(finiteVertex);
    isConvex     = false;
    outwardSign  = NaN;
    if hasOneRing
        vertices_units          = [x_units(:), y_units(:)];
        nextVertices_units      = circshift(vertices_units, -1, 1);
        areaTerms_units2        = vertices_units(:, 1) .* nextVertices_units(:, 2) - vertices_units(:, 2) .* nextVertices_units(:, 1);
        signedDoubleArea_units2 = sum(areaTerms_units2);
        areaTolerance_units2    = 64 * eps * max(1, sum(abs(areaTerms_units2)));
        hasOneRing            = abs(signedDoubleArea_units2) > areaTolerance_units2;
        if hasOneRing
            edges_units          = nextVertices_units - vertices_units;
            nextEdges_units      = circshift(edges_units, -1, 1);
            turns_units2         = edges_units(:, 1) .* nextEdges_units(:, 2) - edges_units(:, 2) .* nextEdges_units(:, 1);
            turnTolerance_units2 = 64 * eps * max(1, max(abs(turns_units2)));
            isConvex           = all(turns_units2 >= -turnTolerance_units2) || all(turns_units2 <= turnTolerance_units2);
            outwardSign        = -sign(signedDoubleArea_units2);
        end
    end
    geometry = struct("Active", active, "x_units", double(x_units(:)), ...
        "y_units", double(y_units(:)), "VertexSpeedBound_units_s", speed_units_s, ...
        "HasOrderedSingleRegion", hasOneRing, "IsConvex", isConvex, "OutwardSign", outwardSign, ...
        "TopologyIsInterpolated", topologyIsInterpolated, ...
        "GeometryModel", string(geometryModel), ...
        "LowerSampleIndex", lowerIndex, "UpperSampleIndex", upperIndex);
end
