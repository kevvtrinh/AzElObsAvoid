function [shape, geometry] = preparedShapeAtTime( ...
    obstacle, queryTime_s, geometryOnly, classifyBoundary)
%% Section 0: Header & Readme
% SYNTAX
%   [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
%       obstacle, queryTime_s)
%   [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
%       obstacle, queryTime_s, geometryOnly, classifyBoundary)
%**************************************************************************
% PURPOSE
%   - Evaluate one prepared obstacle at a physical time.
%**************************************************************************
% INPUTS
%   - obstacle (prepared scalar obstacle)
%       Obstacle history with source-checked InternalPreparation data.
%   - queryTime_s (numeric scalar)
%       Physical time to evaluate.
%   - geometryOnly (logical scalar, optional; default false)
%       Whether to omit polyshape construction when possible.
%   - classifyBoundary (logical scalar, optional; default true)
%       Whether to classify single-ring order and convexity.
%**************************************************************************
% OUTPUTS
%   - shape (polyshape)
%       Protected shape, or empty when geometryOnly omits construction.
%   - geometry (scalar struct)
%       Protected boundary and its prepared interval model. An unprepared,
%       unsupported, or unknown interval throws an error.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Evaluate Prepared Inputs

if nargin < 3
    geometryOnly = false;
end
if nargin < 4
    classifyBoundary = true;
end
preparation = obstacle.InternalPreparation;
time_s      = double(obstacle.time_s(:));
shape       = [];
queryOutsideHistory = numel(time_s) > 1 && ...
    (queryTime_s < time_s(1) || queryTime_s > time_s(end));
if isempty(time_s) || queryOutsideHistory
    geometry = boundaryGeometry( ...
        zeros(0, 1), zeros(0, 1), 0, false, false, 0, "inactive", classifyBoundary);
    if ~geometryOnly
        shape = polyshape();
    end
    return;
end
lowerSampleIndex = find(time_s <= queryTime_s, 1, "last");
upperSampleIndex = find(time_s >= queryTime_s, 1, "first");
if isscalar(time_s)
    lowerSampleIndex = 1;
    upperSampleIndex = 1;
end
if isfield(preparation, 'SamplePrepared')
    covered = preparation.SamplePrepared(lowerSampleIndex) && preparation.SamplePrepared(upperSampleIndex);
    if lowerSampleIndex ~= upperSampleIndex
        covered = covered && preparation.IntervalPrepared(lowerSampleIndex);
    end
    assert(covered, 'preparedShapeAtTime:UnpreparedTime', ...
        'Prepare the requested time before querying internal geometry.');
end
fraction = 0;
if lowerSampleIndex ~= upperSampleIndex
    fraction = (queryTime_s - time_s(lowerSampleIndex)) / ...
        (time_s(upperSampleIndex) - time_s(lowerSampleIndex));
end

%% Section 2: Evaluate The Protected Boundary

x_units                = double(obstacle.x_units{lowerSampleIndex}(:));
y_units                = double(obstacle.y_units{lowerSampleIndex}(:));
topologyIsInterpolated = true;
usesSweptCells         = false;
if lowerSampleIndex == upperSampleIndex
    speed_units_s = preparation.SampleSpeedBound_units_s(lowerSampleIndex);
    geometryModel = "authoritativeSample";
    if ~geometryOnly
        shape = preparation.SampleShapes{lowerSampleIndex};
    end
elseif preparation.MatchingTopology(lowerSampleIndex)
    spanHasMultipleIntervals = preparation.SpanEndSampleIndex(lowerSampleIndex) > ...
        preparation.SpanStartSampleIndex(lowerSampleIndex) + 1;
    if spanHasMultipleIntervals
        firstSampleIndex = preparation.SpanStartSampleIndex(lowerSampleIndex);
        lastSampleIndex  = preparation.SpanEndSampleIndex(lowerSampleIndex);
        fraction = (queryTime_s - time_s(firstSampleIndex)) / ...
            (time_s(lastSampleIndex) - time_s(firstSampleIndex));
        x_units = (1 - fraction) * obstacle.x_units{firstSampleIndex} + ...
            fraction * obstacle.x_units{lastSampleIndex};
        y_units = (1 - fraction) * obstacle.y_units{firstSampleIndex} + ...
            fraction * obstacle.y_units{lastSampleIndex};
    else
        x_units = x_units + fraction * preparation.DeltaX_units{lowerSampleIndex};
        y_units = y_units + fraction * preparation.DeltaY_units{lowerSampleIndex};
    end
    speed_units_s = preparation.IntervalSpeedBound_units_s(lowerSampleIndex);
    geometryModel = preparation.IntervalGeometryModel(lowerSampleIndex);
    if ~geometryOnly && speed_units_s == 0
        shape = preparation.SampleShapes{lowerSampleIndex};
    end
elseif preparation.IntervalIsUnsupported(lowerSampleIndex)
    error('preparedShapeAtTime:UnsupportedContinuousDeformation', ...
        'The obstacle interval has no verified exact continuous geometry model.');
elseif preparation.IntervalIsStationary(lowerSampleIndex) || ...
        preparation.IntervalUsesSweptCells(lowerSampleIndex)
    shape = preparation.IntervalUnionShapes{lowerSampleIndex};
    [x_units, y_units] = boundary(shape);
    speed_units_s          = 0;
    topologyIsInterpolated = false;
    usesSweptCells         = preparation.IntervalUsesSweptCells(lowerSampleIndex);
    geometryModel          = preparation.IntervalGeometryModel(lowerSampleIndex);
else
    error('preparedShapeAtTime:UnknownGeometryModel', ...
        'The prepared obstacle interval has an unknown geometry model.');
end
x_units(~isfinite(x_units)) = NaN;
y_units(~isfinite(y_units)) = NaN;
if ~geometryOnly && (isempty(shape) || isempty(shape.Vertices))
    shape = obstacleAvoidance.geometry.boundaryToShape(x_units, y_units);
end
if nargout < 2
    return;
end
geometry = boundaryGeometry( ...
    x_units, y_units, speed_units_s, topologyIsInterpolated, usesSweptCells, ...
    lowerSampleIndex, geometryModel, classifyBoundary);
end

%% Section 3: Local Functions

function geometry = boundaryGeometry(x_units, y_units, speed_units_s, ...
        topologyIsInterpolated, usesSweptCells, lowerSampleIndex, geometryModel, classifyBoundary)
    % Classify one ordered boundary without changing its vertices or ring order.
    finiteVertex = isfinite(x_units) & isfinite(y_units);
    active       = nnz(finiteVertex) >= 3;
    hasOneRing   = classifyBoundary && active && all(finiteVertex);
    isConvex     = false;
    outwardSign  = NaN;
    if hasOneRing
        vertices_units          = [x_units(:), y_units(:)];
        nextVertices_units      = circshift(vertices_units, -1, 1);
        areaTerms_units2        = vertices_units(:, 1) .* nextVertices_units(:, 2) - vertices_units(:, 2) .* nextVertices_units(:, 1);
        signedDoubleArea_units2 = sum(areaTerms_units2);
        areaTolerance_units2    = 64 * eps * max(1, sum(abs(areaTerms_units2)));
        hasOneRing = abs(signedDoubleArea_units2) > areaTolerance_units2;
        if hasOneRing
            edges_units          = nextVertices_units - vertices_units;
            nextEdges_units      = circshift(edges_units, -1, 1);
            turns_units2         = edges_units(:, 1) .* nextEdges_units(:, 2) - edges_units(:, 2) .* nextEdges_units(:, 1);
            turnTolerance_units2 = 64 * eps * max(1, max(abs(turns_units2)));
            isConvex    = all(turns_units2 >= -turnTolerance_units2) || all(turns_units2 <= turnTolerance_units2);
            outwardSign = -sign(signedDoubleArea_units2);
        end
    end
    geometry = struct( ...
        "Active",                    active, ...
        "x_units",                   double(x_units(:)), ...
        "y_units",                   double(y_units(:)), ...
        "VertexSpeedBound_units_s",  speed_units_s, ...
        "HasOrderedSingleRegion",    hasOneRing, ...
        "IsConvex",                  isConvex, ...
        "OutwardSign",               outwardSign, ...
        "TopologyIsInterpolated",    topologyIsInterpolated, ...
        "UsesSweptCells",            usesSweptCells, ...
        "GeometryModel",             string(geometryModel), ...
        "LowerSampleIndex",          lowerSampleIndex);
end
