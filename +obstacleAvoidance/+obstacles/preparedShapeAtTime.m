function [protectedShape, shapeDetails] = preparedShapeAtTime( ...
    obstacle, requestedTime_s, geometryOnly, classifyBoundary)
%% Section 0: Header & Readme
% SYNTAX
%   [protectedShape, shapeDetails] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
%       obstacle, requestedTime_s)
%   [protectedShape, shapeDetails] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
%       obstacle, requestedTime_s, geometryOnly, classifyBoundary)
%**************************************************************************
% PURPOSE
%   - Return the protected obstacle shape at the requested time, using
%     the boundary motion or interval enclosure verified during preparation.
%**************************************************************************
% INPUTS
%   - obstacle (prepared scalar obstacle)
%       Obstacle history already prepared for the requested time.
%   - requestedTime_s (numeric scalar)
%       Time at which the obstacle shape is needed.
%   - geometryOnly (logical scalar, optional; default false)
%       Whether to omit polyshape construction when possible.
%   - classifyBoundary (logical scalar, optional; default true)
%       Whether to check for one ordered boundary loop and no inward corners.
%**************************************************************************
% OUTPUTS
%   - protectedShape (polyshape or empty array)
%       Protected shape, or empty when geometryOnly omits construction.
%   - shapeDetails (scalar struct)
%       Protected vertices, speed bound, and boundary properties. An
%       unprepared, unsupported, or unknown interval throws an error.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Locate The Requested Time In The Prepared History

if nargin < 3
    geometryOnly = false;
end
if nargin < 4
    classifyBoundary = true;
end

obstaclePreparation = obstacle.InternalPreparation;
sampleTimes_s       = double(obstacle.time_s(:));
protectedShape      = [];

% A single sample applies at every time. A history with several samples
% has no obstacle before its first time or after its last time.
requestedTimeIsOutsideHistory = numel(sampleTimes_s) > 1 && ...
    (requestedTime_s < sampleTimes_s(1) || requestedTime_s > sampleTimes_s(end));
if isempty(sampleTimes_s) || requestedTimeIsOutsideHistory
    shapeDetails = createBoundaryDetails( ...
        zeros(0, 1), zeros(0, 1), 0, false, false, 0, classifyBoundary);
    if ~geometryOnly
        protectedShape = polyshape();
    end
    return;
end

% At a recorded time both indices select the same sample. Between samples,
% they select the two boundaries needed to calculate the shape.
intervalStartSampleIndex = find(sampleTimes_s <= requestedTime_s, 1, "last");
intervalEndSampleIndex   = find(sampleTimes_s >= requestedTime_s, 1, "first");
if isscalar(sampleTimes_s)
    intervalStartSampleIndex = 1;
    intervalEndSampleIndex   = 1;
end

% Partial preparation must include both endpoints and the interval between
% them before its boundary can be evaluated.
if isfield(obstaclePreparation, 'SamplePrepared')
    requestedTimeWasPrepared = obstaclePreparation.SamplePrepared(intervalStartSampleIndex) && ...
        obstaclePreparation.SamplePrepared(intervalEndSampleIndex);
    if intervalStartSampleIndex ~= intervalEndSampleIndex
        requestedTimeWasPrepared = requestedTimeWasPrepared && ...
            obstaclePreparation.IntervalPrepared(intervalStartSampleIndex);
    end
    assert(requestedTimeWasPrepared, 'preparedShapeAtTime:UnpreparedTime', ...
        'Prepare the requested time before querying internal geometry.');
end

% For example, time 4 in [2 6] is halfway through the interval: fraction = 0.5.
motionFraction = 0;
if intervalStartSampleIndex ~= intervalEndSampleIndex
    motionFraction = (requestedTime_s - sampleTimes_s(intervalStartSampleIndex)) / ...
        (sampleTimes_s(intervalEndSampleIndex) - sampleTimes_s(intervalStartSampleIndex));
end

%% Section 2: Evaluate The Protected Boundary

x_units = double(obstacle.x_units{intervalStartSampleIndex}(:));
y_units = double(obstacle.y_units{intervalStartSampleIndex}(:));

topologyIsInterpolated = true;
usesMovingCells        = false;

% Reuse a recorded boundary exactly when its sample time was requested.
if intervalStartSampleIndex == intervalEndSampleIndex
    vertexSpeedBound_units_s = obstaclePreparation.SampleSpeedBound_units_s(intervalStartSampleIndex);
    if ~geometryOnly
        protectedShape = obstaclePreparation.SampleShapes{intervalStartSampleIndex};
    end
elseif obstaclePreparation.MatchingTopology(intervalStartSampleIndex)
    % Corresponding vertices move along straight lines. Preparation can
    % combine adjacent intervals with the same motion into one longer span.
    spanHasMultipleIntervals = obstaclePreparation.SpanEndSampleIndex(intervalStartSampleIndex) > ...
        obstaclePreparation.SpanStartSampleIndex(intervalStartSampleIndex) + 1;
    if spanHasMultipleIntervals
        firstSampleIndex = obstaclePreparation.SpanStartSampleIndex(intervalStartSampleIndex);
        lastSampleIndex  = obstaclePreparation.SpanEndSampleIndex(intervalStartSampleIndex);
        motionFraction   = (requestedTime_s - sampleTimes_s(firstSampleIndex)) / ...
            (sampleTimes_s(lastSampleIndex) - sampleTimes_s(firstSampleIndex));
        x_units = (1 - motionFraction) * obstacle.x_units{firstSampleIndex} + ...
            motionFraction * obstacle.x_units{lastSampleIndex};
        y_units = (1 - motionFraction) * obstacle.y_units{firstSampleIndex} + ...
            motionFraction * obstacle.y_units{lastSampleIndex};
    else
        x_units = x_units + motionFraction * obstaclePreparation.DeltaX_units{intervalStartSampleIndex};
        y_units = y_units + motionFraction * obstaclePreparation.DeltaY_units{intervalStartSampleIndex};
    end
    vertexSpeedBound_units_s = obstaclePreparation.IntervalSpeedBound_units_s(intervalStartSampleIndex);
    if ~geometryOnly && vertexSpeedBound_units_s == 0
        protectedShape = obstaclePreparation.SampleShapes{intervalStartSampleIndex};
    end
elseif obstaclePreparation.IntervalIsUnsupported(intervalStartSampleIndex)
    error('preparedShapeAtTime:UnsupportedContinuousDeformation', ...
        'The obstacle interval has no verified exact continuous geometry model.');
elseif obstaclePreparation.IntervalIsStationary(intervalStartSampleIndex) || ...
        obstaclePreparation.IntervalUsesMovingCells(intervalStartSampleIndex) || ...
        obstaclePreparation.IntervalUsesEndpointHull(intervalStartSampleIndex)
    % These models use a fixed shape covering the whole interval. Its
    % boundary does not move, so its speed bound is zero.
    protectedShape = obstaclePreparation.IntervalUnionShapes{intervalStartSampleIndex};
    [x_units, y_units] = boundary(protectedShape);
    vertexSpeedBound_units_s = 0;
    topologyIsInterpolated   = false;
    usesMovingCells          = obstaclePreparation.IntervalUsesMovingCells(intervalStartSampleIndex);
else
    error('preparedShapeAtTime:UnknownGeometryModel', ...
        'The prepared obstacle interval has an unknown geometry model.');
end

%% Section 3: Return The Shape And Requested Boundary Details

% NaN separates boundary loops. The margin is already included in these
% vertices; building a polyshape here must not add it again.
x_units(~isfinite(x_units)) = NaN;
y_units(~isfinite(y_units)) = NaN;
if ~geometryOnly && (isempty(protectedShape) || isempty(protectedShape.Vertices))
    protectedShape = obstacleAvoidance.geometry.boundaryToShape(x_units, y_units);
end
if nargout < 2
    return;
end
shapeDetails = createBoundaryDetails( ...
    x_units, y_units, vertexSpeedBound_units_s, topologyIsInterpolated, usesMovingCells, ...
    intervalStartSampleIndex, classifyBoundary);
end

%% Section 4: Local Functions

function shapeDetails = createBoundaryDetails(x_units, y_units, vertexSpeedBound_units_s, ...
        topologyIsInterpolated, usesMovingCells, intervalStartSampleIndex, classifyBoundary)
    % Describe the boundary without changing its vertices or loop order.
    % Boundaries with NaN separators need general polygon checks.
    vertexIsFinite        = isfinite(x_units) & isfinite(y_units);
    hasEnoughVertices     = nnz(vertexIsFinite) >= 3;
    hasSingleBoundaryLoop = classifyBoundary && hasEnoughVertices && all(vertexIsFinite);
    boundaryIsConvex      = false;
    outwardNormalSign     = NaN;
    if hasSingleBoundaryLoop
        vertices_units     = [x_units(:), y_units(:)];
        nextVertices_units = circshift(vertices_units, -1, 1);
        areaTerms_units2   = vertices_units(:, 1) .* nextVertices_units(:, 2) - ...
            vertices_units(:, 2) .* nextVertices_units(:, 1);

        % Signed area identifies clockwise versus counterclockwise order.
        % Ignore area smaller than the rounding tolerance when classifying it.
        signedDoubleArea_units2 = sum(areaTerms_units2);
        areaTolerance_units2    = 64 * eps * max(1, sum(abs(areaTerms_units2)));
        hasSingleBoundaryLoop   = abs(signedDoubleArea_units2) > areaTolerance_units2;
        if hasSingleBoundaryLoop
            % A convex loop turns the same way at every corner. Allow tiny
            % rounding differences around zero for points on a straight edge.
            edgeVectors_units        = nextVertices_units - vertices_units;
            nextEdgeVectors_units    = circshift(edgeVectors_units, -1, 1);
            turnCrossProducts_units2 = edgeVectors_units(:, 1) .* nextEdgeVectors_units(:, 2) - ...
                edgeVectors_units(:, 2) .* nextEdgeVectors_units(:, 1);
            turnTolerance_units2 = 64 * eps * max(1, max(abs(turnCrossProducts_units2)));
            boundaryIsConvex     = all(turnCrossProducts_units2 >= -turnTolerance_units2) || ...
                all(turnCrossProducts_units2 <= turnTolerance_units2);

            % +1 selects the left side of an edge; -1 selects the right.
            % For a counterclockwise boundary, the outside is on the right.
            outwardNormalSign = -sign(signedDoubleArea_units2);
        end
    end
    shapeDetails = struct( ...
        "Active",                   hasEnoughVertices, ...
        "x_units",                  double(x_units(:)), ...
        "y_units",                  double(y_units(:)), ...
        "VertexSpeedBound_units_s", vertexSpeedBound_units_s, ...
        "HasOrderedSingleRegion",   hasSingleBoundaryLoop, ...
        "IsConvex",                 boundaryIsConvex, ...
        "OutwardSign",              outwardNormalSign, ...
        "TopologyIsInterpolated",   topologyIsInterpolated, ...
        "UsesMovingCells",          usesMovingCells, ...
        "LowerSampleIndex",         intervalStartSampleIndex);
end
