function [protectedShape, shapeDetails] = preparedShapeAtTime( ...
    obstacle, requestedTime_s, geometryOnly)
%% Section 0: Header & Readme
% SYNTAX
%   [protectedShape, shapeDetails] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
%       obstacle, requestedTime_s)
%   [protectedShape, shapeDetails] = obstacleAvoidance.obstacles.preparedShapeAtTime( ...
%       obstacle, requestedTime_s, geometryOnly)
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
%**************************************************************************
% OUTPUTS
%   - protectedShape (polyshape or empty array)
%       Protected shape, or empty when geometryOnly omits construction.
%   - shapeDetails (scalar struct)
%       Protected vertices and the prepared interval used. An
%       unprepared, unsupported, or unknown interval throws an error.
%**************************************************************************
% UNITS
%   - Position uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Locate The Requested Time In The Prepared History

if nargin < 3
    geometryOnly = false;
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
        zeros(0, 1), zeros(0, 1), false, 0);
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

usesMovingCells = false;

% Reuse a recorded boundary exactly when its sample time was requested.
if intervalStartSampleIndex == intervalEndSampleIndex
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
    % These models use a fixed shape covering the whole interval.
    protectedShape = obstaclePreparation.IntervalUnionShapes{intervalStartSampleIndex};
    [x_units, y_units] = boundary(protectedShape);
    usesMovingCells = obstaclePreparation.IntervalUsesMovingCells(intervalStartSampleIndex);
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
    x_units, y_units, usesMovingCells, intervalStartSampleIndex);
end

%% Section 4: Local Functions

function shapeDetails = createBoundaryDetails(x_units, y_units, usesMovingCells, intervalStartSampleIndex)
    % Report protected coordinates and the prepared interval used by snapshots.
    hasEnoughVertices = nnz(isfinite(x_units) & isfinite(y_units)) >= 3;
    shapeDetails = struct( ...
        "Active",           hasEnoughVertices, ...
        "x_units",          double(x_units(:)), ...
        "y_units",          double(y_units(:)), ...
        "UsesMovingCells",  usesMovingCells, ...
        "LowerSampleIndex", intervalStartSampleIndex);
end
