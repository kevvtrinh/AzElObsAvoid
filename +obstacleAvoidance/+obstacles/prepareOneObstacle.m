function obstacle = prepareOneObstacle(obstacle, preparationVersion, sourceSnapshot, ...
    timeRange_s, previousPreparation, stopAtUnsupported)
%% Section 0: Header & Readme
% SYNTAX
%   obstacle = obstacleAvoidance.obstacles.prepareOneObstacle( ...
%       obstacle, preparationVersion, sourceSnapshot, timeRange_s, ...
%       previousPreparation, stopAtUnsupported)
%**************************************************************************
% PURPOSE
%   - Prepare the sampled obstacle shapes and the occupied space between
%     samples for the requested time range.
%**************************************************************************
% INPUTS
%   - obstacle (scalar struct)
%       Obstacle history in the standard planner format.
%   - preparationVersion (positive integer scalar)
%       Version stored with derived preparation data.
%   - sourceSnapshot (scalar struct)
%       Copy of the obstacle inputs. Later calls compare these fields before
%       reusing prepared geometry.
%   - timeRange_s (1-by-2 numeric row)
%       Earliest and latest requested times. Prepare the samples and
%       intervals needed to cover this range.
%   - previousPreparation (scalar struct or empty)
%       Saved preparation for these same obstacle inputs, if available.
%   - stopAtUnsupported (logical scalar)
%       Stop when a requested interval has no supported geometry model.
%       Otherwise, continue preparing later intervals too.
%**************************************************************************
% OUTPUTS
%   - obstacle (prepared scalar obstacle)
%       InternalPreparation stores the sample shapes, interval models, and
%       flags showing which requested times have been prepared.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Select Samples And Intervals Needed For This Request

validateattributes(preparationVersion, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
time_s        = obstacle.time_s;
sampleCount   = numel(time_s);
intervalCount = sampleCount - 1;
% A sample is a shape at one time. An interval is the time between two
% samples. Include both endpoints of every interval the request overlaps.
neededSamples   = time_s >= timeRange_s(1) & time_s <= timeRange_s(2);
neededIntervals = time_s(1:end - 1) < timeRange_s(2) & time_s(2:end) > timeRange_s(1);
intervalIndices = find(neededIntervals);

neededSamples([intervalIndices; intervalIndices + 1]) = true;
if sampleCount == 1
    neededSamples(1) = true;
end

%% Section 2: Create Or Reuse Prepared Obstacle Data

if isempty(previousPreparation)
    usesUnbufferedSourceIndex = obstacle.UsesSourceIndex && ...
        obstacle.safetyMargin_units == 0;
    preparation = struct( ...
        'PreparationVersion',                              preparationVersion, ...
        'SourceSnapshot',                                  sourceSnapshot, ...
        'SamplePrepared',                                  false(sampleCount, 1), ...
        'IntervalPrepared',                                false(intervalCount, 1), ...
        'SampleShapes',                                    {cell(sampleCount, 1)}, ...
        'SampleEdgeStart_units',                           {cell(sampleCount, 1)}, ...
        'SampleEdgeEnd_units',                             {cell(sampleCount, 1)}, ...
        'IntervalUnionShapes',                             {cell(intervalCount, 1)}, ...
        'IntervalUnionEdgeStart_units',                    {cell(intervalCount, 1)}, ...
        'IntervalUnionEdgeEnd_units',                      {cell(intervalCount, 1)}, ...
        'IntervalStartRegions_units',                      {cell(intervalCount, 1)}, ...
        'IntervalEndRegions_units',                        {cell(intervalCount, 1)}, ...
        'DeltaX_units',                                    {cell(intervalCount, 1)}, ...
        'DeltaY_units',                                    {cell(intervalCount, 1)}, ...
        'MatchingTopology',                                false(intervalCount, 1), ...
        'IntervalGeometryModel',                           strings(intervalCount, 1), ...
        'IntervalHasExactPartition',                       false(intervalCount, 1), ...
        'IntervalIsStationary',                            false(intervalCount, 1), ...
        'IntervalUsesMovingCells',                         false(intervalCount, 1), ...
        'IntervalUsesEndpointHull',                        false(intervalCount, 1), ...
        'IntervalIsUnsupported',                           false(intervalCount, 1), ...
        'IntervalPartitionReused',                         false(intervalCount, 1), ...
        'IntervalMovingCellCount',                         zeros(intervalCount, 2), ...
        'IntervalMovingCellTiming_s',                      zeros(intervalCount, 4), ...
        'IntervalMovingCellUncoveredProtectedArea_units2', zeros(intervalCount, 2), ...
        'IntervalEndpointHullAddedArea_units2',            zeros(intervalCount, 1), ...
        'IntervalProofReason',                             strings(intervalCount, 1), ...
        'SpanStartSampleIndex',                            (1:intervalCount).', ...
        'SpanEndSampleIndex',                              (2:sampleCount).', ...
        'MergedIntervalCount',                             0, ...
        'RejectedMergeSpanSampleIndex',                    zeros(0, 2), ...
        'CandidateSpanEndSampleIndex',                     findCandidateSpanEnds(obstacle, usesUnbufferedSourceIndex), ...
        'IntervalSpeedBound_units_s',                      Inf(intervalCount, 1), ...
        'SampleSpeedBound_units_s',                        Inf(sampleCount, 1), ...
        'IsTimeInvariant',                                 false);
    % Identical coordinates at every sample establish a static shape.
    % Multiple samples still limit its active time range; one applies at all times.
    preparation.SamplesExactlyEqual = all(cellfun( ...
        @(x, y) isequaln(x, obstacle.x_units{1}) && isequaln(y, obstacle.y_units{1}), ...
        obstacle.x_units, obstacle.y_units));
    preparation.IsTimeInvariant = preparation.SamplesExactlyEqual;
else
    preparation = previousPreparation;
end
% A previous call may already have found an unsupported interval. Preserve
% that result so the planner can report why this request cannot proceed.
if stopAtUnsupported && any(neededIntervals & preparation.IntervalPrepared & ...
        preparation.IntervalIsUnsupported)
    obstacle.InternalPreparation = preparation;
    return;
end

% A span is a group of consecutive intervals proposed to share one linear
% vertex-motion model. Prepare the whole group when any part is requested,
% so later requests and independent validation use the same geometry.
spanEndSampleIndices   = preparation.CandidateSpanEndSampleIndex;
spanStartSampleIndices = [1; find(diff(spanEndSampleIndices) ~= 0) + 1];
if isempty(spanEndSampleIndices)
    spanStartSampleIndices = zeros(0, 1);
end
for firstIntervalIndex = reshape(spanStartSampleIndices, 1, [])
    spanIntervalIndices = firstIntervalIndex:spanEndSampleIndices(firstIntervalIndex) - 1;
    if any(neededIntervals(spanIntervalIndices))
        neededIntervals(spanIntervalIndices) = true;
        neededSamples(firstIntervalIndex:spanEndSampleIndices(firstIntervalIndex)) = true;
    end
end

%% Section 3: Prepare New Samples And The Space Occupied Between Them

% Separate intervals can be prepared independently. Groups proposed for
% merging stay sequential because rejecting a merge changes the next group.
newIntervalIndices         = find(neededIntervals & ~preparation.IntervalPrepared);
intervalCanRunIndependently = preparation.CandidateSpanEndSampleIndex == (2:sampleCount).' & ...
    [true; diff(preparation.CandidateSpanEndSampleIndex) ~= 0];
independentIntervalIndices = newIntervalIndices(intervalCanRunIndependently(newIntervalIndices));
sampleResults              = cell(sampleCount, 1);
intervalResults            = cell(intervalCount, 1);

% Declared vertex matches apply directly only when no margin changed the
% vertices. With a margin, test translation before trying enclosure models.
protectedVerticesKeepIndices = obstacle.UsesSourceIndex && obstacle.safetyMargin_units == 0;
onlyCheckTranslation         = obstacle.UsesSourceIndex && obstacle.safetyMargin_units > 0;
useBackgroundWorkers         = false;
% The 32-interval threshold avoids worker overhead on small histories.
% Three paired R2024b runs measured median preparation times of 0.884 s
% serial and 0.491 s with four batches, including dispatch and collection.
parallelIntervalThreshold = 32;
% The owner allows at most four workers on this machine so that other MATLAB
% processes running beside this one keep cores of their own.
maximumWorkerCount = 4;
if numel(independentIntervalIndices) >= parallelIntervalThreshold && exist("backgroundPool", "builtin") == 5
    workerPool           = backgroundPool;
    useBackgroundWorkers = workerPool.NumWorkers > 1 && ~workerPool.Busy;
end
if useBackgroundWorkers
    workerCount            = min(maximumWorkerCount, workerPool.NumWorkers);
    sampleIndices          = find(neededSamples & ~preparation.SamplePrepared);
    intervalSampleIndices  = unique([independentIntervalIndices; independentIntervalIndices + 1]);
    remainingSampleIndices = setdiff(sampleIndices, intervalSampleIndices, 'stable');

    preparationFutures(1, workerCount) = parallel.FevalFuture;
    batchSampleIndices   = cell(workerCount, 1);
    batchIntervalIndices = cell(workerCount, 1);

    % Each worker gets a group of independent intervals and the sample
    % shapes they need. Distribute any remaining samples across the workers.
    for workerIndex = 1:workerCount
        firstBatchIndex = floor((workerIndex - 1) * numel(independentIntervalIndices) / workerCount) + 1;
        finalBatchIndex = floor(workerIndex * numel(independentIntervalIndices) / workerCount);
        batchIntervalIndices{workerIndex} = independentIntervalIndices(firstBatchIndex:finalBatchIndex);
        extraSampleIndices = remainingSampleIndices(workerIndex:workerCount:end);
        batchSampleIndices{workerIndex} = unique([ ...
            batchIntervalIndices{workerIndex}; ...
            batchIntervalIndices{workerIndex} + 1; ...
            extraSampleIndices]);
        [~, lowerSampleIndex] = ismember( ...
            batchIntervalIndices{workerIndex}, batchSampleIndices{workerIndex});
        [~, upperSampleIndex] = ismember( ...
            batchIntervalIndices{workerIndex} + 1, batchSampleIndices{workerIndex});
        preparationFutures(workerIndex) = parfeval(workerPool, @prepareCombinedBatch, 2, ...
            obstacle.x_units(batchSampleIndices{workerIndex}), ...
            obstacle.y_units(batchSampleIndices{workerIndex}), ...
            lowerSampleIndex, upperSampleIndex, protectedVerticesKeepIndices, onlyCheckTranslation);
    end
    % Cancel outstanding work if collection fails or this function exits.
    cancelWorkersOnExit = onCleanup(@() cancel(preparationFutures));
    for workerIndex = 1:workerCount
        [batchSampleResults, batchIntervalResults] = fetchOutputs(preparationFutures(workerIndex));
        sampleResults(batchSampleIndices{workerIndex})     = batchSampleResults;
        intervalResults(batchIntervalIndices{workerIndex}) = batchIntervalResults;
    end
    clear cancelWorkersOnExit;
end

% Apply results in history order, whether calculated here or by a worker.
% This keeps reuse of the preceding interval and reported errors consistent.
for intervalIndex = reshape(find(neededIntervals & ~preparation.IntervalPrepared), 1, [])
    if preparation.IntervalPrepared(intervalIndex)
        % An earlier iteration may have prepared this interval as part of a span.
        continue;
    end
    preparation = prepareSourceInterval(preparation, obstacle, intervalIndex, ...
        sampleResults, intervalResults, protectedVerticesKeepIndices, onlyCheckTranslation);
    if stopAtUnsupported && preparation.IntervalIsUnsupported(intervalIndex)
        break;
    end
end
% A request at an exact sample time may need a shape but no interval model.
% Finish these remaining samples unless preparation deliberately stopped early.
if ~stopAtUnsupported || ~any(neededIntervals & preparation.IntervalPrepared & ...
        preparation.IntervalIsUnsupported)
    preparation = prepareSamples(preparation, obstacle, find(neededSamples).', sampleResults);
end

%% Section 4: Update Cached Motion Bounds And Static Status

% At a sample, use the larger speed bound from its two neighboring intervals.
preparation.SampleSpeedBound_units_s = max([0; preparation.IntervalSpeedBound_units_s], ...
    [preparation.IntervalSpeedBound_units_s; 0]);
% An interval enclosure can occupy more space than its endpoint shapes.
% These shapes can jump at sample times, so do not assume a finite speed there.
enclosureIntervalIndices = find(preparation.IntervalUsesMovingCells | ...
    preparation.IntervalUsesEndpointHull);
preparation.SampleSpeedBound_units_s(unique( ...
    [enclosureIntervalIndices; enclosureIntervalIndices + 1])) = Inf;
staticIntervals = preparation.IntervalIsStationary | ...
    (preparation.MatchingTopology & preparation.IntervalSpeedBound_units_s == 0);
preparation.IsTimeInvariant = preparation.IsTimeInvariant || ...
    (all(preparation.IntervalPrepared) && all(staticIntervals));
if preparation.IsTimeInvariant
    preparation.SampleSpeedBound_units_s(:) = 0;
end
mergedSpanSampleIndex = unique([preparation.SpanStartSampleIndex, ...
    preparation.SpanEndSampleIndex], 'rows', 'stable');
preparation.MergedSpanTime_s = reshape(time_s(mergedSpanSampleIndex), ...
    size(mergedSpanSampleIndex));
obstacle.InternalPreparation = preparation;
end

%% Section 5: Local Functions

function preparation = prepareSourceInterval(preparation, obstacle, intervalIndex, ...
        sampleResults, intervalResults, protectedVerticesKeepIndices, onlyCheckTranslation)
    % Choose how to represent occupied space between these samples:
    % verified vertex motion, an unchanged shape, moving-cell enclosures,
    % then an enclosure around both endpoint shapes. If none is supported,
    % mark the interval unsupported so planning can report it.
    time_s           = obstacle.time_s;
    finalSampleIndex = preparation.CandidateSpanEndSampleIndex(intervalIndex);
    preparation = prepareSamples(preparation, obstacle, intervalIndex:finalSampleIndex, sampleResults);

    lowerX_units = obstacle.x_units{intervalIndex};
    lowerY_units = obstacle.y_units{intervalIndex};
    upperX_units = obstacle.x_units{finalSampleIndex};
    upperY_units = obstacle.y_units{finalSampleIndex};
    reusableStartRegions_units   = cell(0, 1);
    precedingPartitionIsReusable = intervalIndex > 1 && ...
        preparation.IntervalPrepared(intervalIndex - 1) && ...
        preparation.IntervalHasExactPartition(intervalIndex - 1) && ...
        ~isempty(preparation.IntervalEndRegions_units{intervalIndex - 1});
    % A partition divides the obstacle into convex regions. A translating
    % interval can reuse the preceding end regions as its starting regions.
    if precedingPartitionIsReusable
        reusableStartRegions_units = preparation.IntervalEndRegions_units{intervalIndex - 1};
    end
    % Matching vertex indices refer to the original obstacle boundary.
    % Adding a safety margin can change those vertices. In that case, only
    % simple translation can use the protected vertices directly; other
    % motion models must start from the original boundary.
    usesSourceIndex     = obstacle.UsesSourceIndex;
    preserveAlignment   = protectedVerticesKeepIndices || finalSampleIndex > intervalIndex + 1;
    intervalPreparation = intervalResults{intervalIndex};
    if isa(intervalPreparation, 'MException')
        rethrow(intervalPreparation);
    end
    if isempty(intervalPreparation)
        [interpolationIsVerified, alignedUpper_units, startRegions_units, endRegions_units, ...
            geometryModel, hasExactPartition, partitionReused] = ...
            obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
            createBoundarySample(lowerX_units, lowerY_units, preparation.SampleShapes{intervalIndex}), ...
            createBoundarySample(upperX_units, upperY_units, preparation.SampleShapes{finalSampleIndex}), ...
            reusableStartRegions_units, ...
            createIntervalCheckOptions(preserveAlignment, onlyCheckTranslation, false));
    else
        [interpolationIsVerified, alignedUpper_units, startRegions_units, endRegions_units, ...
            geometryModel, hasExactPartition, partitionReused] = intervalPreparation{:};
    end
    if ~interpolationIsVerified && finalSampleIndex > intervalIndex + 1
        % Equal vertex velocities suggested merging these intervals, but
        % the full group did not pass the geometry check. Prepare the original
        % intervals separately.
        preparation.RejectedMergeSpanSampleIndex(end + 1, :) = [intervalIndex, finalSampleIndex];
        preparation.CandidateSpanEndSampleIndex(intervalIndex:finalSampleIndex - 1) = ...
            (intervalIndex + 1:finalSampleIndex).';
        finalSampleIndex = intervalIndex + 1;
        upperX_units     = obstacle.x_units{finalSampleIndex};
        upperY_units     = obstacle.y_units{finalSampleIndex};
        [interpolationIsVerified, alignedUpper_units, startRegions_units, endRegions_units, ...
            geometryModel, hasExactPartition, partitionReused] = ...
            obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
            createBoundarySample(lowerX_units, lowerY_units, preparation.SampleShapes{intervalIndex}), ...
            createBoundarySample(upperX_units, upperY_units, preparation.SampleShapes{finalSampleIndex}), ...
            reusableStartRegions_units, ...
            createIntervalCheckOptions(protectedVerticesKeepIndices, onlyCheckTranslation, false));
    end
    intervalIsStationary     = false;
    intervalUsesMovingCells  = false;
    intervalUsesEndpointHull = false;
    intervalIsUnsupported    = false;
    intervalProofReason      = "";

    preparation.MatchingTopology(intervalIndex)        = interpolationIsVerified;
    preparation.IntervalPartitionReused(intervalIndex) = partitionReused;
    if interpolationIsVerified
        % Each vertex follows a straight line between its matched positions.
        % speed = distance between positions / time between samples.
        preparation.DeltaX_units{intervalIndex} = alignedUpper_units(:, 1) - lowerX_units;
        preparation.DeltaY_units{intervalIndex} = alignedUpper_units(:, 2) - lowerY_units;
        preparation.IntervalStartRegions_units{intervalIndex} = startRegions_units;
        preparation.IntervalEndRegions_units{intervalIndex}   = endRegions_units;
        vertexSpeed_units_s = hypot( ...
            preparation.DeltaX_units{intervalIndex}, ...
            preparation.DeltaY_units{intervalIndex}) / ...
            (time_s(finalSampleIndex) - time_s(intervalIndex));
        preparation.IntervalSpeedBound_units_s(intervalIndex) = ...
            max([0; vertexSpeed_units_s(isfinite(vertexSpeed_units_s))]);
    else
        firstShape           = preparation.SampleShapes{intervalIndex};
        lastShape            = preparation.SampleShapes{finalSampleIndex};
        sampleShapesAreEqual = compareShapes(firstShape, lastShape);
        if sampleShapesAreEqual
            shape                = firstShape;
            geometryModel        = "staticEquivalentSamples";
            intervalIsStationary = true;
        else
            % Build regions covering the motion of the original boundary,
            % then include its safety margin once in those regions.
            lowerOriginal_units = [obstacle.originalX_units{intervalIndex}, ...
                obstacle.originalY_units{intervalIndex}];
            upperOriginal_units = [obstacle.originalX_units{finalSampleIndex}, ...
                obstacle.originalY_units{finalSampleIndex}];
            [movingCellsSupported, movingCellShape, movingCellRegions_units, movingCellCounts, movingCellTiming_s] = ...
                obstacleAvoidance.obstacles.createMovingCells( ...
                lowerOriginal_units, upperOriginal_units, ...
                obstacle.safetyMargin_units, usesSourceIndex);
            if movingCellsSupported
                % The region covering motion between samples must contain
                % both supplied shapes, including their safety margins.
                uncoveredArea_units2 = [area(subtract(firstShape, movingCellShape)), ...
                    area(subtract(lastShape, movingCellShape))];
                preparation.IntervalMovingCellUncoveredProtectedArea_units2(intervalIndex, :) = ...
                    uncoveredArea_units2;
                % Allow only area differences on the scale of floating-point
                % roundoff when comparing the two polygon constructions.
                areaTolerance_units2 = 4096 * eps(max([1, area(firstShape), area(lastShape)]));
                movingCellsSupported = all(uncoveredArea_units2 <= areaTolerance_units2);
            end
            if movingCellsSupported
                shape         = movingCellShape;
                geometryModel = "movingConvexCells";
                preparation.IntervalStartRegions_units{intervalIndex} = movingCellRegions_units;
                preparation.IntervalEndRegions_units{intervalIndex}   = movingCellRegions_units;
                intervalUsesMovingCells = true;
            else
                % With no usable moving-cell enclosure, surround both sample
                % boundaries with a convex hull, like a taut rubber band.
                % This may block extra space; record its added area below.
                lowerProtected_units = [lowerX_units, lowerY_units];
                upperProtected_units = [upperX_units, upperY_units];
                [hullSupported, endpointHullShape, endpointHullRegions_units, addedArea_units2] = ...
                    obstacleAvoidance.obstacles.buildEndpointHullCell( ...
                    lowerProtected_units, upperProtected_units, firstShape, lastShape);
                if hullSupported
                    shape         = endpointHullShape;
                    geometryModel = "endpointConvexHull";
                    preparation.IntervalStartRegions_units{intervalIndex} = endpointHullRegions_units;
                    preparation.IntervalEndRegions_units{intervalIndex}   = endpointHullRegions_units;
                    preparation.IntervalEndpointHullAddedArea_units2(intervalIndex) = addedArea_units2;
                    intervalUsesEndpointHull = true;
                else
                    shape                 = polyshape();
                    geometryModel         = "unsupportedContinuousDeformation";
                    intervalProofReason   = "degenerateEndpointGeometry";
                    intervalIsUnsupported = true;
                end
            end
            preparation.IntervalMovingCellCount(intervalIndex, :)    = movingCellCounts;
            preparation.IntervalMovingCellTiming_s(intervalIndex, :) = movingCellTiming_s;
        end
        preparation.IntervalUnionShapes{intervalIndex}       = shape;
        preparation.IntervalSpeedBound_units_s(intervalIndex) = 0;
        [preparation.IntervalUnionEdgeStart_units{intervalIndex}, ...
            preparation.IntervalUnionEdgeEnd_units{intervalIndex}] = ...
            obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
    end
    classifiedIntervalIndices = intervalIndex;
    if finalSampleIndex > intervalIndex + 1
        % Split the verified group model back into the original time
        % intervals, keeping the same region order and supplied sample shapes.
        spanDisplacement_units = [preparation.DeltaX_units{intervalIndex}, ...
            preparation.DeltaY_units{intervalIndex}];
        spanStartRegions_units = startRegions_units;
        spanEndRegions_units   = endRegions_units;
        for spanIntervalIndex = intervalIndex:finalSampleIndex - 1
            timeFraction = (time_s(spanIntervalIndex:spanIntervalIndex + 1) - time_s(intervalIndex)) / ...
                (time_s(finalSampleIndex) - time_s(intervalIndex));
            preparation.DeltaX_units{spanIntervalIndex} = diff(timeFraction) * spanDisplacement_units(:, 1);
            preparation.DeltaY_units{spanIntervalIndex} = diff(timeFraction) * spanDisplacement_units(:, 2);
            for regionIndex = 1:numel(spanStartRegions_units)
                regionDisplacement_units = spanEndRegions_units{regionIndex} - spanStartRegions_units{regionIndex};
                startRegions_units{regionIndex} = ...
                    spanStartRegions_units{regionIndex} + timeFraction(1) * regionDisplacement_units;
                endRegions_units{regionIndex} = ...
                    spanStartRegions_units{regionIndex} + timeFraction(2) * regionDisplacement_units;
            end
            preparation.IntervalStartRegions_units{spanIntervalIndex} = startRegions_units;
            preparation.IntervalEndRegions_units{spanIntervalIndex}   = endRegions_units;
        end
        spanIntervalIndices       = intervalIndex:finalSampleIndex - 1;
        classifiedIntervalIndices = spanIntervalIndices;

        preparation.IntervalPrepared(spanIntervalIndices) = true;
        preparation.MatchingTopology(spanIntervalIndices) = true;
        preparation.IntervalSpeedBound_units_s(spanIntervalIndices) = ...
            preparation.IntervalSpeedBound_units_s(intervalIndex);
        preparation.IntervalPartitionReused(intervalIndex + 1:finalSampleIndex - 1) = true;
        preparation.SpanStartSampleIndex(spanIntervalIndices) = intervalIndex;
        preparation.SpanEndSampleIndex(spanIntervalIndices)   = finalSampleIndex;
        preparation.MergedIntervalCount = preparation.MergedIntervalCount + numel(spanIntervalIndices) - 1;
    end
    preparation.IntervalGeometryModel(classifiedIntervalIndices)     = geometryModel;
    preparation.IntervalHasExactPartition(classifiedIntervalIndices) = hasExactPartition;
    preparation.IntervalIsStationary(classifiedIntervalIndices)      = intervalIsStationary;
    preparation.IntervalUsesMovingCells(classifiedIntervalIndices)   = intervalUsesMovingCells;
    preparation.IntervalUsesEndpointHull(classifiedIntervalIndices)  = intervalUsesEndpointHull;
    preparation.IntervalIsUnsupported(classifiedIntervalIndices)     = intervalIsUnsupported;
    preparation.IntervalProofReason(classifiedIntervalIndices)       = ...
        intervalProofReason;
    preparation.IntervalPrepared(intervalIndex) = true;
end

function preparedResults = prepareSampleBatch(x_units, y_units)
    % Prepare each sample shape and its edges. Save errors until the main
    % loop reaches this sample, so worker timing cannot change which error
    % is reported first.
    preparedResults = cell(numel(x_units), 1);
    for sampleIndex = 1:numel(x_units)
        try
            shape = obstacleAvoidance.geometry.boundaryToShape( ...
                x_units{sampleIndex}, y_units{sampleIndex});
            [edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
            preparedResults{sampleIndex} = {shape, edgeStart_units, edgeEnd_units};
        catch exception
            preparedResults{sampleIndex} = exception;
        end
    end
end

function [sampleResults, intervalResults] = prepareCombinedBatch( ...
        x_units, y_units, lowerSampleIndex, upperSampleIndex, ...
        protectedVerticesKeepIndices, onlyCheckTranslation)
    % One worker prepares its sample shapes, then the intervals using them.
    sampleResults = prepareSampleBatch(x_units, y_units);
    sampleShapes  = cell(numel(sampleResults), 1);
    for sampleIndex = 1:numel(sampleResults)
        if ~isa(sampleResults{sampleIndex}, 'MException')
            sampleShapes{sampleIndex} = sampleResults{sampleIndex}{1};
        end
    end
    intervalResults = prepareIntervalBatch( ...
        x_units(lowerSampleIndex), y_units(lowerSampleIndex), ...
        x_units(upperSampleIndex), y_units(upperSampleIndex), ...
        sampleShapes(lowerSampleIndex), sampleShapes(upperSampleIndex), ...
        protectedVerticesKeepIndices, onlyCheckTranslation);
end

function preparedResults = prepareIntervalBatch(lowerX_units, lowerY_units, upperX_units, upperY_units, ...
        lowerShapes, upperShapes, protectedVerticesKeepIndices, onlyCheckTranslation)
    % Translation may reuse the previous interval's regions. Leave its
    % result empty so the main loop can perform that step in history order.
    preparedResults = cell(numel(lowerX_units), 1);
    for batchIndex = 1:numel(lowerX_units)
        if isempty(lowerShapes{batchIndex}) || isempty(upperShapes{batchIndex})
            continue;
        end
        try
            intervalPreparation = cell(1, 7);
            [intervalPreparation{:}, needsPreviousInterval] = ...
                obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
                createBoundarySample(lowerX_units{batchIndex}, lowerY_units{batchIndex}, ...
                    lowerShapes{batchIndex}), ...
                createBoundarySample(upperX_units{batchIndex}, upperY_units{batchIndex}, ...
                    upperShapes{batchIndex}), ...
                cell(0, 1), createIntervalCheckOptions(protectedVerticesKeepIndices, onlyCheckTranslation, true));
            if ~needsPreviousInterval
                preparedResults{batchIndex} = intervalPreparation;
            end
        catch exception
            preparedResults{batchIndex} = exception;
        end
    end
end

function preparation = prepareSamples(preparation, obstacle, sampleIndices, sampleResults)
    % Store each sample shape and its edges once. Use a worker result when
    % available; otherwise run the same calculation here.
    for sampleIndex = sampleIndices(~preparation.SamplePrepared(sampleIndices))
        samplePreparation = sampleResults{sampleIndex};
        if isempty(samplePreparation)
            serialSampleResults = prepareSampleBatch( ...
                obstacle.x_units(sampleIndex), obstacle.y_units(sampleIndex));
            samplePreparation = serialSampleResults{1};
        end
        if isa(samplePreparation, 'MException')
            rethrow(samplePreparation);
        end
        preparation.SampleShapes{sampleIndex}          = samplePreparation{1};
        preparation.SampleEdgeStart_units{sampleIndex} = samplePreparation{2};
        preparation.SampleEdgeEnd_units{sampleIndex}   = samplePreparation{3};
        preparation.SamplePrepared(sampleIndex)        = true;
    end
end

function sampleShapesAreEqual = compareShapes(firstShape, secondShape)
    % Two shapes cover the same area when neither extends beyond the other,
    % within the tolerance for polygon arithmetic.
    firstArea_units2     = area(firstShape);
    secondArea_units2    = area(secondShape);
    areaScale_units2     = max([1, firstArea_units2, secondArea_units2]);
    areaTolerance_units2 = 512 * eps(areaScale_units2);
    % If one area is clearly larger, skip the costly polygon subtraction.
    % Use 2 x tolerance for this quick check, then the stricter tolerance
    % for the actual area left outside the other shape.
    firstIsContained = firstArea_units2 <= secondArea_units2 + 2 * areaTolerance_units2 && ...
        area(subtract(firstShape, secondShape)) <= areaTolerance_units2;
    secondIsContained = secondArea_units2 <= firstArea_units2 + 2 * areaTolerance_units2 && ...
        area(subtract(secondShape, firstShape)) <= areaTolerance_units2;
    sampleShapesAreEqual = firstIsContained && secondIsContained;
end

function finalSampleIndices = findCandidateSpanEnds(obstacle, usesSourceIndex)
    % Propose grouping consecutive intervals when each vertex keeps its
    % velocity and matching order. For example, a vertex at x = 0, 1, 2
    % at t = 0, 1, 2 follows the same line over the whole span.
    % The main interval check still verifies the geometry of that full span.
    time_s             = obstacle.time_s;
    intervalCount      = numel(time_s) - 1;
    finalSampleIndices = (2:intervalCount + 1).';
    intervalIndex      = 1;
    while intervalIndex < intervalCount
        spanStartVertices_units = [obstacle.x_units{intervalIndex}, obstacle.y_units{intervalIndex}];
        currentVertices_units   = [obstacle.x_units{intervalIndex + 1}, obstacle.y_units{intervalIndex + 1}];

        samplePairCannotBeMerged = size(spanStartVertices_units, 1) < 3 || ...
            ~isequal(size(spanStartVertices_units), size(currentVertices_units)) || ...
            ~all(isfinite([spanStartVertices_units; currentVertices_units]), 'all');
        if samplePairCannotBeMerged
            intervalIndex = intervalIndex + 1;
            continue;
        end
        currentVertexVelocity_units_s = (currentVertices_units - spanStartVertices_units) / ...
            diff(time_s(intervalIndex:intervalIndex + 1));
        lastIntervalIndex = intervalIndex;
        while lastIntervalIndex < intervalCount
            nextVertices_units = [obstacle.x_units{lastIntervalIndex + 2}, ...
                obstacle.y_units{lastIntervalIndex + 2}];
            nextSampleCannotBeMerged = ~isequal(size(currentVertices_units), size(nextVertices_units)) || ...
                ~all(isfinite(nextVertices_units), 'all');
            if nextSampleCannotBeMerged
                break;
            end
            nextVertexVelocity_units_s = (nextVertices_units - currentVertices_units) / ...
                diff(time_s(lastIntervalIndex + 1:lastIntervalIndex + 2));
            coordinateScale_units = max([1; abs(spanStartVertices_units(:)); ...
                abs(currentVertices_units(:)); abs(nextVertices_units(:))]);
            vertexVelocityChanged = any(abs(nextVertexVelocity_units_s - currentVertexVelocity_units_s) > ...
                64 * eps(coordinateScale_units), 'all');
            if vertexVelocityChanged
                break;
            end
            % A tiny velocity difference can accumulate over a long time:
            % position difference = velocity difference x elapsed time.
            % Check that the proposed straight vertex motion also passes
            % through every supplied sample within coordinate roundoff.
            spanStartTime_s = time_s(intervalIndex);
            spanEndTime_s   = time_s(lastIntervalIndex + 2);
            samplesFollowLinearMotion = true;
            for interiorIndex = intervalIndex + 1:lastIntervalIndex + 1
                timeFraction            = (time_s(interiorIndex) - spanStartTime_s) / (spanEndTime_s - spanStartTime_s);
                predictedVertices_units = ...
                    spanStartVertices_units + timeFraction * (nextVertices_units - spanStartVertices_units);
                interiorVertices_units  = [obstacle.x_units{interiorIndex}, obstacle.y_units{interiorIndex}];
                positionError_units     = max(abs(interiorVertices_units - predictedVertices_units), [], 'all');
                if positionError_units > 64 * eps(coordinateScale_units)
                    samplesFollowLinearMotion = false;
                    break;
                end
            end
            if ~samplesFollowLinearMotion
                break;
            end
            % Without declared vertex matches, confirm that matching adjacent
            % samples would keep the same first vertex and direction.
            if ~usesSourceIndex
                nextKeepsAlignment = isequal( ...
                    obstacleAvoidance.obstacles.alignCorrespondingRing(currentVertices_units, nextVertices_units), ...
                    nextVertices_units);
                if ~nextKeepsAlignment
                    break;
                end
                if lastIntervalIndex == intervalIndex
                    upperKeepsAlignment = isequal( ...
                        obstacleAvoidance.obstacles.alignCorrespondingRing(spanStartVertices_units, currentVertices_units), ...
                        currentVertices_units);
                    if ~upperKeepsAlignment
                        break;
                    end
                end
            end
            lastIntervalIndex             = lastIntervalIndex + 1;
            currentVertexVelocity_units_s = nextVertexVelocity_units_s;
            currentVertices_units         = nextVertices_units;
        end
        finalSampleIndices(intervalIndex:lastIntervalIndex) = lastIntervalIndex + 1;
        intervalIndex = lastIntervalIndex + 1;
    end
end

function boundarySample = createBoundarySample(x_units, y_units, shape)
    % Keep the protected boundary and its polygon together for geometry checks.
    boundarySample = struct( ...
        'X_units', x_units, ...
        'Y_units', y_units, ...
        'Shape',   shape);
end

function checkOptions = createIntervalCheckOptions(preserveAlignment, onlyCheckTranslation, deferTranslation)
    % Select the vertex-order and translation checks for one interval.
    checkOptions = struct( ...
        'PreserveAlignment', preserveAlignment, ...
        'TranslationOnly',   onlyCheckTranslation, ...
        'DeferTranslation',  deferTranslation);
end
