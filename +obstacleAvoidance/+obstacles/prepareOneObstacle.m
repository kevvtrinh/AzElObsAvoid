function obstacle = prepareOneObstacle(obstacle, preparationVersion, sourceSnapshot, ...
    timeRange_s, previous, stopAtUnsupported)
%% Section 0: Header & Readme
% SYNTAX
%   obstacle = obstacleAvoidance.obstacles.prepareOneObstacle( ...
%       obstacle, preparationVersion, sourceSnapshot, timeRange_s, ...
%       previous, stopAtUnsupported)
%**************************************************************************
% PURPOSE
%   - Prepare requested entries of one canonical obstacle history.
%**************************************************************************
% INPUTS
%   - obstacle (scalar struct)
%       Canonical obstacle history to prepare.
%   - preparationVersion (positive integer scalar)
%       Version stored with derived preparation data.
%   - sourceSnapshot (scalar struct)
%       Source fields used to verify preparation-cache identity.
%   - timeRange_s (1-by-2 numeric row)
%       Requested source interval.
%   - previous (scalar struct or empty)
%       Source-checked preparation data eligible for extension.
%   - stopAtUnsupported (logical scalar)
%       Whether preparation stops at the first unsupported touched interval.
%**************************************************************************
% OUTPUTS
%   - obstacle (prepared scalar obstacle)
%       InternalPreparation contains reusable source-derived geometry.
%       Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry uses coordinate units; time uses seconds.
%**************************************************************************

%% Section 1: Select Source Samples Without Changing The History

validateattributes(preparationVersion, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
time_s          = obstacle.time_s;
sampleCount     = numel(time_s);
intervalCount   = sampleCount - 1;
neededSamples   = time_s >= timeRange_s(1) & time_s <= timeRange_s(2);
neededIntervals = time_s(1:end - 1) < timeRange_s(2) & time_s(2:end) > timeRange_s(1);
intervalIndices = find(neededIntervals);
neededSamples([intervalIndices; intervalIndices + 1]) = true;
if sampleCount == 1
    neededSamples(1) = true;
end

%% Section 2: Extend The Single Source-Checked Preparation Record

if isempty(previous)
    usesUnbufferedSourceIndex = obstacle.UsesSourceIndex && ...
        obstacle.safetyMargin_units == 0;
    preparation = struct( ...
        'PreparationVersion',                         preparationVersion, ...
        'SourceSnapshot',                             sourceSnapshot, ...
        'SamplePrepared',                             false(sampleCount, 1), ...
        'IntervalPrepared',                           false(intervalCount, 1), ...
        'SampleShapes',                               {cell(sampleCount, 1)}, ...
        'SampleEdgeStart_units',                      {cell(sampleCount, 1)}, ...
        'SampleEdgeEnd_units',                        {cell(sampleCount, 1)}, ...
        'IntervalUnionShapes',                        {cell(intervalCount, 1)}, ...
        'IntervalUnionEdgeStart_units',               {cell(intervalCount, 1)}, ...
        'IntervalUnionEdgeEnd_units',                 {cell(intervalCount, 1)}, ...
        'IntervalStartRegions_units',                 {cell(intervalCount, 1)}, ...
        'IntervalEndRegions_units',                   {cell(intervalCount, 1)}, ...
        'DeltaX_units',                               {cell(intervalCount, 1)}, ...
        'DeltaY_units',                               {cell(intervalCount, 1)}, ...
        'MatchingTopology',                           false(intervalCount, 1), ...
        'IntervalGeometryModel',                      strings(intervalCount, 1), ...
        'IntervalHasExactPartition',                  false(intervalCount, 1), ...
        'IntervalIsStationary',                       false(intervalCount, 1), ...
        'IntervalUsesMovingCells',                     false(intervalCount, 1), ...
        'IntervalUsesEndpointHull',                   false(intervalCount, 1), ...
        'IntervalIsUnsupported',                      false(intervalCount, 1), ...
        'IntervalPartitionReused',                    false(intervalCount, 1), ...
        'IntervalMovingCellCount',                     zeros(intervalCount, 2), ...
        'IntervalMovingCellTiming_s',                      zeros(intervalCount, 4), ...
        'IntervalMovingCellUncoveredProtectedArea_units2', zeros(intervalCount, 2), ...
        'IntervalEndpointHullAddedArea_units2',       zeros(intervalCount, 1), ...
        'IntervalProofReason',                strings(intervalCount, 1), ...
        'SpanStartSampleIndex',                       (1:intervalCount).', ...
        'SpanEndSampleIndex',                         (2:sampleCount).', ...
        'MergedIntervalCount',                        0, ...
        'RejectedMergeSpanSampleIndex',               zeros(0, 2), ...
        'CandidateSpanEndSampleIndex',                affineSpanEnds(obstacle, usesUnbufferedSourceIndex), ...
        'IntervalSpeedBound_units_s',                 Inf(intervalCount, 1), ...
        'SampleSpeedBound_units_s',                   Inf(sampleCount, 1), ...
        'IsTimeInvariant',                            false);
    % Exact numeric equality can establish a globally static shape without
    % constructing polygons outside the requested window. Activity still uses time_s.
    preparation.SamplesExactlyEqual = all(cellfun( ...
        @(x, y) isequaln(x, obstacle.x_units{1}) && isequaln(y, obstacle.y_units{1}), ...
        obstacle.x_units, obstacle.y_units));
    preparation.IsTimeInvariant = preparation.SamplesExactlyEqual;
else
    preparation = previous;
end
if stopAtUnsupported && any(neededIntervals & preparation.IntervalPrepared & ...
        preparation.IntervalIsUnsupported)
    obstacle.InternalPreparation = preparation;
    return;
end

% Canonical spans are source-derived and independent of the query window.
% Prepare an entire touched span so independent rebuilds clip identical cells.
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

%% Section 3: Prepare Each Newly Requested Source Interval Once

% Only isolated candidate intervals have fixed endpoints before proof.
% A rejected merged span changes its successors, so all spans stay sequential.
newIntervalIndices   = find(neededIntervals & ~preparation.IntervalPrepared);
isolatedIntervals    = preparation.CandidateSpanEndSampleIndex == (2:sampleCount).' & ...
    [true; diff(preparation.CandidateSpanEndSampleIndex) ~= 0];
independentIndices   = newIntervalIndices(isolatedIntervals(newIntervalIndices));
sampleProducts       = cell(sampleCount, 1);
intervalProducts     = cell(intervalCount, 1);
protectedKeepsIndex  = obstacle.UsesSourceIndex && obstacle.safetyMargin_units == 0;
translationOnly      = obstacle.UsesSourceIndex && obstacle.safetyMargin_units > 0;
useBackgroundWorkers = false;
% At 32 dense intervals, three paired R2024b runs measured median preparation
% of 0.884 s serial versus 0.491 s with four batches, including both barriers.
parallelIntervalThreshold = 32;
% The owner allows at most four workers on this machine so that other MATLAB
% processes running beside this one keep cores of their own.
maximumWorkerCount        = 4;
if numel(independentIndices) >= parallelIntervalThreshold && exist("backgroundPool", "builtin") == 5
    workerPool           = backgroundPool;
    useBackgroundWorkers = workerPool.NumWorkers > 1 && ~workerPool.Busy;
end
if useBackgroundWorkers
    workerCount   = min(maximumWorkerCount, workerPool.NumWorkers);
    sampleIndices = find(neededSamples & ~preparation.SamplePrepared);
    intervalSampleIndices  = unique([independentIndices; independentIndices + 1]);
    remainingSampleIndices = setdiff(sampleIndices, intervalSampleIndices, 'stable');
    combinedFutures(1, workerCount) = parallel.FevalFuture;
    batchSampleIndices = cell(workerCount, 1);
    batchIntervalIndices = cell(workerCount, 1);
    for workerIndex = 1:workerCount
        firstBatchIndex = floor((workerIndex - 1) * numel(independentIndices) / workerCount) + 1;
        finalBatchIndex = floor(workerIndex * numel(independentIndices) / workerCount);
        batchIntervalIndices{workerIndex} = independentIndices(firstBatchIndex:finalBatchIndex);
        extraSampleIndices = remainingSampleIndices(workerIndex:workerCount:end);
        batchSampleIndices{workerIndex} = unique([ ...
            batchIntervalIndices{workerIndex}; ...
            batchIntervalIndices{workerIndex} + 1; ...
            extraSampleIndices]);
        [~, lowerSampleIndex] = ismember( ...
            batchIntervalIndices{workerIndex}, batchSampleIndices{workerIndex});
        [~, upperSampleIndex] = ismember( ...
            batchIntervalIndices{workerIndex} + 1, batchSampleIndices{workerIndex});
        combinedFutures(workerIndex) = parfeval(workerPool, @prepareCombinedBatch, 2, ...
            obstacle.x_units(batchSampleIndices{workerIndex}), ...
            obstacle.y_units(batchSampleIndices{workerIndex}), ...
            lowerSampleIndex, upperSampleIndex, protectedKeepsIndex, translationOnly);
    end
    combinedCleanup = onCleanup(@() cancel(combinedFutures));
    for workerIndex = 1:workerCount
        [batchSamples, batchIntervals] = fetchOutputs(combinedFutures(workerIndex));
        sampleProducts(batchSampleIndices{workerIndex}) = batchSamples;
        intervalProducts(batchIntervalIndices{workerIndex}) = batchIntervals;
    end
    clear combinedCleanup;
end

for intervalIndex = reshape(find(neededIntervals & ~preparation.IntervalPrepared), 1, [])
    if preparation.IntervalPrepared(intervalIndex)
        continue;
    end
    preparation = prepareSourceInterval(preparation, obstacle, intervalIndex, ...
        sampleProducts, intervalProducts, protectedKeepsIndex, translationOnly);
    if stopAtUnsupported && preparation.IntervalIsUnsupported(intervalIndex)
        break;
    end
end
if ~stopAtUnsupported || ~any(neededIntervals & preparation.IntervalPrepared & ...
        preparation.IntervalIsUnsupported)
    preparation = prepareSamples(preparation, obstacle, find(neededSamples).', sampleProducts);
end

%% Section 4: Update Cached Motion Bounds And Static Status

preparation.SampleSpeedBound_units_s = max([0; preparation.IntervalSpeedBound_units_s], ...
    [preparation.IntervalSpeedBound_units_s; 0]);
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
        sampleProducts, intervalProducts, protectedKeepsIndex, translationOnly)
    % Prove one requested source interval, or the merged span it starts,
    % and record its geometry model on every interval it covers.
    time_s = obstacle.time_s;
    finalSampleIndex = preparation.CandidateSpanEndSampleIndex(intervalIndex);
    preparation = prepareSamples(preparation, obstacle, intervalIndex:finalSampleIndex, sampleProducts);
    lowerX_units = obstacle.x_units{intervalIndex};
    lowerY_units = obstacle.y_units{intervalIndex};
    upperX_units = obstacle.x_units{finalSampleIndex};
    upperY_units = obstacle.y_units{finalSampleIndex};
    reusableStartRegions_units = cell(0, 1);
    precedingPartitionIsReusable = intervalIndex > 1 && ...
        preparation.IntervalPrepared(intervalIndex - 1) && ...
        preparation.IntervalHasExactPartition(intervalIndex - 1) && ...
        ~isempty(preparation.IntervalEndRegions_units{intervalIndex - 1});
    if precedingPartitionIsReusable
        reusableStartRegions_units = preparation.IntervalEndRegions_units{intervalIndex - 1};
    end
    % A declared source-index correspondence describes the original rings.
    % Protected rings are only those rings when no margin was applied;
    % buffered rings carry no index correspondence, so they admit only the
    % translation proof, and every other motion uses the original rings.
    usesSourceIndex   = obstacle.UsesSourceIndex;
    preserveAlignment = protectedKeepsIndex || finalSampleIndex > intervalIndex + 1;
    product = intervalProducts{intervalIndex};
    if isa(product, 'MException')
        rethrow(product);
    end
    if isempty(product)
        [matched, alignedUpper_units, startRegions_units, endRegions_units, ...
            geometryModel, hasExactPartition, partitionReused] = ...
            obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
            lowerX_units, lowerY_units, upperX_units, upperY_units, ...
            preparation.SampleShapes{intervalIndex}, ...
            preparation.SampleShapes{finalSampleIndex}, ...
            reusableStartRegions_units, preserveAlignment, translationOnly, false);
    else
        [matched, alignedUpper_units, startRegions_units, endRegions_units, ...
            geometryModel, hasExactPartition, partitionReused] = product{:};
    end
    if ~matched && finalSampleIndex > intervalIndex + 1
        % Velocity equality proposes a reduction; without a shared full-span
        % exact partition no merge is permitted. Prepare the source intervals.
        preparation.RejectedMergeSpanSampleIndex(end + 1, :) = [intervalIndex, finalSampleIndex];
        preparation.CandidateSpanEndSampleIndex(intervalIndex:finalSampleIndex - 1) = ...
            (intervalIndex + 1:finalSampleIndex).';
        finalSampleIndex = intervalIndex + 1;
        upperX_units = obstacle.x_units{finalSampleIndex};
        upperY_units = obstacle.y_units{finalSampleIndex};
        [matched, alignedUpper_units, startRegions_units, endRegions_units, ...
            geometryModel, hasExactPartition, partitionReused] = ...
            obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
            lowerX_units, lowerY_units, upperX_units, upperY_units, ...
            preparation.SampleShapes{intervalIndex}, ...
            preparation.SampleShapes{finalSampleIndex}, ...
            reusableStartRegions_units, protectedKeepsIndex, translationOnly, false);
    end
    intervalIsStationary        = false;
    intervalUsesMovingCells      = false;
    intervalUsesEndpointHull    = false;
    intervalIsUnsupported       = false;
    intervalProofReason = "";
    preparation.MatchingTopology(intervalIndex) = matched;
    preparation.IntervalPartitionReused(intervalIndex) = partitionReused;
    if matched
        preparation.DeltaX_units{intervalIndex} = alignedUpper_units(:, 1) - lowerX_units;
        preparation.DeltaY_units{intervalIndex} = alignedUpper_units(:, 2) - lowerY_units;
        preparation.IntervalStartRegions_units{intervalIndex} = startRegions_units;
        preparation.IntervalEndRegions_units{intervalIndex}   = endRegions_units;
        speed_units_s = hypot( ...
            preparation.DeltaX_units{intervalIndex}, ...
            preparation.DeltaY_units{intervalIndex}) / ...
            (time_s(finalSampleIndex) - time_s(intervalIndex));
        preparation.IntervalSpeedBound_units_s(intervalIndex) = ...
            max([0; speed_units_s(isfinite(speed_units_s))]);
    else
        firstShape = preparation.SampleShapes{intervalIndex};
        lastShape  = preparation.SampleShapes{finalSampleIndex};
        equivalent = compareShapes(firstShape, lastShape);
        if equivalent
            shape                = firstShape;
            geometryModel        = "staticEquivalentSamples";
            intervalIsStationary = true;
        else
            lowerOriginal_units = [obstacle.originalX_units{intervalIndex}, ...
                obstacle.originalY_units{intervalIndex}];
            upperOriginal_units = [obstacle.originalX_units{finalSampleIndex}, ...
                obstacle.originalY_units{finalSampleIndex}];
            [movingCellsSupported, movingCellShape, movingCellRegions_units, counts, timing_s] = ...
                obstacleAvoidance.obstacles.createMovingCells( ...
                lowerOriginal_units, upperOriginal_units, ...
                obstacle.safetyMargin_units, usesSourceIndex);
            if movingCellsSupported
                % Prove both supplied protected samples against the
                % given moving-cell enclosure without replacing either sample.
                uncoveredArea_units2 = [area(subtract(firstShape, movingCellShape)), ...
                    area(subtract(lastShape, movingCellShape))];
                preparation.IntervalMovingCellUncoveredProtectedArea_units2(intervalIndex, :) = ...
                    uncoveredArea_units2;
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
                    shape                       = polyshape();
                    geometryModel               = "unsupportedContinuousDeformation";
                    intervalProofReason = "degenerateEndpointGeometry";
                    intervalIsUnsupported       = true;
                end
            end
            preparation.IntervalMovingCellCount(intervalIndex, :) = counts;
            preparation.IntervalMovingCellTiming_s(intervalIndex, :)  = timing_s;
        end
        preparation.IntervalUnionShapes{intervalIndex} = shape;
        preparation.IntervalSpeedBound_units_s(intervalIndex) = 0;
        [preparation.IntervalUnionEdgeStart_units{intervalIndex}, ...
            preparation.IntervalUnionEdgeEnd_units{intervalIndex}] = ...
            obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
    end
    classifiedIntervalIndices = intervalIndex;
    if finalSampleIndex > intervalIndex + 1
        % One proven partition restricts to every source subinterval with
        % identical face indices. Source samples themselves remain supplied.
        spanDelta_units = [preparation.DeltaX_units{intervalIndex}, ...
            preparation.DeltaY_units{intervalIndex}];
        spanStartRegions_units = startRegions_units;
        spanEndRegions_units   = endRegions_units;
        for sourceIndex = intervalIndex:finalSampleIndex - 1
            fraction = (time_s(sourceIndex:sourceIndex + 1) - time_s(intervalIndex)) / ...
                (time_s(finalSampleIndex) - time_s(intervalIndex));
            preparation.DeltaX_units{sourceIndex} = diff(fraction) * spanDelta_units(:, 1);
            preparation.DeltaY_units{sourceIndex} = diff(fraction) * spanDelta_units(:, 2);
            for faceIndex = 1:numel(spanStartRegions_units)
                delta_units = spanEndRegions_units{faceIndex} - spanStartRegions_units{faceIndex};
                startRegions_units{faceIndex} = spanStartRegions_units{faceIndex} + fraction(1) * delta_units;
                endRegions_units{faceIndex} = spanStartRegions_units{faceIndex} + fraction(2) * delta_units;
            end
            preparation.IntervalStartRegions_units{sourceIndex} = startRegions_units;
            preparation.IntervalEndRegions_units{sourceIndex} = endRegions_units;
        end
        spanIndices = intervalIndex:finalSampleIndex - 1;
        classifiedIntervalIndices = spanIndices;
        preparation.IntervalPrepared(spanIndices) = true;
        preparation.MatchingTopology(spanIndices) = true;
        preparation.IntervalSpeedBound_units_s(spanIndices) = preparation.IntervalSpeedBound_units_s(intervalIndex);
        preparation.IntervalPartitionReused(intervalIndex + 1:finalSampleIndex - 1) = true;
        preparation.SpanStartSampleIndex(spanIndices) = intervalIndex;
        preparation.SpanEndSampleIndex(spanIndices) = finalSampleIndex;
        preparation.MergedIntervalCount = preparation.MergedIntervalCount + numel(spanIndices) - 1;
    end
    preparation.IntervalGeometryModel(classifiedIntervalIndices)     = geometryModel;
    preparation.IntervalHasExactPartition(classifiedIntervalIndices) = hasExactPartition;
    preparation.IntervalIsStationary(classifiedIntervalIndices)      = intervalIsStationary;
    preparation.IntervalUsesMovingCells(classifiedIntervalIndices)    = intervalUsesMovingCells;
    preparation.IntervalUsesEndpointHull(classifiedIntervalIndices)  = intervalUsesEndpointHull;
    preparation.IntervalIsUnsupported(classifiedIntervalIndices)     = intervalIsUnsupported;
    preparation.IntervalProofReason(classifiedIntervalIndices) = ...
        intervalProofReason;
    preparation.IntervalPrepared(intervalIndex) = true;
end

function products = prepareSampleBatch(x_units, y_units)
    % Keep exceptions as data until the reference pass reaches their sample.
    products = cell(numel(x_units), 1);
    for sampleIndex = 1:numel(x_units)
        try
            shape = obstacleAvoidance.geometry.boundaryToShape( ...
                x_units{sampleIndex}, y_units{sampleIndex});
            [edgeStart_units, edgeEnd_units] = obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
            products{sampleIndex} = {shape, edgeStart_units, edgeEnd_units};
        catch exception
            products{sampleIndex} = exception;
        end
    end
end

function [sampleProducts, intervalProducts] = prepareCombinedBatch( ...
        x_units, y_units, lowerSampleIndex, upperSampleIndex, ...
        protectedKeepsIndex, translationOnly)
    sampleProducts = prepareSampleBatch(x_units, y_units);
    sampleShapes   = cell(numel(sampleProducts), 1);
    for sampleIndex = 1:numel(sampleProducts)
        if ~isa(sampleProducts{sampleIndex}, 'MException')
            sampleShapes{sampleIndex} = sampleProducts{sampleIndex}{1};
        end
    end
    intervalProducts = prepareIntervalBatch( ...
        x_units(lowerSampleIndex), y_units(lowerSampleIndex), ...
        x_units(upperSampleIndex), y_units(upperSampleIndex), ...
        sampleShapes(lowerSampleIndex), sampleShapes(upperSampleIndex), ...
        protectedKeepsIndex, translationOnly);
end

function products = prepareIntervalBatch(lowerX_units, lowerY_units, upperX_units, upperY_units, ...
        lowerShapes, upperShapes, protectedKeepsIndex, translationOnly)
    % Translation returns no product: only the sequential pass owns reuse.
    products = cell(numel(lowerX_units), 1);
    for batchIndex = 1:numel(lowerX_units)
        if isempty(lowerShapes{batchIndex}) || isempty(upperShapes{batchIndex})
            continue;
        end
        try
            product = cell(1, 7);
            [product{:}, dependsOnPrevious] = ...
                obstacleAvoidance.obstacles.alignVerifiedSingleRing( ...
                lowerX_units{batchIndex}, lowerY_units{batchIndex}, ...
                upperX_units{batchIndex}, upperY_units{batchIndex}, ...
                lowerShapes{batchIndex}, upperShapes{batchIndex}, ...
                cell(0, 1), protectedKeepsIndex, translationOnly, true);
            if ~dependsOnPrevious
                products{batchIndex} = product;
            end
        catch exception
            products{batchIndex} = exception;
        end
    end
end

function preparation = prepareSamples(preparation, obstacle, sampleIndices, sampleProducts)
    % Prepare only source samples that are not already cached.
    for sampleIndex = sampleIndices(~preparation.SamplePrepared(sampleIndices))
        product = sampleProducts{sampleIndex};
        if isempty(product)
            % A missing worker product is prepared here by the same batch
            % implementation, so both paths share one sample computation.
            serialProducts = prepareSampleBatch( ...
                obstacle.x_units(sampleIndex), obstacle.y_units(sampleIndex));
            product = serialProducts{1};
        end
        if isa(product, 'MException')
            rethrow(product);
        end
        preparation.SampleShapes{sampleIndex}          = product{1};
        preparation.SampleEdgeStart_units{sampleIndex} = product{2};
        preparation.SampleEdgeEnd_units{sampleIndex}   = product{3};
        preparation.SamplePrepared(sampleIndex)        = true;
    end
end

function equivalent = compareShapes(firstShape, secondShape)
    % Check equality and containment using shape differences.
    firstArea_units2     = area(firstShape);
    secondArea_units2    = area(secondShape);
    areaScale_units2     = max([1, firstArea_units2, secondArea_units2]);
    areaTolerance_units2 = 512 * eps(areaScale_units2);
    % A larger polygon cannot fit inside a smaller one: the area difference
    % bounds the subtraction from below. Keep a full extra tolerance of
    % roundoff reserve and perform the original Boolean check near equality.
    firstIsContained = firstArea_units2 <= secondArea_units2 + 2 * areaTolerance_units2 && ...
        area(subtract(firstShape, secondShape)) <= areaTolerance_units2;
    secondIsContained = secondArea_units2 <= firstArea_units2 + 2 * areaTolerance_units2 && ...
        area(subtract(secondShape, firstShape)) <= areaTolerance_units2;
    equivalent = firstIsContained && secondIsContained;
end

function finalSampleIndices = affineSpanEnds(obstacle, usesSourceIndex)
    % Equal velocities plus unchanged per-interval alignment propose spans.
    % The main stage proves the entire span with one shared face partition.
    % A declared source-index correspondence needs no alignment check.
    time_s             = obstacle.time_s;
    intervalCount      = numel(time_s) - 1;
    finalSampleIndices = (2:intervalCount + 1).';
    intervalIndex      = 1;
    while intervalIndex < intervalCount
        lower_units = [obstacle.x_units{intervalIndex}, obstacle.y_units{intervalIndex}];
        upper_units = [obstacle.x_units{intervalIndex + 1}, obstacle.y_units{intervalIndex + 1}];
        pairIsUnusableRing = size(lower_units, 1) < 3 || ...
            ~isequal(size(lower_units), size(upper_units)) || ...
            ~all(isfinite([lower_units; upper_units]), 'all');
        if pairIsUnusableRing
            intervalIndex = intervalIndex + 1;
            continue;
        end
        velocity_units_s = (upper_units - lower_units) / ...
            diff(time_s(intervalIndex:intervalIndex + 1));
        lastIntervalIndex = intervalIndex;
        while lastIntervalIndex < intervalCount
            next_units = [obstacle.x_units{lastIntervalIndex + 2}, ...
                obstacle.y_units{lastIntervalIndex + 2}];
            nextIsUnusableRing = ~isequal(size(upper_units), size(next_units)) || ...
                ~all(isfinite(next_units), 'all');
            if nextIsUnusableRing
                break;
            end
            nextVelocity_units_s = (next_units - upper_units) / ...
                diff(time_s(lastIntervalIndex + 1:lastIntervalIndex + 2));
            coordinateScale_units = max([1; abs(lower_units(:)); ...
                abs(upper_units(:)); abs(next_units(:))]);
            velocityChanged = any(abs(nextVelocity_units_s - velocity_units_s) > ...
                64 * eps(coordinateScale_units), 'all');
            if velocityChanged
                break;
            end
            % Equal velocities are only a proposal: over a long interval a
            % sub-epsilon velocity difference is still a real displacement.
            % The merged affine span must reproduce every interior
            % supplied sample in position.
            spanStartTime_s = time_s(intervalIndex);
            spanEndTime_s   = time_s(lastIntervalIndex + 2);
            spanIsAffine    = true;
            for interiorIndex = intervalIndex + 1:lastIntervalIndex + 1
                fraction        = (time_s(interiorIndex) - spanStartTime_s) / (spanEndTime_s - spanStartTime_s);
                predicted_units = lower_units + fraction * (next_units - lower_units);
                interior_units  = [obstacle.x_units{interiorIndex}, obstacle.y_units{interiorIndex}];
                positionError_units = max(abs(interior_units - predicted_units), [], 'all');
                if positionError_units > 64 * eps(coordinateScale_units)
                    spanIsAffine = false;
                    break;
                end
            end
            if ~spanIsAffine
                break;
            end
            if ~usesSourceIndex
                nextKeepsAlignment = isequal( ...
                    obstacleAvoidance.obstacles.alignCorrespondingRing(upper_units, next_units), ...
                    next_units);
                if ~nextKeepsAlignment
                    break;
                end
                if lastIntervalIndex == intervalIndex
                    upperKeepsAlignment = isequal( ...
                        obstacleAvoidance.obstacles.alignCorrespondingRing(lower_units, upper_units), ...
                        upper_units);
                    if ~upperKeepsAlignment
                        break;
                    end
                end
            end
            lastIntervalIndex = lastIntervalIndex + 1;
            velocity_units_s = nextVelocity_units_s;
            upper_units      = next_units;
        end
        finalSampleIndices(intervalIndex:lastIntervalIndex) = lastIntervalIndex + 1;
        intervalIndex = lastIntervalIndex + 1;
    end
end
