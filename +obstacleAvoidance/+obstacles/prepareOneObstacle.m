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
        'IntervalUsesSweptCells',                     false(intervalCount, 1), ...
        'IntervalUsesEndpointHull',                   false(intervalCount, 1), ...
        'IntervalIsUnsupported',                      false(intervalCount, 1), ...
        'IntervalPartitionReused',                    false(intervalCount, 1), ...
        'IntervalSweptCellCount',                     zeros(intervalCount, 2), ...
        'IntervalSweptTiming_s',                      zeros(intervalCount, 4), ...
        'IntervalSweptUncoveredProtectedArea_units2', zeros(intervalCount, 2), ...
        'IntervalEndpointHullAddedArea_units2',       zeros(intervalCount, 1), ...
        'IntervalCertificationReason',                strings(intervalCount, 1), ...
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

% Only isolated candidate intervals have fixed endpoints before certification.
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
    % translation certificate, and every other motion uses the original rings.
    usesSourceIndex   = obstacle.UsesSourceIndex;
    preserveAlignment = protectedKeepsIndex || finalSampleIndex > intervalIndex + 1;
    product = intervalProducts{intervalIndex};
    if isa(product, 'MException')
        rethrow(product);
    end
    if isempty(product)
        [matched, alignedUpper_units, startRegions_units, endRegions_units, ...
            geometryModel, hasExactPartition, partitionReused] = alignVerifiedSingleRing( ...
            lowerX_units, lowerY_units, upperX_units, upperY_units, ...
            preparation.SampleShapes{intervalIndex}, ...
            preparation.SampleShapes{finalSampleIndex}, ...
            reusableStartRegions_units, preserveAlignment, translationOnly);
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
            geometryModel, hasExactPartition, partitionReused] = alignVerifiedSingleRing( ...
            lowerX_units, lowerY_units, upperX_units, upperY_units, ...
            preparation.SampleShapes{intervalIndex}, ...
            preparation.SampleShapes{finalSampleIndex}, ...
            reusableStartRegions_units, protectedKeepsIndex, translationOnly);
    end
    intervalIsStationary        = false;
    intervalUsesSweptCells      = false;
    intervalUsesEndpointHull    = false;
    intervalIsUnsupported       = false;
    intervalCertificationReason = "";
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
            [sweptSupported, sweptShape, sweptRegions_units, counts, timing_s] = ...
                obstacleAvoidance.obstacles.createSweptCorrespondingCells( ...
                lowerOriginal_units, upperOriginal_units, ...
                obstacle.safetyMargin_units, usesSourceIndex);
            if sweptSupported
                % Certify both authoritative protected samples against the
                % prescribed swept enclosure without replacing either sample.
                uncoveredArea_units2 = [area(subtract(firstShape, sweptShape)), ...
                    area(subtract(lastShape, sweptShape))];
                preparation.IntervalSweptUncoveredProtectedArea_units2(intervalIndex, :) = ...
                    uncoveredArea_units2;
                areaTolerance_units2 = 4096 * eps(max([1, area(firstShape), area(lastShape)]));
                sweptSupported = all(uncoveredArea_units2 <= areaTolerance_units2);
            end
            if sweptSupported
                shape         = sweptShape;
                geometryModel = "sweptCorrespondingConvexCells";
                preparation.IntervalStartRegions_units{intervalIndex} = sweptRegions_units;
                preparation.IntervalEndRegions_units{intervalIndex}   = sweptRegions_units;
                intervalUsesSweptCells = true;
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
                    intervalCertificationReason = "degenerateEndpointGeometry";
                    intervalIsUnsupported       = true;
                end
            end
            preparation.IntervalSweptCellCount(intervalIndex, :) = counts;
            preparation.IntervalSweptTiming_s(intervalIndex, :)  = timing_s;
        end
        preparation.IntervalUnionShapes{intervalIndex} = shape;
        preparation.IntervalSpeedBound_units_s(intervalIndex) = 0;
        [preparation.IntervalUnionEdgeStart_units{intervalIndex}, ...
            preparation.IntervalUnionEdgeEnd_units{intervalIndex}] = ...
            obstacleAvoidance.geometry.boundaryToEdges(shape, 0);
    end
    classifiedIntervalIndices = intervalIndex;
    if finalSampleIndex > intervalIndex + 1
        % One certified partition restricts to every source subinterval with
        % identical face indices. Source samples themselves remain authoritative.
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
    preparation.IntervalUsesSweptCells(classifiedIntervalIndices)    = intervalUsesSweptCells;
    preparation.IntervalUsesEndpointHull(classifiedIntervalIndices)  = intervalUsesEndpointHull;
    preparation.IntervalIsUnsupported(classifiedIntervalIndices)     = intervalIsUnsupported;
    preparation.IntervalCertificationReason(classifiedIntervalIndices) = ...
        intervalCertificationReason;
    preparation.IntervalPrepared(intervalIndex) = true;
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
enclosureIntervalIndices = find(preparation.IntervalUsesSweptCells | ...
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
            [product{:}, dependsOnPrevious] = alignVerifiedSingleRing( ...
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

function [verified, alignedUpper_units, startRegions_units, endRegions_units, ...
        geometryModel, hasExactPartition, partitionReused, dependsOnPrevious] = alignVerifiedSingleRing( ...
        lowerX_units, lowerY_units, upperX_units, upperY_units, lowerShape, ...
        upperShape, reusableStartRegions_units, preserveAlignment, translationOnly, deferTranslation)
    % Align rings, then certify either one moving convex region or an exact
    % moving convex partition of the complete interpolated polygon.
    % translationOnly stops after the index-preserving translation check.
    if nargin < 10
        deferTranslation = false;
    end
    dependsOnPrevious = false;
    lower_units        = [lowerX_units(:), lowerY_units(:)];
    upper_units        = [upperX_units(:), upperY_units(:)];
    verified           = false;
    alignedUpper_units = zeros(0, 2);
    startRegions_units = cell(0, 1);
    endRegions_units   = cell(0, 1);
    geometryModel      = "";
    hasExactPartition  = false;
    partitionReused    = false;
    lowerFinite = all(isfinite(lower_units), 2);
    upperFinite = all(isfinite(upper_units), 2);
    if isequal(lowerFinite, upperFinite) && nnz(lowerFinite) >= 3
        finiteDelta_units = upper_units(lowerFinite, :) - lower_units(lowerFinite, :);
        coordinateScale_units = max([1; abs(lower_units(lowerFinite, 1)); ...
            abs(lower_units(lowerFinite, 2)); abs(upper_units(upperFinite, 1)); ...
            abs(upper_units(upperFinite, 2))]);
        translationTolerance_units = 512 * eps(coordinateScale_units);
        if max(abs(finiteDelta_units - finiteDelta_units(1, :)), [], 'all') <= translationTolerance_units
            if deferTranslation
                dependsOnPrevious = true;
                return;
            end
            alignedUpper_units = upper_units;
            startRegions_units = reusableStartRegions_units;
            if isempty(startRegions_units)
                startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape);
            else
                partitionReused = true;
            end
            endRegions_units = cellfun(@(region) region + finiteDelta_units(1, :), ...
                startRegions_units, 'UniformOutput', false);
            verified         = true;
            geometryModel    = "linearCorrespondingConvexPartition";
            hasExactPartition = true;
            return;
        end
    end
    isSingleRing = size(lower_units, 1) >= 3 && ...
        isequal(size(lower_units), size(upper_units)) && ...
        all(isfinite(lower_units), "all") && all(isfinite(upper_units), "all");
    if ~isSingleRing || translationOnly
        return;
    end

    % Identical rings already attain the first possible zero-distance match.
    if isequal(lower_units, upper_units)
        alignedUpper_units = upper_units;
        verified           = true;
        geometryModel      = "linearCorrespondingVertices";
        return;
    end
    if preserveAlignment
        alignedUpper_units = upper_units;
    else
        alignedUpper_units = obstacleAvoidance.obstacles.alignCorrespondingRing(lower_units, upper_units);
    end
    delta_units                = alignedUpper_units - lower_units;
    coordinateScale_units      = max([1; abs(lower_units(:)); abs(alignedUpper_units(:))]);
    translationTolerance_units = 512 * eps(coordinateScale_units);
    isTranslation              = max(abs(delta_units - delta_units(1, :)), [], "all") <= ...
        translationTolerance_units;
    if isTranslation
        if deferTranslation
            dependsOnPrevious = true;
            return;
        end
        % A translation preserves every face of one exact partition. Build
        % the terminal faces by translating those same faces; this avoids
        % relying on polyshape's vertex ordering after cyclic/reversed input.
        startRegions_units = reusableStartRegions_units;
        if isempty(startRegions_units)
            startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape);
        else
            partitionReused = true;
        end
        endRegions_units = cellfun(@(region) region + delta_units(1, :), ...
            startRegions_units, 'UniformOutput', false);
        verified = ~isempty(startRegions_units);
        if verified
            geometryModel = "linearCorrespondingConvexPartition";
            hasExactPartition = true;
            return;
        end
    end
    isConvexMotion = remainsStrictlyConvex( ...
        lower_units, alignedUpper_units, coordinateScale_units);
    verified = isConvexMotion;
    if verified
        geometryModel = "linearCorrespondingVertices";
        return;
    end
    [globalAffineVerified, exactGlobalAffineVerified] = verifiedGlobalAffineMap( ...
        lower_units, alignedUpper_units, coordinateScale_units);
    topologyMotionVerified = exactGlobalAffineVerified;
    if ~topologyMotionVerified
        topologyMotionVerified = movingBoundaryRemainsSimple( ...
            lower_units, alignedUpper_units, coordinateScale_units);
    end
    [partitionVerified, startRegions_units, endRegions_units] = ...
        createVerifiedMovingPartition( ...
        lower_units, alignedUpper_units, lowerShape, upperShape, ...
        coordinateScale_units, topologyMotionVerified);
    verified = partitionVerified && (globalAffineVerified || topologyMotionVerified);
    if verified
        geometryModel = "linearCorrespondingConvexPartition";
        hasExactPartition = true;
        return;
    end
    % Every verified branch above returned, so this cleanup is unconditional.
    alignedUpper_units = zeros(0, 2);
    startRegions_units = cell(0, 1);
    endRegions_units   = cell(0, 1);
end

function [verified, exactVerified] = verifiedGlobalAffineMap( ...
        lower_units, upper_units, coordinateScale_units)
    % A single affine map preserves every edge and face. Its linear blend
    % with identity is valid when the determinant stays strictly positive.
    source = [lower_units, ones(size(lower_units, 1), 1)];
    transformation = source \ upper_units;
    residual_units = source * transformation - upper_units;
    residualTolerance_units = 4096 * eps(coordinateScale_units);
    if max(abs(residual_units), [], 'all') > residualTolerance_units
        verified = false;
        exactVerified = false;
        return;
    end
    linearMap = transformation(1:2, :);
    deltaMap = linearMap - eye(2);
    determinantCoefficients = [det(deltaMap), ...
        deltaMap(1, 1) + deltaMap(2, 2), 1];
    candidateTau = [0; 1];
    if determinantCoefficients(1) ~= 0
        stationaryTau = -determinantCoefficients(2) / (2 * determinantCoefficients(1));
        if stationaryTau > 0 && stationaryTau < 1
            candidateTau(end + 1, 1) = stationaryTau;
        end
    end
    determinants = polyval(determinantCoefficients, candidateTau);
    determinantTolerance = 4096 * eps(max(1, norm(linearMap, 'fro') ^ 2));
    verified = all(determinants > determinantTolerance);
    exactVerified = verified && all(residual_units == 0, 'all');
end

function [verified, startRegions_units, endRegions_units] = createVerifiedMovingPartition( ...
        lower_units, upper_units, lowerShape, upperShape, coordinateScale_units, ...
        topologyMotionVerified)
    % Carry one exact lower-sample partition through the supplied vertex
    % correspondence. Every face must remain convex for the whole interval.
    verified = false;
    startRegions_units = obstacleAvoidance.geometry.convexRegions(lowerShape);
    endRegions_units = cell(size(startRegions_units));
    sourceIndexRegions = cell(size(startRegions_units));
    for regionIndex = 1:numel(startRegions_units)
        [isSourceVertex, sourceIndex] = ismember( ...
            startRegions_units{regionIndex}, lower_units, 'rows');
        if ~all(isSourceVertex) || numel(unique(sourceIndex)) ~= numel(sourceIndex)
            startRegions_units = cell(0, 1);
            endRegions_units = cell(0, 1);
            return;
        end
        sourceIndexRegions{regionIndex} = sourceIndex;
        endRegions_units{regionIndex} = upper_units(sourceIndex, :);
        if ~remainsStrictlyConvex(startRegions_units{regionIndex}, ...
                endRegions_units{regionIndex}, coordinateScale_units)
            [verified, startRegions_units, endRegions_units] = createVerifiedMovingTriangles( ...
                lower_units, upper_units, lowerShape, upperShape, ...
                coordinateScale_units, topologyMotionVerified);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceIndexRegions, ...
        lower_units, upper_units, lowerShape, upperShape, topologyMotionVerified);
end

function [verified, startRegions_units, endRegions_units] = createVerifiedMovingTriangles( ...
        lower_units, upper_units, lowerShape, upperShape, coordinateScale_units, ...
        topologyMotionVerified)
    % A triangle mesh is the non-heuristic fallback partition. It is
    % accepted only when every mesh point is an original corresponding vertex.
    verified = false;
    startRegions_units = cell(0, 1);
    endRegions_units   = cell(0, 1);
    mesh = triangulation(lowerShape);
    [isSourceVertex, sourceIndex] = ismember(mesh.Points, lower_units, 'rows');
    if ~all(isSourceVertex) || numel(unique(sourceIndex)) ~= numel(sourceIndex)
        return;
    end
    faceCount = size(mesh.ConnectivityList, 1);
    startRegions_units = cell(faceCount, 1);
    endRegions_units   = cell(faceCount, 1);
    sourceIndexRegions = cell(faceCount, 1);
    for faceIndex = 1:faceCount
        pointIndex = mesh.ConnectivityList(faceIndex, :);
        sourceIndexRegions{faceIndex} = sourceIndex(pointIndex);
        startRegions_units{faceIndex} = mesh.Points(pointIndex, :);
        endRegions_units{faceIndex} = upper_units(sourceIndexRegions{faceIndex}, :);
        if ~remainsStrictlyConvex(startRegions_units{faceIndex}, ...
                endRegions_units{faceIndex}, coordinateScale_units)
            startRegions_units = cell(0, 1);
            endRegions_units = cell(0, 1);
            return;
        end
    end
    verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceIndexRegions, ...
        lower_units, upper_units, lowerShape, upperShape, topologyMotionVerified);
end

function verified = partitionMatchesEndpointShapes( ...
        startRegions_units, endRegions_units, sourceIndexRegions, ...
        lower_units, upper_units, lowerShape, upperShape, topologyMotionVerified)
    % Endpoint Boolean equality catches mapping or triangulation changes
    % before the continuous face certificates are trusted.
    if topologyMotionVerified && partitionHasExactRingTopology( ...
            sourceIndexRegions, lower_units, upper_units, lowerShape, upperShape)
        verified = true;
        return;
    end
    startUnion = unionRegions(startRegions_units);
    endUnion = unionRegions(endRegions_units);
    areaScale_units2 = max([1, area(lowerShape), area(upperShape)]);
    areaTolerance_units2 = 4096 * eps(areaScale_units2);
    verified = area(xor(startUnion, lowerShape)) <= areaTolerance_units2 && ...
        area(xor(endUnion, upperShape)) <= areaTolerance_units2;
end

function verified = partitionHasExactRingTopology( ...
        sourceIndexRegions, lower_units, upper_units, lowerShape, upperShape)
    % For an unsimplified simple ring, a partition is carried exactly when
    % every outer edge is one source edge and every interior edge is shared
    % once in each direction. Strict face-motion and boundary certificates
    % are applied by the caller after this endpoint topology check.
    vertexCount = size(lower_units, 1);
    verified = isequal(size(lower_units), size(upper_units)) && ...
        size(unique(lower_units, 'rows'), 1) == vertexCount && ...
        size(unique(upper_units, 'rows'), 1) == vertexCount && ...
        shapeHasExactRingBoundary(lowerShape, lower_units) && ...
        shapeHasExactRingBoundary(upperShape, upper_units) && ...
        all(cellfun(@numel, sourceIndexRegions) >= 3);
    if ~verified || isempty(sourceIndexRegions)
        return;
    end

    edgeCount = sum(cellfun(@numel, sourceIndexRegions));
    edgeStart = zeros(edgeCount, 1);
    edgeEnd   = zeros(edgeCount, 1);
    firstEdge = 1;
    for regionIndex = 1:numel(sourceIndexRegions)
        sourceIndex = sourceIndexRegions{regionIndex}(:);
        finalEdge = firstEdge + numel(sourceIndex) - 1;
        edgeStart(firstEdge:finalEdge) = sourceIndex;
        edgeEnd(firstEdge:finalEdge)   = circshift(sourceIndex, -1);
        firstEdge = finalEdge + 1;
    end
    undirectedEdges = [min(edgeStart, edgeEnd), max(edgeStart, edgeEnd)];
    [uniqueEdges, ~, edgeGroup] = unique(undirectedEdges, 'rows');
    multiplicity = accumarray(edgeGroup, 1);
    forwardCount = accumarray(edgeGroup, edgeStart < edgeEnd);
    if any(multiplicity > 2) || ...
            any(forwardCount(multiplicity == 2) ~= 1) || ...
            vertexCount - size(uniqueEdges, 1) + numel(sourceIndexRegions) ~= 1
        verified = false;
        return;
    end

    nextSourceIndex = [2:vertexCount, 1].';
    expectedBoundaryEdges = sortrows([ ...
        min((1:vertexCount).', nextSourceIndex), ...
        max((1:vertexCount).', nextSourceIndex)]);
    actualBoundaryEdges = sortrows(uniqueEdges(multiplicity == 1, :));
    boundaryOccurrence = multiplicity(edgeGroup) == 1;
    directedBoundaryEdges = sortrows([ ...
        edgeStart(boundaryOccurrence), edgeEnd(boundaryOccurrence)]);
    forwardBoundaryEdges = sortrows([(1:vertexCount).', nextSourceIndex]);
    reverseBoundaryEdges = sortrows([nextSourceIndex, (1:vertexCount).']);
    verified = isequal(actualBoundaryEdges, expectedBoundaryEdges) && ...
        (isequal(directedBoundaryEdges, forwardBoundaryEdges) || ...
        isequal(directedBoundaryEdges, reverseBoundaryEdges));
end

function verified = shapeHasExactRingBoundary(shape, vertices_units)
    components = regions(shape);
    if numel(components) ~= 1 || components.NumHoles ~= 0
        verified = false;
        return;
    end
    shapeVertices_units = components.Vertices;
    [shapeVertexIsSource, shapeSourceIndex] = ismember( ...
        shapeVertices_units, vertices_units, 'rows');
    if ~isequal(size(shapeVertices_units), size(vertices_units)) || ...
            ~all(shapeVertexIsSource) || numel(unique(shapeSourceIndex)) ~= numel(shapeSourceIndex)
        verified = false;
        return;
    end
    nextShapeIndex = circshift(shapeSourceIndex, -1);
    shapeEdges = sortrows([ ...
        min(shapeSourceIndex, nextShapeIndex), max(shapeSourceIndex, nextShapeIndex)]);
    vertexCount = size(vertices_units, 1);
    nextSourceIndex = [2:vertexCount, 1].';
    sourceEdges = sortrows([ ...
        min((1:vertexCount).', nextSourceIndex), max((1:vertexCount).', nextSourceIndex)]);
    verified = isequal(shapeEdges, sourceEdges);
end

function shape = unionRegions(regions_units)
    % Union exact convex faces without simplifying away collinear vertices.
    shape = polyshape();
    for regionIndex = 1:numel(regions_units)
        region_units = regions_units{regionIndex};
        shape = union(shape, polyshape( ...
            region_units, 'Simplify', false, 'KeepCollinearPoints', true));
    end
end

function verified = movingBoundaryRemainsSimple(lower_units, upper_units, coordinateScale_units)
    % A crossing can begin or end only when one endpoint becomes collinear
    % with the other moving edge. Split time at every such quadratic root,
    % then test the roots and the open intervals between them.
    verified                    = true;
    vertexCount                 = size(lower_units, 1);
    delta_units                 = upper_units - lower_units;
    orientationTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    positionTolerance_units     = 4096 * eps(coordinateScale_units);

    % Each endpoint is affine in time. These bounds contain every point on
    % each moving edge throughout the interval, so disjoint ranges certify
    % separation without solving any orientation polynomial.
    nextIndex         = [2:vertexCount, 1];
    edgeMinimum_units = min(min(lower_units, lower_units(nextIndex, :)), ...
        min(upper_units, upper_units(nextIndex, :)));
    edgeMaximum_units = max(max(lower_units, lower_units(nextIndex, :)), ...
        max(upper_units, upper_units(nextIndex, :)));
    for firstIndex = 1:vertexCount
        firstNext     = mod(firstIndex, vertexCount) + 1;
        secondIndices = (firstIndex + 1:vertexCount).';
        overlapping   = all(edgeMaximum_units(firstIndex, :) >= ...
            edgeMinimum_units(secondIndices, :) - positionTolerance_units & ...
            edgeMaximum_units(secondIndices, :) >= ...
            edgeMinimum_units(firstIndex, :) - positionTolerance_units, 2);
        for secondIndex = reshape(secondIndices(overlapping), 1, [])
            secondNext = mod(secondIndex, vertexCount) + 1;
            edgesShareAnEndpoint = secondIndex == firstNext || secondNext == firstIndex;
            if edgesShareAnEndpoint
                continue;
            end
            triples = [firstIndex, firstNext, secondIndex; ...
                firstIndex, firstNext, secondNext; ...
                secondIndex, secondNext, firstIndex; ...
                secondIndex, secondNext, firstNext];
            criticalTau = [0; 1];
            for tripleIndex = 1:4
                indices      = triples(tripleIndex, :);
                coefficients = orientationCoefficients( ...
                    lower_units(indices, :), delta_units(indices, :));
                orientationIsIndeterminate = all(abs(coefficients) <= orientationTolerance_units2);
                if orientationIsIndeterminate
                    verified = false;
                    return;
                end
                rootsTau    = realPolynomialRoots(coefficients, orientationTolerance_units2);
                criticalTau = [criticalTau; rootsTau]; %#ok<AGROW>
            end
            criticalTau = unique(min(1, max(0, criticalTau)));
            probeTau    = [criticalTau; (criticalTau(1:end - 1) + criticalTau(2:end)) / 2];
            for tau = reshape(probeTau, 1, [])
                points_units = lower_units + tau * delta_units;
                if segmentsIntersect(points_units(firstIndex, :), points_units(firstNext, :), ...
                        points_units(secondIndex, :), points_units(secondNext, :), ...
                        orientationTolerance_units2, positionTolerance_units)
                    verified = false;
                    return;
                end
            end
        end
    end
end

function coefficients = orientationCoefficients(points_units, delta_units)
    % Expand the triple's signed area as a quadratic in the interval fraction.
    firstEdge_units   = points_units(2, :) - points_units(1, :);
    secondEdge_units  = points_units(3, :) - points_units(1, :);
    firstDelta_units  = delta_units(2, :) - delta_units(1, :);
    secondDelta_units = delta_units(3, :) - delta_units(1, :);
    coefficients = [cross2d(firstDelta_units, secondDelta_units), ...
        cross2d(firstDelta_units, secondEdge_units) + cross2d(firstEdge_units, secondDelta_units), ...
        cross2d(firstEdge_units, secondEdge_units)];
end

function rootsTau = realPolynomialRoots(coefficients, tolerance)
    % Return the real roots inside [0, 1], degrading to the linear case when
    % the leading coefficients are within tolerance of zero.
    leadingIsNegligible = abs(coefficients(1)) <= tolerance;
    linearIsNegligible  = abs(coefficients(2)) <= tolerance;
    if leadingIsNegligible && linearIsNegligible
        rootsTau = zeros(0, 1);
    elseif leadingIsNegligible
        rootsTau = -coefficients(3) / coefficients(2);
    else
        candidate          = roots(coefficients);
        imaginaryTolerance = 1024 * eps(max(1, max(abs(candidate))));
        rootsTau           = real(candidate(abs(imag(candidate)) <= imaginaryTolerance));
    end
    timeTolerance = 1024 * eps;
    rootsTau = rootsTau(rootsTau >= -timeTolerance & rootsTau <= 1 + timeTolerance);
end

function intersects = segmentsIntersect(firstStart_units, firstEnd_units, ...
        secondStart_units, secondEnd_units, orientationTolerance_units2, positionTolerance_units)
    % Report a closed-segment intersection using tolerance-snapped orientations.
    firstDirection_units  = firstEnd_units - firstStart_units;
    secondDirection_units = secondEnd_units - secondStart_units;
    orientations = [cross2d(firstDirection_units, secondStart_units - firstStart_units), ...
        cross2d(firstDirection_units, secondEnd_units - firstStart_units), ...
        cross2d(secondDirection_units, firstStart_units - secondStart_units), ...
        cross2d(secondDirection_units, firstEnd_units - secondStart_units)];
    signs = sign(orientations);
    signs(abs(orientations) <= orientationTolerance_units2) = 0;
    lowerCorner_units = max(min([firstStart_units; firstEnd_units]), ...
        min([secondStart_units; secondEnd_units]));
    upperCorner_units = min(max([firstStart_units; firstEnd_units]), ...
        max([secondStart_units; secondEnd_units]));
    boundingBoxesOverlap = all(lowerCorner_units <= upperCorner_units + positionTolerance_units);
    intersects = boundingBoxesOverlap && ...
        signs(1) * signs(2) <= 0 && signs(3) * signs(4) <= 0;
end

function verified = remainsStrictlyConvex(lower_units, upper_units, coordinateScale_units)
    % Check that interpolated turns keep the same nonzero sign on [0, 1].
    lowerEdge_units      = circshift(lower_units, -1, 1) - lower_units;
    upperEdge_units      = circshift(upper_units, -1, 1) - upper_units;
    lowerTurn_units2     = cross2d(lowerEdge_units, circshift(lowerEdge_units, -1, 1));
    orientation          = sign(sum(lowerTurn_units2));
    turnTolerance_units2 = 4096 * eps(coordinateScale_units ^ 2);
    if orientation == 0 || any(orientation * lowerTurn_units2 <= turnTolerance_units2)
        verified = false;
        return;
    end
    edgeDelta_units     = upperEdge_units - lowerEdge_units;
    nextLowerEdge_units = circshift(lowerEdge_units, -1, 1);
    nextEdgeDelta_units = circshift(edgeDelta_units, -1, 1);
    constant_units2     = cross2d(lowerEdge_units, nextLowerEdge_units);
    linear_units2       = cross2d(edgeDelta_units, nextLowerEdge_units) + ...
        cross2d(lowerEdge_units, nextEdgeDelta_units);
    quadratic_units2    = cross2d(edgeDelta_units, nextEdgeDelta_units);
    verified            = true;
    for vertexIndex = 1:size(lower_units, 1)
        candidateTau = [0; 1];
        if quadratic_units2(vertexIndex) ~= 0
            stationaryTau = -linear_units2(vertexIndex) / (2 * quadratic_units2(vertexIndex));
            if stationaryTau > 0 && stationaryTau < 1
                candidateTau(end + 1, 1) = stationaryTau; %#ok<AGROW>
            end
        end
        turn_units2 = constant_units2(vertexIndex) + ...
            linear_units2(vertexIndex) * candidateTau + ...
            quadratic_units2(vertexIndex) * candidateTau .^ 2;
        if any(orientation * turn_units2 <= turnTolerance_units2)
            verified = false;
            return;
        end
    end
end

function value = cross2d(first_units, second_units)
    % Return row-wise signed two-dimensional cross products.
    value = first_units(:, 1) .* second_units(:, 2) - first_units(:, 2) .* second_units(:, 1);
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
    % The main stage certifies the entire span with one shared face partition.
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
            % authoritative sample in position.
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
