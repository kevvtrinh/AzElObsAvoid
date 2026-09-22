function [isClear, blockingCellIndices, collisionTimes_s] = affineEdgesAreClear( ...
    startNodeIndices, endNodeIndices, startTime_s, endTime_s, movingCellLookup)
%% Section 0: Header & Readme
% SYNTAX
%   [isClear, blockingCellIndices, collisionTimes_s] = ...
%       obstacleAvoidance.search.affineEdgesAreClear( ...
%       startNodeIndices, endNodeIndices, startTime_s, endTime_s, movingCellLookup)
%**************************************************************************
% PURPOSE
%   - Check straight paths over a shared travel interval against moving
%     obstacle regions. Each cell is a convex region (no inward corners)
%     whose vertices move at constant velocity during its active interval.
%   - Return the first blocking region and, when found, a time at which the
%     path point is clearly inside it.
%**************************************************************************
% INPUTS
%   - startNodeIndices, endNodeIndices (N-by-1 numeric)
%       Node indices of the segment endpoints.
%   - startTime_s, endTime_s (numeric scalars)
%       Start and end of travel, with end >= start. Equal times check only
%       the segment's start position.
%   - movingCellLookup (scalar struct)
%       Regions, boundary directions, node positions, and saved candidate
%       regions for each node pair, as returned by createMovingCellIndex.
%**************************************************************************
% OUTPUTS
%   - isClear (N-by-1 logical)
%       True when the segment stays clear of every region, including its
%       boundary contact tolerance.
%   - blockingCellIndices (N-by-1 uint32)
%       First blocking cell of each blocked segment, zero when clear.
%   - collisionTimes_s (N-by-1 numeric)
%       A late time found clearly inside the blocking region. NaN means
%       the segment is clear or no clearly-inside time was found. The arrival
%       search needs a strict interior point to reuse a collision when
%       checking later arrival times.
%**************************************************************************
% UNITS
%   - Positions use coordinate units; times use seconds. Fractions from
%     0 to 1 describe progress through an interval.
%**************************************************************************

%% Section 1: Read The Regions And Initialize Segment Results

segmentStart_units       = movingCellLookup.NodePosition_units(startNodeIndices, :);
segmentEnd_units         = movingCellLookup.NodePosition_units(endNodeIndices, :);
timedRegions             = movingCellLookup.Cells;
nodePairCache            = movingCellLookup;
regionIsCounterclockwise = movingCellLookup.CellIsCounterclockwise;

segmentCount        = size(segmentStart_units, 1);
isClear             = true(segmentCount, 1);
blockingCellIndices = zeros(segmentCount, 1, "uint32");
collisionTimes_s    = NaN(segmentCount, 1);
if segmentCount == 0 || isempty(timedRegions.Regions_units)
    return
end
if segmentCount == 1
    [isClear, blockingCellIndices, collisionTimes_s] = singleTimedSegmentIsClear( ...
        segmentStart_units, segmentEnd_units, startNodeIndices, endNodeIndices, ...
        startTime_s, endTime_s, timedRegions, nodePairCache, regionIsCounterclockwise);
    return
end

%% Section 2: Select Regions That Can Meet Each Timed Segment

% The saved box entry/exit fractions show where a segment can meet a region.
% Keep that pair only when those fractions also overlap its active times.
% Passing this quick check means a full moving-boundary check is still needed.
regionIndicesBySegment = cell(segmentCount, 1);
segmentIndexBlocks     = cell(segmentCount, 1);
traversalDuration_s    = endTime_s - startTime_s;
for segmentIndex = 1:segmentCount
    [pairRegionIndices, boxEntryFractions, boxExitFractions, activeStartTimes_s, activeEndTimes_s] = ...
        getCandidateRegionsForNodePair( ...
        startNodeIndices(segmentIndex), endNodeIndices(segmentIndex), nodePairCache);
    if isempty(pairRegionIndices)
        continue
    end

    % Entries are ordered by start time. Ignore regions that start after
    % travel ends, then remove those that stop before travel begins.
    lastStartedRegionOffset = obstacleAvoidance.search.sortedUpperBound(activeStartTimes_s, endTime_s);
    if lastStartedRegionOffset == 0
        continue
    end
    candidateCacheIndices = (1:lastStartedRegionOffset).';
    candidateCacheIndices = candidateCacheIndices( ...
        activeEndTimes_s(candidateCacheIndices) >= startTime_s);
    if isempty(candidateCacheIndices)
        continue
    end
    pairRegionIndices  = pairRegionIndices(candidateCacheIndices);
    activeIntervals_s  = timedRegions.ActiveTimeInterval_s(pairRegionIndices, :);
    overlapStart_s     = max(startTime_s, activeIntervals_s(:, 1));
    overlapEnd_s       = min(endTime_s, activeIntervals_s(:, 2));
    overlapsActiveTime = overlapStart_s <= overlapEnd_s;
    if traversalDuration_s > 0
        overlapStartFractions = (overlapStart_s - startTime_s) / traversalDuration_s;
        overlapEndFractions   = (overlapEnd_s - startTime_s) / traversalDuration_s;
        absoluteTimeScale_s   = max([ ...
            repmat(max(abs([startTime_s, endTime_s])), numel(pairRegionIndices), 1), ...
            abs(activeIntervals_s)], [], 2);
        % Convert time-rounding uncertainty to a path fraction. Widen the
        % quick check so rounding cannot discard a possible collision.
        timeRoundingFraction = 512 * eps(max(1, absoluteTimeScale_s)) / traversalDuration_s;
        timeRoundingFraction(~isfinite(timeRoundingFraction)) = Inf;
        overlapStartFractions = overlapStartFractions - timeRoundingFraction;
        overlapEndFractions   = overlapEndFractions + timeRoundingFraction;
    else
        overlapStartFractions = zeros(size(overlapStart_s));
        overlapEndFractions   = zeros(size(overlapEnd_s));
    end
    candidateBoxEntryFractions = boxEntryFractions(candidateCacheIndices);
    candidateBoxExitFractions  = boxExitFractions(candidateCacheIndices);
    pathCouldMeetRegion        = candidateBoxExitFractions >= overlapStartFractions & ...
        candidateBoxEntryFractions <= overlapEndFractions;
    % Keep both indices for every surviving segment/region pair.
    keptCandidateIndices = find(overlapsActiveTime & pathCouldMeetRegion);
    regionIndicesBySegment{segmentIndex} = pairRegionIndices(keptCandidateIndices);
    segmentIndexBlocks{segmentIndex} = repmat(segmentIndex, numel(keptCandidateIndices), 1);
end

% Join the candidate pairs, then group them by region. Checking regions
% in index order preserves the first blocking region for each segment.
candidateRegionIndices  = vertcat(regionIndicesBySegment{:});
candidateSegmentIndices = vertcat(segmentIndexBlocks{:});
if isempty(candidateRegionIndices)
    return
end
[candidateRegionIndices, regionSortOrder] = sort(candidateRegionIndices);
candidateSegmentIndices = candidateSegmentIndices(regionSortOrder);
regionGroupStarts       = [1; 1 + find(diff(candidateRegionIndices) ~= 0)];
regionGroupEnds         = [regionGroupStarts(2:end) - 1; numel(candidateRegionIndices)];

%% Section 3: Check The Full Motion Against Each Candidate Region

for regionGroupIndex = 1:numel(regionGroupStarts)
    if ~any(isClear)
        break
    end
    regionIndex     = candidateRegionIndices(regionGroupStarts(regionGroupIndex));
    regionGroupRows = regionGroupStarts(regionGroupIndex):regionGroupEnds(regionGroupIndex);
    segmentsToCheck = candidateSegmentIndices(regionGroupRows);
    segmentsToCheck = segmentsToCheck(isClear(segmentsToCheck));
    if isempty(segmentsToCheck)
        continue
    end
    overlapStart_s = max(startTime_s, timedRegions.ActiveTimeInterval_s(regionIndex, 1));
    overlapEnd_s   = min(endTime_s, timedRegions.ActiveTimeInterval_s(regionIndex, 2));

    if traversalDuration_s > 0
        pathStartFraction = (overlapStart_s - startTime_s) / traversalDuration_s;
        pathEndFraction   = (overlapEnd_s - startTime_s) / traversalDuration_s;
    else
        pathStartFraction = 0;
        pathEndFraction   = 0;
    end

    % Move both the path endpoints and region vertices to the time they
    % share. For example, overlap [2 6] within travel [0 10] uses path
    % fractions [0.2 0.6]. The region uses its own active time interval.
    pathStart_units = segmentStart_units(segmentsToCheck, :) + pathStartFraction .* ...
        (segmentEnd_units(segmentsToCheck, :) - segmentStart_units(segmentsToCheck, :));
    pathEnd_units = segmentStart_units(segmentsToCheck, :) + pathEndFraction .* ...
        (segmentEnd_units(segmentsToCheck, :) - segmentStart_units(segmentsToCheck, :));

    regionDuration_s    = diff(timedRegions.ActiveTimeInterval_s(regionIndex, :));
    regionStartFraction = (overlapStart_s - timedRegions.ActiveTimeInterval_s(regionIndex, 1)) / regionDuration_s;
    regionEndFraction   = (overlapEnd_s - timedRegions.ActiveTimeInterval_s(regionIndex, 1)) / regionDuration_s;

    regionStart_units              = timedRegions.Regions_units{regionIndex};
    regionVertexDisplacement_units = timedRegions.EndRegions_units{regionIndex} - regionStart_units;
    overlapRegionStart_units       = regionStart_units + regionStartFraction .* regionVertexDisplacement_units;
    overlapRegionEnd_units         = regionStart_units + regionEndFraction .* regionVertexDisplacement_units;
    if isscalar(segmentsToCheck)
        [pointTouchesRegion, collisionFraction] = movingPointTouchesConvexRegion( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, ...
            overlapRegionEnd_units, regionIsCounterclockwise(regionIndex));
    else
        [pointTouchesRegion, collisionFraction] = movingPointsTouchConvexRegion( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, ...
            overlapRegionEnd_units, regionIsCounterclockwise(regionIndex));
    end
    blockedSegmentIndices = segmentsToCheck(pointTouchesRegion);
    isClear(blockedSegmentIndices) = false;
    blockingCellIndices(blockedSegmentIndices) = uint32(regionIndex);
    collisionTimes_s(blockedSegmentIndices) = overlapStart_s + ...
        collisionFraction(pointTouchesRegion) .* (overlapEnd_s - overlapStart_s);
end
end

%% Section 4: Local Functions

function [isClear, blockingCellIndex, collisionTime_s] = singleTimedSegmentIsClear( ...
        segmentStart_units, segmentEnd_units, startNodeIndex, endNodeIndex, ...
        startTime_s, endTime_s, timedRegions, nodePairCache, regionIsCounterclockwise)
    % Use the same collision rules for one segment, without constructing
    % arrays that group several segments by region.
    isClear           = true;
    blockingCellIndex = uint32(0);
    collisionTime_s   = NaN;
    [pairRegionIndices, boxEntryFractions, boxExitFractions, activeStartTimes_s, activeEndTimes_s] = ...
        getCandidateRegionsForNodePair(startNodeIndex, endNodeIndex, nodePairCache);
    if isempty(pairRegionIndices)
        return
    end

    % Entries are ordered by start time. Ignore regions that start after
    % travel ends, then remove those that stop before travel begins.
    lastStartedRegionOffset = obstacleAvoidance.search.sortedUpperBound(activeStartTimes_s, endTime_s);
    if lastStartedRegionOffset == 0
        return
    end
    candidateCacheIndices = (1:lastStartedRegionOffset).';
    candidateCacheIndices = candidateCacheIndices( ...
        activeEndTimes_s(candidateCacheIndices) >= startTime_s);
    if isempty(candidateCacheIndices)
        return
    end
    pairRegionIndices   = pairRegionIndices(candidateCacheIndices);
    activeIntervals_s   = timedRegions.ActiveTimeInterval_s(pairRegionIndices, :);
    overlapStart_s      = max(startTime_s, activeIntervals_s(:, 1));
    overlapEnd_s        = min(endTime_s, activeIntervals_s(:, 2));
    overlapsActiveTime  = overlapStart_s <= overlapEnd_s;
    traversalDuration_s = endTime_s - startTime_s;
    if traversalDuration_s > 0
        overlapStartFractions = (overlapStart_s - startTime_s) / traversalDuration_s;
        overlapEndFractions   = (overlapEnd_s - startTime_s) / traversalDuration_s;
        absoluteTimeScale_s   = max([ ...
            repmat(max(abs([startTime_s, endTime_s])), numel(pairRegionIndices), 1), ...
            abs(activeIntervals_s)], [], 2);
        % Convert time-rounding uncertainty to a path fraction. Widen the
        % quick check so rounding cannot discard a possible collision.
        timeRoundingFraction = 512 * eps(max(1, absoluteTimeScale_s)) / traversalDuration_s;
        timeRoundingFraction(~isfinite(timeRoundingFraction)) = Inf;
        overlapStartFractions = overlapStartFractions - timeRoundingFraction;
        overlapEndFractions   = overlapEndFractions + timeRoundingFraction;
    else
        overlapStartFractions = zeros(size(overlapStart_s));
        overlapEndFractions   = zeros(size(overlapEnd_s));
    end
    candidateBoxEntryFractions = boxEntryFractions(candidateCacheIndices);
    candidateBoxExitFractions  = boxExitFractions(candidateCacheIndices);
    pathCouldMeetRegion        = candidateBoxExitFractions >= overlapStartFractions & ...
        candidateBoxEntryFractions <= overlapEndFractions;
    candidateRegionIndices = sort(pairRegionIndices(overlapsActiveTime & pathCouldMeetRegion));

    for regionIndex = reshape(candidateRegionIndices, 1, [])
        overlapStart_s = max(startTime_s, timedRegions.ActiveTimeInterval_s(regionIndex, 1));
        overlapEnd_s   = min(endTime_s, timedRegions.ActiveTimeInterval_s(regionIndex, 2));
        if traversalDuration_s > 0
            pathStartFraction = (overlapStart_s - startTime_s) / traversalDuration_s;
            pathEndFraction   = (overlapEnd_s - startTime_s) / traversalDuration_s;
        else
            pathStartFraction = 0;
            pathEndFraction   = 0;
        end
        segmentDisplacement_units = segmentEnd_units - segmentStart_units;
        pathStart_units           = segmentStart_units + pathStartFraction .* segmentDisplacement_units;
        pathEnd_units             = segmentStart_units + pathEndFraction .* segmentDisplacement_units;

        regionDuration_s    = diff(timedRegions.ActiveTimeInterval_s(regionIndex, :));
        regionStartFraction = (overlapStart_s - timedRegions.ActiveTimeInterval_s(regionIndex, 1)) / regionDuration_s;
        regionEndFraction   = (overlapEnd_s - timedRegions.ActiveTimeInterval_s(regionIndex, 1)) / regionDuration_s;

        regionStart_units              = timedRegions.Regions_units{regionIndex};
        regionVertexDisplacement_units = timedRegions.EndRegions_units{regionIndex} - regionStart_units;
        overlapRegionStart_units       = regionStart_units + regionStartFraction .* regionVertexDisplacement_units;
        overlapRegionEnd_units         = regionStart_units + regionEndFraction .* regionVertexDisplacement_units;
        [pointTouchesRegion, collisionFraction] = movingPointTouchesConvexRegion( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, overlapRegionEnd_units, ...
            regionIsCounterclockwise(regionIndex));
        if pointTouchesRegion
            isClear           = false;
            blockingCellIndex = uint32(regionIndex);
            collisionTime_s   = overlapStart_s + collisionFraction * (overlapEnd_s - overlapStart_s);
            return
        end
    end
end

function [regionIndices, boxEntryFractions, boxExitFractions, activeStartTimes_s, activeEndTimes_s] = ...
        getCandidateRegionsForNodePair(startNodeIndex, endNodeIndex, nodePairCache)
    % Reuse the node pair's candidate regions when saved. Large searches
    % calculate the same entry on demand to limit memory use.
    if nodePairCache.IsPrecomputed
        pairIndex          = startNodeIndex + nodePairCache.NodeCount * (endNodeIndex - 1);
        regionIndices      = nodePairCache.CellIndices{pairIndex};
        boxEntryFractions  = nodePairCache.QEnter{pairIndex};
        boxExitFractions   = nodePairCache.QExit{pairIndex};
        activeStartTimes_s = nodePairCache.ActiveStart_s{pairIndex};
        activeEndTimes_s   = nodePairCache.ActiveEnd_s{pairIndex};
        return
    end
    [regionIndices, boxEntryFractions, boxExitFractions, activeStartTimes_s, activeEndTimes_s] = ...
        obstacleAvoidance.search.computePairCellEntry( ...
        nodePairCache.NodePosition_units(startNodeIndex, :), ...
        nodePairCache.NodePosition_units(endNodeIndex, :), ...
        nodePairCache.CellLower_units, nodePairCache.CellUpper_units, nodePairCache.ActiveIntervals_s);
end

function [pointTouchesRegion, strictCollisionFraction] = movingPointsTouchConvexRegion( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
        isCounterclockwise)
    % Check several moving points against the same moving convex region.
    % Each row is a boundary edge; each column is a moving path point.
    % A point lies inside only when it is on the inside side of every edge.
    pointCount              = size(pointStart_units, 1);
    pointTouchesRegion      = false(pointCount, 1);
    strictCollisionFraction = NaN(pointCount, 1);
    if pointCount == 0
        return
    end
    if pointCount == 1
        [pointTouchesRegion, strictCollisionFraction] = movingPointTouchesConvexRegion( ...
            pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
            isCounterclockwise);
        return
    end

    vertexCount       = size(regionStart_units, 1);
    nextVertexIndices = [2:vertexCount, 1];

    boundaryEdgeStart_units  = regionStart_units(nextVertexIndices, :) - regionStart_units;
    boundaryEdgeChange_units = ...
        (regionEnd_units(nextVertexIndices, :) - regionEnd_units) - boundaryEdgeStart_units;
    regionVertexDisplacement_units = regionEnd_units - regionStart_units;
    pointDisplacement_units        = pointEnd_units - pointStart_units;

    % Use each boundary vertex as the origin for its point-to-edge test.
    % Both the edge vector and point offset change linearly with fraction u.
    % Their cross product is A x u^2 + B x u + C, with units of area.
    relativeStartX_units        = pointStart_units(:, 1).' - regionStart_units(:, 1);
    relativeStartY_units        = pointStart_units(:, 2).' - regionStart_units(:, 2);
    relativeChangeX_units       = pointDisplacement_units(:, 1).' - regionVertexDisplacement_units(:, 1);
    relativeChangeY_units       = pointDisplacement_units(:, 2).' - regionVertexDisplacement_units(:, 2);
    quadraticCoefficient_units2 = boundaryEdgeChange_units(:, 1) .* relativeChangeY_units - ...
        boundaryEdgeChange_units(:, 2) .* relativeChangeX_units;
    linearCoefficient_units2 = (boundaryEdgeChange_units(:, 1) .* relativeStartY_units - ...
        boundaryEdgeChange_units(:, 2) .* relativeStartX_units) + ...
        (boundaryEdgeStart_units(:, 1) .* relativeChangeY_units - ...
        boundaryEdgeStart_units(:, 2) .* relativeChangeX_units);
    constantCoefficient_units2 = boundaryEdgeStart_units(:, 1) .* relativeStartY_units - ...
        boundaryEdgeStart_units(:, 2) .* relativeStartX_units;
    regionScale_units        = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units         = max(max(abs(pointStart_units), [], 2), max(abs(pointEnd_units), [], 2));
    coordinateScale_units    = max(1, max(regionScale_units, pointScale_units));
    edgeSideTolerance_units2 = obstacleAvoidance.search.createResidualBound(coordinateScale_units);

    % Solve where each edge-side value becomes zero. These fractions mark
    % possible boundary crossings; NaN means no usable crossing in [0 1].
    coefficientScale = max(1, max(max(abs(quadraticCoefficient_units2), ...
        abs(linearCoefficient_units2)), abs(constantCoefficient_units2)));
    rootTolerance             = 256 * eps(coefficientScale);
    quadraticTermIsNegligible = abs(quadraticCoefficient_units2) <= rootTolerance;
    discriminant_units4       = linearCoefficient_units2 .* linearCoefficient_units2 - ...
        4 * quadraticCoefficient_units2 .* constantCoefficient_units2;
    hasQuadraticRoots       = ~quadraticTermIsNegligible & discriminant_units4 >= -rootTolerance;
    discriminantRoot_units2 = sqrt(max(0, discriminant_units4));
    firstRootFraction       = (-linearCoefficient_units2 - discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    secondRootFraction = (-linearCoefficient_units2 + discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    linearRootFraction = -constantCoefficient_units2 ./ linearCoefficient_units2;
    hasLinearRoot      = quadraticTermIsNegligible & abs(linearCoefficient_units2) > rootTolerance;

    % A negligible quadratic term uses the linear solution. Discard missing
    % roots and roots outside the interval before choosing check fractions.
    firstRootFraction(quadraticTermIsNegligible) = linearRootFraction(quadraticTermIsNegligible);
    firstRootFraction(~hasQuadraticRoots & ~hasLinearRoot) = NaN;
    secondRootFraction(~hasQuadraticRoots) = NaN;
    firstRootFraction(~(firstRootFraction >= 0 & firstRootFraction <= 1)) = NaN;
    secondRootFraction(~(secondRootFraction >= 0 & secondRootFraction <= 1)) = NaN;

    % Check the endpoints, crossings, and one midpoint between crossings.
    % Between consecutive roots, each edge-side value keeps the same sign.
    eventFractions = sort( ...
        [zeros(pointCount, 1), ones(pointCount, 1), firstRootFraction.', secondRootFraction.'], 2);
    checkFractions        = [eventFractions, (eventFractions(:, 1:end - 1) + eventFractions(:, 2:end)) / 2];
    checkFractionsByPoint = reshape(checkFractions, 1, pointCount, []);
    edgeSideValues_units2 = quadraticCoefficient_units2 .* checkFractionsByPoint.^2 + ...
        linearCoefficient_units2 .* checkFractionsByPoint + constantCoefficient_units2;

    % Counterclockwise boundaries have their inside on the left (+ cross
    % product); clockwise boundaries have it on the right (- cross product).
    % Boundary contact blocks travel. Reusing a collision for later arrivals
    % requires a point farther inside: more than 16 x the side tolerance.
    edgeSideTolerance_units2 = reshape(edgeSideTolerance_units2, 1, pointCount);
    if isCounterclockwise
        onInsideSide        = edgeSideValues_units2 >= -edgeSideTolerance_units2;
        clearlyOnInsideSide = edgeSideValues_units2 > 16 * edgeSideTolerance_units2;
    else
        onInsideSide        = edgeSideValues_units2 <= edgeSideTolerance_units2;
        clearlyOnInsideSide = edgeSideValues_units2 < -16 * edgeSideTolerance_units2;
    end

    % A point touches the region when all edges accept it at any check time.
    pointTouchesRegion    = reshape(any(all(onInsideSide, 1), 3), pointCount, 1);
    pointIsStrictlyInside = reshape(all(clearlyOnInsideSide, 1), pointCount, []);
    for pointIndex = reshape(find(pointTouchesRegion), 1, [])
        strictInsideFractions = checkFractions(pointIndex, pointIsStrictlyInside(pointIndex, :));
        if ~isempty(strictInsideFractions)
            % Find a later point that remains clearly inside this region.
            % The arrival search can use it to check whether slower travel
            % on this same segment would still collide at that physical time.
            % Refine toward the next crossing while retaining an inside point.
            lastInsideFraction   = max(strictInsideFractions);
            finiteEventFractions = eventFractions(pointIndex, isfinite(eventFractions(pointIndex, :)));
            laterEventFractions  = finiteEventFractions(finiteEventFractions > lastInsideFraction);
            if ~isempty(laterEventFractions)
                upperSearchFraction = min(laterEventFractions);
                for refinementIndex = 1:12
                    trialFraction              = (lastInsideFraction + upperSearchFraction) / 2;
                    trialEdgeSideValues_units2 = ...
                        quadraticCoefficient_units2(:, pointIndex) * trialFraction ^ 2 + ...
                        linearCoefficient_units2(:, pointIndex) * trialFraction + ...
                        constantCoefficient_units2(:, pointIndex);
                    if isCounterclockwise
                        trialIsStrict = all(trialEdgeSideValues_units2 > ...
                            16 * edgeSideTolerance_units2(pointIndex));
                    else
                        trialIsStrict = all(trialEdgeSideValues_units2 < ...
                            -16 * edgeSideTolerance_units2(pointIndex));
                    end
                    if trialIsStrict
                        lastInsideFraction = trialFraction;
                    else
                        upperSearchFraction = trialFraction;
                    end
                end
            end
            strictCollisionFraction(pointIndex) = lastInsideFraction;
        end
    end
end

function [pointTouchesRegion, strictCollisionFraction] = movingPointTouchesConvexRegion( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
        isCounterclockwise)
    % Check one moving point with two-dimensional arrays. The batch helper
    % uses an edge x point x check-fraction array for several points.
    vertexCount       = size(regionStart_units, 1);
    nextVertexIndices = [2:vertexCount, 1];

    boundaryEdgeStart_units  = regionStart_units(nextVertexIndices, :) - regionStart_units;
    boundaryEdgeChange_units = ...
        (regionEnd_units(nextVertexIndices, :) - regionEnd_units) - boundaryEdgeStart_units;
    regionVertexDisplacement_units = regionEnd_units - regionStart_units;
    pointDisplacement_units        = pointEnd_units - pointStart_units;

    % edge-side value = cross(edge vector, point - edge start).
    % Linear vertex and point motion make this a quadratic in fraction u.
    relativeStartX_units        = pointStart_units(1) - regionStart_units(:, 1);
    relativeStartY_units        = pointStart_units(2) - regionStart_units(:, 2);
    relativeChangeX_units       = pointDisplacement_units(1) - regionVertexDisplacement_units(:, 1);
    relativeChangeY_units       = pointDisplacement_units(2) - regionVertexDisplacement_units(:, 2);
    quadraticCoefficient_units2 = boundaryEdgeChange_units(:, 1) .* relativeChangeY_units - ...
        boundaryEdgeChange_units(:, 2) .* relativeChangeX_units;
    linearCoefficient_units2 = (boundaryEdgeChange_units(:, 1) .* relativeStartY_units - ...
        boundaryEdgeChange_units(:, 2) .* relativeStartX_units) + ...
        (boundaryEdgeStart_units(:, 1) .* relativeChangeY_units - ...
        boundaryEdgeStart_units(:, 2) .* relativeChangeX_units);
    constantCoefficient_units2 = boundaryEdgeStart_units(:, 1) .* relativeStartY_units - ...
        boundaryEdgeStart_units(:, 2) .* relativeStartX_units;
    regionScale_units = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units  = max(max(abs(pointStart_units), [], 2), ...
        max(abs(pointEnd_units), [], 2));
    coordinateScale_units    = max(1, max(regionScale_units, pointScale_units));
    edgeSideTolerance_units2 = obstacleAvoidance.search.createResidualBound(coordinateScale_units);

    % Reverse clockwise edge signs so a positive value always means inside.
    if ~isCounterclockwise
        quadraticCoefficient_units2 = -quadraticCoefficient_units2;
        linearCoefficient_units2    = -linearCoefficient_units2;
        constantCoefficient_units2  = -constantCoefficient_units2;
    end
    pointTouchesRegion      = false;
    strictCollisionFraction = NaN;

    % An endpoint clearly inside every edge already proves a collision.
    endEdgeSideValues_units2 = quadraticCoefficient_units2 + linearCoefficient_units2 + constantCoefficient_units2;
    if all(endEdgeSideValues_units2 > 16 * edgeSideTolerance_units2)
        pointTouchesRegion      = true;
        strictCollisionFraction = 1;
        return
    end

    % If one edge keeps the point outside for the whole interval, the point
    % cannot enter the region. Check the quadratic maximum at the endpoints
    % and its peak, allowing 2 x the tolerance before declaring it clear.
    maximumEdgeSideValues_units2 = max(constantCoefficient_units2, endEdgeSideValues_units2);
    quadraticOpensDownward = quadraticCoefficient_units2 < 0;
    peakFraction          = -linearCoefficient_units2 ./ (2 * quadraticCoefficient_units2);
    peakIsWithinInterval = quadraticOpensDownward & peakFraction > 0 & peakFraction < 1;
    if any(peakIsWithinInterval)
        peakEdgeSideValues_units2 = quadraticCoefficient_units2 .* peakFraction.^2 + ...
            linearCoefficient_units2 .* peakFraction + constantCoefficient_units2;
        maximumEdgeSideValues_units2(peakIsWithinInterval) = max( ...
            maximumEdgeSideValues_units2(peakIsWithinInterval), ...
            peakEdgeSideValues_units2(peakIsWithinInterval));
    end
    if any(maximumEdgeSideValues_units2 < -2 * edgeSideTolerance_units2)
        return
    end

    coefficientScale = max(1, max(max(abs(quadraticCoefficient_units2), ...
        abs(linearCoefficient_units2)), abs(constantCoefficient_units2)));
    rootTolerance             = 256 * eps(coefficientScale);
    quadraticTermIsNegligible = abs(quadraticCoefficient_units2) <= rootTolerance;
    discriminant_units4       = linearCoefficient_units2 .* linearCoefficient_units2 - ...
        4 * quadraticCoefficient_units2 .* constantCoefficient_units2;
    hasQuadraticRoots       = ~quadraticTermIsNegligible & discriminant_units4 >= -rootTolerance;
    discriminantRoot_units2 = sqrt(max(0, discriminant_units4));
    firstRootFraction       = (-linearCoefficient_units2 - discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    secondRootFraction = (-linearCoefficient_units2 + discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    linearRootFraction = -constantCoefficient_units2 ./ linearCoefficient_units2;
    hasLinearRoot      = quadraticTermIsNegligible & abs(linearCoefficient_units2) > rootTolerance;

    % A negligible quadratic term uses the linear solution. Discard missing
    % roots and roots outside the interval before choosing check fractions.
    firstRootFraction(quadraticTermIsNegligible) = linearRootFraction(quadraticTermIsNegligible);
    firstRootFraction(~hasQuadraticRoots & ~hasLinearRoot) = NaN;
    secondRootFraction(~hasQuadraticRoots) = NaN;
    firstRootFraction(~(firstRootFraction >= 0 & firstRootFraction <= 1)) = NaN;
    secondRootFraction(~(secondRootFraction >= 0 & secondRootFraction <= 1)) = NaN;

    % As in the batch helper, test every boundary crossing and the intervals
    % between crossings, rather than relying on evenly spaced time samples.
    finiteRootFractions = [firstRootFraction(isfinite(firstRootFraction)); ...
        secondRootFraction(isfinite(secondRootFraction))];
    eventFractions        = sort([0, 1, finiteRootFractions.']);
    checkFractions        = [eventFractions, (eventFractions(1:end - 1) + eventFractions(2:end)) / 2];
    edgeSideValues_units2 = quadraticCoefficient_units2 .* checkFractions.^2 + ...
        linearCoefficient_units2 .* checkFractions + constantCoefficient_units2;
    onInsideSide          = edgeSideValues_units2 >= -edgeSideTolerance_units2;
    pointIsStrictlyInside = all(edgeSideValues_units2 > 16 * edgeSideTolerance_units2, 1);
    pointTouchesRegion    = any(all(onInsideSide, 1));
    if ~pointTouchesRegion || ~any(pointIsStrictlyInside)
        return
    end

    % Start with the latest clearly-inside check and move toward the next
    % boundary event. Keep the last inside fraction after 12 refinements.
    lastInsideFraction   = max(checkFractions(pointIsStrictlyInside));
    finiteEventFractions = eventFractions(isfinite(eventFractions));
    laterEventFractions  = finiteEventFractions(finiteEventFractions > lastInsideFraction);
    if ~isempty(laterEventFractions)
        upperSearchFraction = min(laterEventFractions);
        for refinementIndex = 1:12
            trialFraction              = (lastInsideFraction + upperSearchFraction) / 2;
            trialEdgeSideValues_units2 = quadraticCoefficient_units2 * trialFraction ^ 2 + ...
                linearCoefficient_units2 * trialFraction + constantCoefficient_units2;
            trialIsStrict = all(trialEdgeSideValues_units2 > 16 * edgeSideTolerance_units2);
            if trialIsStrict
                lastInsideFraction = trialFraction;
            else
                upperSearchFraction = trialFraction;
            end
        end
    end
    strictCollisionFraction = lastInsideFraction;
end
