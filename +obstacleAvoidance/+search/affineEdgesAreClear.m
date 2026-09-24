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

%% Section 1: Read The Segments And Initialize Their Results

segmentStart_units  = movingCellLookup.NodePosition_units(startNodeIndices, :);
segmentEnd_units    = movingCellLookup.NodePosition_units(endNodeIndices, :);
segmentCount        = size(segmentStart_units, 1);
isClear             = true(segmentCount, 1);
blockingCellIndices = zeros(segmentCount, 1, "uint32");
collisionTimes_s    = NaN(segmentCount, 1);
if segmentCount == 0 || isempty(movingCellLookup.Cells.Regions_units)
    return
end

%% Section 2: Check One Segment Without Building Region Groups

% With one segment, check its candidate regions in index order and stop at
% the first one it touches. This gives the same result as Section 3 without
% building the arrays that group several segments by region.
if segmentCount == 1
    candidateRegionIndices = selectCandidateRegions( ...
        startNodeIndices, endNodeIndices, startTime_s, endTime_s, movingCellLookup);
    for regionIndex = reshape(sort(candidateRegionIndices), 1, [])
        [touchesRegion, collisionTime_s] = checkSegmentsAgainstRegion( ...
            segmentStart_units, segmentEnd_units, startTime_s, endTime_s, ...
            regionIndex, movingCellLookup);
        if touchesRegion
            isClear             = false;
            blockingCellIndices = uint32(regionIndex);
            collisionTimes_s    = collisionTime_s;
            return
        end
    end
    return
end

%% Section 3: Group Several Segments By Region And Check Them Together

% Collect every surviving segment/region pair, keeping both indices.
regionIndicesBySegment = cell(segmentCount, 1);
segmentIndexBlocks     = cell(segmentCount, 1);
for segmentIndex = 1:segmentCount
    regionIndicesBySegment{segmentIndex} = selectCandidateRegions( ...
        startNodeIndices(segmentIndex), endNodeIndices(segmentIndex), ...
        startTime_s, endTime_s, movingCellLookup);
    segmentIndexBlocks{segmentIndex} = repmat(segmentIndex, ...
        numel(regionIndicesBySegment{segmentIndex}), 1);
end
candidateRegionIndices  = vertcat(regionIndicesBySegment{:});
candidateSegmentIndices = vertcat(segmentIndexBlocks{:});
if isempty(candidateRegionIndices)
    return
end

% Check regions in index order so each segment keeps its first blocking
% region. A segment already blocked is not checked again.
[candidateRegionIndices, regionSortOrder] = sort(candidateRegionIndices);
candidateSegmentIndices = candidateSegmentIndices(regionSortOrder);
regionGroupStarts       = [1; 1 + find(diff(candidateRegionIndices) ~= 0)];
regionGroupEnds         = [regionGroupStarts(2:end) - 1; numel(candidateRegionIndices)];
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
    [touchesRegion, groupCollisionTimes_s] = checkSegmentsAgainstRegion( ...
        segmentStart_units(segmentsToCheck, :), segmentEnd_units(segmentsToCheck, :), ...
        startTime_s, endTime_s, regionIndex, movingCellLookup);
    blockedSegmentIndices = segmentsToCheck(touchesRegion);
    isClear(blockedSegmentIndices)             = false;
    blockingCellIndices(blockedSegmentIndices) = uint32(regionIndex);
    collisionTimes_s(blockedSegmentIndices)    = groupCollisionTimes_s(touchesRegion);
end
end

%% Section 4: Local Functions

function regionIndices = selectCandidateRegions( ...
        startNodeIndex, endNodeIndex, startTime_s, endTime_s, movingCellLookup)
    % Read the node pair's candidate regions, then keep those this travel
    % can meet. The saved box entry/exit fractions show where the segment
    % can meet a region; keep the pair only when those fractions also
    % overlap the region's active times. Passing this quick check still
    % needs the full moving-boundary check.
    if movingCellLookup.IsPrecomputed
        pairIndex          = startNodeIndex + movingCellLookup.NodeCount * (endNodeIndex - 1);
        regionIndices      = movingCellLookup.CellIndices{pairIndex};
        boxEntryFractions  = movingCellLookup.QEnter{pairIndex};
        boxExitFractions   = movingCellLookup.QExit{pairIndex};
        activeStartTimes_s = movingCellLookup.ActiveStart_s{pairIndex};
        activeEndTimes_s   = movingCellLookup.ActiveEnd_s{pairIndex};
    else
        % Large searches calculate the same entry on demand to limit memory use.
        [regionIndices, boxEntryFractions, boxExitFractions, activeStartTimes_s, activeEndTimes_s] = ...
            obstacleAvoidance.search.computePairCellEntry( ...
            movingCellLookup.NodePosition_units(startNodeIndex, :), ...
            movingCellLookup.NodePosition_units(endNodeIndex, :), ...
            movingCellLookup.CellLower_units, movingCellLookup.CellUpper_units, ...
            movingCellLookup.ActiveIntervals_s);
    end
    if isempty(regionIndices)
        return
    end

    % Entries are ordered by start time. Ignore regions that start after
    % travel ends, then remove those that stop before travel begins.
    lastStartedRegionOffset = obstacleAvoidance.search.sortedUpperBound(activeStartTimes_s, endTime_s);
    candidateCacheIndices   = (1:lastStartedRegionOffset).';
    candidateCacheIndices   = candidateCacheIndices(activeEndTimes_s(candidateCacheIndices) >= startTime_s);
    regionIndices           = regionIndices(candidateCacheIndices);
    if isempty(regionIndices)
        return
    end
    activeIntervals_s   = movingCellLookup.Cells.ActiveTimeInterval_s(regionIndices, :);
    overlapStart_s      = max(startTime_s, activeIntervals_s(:, 1));
    overlapEnd_s        = min(endTime_s, activeIntervals_s(:, 2));
    traversalDuration_s = endTime_s - startTime_s;
    if traversalDuration_s > 0
        % Convert time-rounding uncertainty to a path fraction. Widen the
        % quick check so rounding cannot discard a possible collision.
        absoluteTimeScale_s = max([ ...
            repmat(max(abs([startTime_s, endTime_s])), numel(regionIndices), 1), ...
            abs(activeIntervals_s)], [], 2);
        timeRoundingFraction = 512 * eps(max(1, absoluteTimeScale_s)) / traversalDuration_s;
        timeRoundingFraction(~isfinite(timeRoundingFraction)) = Inf;
        overlapStartFractions = (overlapStart_s - startTime_s) / traversalDuration_s - timeRoundingFraction;
        overlapEndFractions   = (overlapEnd_s - startTime_s) / traversalDuration_s + timeRoundingFraction;
    else
        overlapStartFractions = zeros(size(overlapStart_s));
        overlapEndFractions   = zeros(size(overlapEnd_s));
    end
    pathCouldMeetRegion = overlapStart_s <= overlapEnd_s & ...
        boxExitFractions(candidateCacheIndices) >= overlapStartFractions & ...
        boxEntryFractions(candidateCacheIndices) <= overlapEndFractions;
    regionIndices = regionIndices(pathCouldMeetRegion);
end

function [touchesRegion, collisionTimes_s] = checkSegmentsAgainstRegion( ...
        segmentStart_units, segmentEnd_units, startTime_s, endTime_s, ...
        regionIndex, movingCellLookup)
    % Move the path endpoints and region vertices to the time they share.
    % For example, overlap [2 6] within travel [0 10] uses path fractions
    % [0.2 0.6]. The region uses its own active time interval.
    timedRegions        = movingCellLookup.Cells;
    activeInterval_s    = timedRegions.ActiveTimeInterval_s(regionIndex, :);
    overlapStart_s      = max(startTime_s, activeInterval_s(1));
    overlapEnd_s        = min(endTime_s, activeInterval_s(2));
    traversalDuration_s = endTime_s - startTime_s;
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

    regionDuration_s               = diff(activeInterval_s);
    regionStartFraction            = (overlapStart_s - activeInterval_s(1)) / regionDuration_s;
    regionEndFraction              = (overlapEnd_s - activeInterval_s(1)) / regionDuration_s;
    regionStart_units              = timedRegions.Regions_units{regionIndex};
    regionVertexDisplacement_units = timedRegions.EndRegions_units{regionIndex} - regionStart_units;
    overlapRegionStart_units       = regionStart_units + regionStartFraction .* regionVertexDisplacement_units;
    overlapRegionEnd_units         = regionStart_units + regionEndFraction .* regionVertexDisplacement_units;

    [touchesRegion, collisionFractions] = movingPointsTouchConvexRegion( ...
        pathStart_units, pathEnd_units, overlapRegionStart_units, overlapRegionEnd_units, ...
        movingCellLookup.CellIsCounterclockwise(regionIndex));
    % A clear point has a NaN fraction, so its time is NaN too. The formula
    % keeps the fraction's numeric class, as the single-segment path returns it.
    collisionTimes_s = overlapStart_s + collisionFractions .* (overlapEnd_s - overlapStart_s);
end

function [pointTouchesRegion, strictCollisionFraction] = movingPointsTouchConvexRegion( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
        isCounterclockwise)
    % Check one or more moving points against the same moving convex region.
    % Each row is a boundary edge; each column is a moving path point.
    % A point lies inside only when it is on the inside side of every edge.
    pointCount              = size(pointStart_units, 1);
    pointTouchesRegion      = false(pointCount, 1);
    strictCollisionFraction = NaN(pointCount, 1);
    if pointCount == 0
        return
    end

    % Use each boundary vertex as the origin for its point-to-edge test.
    % Both the edge vector and point offset change linearly with fraction u.
    % Their cross product is A x u^2 + B x u + C, with units of area.
    vertexCount                    = size(regionStart_units, 1);
    nextVertexIndices              = [2:vertexCount, 1];
    boundaryEdgeStart_units        = regionStart_units(nextVertexIndices, :) - regionStart_units;
    boundaryEdgeChange_units       = ...
        (regionEnd_units(nextVertexIndices, :) - regionEnd_units) - boundaryEdgeStart_units;
    regionVertexDisplacement_units = regionEnd_units - regionStart_units;
    pointDisplacement_units        = pointEnd_units - pointStart_units;
    relativeStartX_units           = pointStart_units(:, 1).' - regionStart_units(:, 1);
    relativeStartY_units           = pointStart_units(:, 2).' - regionStart_units(:, 2);
    relativeChangeX_units          = pointDisplacement_units(:, 1).' - regionVertexDisplacement_units(:, 1);
    relativeChangeY_units          = pointDisplacement_units(:, 2).' - regionVertexDisplacement_units(:, 2);
    quadraticCoefficient_units2    = boundaryEdgeChange_units(:, 1) .* relativeChangeY_units - ...
        boundaryEdgeChange_units(:, 2) .* relativeChangeX_units;
    linearCoefficient_units2 = (boundaryEdgeChange_units(:, 1) .* relativeStartY_units - ...
        boundaryEdgeChange_units(:, 2) .* relativeStartX_units) + ...
        (boundaryEdgeStart_units(:, 1) .* relativeChangeY_units - ...
        boundaryEdgeStart_units(:, 2) .* relativeChangeX_units);
    constantCoefficient_units2 = boundaryEdgeStart_units(:, 1) .* relativeStartY_units - ...
        boundaryEdgeStart_units(:, 2) .* relativeStartX_units;

    % Counterclockwise boundaries have their inside on the left (+ cross
    % product). Reverse clockwise signs once so a positive value always
    % means inside. Negating a number is exact, so the two root formulas
    % only swap places and the sorted crossing fractions stay the same.
    if ~isCounterclockwise
        quadraticCoefficient_units2 = -quadraticCoefficient_units2;
        linearCoefficient_units2    = -linearCoefficient_units2;
        constantCoefficient_units2  = -constantCoefficient_units2;
    end
    regionScale_units        = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units         = max(max(abs(pointStart_units), [], 2), max(abs(pointEnd_units), [], 2));
    coordinateScale_units    = max(1, max(regionScale_units, pointScale_units));
    edgeSideTolerance_units2 = obstacleAvoidance.search.createResidualBound(coordinateScale_units);

    % For one point, two quick answers avoid solving for crossings.
    if pointCount == 1
        % An endpoint clearly inside every edge already proves a collision.
        endEdgeSideValues_units2 = quadraticCoefficient_units2 + linearCoefficient_units2 + ...
            constantCoefficient_units2;
        if all(endEdgeSideValues_units2 > 16 * edgeSideTolerance_units2)
            pointTouchesRegion      = true;
            strictCollisionFraction = 1;
            return
        end

        % If one edge keeps the point outside for the whole interval, the
        % point cannot enter the region. Check the quadratic maximum at the
        % endpoints and its peak, allowing 2 x the tolerance before
        % declaring it clear.
        maximumEdgeSideValues_units2 = max(constantCoefficient_units2, endEdgeSideValues_units2);
        peakFraction                 = -linearCoefficient_units2 ./ (2 * quadraticCoefficient_units2);
        peakIsWithinInterval         = quadraticCoefficient_units2 < 0 & peakFraction > 0 & peakFraction < 1;
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
    end

    % Solve where each edge-side value becomes zero. These fractions mark
    % possible boundary crossings; NaN means no usable crossing in [0 1].
    coefficientScale = max(1, max(max(abs(quadraticCoefficient_units2), ...
        abs(linearCoefficient_units2)), abs(constantCoefficient_units2)));
    rootTolerance             = 256 * eps(coefficientScale);
    quadraticTermIsNegligible = abs(quadraticCoefficient_units2) <= rootTolerance;
    discriminant_units4       = linearCoefficient_units2 .* linearCoefficient_units2 - ...
        4 * quadraticCoefficient_units2 .* constantCoefficient_units2;
    hasQuadraticRoots         = ~quadraticTermIsNegligible & discriminant_units4 >= -rootTolerance;
    hasLinearRoot             = quadraticTermIsNegligible & abs(linearCoefficient_units2) > rootTolerance;
    discriminantRoot_units2   = sqrt(max(0, discriminant_units4));
    firstRootFraction         = (-linearCoefficient_units2 - discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    secondRootFraction = (-linearCoefficient_units2 + discriminantRoot_units2) ./ ...
        (2 * quadraticCoefficient_units2);
    linearRootFraction = -constantCoefficient_units2 ./ linearCoefficient_units2;

    % A negligible quadratic term uses the linear solution. Discard missing
    % roots and roots outside the interval before choosing check fractions.
    firstRootFraction(quadraticTermIsNegligible) = linearRootFraction(quadraticTermIsNegligible);
    firstRootFraction(~hasQuadraticRoots & ~hasLinearRoot) = NaN;
    secondRootFraction(~hasQuadraticRoots) = NaN;
    firstRootFraction(~(firstRootFraction >= 0 & firstRootFraction <= 1)) = NaN;
    secondRootFraction(~(secondRootFraction >= 0 & secondRootFraction <= 1)) = NaN;

    % Check the endpoints, crossings, and one midpoint between crossings.
    % Between consecutive roots, each edge-side value keeps the same sign.
    if pointCount == 1
        % One point: keep only the roots that exist.
        finiteRootFractions = [firstRootFraction(isfinite(firstRootFraction)); ...
            secondRootFraction(isfinite(secondRootFraction))];
        eventFractions        = sort([0, 1, finiteRootFractions.']);
        checkFractions        = [eventFractions, (eventFractions(1:end - 1) + eventFractions(2:end)) / 2];
        checkFractionsByPoint = checkFractions;
    else
        % Several points share one array, so a missing root stays as NaN in
        % its row; it sorts last and a NaN check never counts as inside.
        eventFractions = sort( ...
            [zeros(pointCount, 1), ones(pointCount, 1), firstRootFraction.', secondRootFraction.'], 2);
        checkFractions        = [eventFractions, (eventFractions(:, 1:end - 1) + eventFractions(:, 2:end)) / 2];
        checkFractionsByPoint = reshape(checkFractions, 1, pointCount, []);
    end
    edgeSideValues_units2 = quadraticCoefficient_units2 .* checkFractionsByPoint.^2 + ...
        linearCoefficient_units2 .* checkFractionsByPoint + constantCoefficient_units2;

    % Boundary contact blocks travel. Reusing a collision for later arrivals
    % requires a point farther inside: more than 16 x the side tolerance.
    edgeSideTolerance_units2 = reshape(edgeSideTolerance_units2, 1, pointCount);
    pointIsInside            = reshape(all(edgeSideValues_units2 >= -edgeSideTolerance_units2, 1), pointCount, []);
    pointIsStrictlyInside    = reshape(all(edgeSideValues_units2 > 16 * edgeSideTolerance_units2, 1), pointCount, []);
    pointTouchesRegion       = any(pointIsInside, 2);
    for pointIndex = reshape(find(pointTouchesRegion), 1, [])
        strictInsideFractions = checkFractions(pointIndex, pointIsStrictlyInside(pointIndex, :));
        if isempty(strictInsideFractions)
            continue
        end

        % Find a later point that remains clearly inside this region. The
        % arrival search can use it to check whether slower travel on this
        % same segment would still collide at that physical time. Refine
        % toward the next crossing while keeping an inside point.
        lastInsideFraction  = max(strictInsideFractions);
        pointEventFractions = eventFractions(pointIndex, :);
        laterEventFractions = pointEventFractions(isfinite(pointEventFractions) & ...
            pointEventFractions > lastInsideFraction);
        if ~isempty(laterEventFractions)
            upperSearchFraction = min(laterEventFractions);
            for refinementIndex = 1:12
                trialFraction              = (lastInsideFraction + upperSearchFraction) / 2;
                trialEdgeSideValues_units2 = quadraticCoefficient_units2(:, pointIndex) * trialFraction ^ 2 + ...
                    linearCoefficient_units2(:, pointIndex) * trialFraction + ...
                    constantCoefficient_units2(:, pointIndex);
                if all(trialEdgeSideValues_units2 > 16 * edgeSideTolerance_units2(pointIndex))
                    lastInsideFraction = trialFraction;
                else
                    upperSearchFraction = trialFraction;
                end
            end
        end
        if pointCount == 1
            % Assign directly so the fraction keeps its own numeric class.
            strictCollisionFraction = lastInsideFraction;
        else
            strictCollisionFraction(pointIndex) = lastInsideFraction;
        end
    end
end
