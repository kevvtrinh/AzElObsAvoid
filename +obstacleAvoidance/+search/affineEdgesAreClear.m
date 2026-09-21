function [isClear, blockingCellIndices, collisionTimes_s] = affineEdgesAreClear( ...
    firstNodeIndices, secondNodeIndices, first_s, second_s, index)
%% Section 0: Header & Readme
% SYNTAX
%   [isClear, blockingCellIndices, collisionTimes_s] = ...
%       obstacleAvoidance.search.affineEdgesAreClear( ...
%       firstNodeIndices, secondNodeIndices, first_s, second_s, index)
%**************************************************************************
% PURPOSE
%   - Decide exactly whether straight segments traversed over one clock
%     interval stay clear of every affine moving convex cell, and for a
%     blocked segment name the first blocking cell and a collision time.
%**************************************************************************
% INPUTS
%   - firstNodeIndices, secondNodeIndices (N-by-1 numeric)
%       Node indices of the segment endpoints.
%   - first_s, second_s (numeric scalars)
%       Clock interval of the traversal; equal clocks are a stationary point.
%   - index (scalar struct)
%       The moving-cell index from createMovingCellIndex: the cells, their
%       orientation, the node positions, and the pair cache.
%**************************************************************************
% OUTPUTS
%   - isClear (N-by-1 logical)
%       True when the segment never touches any cell.
%   - blockingCellIndices (N-by-1 uint32)
%       First blocking cell of each blocked segment, zero when clear.
%   - collisionTimes_s (N-by-1 numeric)
%       Latest clock at which a blocked segment is strictly inside its
%       blocking cell; NaN when the segment is clear or its contact is
%       tolerance-only, in which case no layer proof is issued.
%**************************************************************************
% UNITS
%   - Positions are coordinate units and clocks are seconds.
%**************************************************************************

%% Section 1: Prove Every Segment Against The Affine Cells

first_units            = index.NodePosition_units(firstNodeIndices, :);
second_units           = index.NodePosition_units(secondNodeIndices, :);
cells                  = index.Cells;
pairCache              = index;
cellIsCounterclockwise = index.CellIsCounterclockwise;
% A path point and every vertex of a time cell are affine in time. Each
% convex half-space residual is therefore quadratic; its real roots
% partition the clock into intervals of constant inside/outside sign.
edgeCount           = size(first_units, 1);
isClear             = true(edgeCount, 1);
blockingCellIndices = zeros(edgeCount, 1, "uint32");
collisionTimes_s      = NaN(edgeCount, 1);
if edgeCount == 0 || isempty(cells.Regions_units)
    return
end
if edgeCount == 1
    [isClear, blockingCellIndices, collisionTimes_s] = affineSingleEdgeIsClear( ...
        first_units, second_units, firstNodeIndices, secondNodeIndices, ...
        first_s, second_s, cells, pairCache, cellIsCounterclockwise);
    return
end

% Intersect the cached spatial segment/box clock with each cell's active
% portion of this edge clock. Only surviving edge/cell pairs reach the
% unchanged exact affine residual calculation below.
cellBlocks = cell(edgeCount, 1);
edgeBlocks = cell(edgeCount, 1);
edgeDuration_s = second_s - first_s;
for edgeIndex = 1:edgeCount
    [pairCellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
        pairCellCandidates( ...
        firstNodeIndices(edgeIndex), secondNodeIndices(edgeIndex), pairCache);
    if isempty(pairCellIndices)
        continue
    end
    finalActiveOffset = obstacleAvoidance.search.sortedUpperBound(activeStart_s, second_s);
    if finalActiveOffset == 0
        continue
    end
    pairOffsets = (1:finalActiveOffset).';
    pairOffsets = pairOffsets(activeEnd_s(pairOffsets) >= first_s);
    if isempty(pairOffsets)
        continue
    end
    pairCellIndices = pairCellIndices(pairOffsets);
    active_s = cells.ActiveTimeInterval_s(pairCellIndices, :);
    overlapStart_s = max(first_s, active_s(:, 1));
    overlapEnd_s   = min(second_s, active_s(:, 2));
    meetsClock = overlapStart_s <= overlapEnd_s;
    if edgeDuration_s > 0
        overlapClockStart = (overlapStart_s - first_s) / edgeDuration_s;
        overlapClockEnd   = (overlapEnd_s - first_s) / edgeDuration_s;
        absoluteTimeScale_s = max([ ...
            repmat(max(abs([first_s, second_s])), numel(pairCellIndices), 1), ...
            abs(active_s)], [], 2);
        clockGuard = 512 * eps(max(1, absoluteTimeScale_s)) / edgeDuration_s;
        clockGuard(~isfinite(clockGuard)) = Inf;
        overlapClockStart = overlapClockStart - clockGuard;
        overlapClockEnd   = overlapClockEnd + clockGuard;
    else
        overlapClockStart = zeros(size(overlapStart_s));
        overlapClockEnd   = zeros(size(overlapEnd_s));
    end
    pairQEnter = qEnter(pairOffsets);
    pairQExit  = qExit(pairOffsets);
    meetsPathClock = pairQExit >= overlapClockStart & ...
        pairQEnter <= overlapClockEnd;
    retainedOffsets = find(meetsClock & meetsPathClock);
    cellBlocks{edgeIndex} = pairCellIndices(retainedOffsets);
    edgeBlocks{edgeIndex} = repmat(edgeIndex, numel(retainedOffsets), 1);
end
candidateCellIndices = vertcat(cellBlocks{:});
candidateEdgeIndices = vertcat(edgeBlocks{:});
if isempty(candidateCellIndices)
    return
end
[candidateCellIndices, order] = sort(candidateCellIndices);
candidateEdgeIndices = candidateEdgeIndices(order);
firstBlockOffsets = [1; 1 + find(diff(candidateCellIndices) ~= 0)];
finalBlockOffsets = [firstBlockOffsets(2:end) - 1; numel(candidateCellIndices)];

for blockIndex = 1:numel(firstBlockOffsets)
    if ~any(isClear)
        break
    end
    cellIndex = candidateCellIndices(firstBlockOffsets(blockIndex));
    blockOffsets = firstBlockOffsets(blockIndex):finalBlockOffsets(blockIndex);
    candidateIndices = candidateEdgeIndices(blockOffsets);
    candidateIndices = candidateIndices(isClear(candidateIndices));
    if isempty(candidateIndices)
        continue
    end
    overlapStart_s = max(first_s, cells.ActiveTimeInterval_s(cellIndex, 1));
    overlapEnd_s   = min(second_s, cells.ActiveTimeInterval_s(cellIndex, 2));

    if edgeDuration_s > 0
        startFraction = (overlapStart_s - first_s) / edgeDuration_s;
        endFraction   = (overlapEnd_s - first_s) / edgeDuration_s;
    else
        startFraction = 0;
        endFraction   = 0;
    end

    pathStart_units = first_units(candidateIndices, :) + startFraction .* ...
        (second_units(candidateIndices, :) - first_units(candidateIndices, :));
    pathEnd_units = first_units(candidateIndices, :) + endFraction .* ...
        (second_units(candidateIndices, :) - first_units(candidateIndices, :));

    cellDuration_s    = diff(cells.ActiveTimeInterval_s(cellIndex, :));
    cellStartFraction = (overlapStart_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
    cellEndFraction   = (overlapEnd_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
    regionStart_units        = cells.Regions_units{cellIndex};
    regionDelta_units        = cells.EndRegions_units{cellIndex} - regionStart_units;
    overlapRegionStart_units = regionStart_units + cellStartFraction .* regionDelta_units;
    overlapRegionEnd_units   = regionStart_units + cellEndFraction .* regionDelta_units;
    if isscalar(candidateIndices)
        [pointTouchesCell, collisionClock] = affinePointTouchesConvexScalar( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, ...
            overlapRegionEnd_units, cellIsCounterclockwise(cellIndex));
    else
        [pointTouchesCell, collisionClock] = affinePointsTouchConvex( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, ...
            overlapRegionEnd_units, cellIsCounterclockwise(cellIndex));
    end
    blockedIndices = candidateIndices(pointTouchesCell);
    isClear(blockedIndices) = false;
    blockingCellIndices(blockedIndices) = uint32(cellIndex);
    collisionTimes_s(blockedIndices) = overlapStart_s + ...
        collisionClock(pointTouchesCell) .* (overlapEnd_s - overlapStart_s);
end
end

%% Section 2: Local Functions

function [isClear, blockingCellIndex, collisionTime_s] = affineSingleEdgeIsClear( ...
        first_units, second_units, firstNodeIndex, secondNodeIndex, ...
        first_s, second_s, cells, pairCache, cellIsCounterclockwise)
    % Preserve the exact batch predicate while avoiding block assembly for one edge.
    isClear           = true;
    blockingCellIndex = uint32(0);
    collisionTime_s     = NaN;
    [pairCellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
        pairCellCandidates(firstNodeIndex, secondNodeIndex, pairCache);
    if isempty(pairCellIndices)
        return
    end

    finalActiveOffset = obstacleAvoidance.search.sortedUpperBound(activeStart_s, second_s);
    if finalActiveOffset == 0
        return
    end
    pairOffsets = (1:finalActiveOffset).';
    pairOffsets = pairOffsets(activeEnd_s(pairOffsets) >= first_s);
    if isempty(pairOffsets)
        return
    end
    pairCellIndices = pairCellIndices(pairOffsets);
    active_s        = cells.ActiveTimeInterval_s(pairCellIndices, :);
    overlapStart_s  = max(first_s, active_s(:, 1));
    overlapEnd_s    = min(second_s, active_s(:, 2));
    meetsClock      = overlapStart_s <= overlapEnd_s;
    edgeDuration_s  = second_s - first_s;
    if edgeDuration_s > 0
        overlapClockStart = (overlapStart_s - first_s) / edgeDuration_s;
        overlapClockEnd   = (overlapEnd_s - first_s) / edgeDuration_s;
        absoluteTimeScale_s = max([ ...
            repmat(max(abs([first_s, second_s])), numel(pairCellIndices), 1), ...
            abs(active_s)], [], 2);
        clockGuard = 512 * eps(max(1, absoluteTimeScale_s)) / edgeDuration_s;
        clockGuard(~isfinite(clockGuard)) = Inf;
        overlapClockStart = overlapClockStart - clockGuard;
        overlapClockEnd   = overlapClockEnd + clockGuard;
    else
        overlapClockStart = zeros(size(overlapStart_s));
        overlapClockEnd   = zeros(size(overlapEnd_s));
    end
    pairQEnter = qEnter(pairOffsets);
    pairQExit  = qExit(pairOffsets);
    meetsPathClock = pairQExit >= overlapClockStart & pairQEnter <= overlapClockEnd;
    candidateCellIndices = sort(pairCellIndices(meetsClock & meetsPathClock));

    for cellIndex = reshape(candidateCellIndices, 1, [])
        overlapStart_s = max(first_s, cells.ActiveTimeInterval_s(cellIndex, 1));
        overlapEnd_s   = min(second_s, cells.ActiveTimeInterval_s(cellIndex, 2));
        if edgeDuration_s > 0
            startFraction = (overlapStart_s - first_s) / edgeDuration_s;
            endFraction   = (overlapEnd_s - first_s) / edgeDuration_s;
        else
            startFraction = 0;
            endFraction   = 0;
        end
        displacement_units = second_units - first_units;
        pathStart_units     = first_units + startFraction .* displacement_units;
        pathEnd_units       = first_units + endFraction .* displacement_units;

        cellDuration_s    = diff(cells.ActiveTimeInterval_s(cellIndex, :));
        cellStartFraction = (overlapStart_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
        cellEndFraction   = (overlapEnd_s - cells.ActiveTimeInterval_s(cellIndex, 1)) / cellDuration_s;
        regionStart_units = cells.Regions_units{cellIndex};
        regionDelta_units = cells.EndRegions_units{cellIndex} - regionStart_units;
        overlapRegionStart_units = regionStart_units + cellStartFraction .* regionDelta_units;
        overlapRegionEnd_units   = regionStart_units + cellEndFraction .* regionDelta_units;
        [pointTouchesCell, collisionClock] = affinePointTouchesConvexScalar( ...
            pathStart_units, pathEnd_units, overlapRegionStart_units, overlapRegionEnd_units, ...
            cellIsCounterclockwise(cellIndex));
        if pointTouchesCell
            isClear           = false;
            blockingCellIndex = uint32(cellIndex);
            collisionTime_s     = overlapStart_s + collisionClock * (overlapEnd_s - overlapStart_s);
            return
        end
    end
end

function [cellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
        pairCellCandidates(firstNodeIndex, secondNodeIndex, cache)
    % Return a precomputed entry or calculate that exact entry on demand.
    if cache.IsPrecomputed
        pairIndex    = firstNodeIndex + cache.NodeCount * (secondNodeIndex - 1);
        cellIndices  = cache.CellIndices{pairIndex};
        qEnter       = cache.QEnter{pairIndex};
        qExit        = cache.QExit{pairIndex};
        activeStart_s = cache.ActiveStart_s{pairIndex};
        activeEnd_s   = cache.ActiveEnd_s{pairIndex};
        return
    end
    [cellIndices, qEnter, qExit, activeStart_s, activeEnd_s] = ...
        obstacleAvoidance.search.computePairCellEntry( ...
        cache.NodePosition_units(firstNodeIndex, :), ...
        cache.NodePosition_units(secondNodeIndex, :), ...
        cache.CellLower_units, cache.CellUpper_units, cache.ActiveIntervals_s);
end

function [pointTouchesCell, strictCollisionClock] = affinePointsTouchConvex( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
        isCounterclockwise)
    % Test path points against one affine moving convex cell in one batch.
    % Residual roots, clock ends, and interval midpoints form exact sign probes.
    % Rows are cell edges and columns are path points, preserving the original
    % point-by-point arithmetic and residual tolerance.
    pointCount       = size(pointStart_units, 1);
    pointTouchesCell = false(pointCount, 1);
    strictCollisionClock = NaN(pointCount, 1);
    if pointCount == 0
        return
    end
    if pointCount == 1
        [pointTouchesCell, strictCollisionClock] = affinePointTouchesConvexScalar( ...
            pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
            isCounterclockwise);
        return
    end

    vertexCount       = size(regionStart_units, 1);
    following         = [2:vertexCount, 1];
    edgeStart_units   = regionStart_units(following, :) - regionStart_units;
    edgeDelta_units   = (regionEnd_units(following, :) - regionEnd_units) - edgeStart_units;
    regionDelta_units = regionEnd_units - regionStart_units;
    pointDelta_units  = pointEnd_units - pointStart_units;

    relativeStartX_units = pointStart_units(:, 1).' - regionStart_units(:, 1);
    relativeStartY_units = pointStart_units(:, 2).' - regionStart_units(:, 2);
    relativeDeltaX_units = pointDelta_units(:, 1).' - regionDelta_units(:, 1);
    relativeDeltaY_units = pointDelta_units(:, 2).' - regionDelta_units(:, 2);
    quadratic_units2 = edgeDelta_units(:, 1) .* relativeDeltaY_units - ...
        edgeDelta_units(:, 2) .* relativeDeltaX_units;
    linear_units2 = (edgeDelta_units(:, 1) .* relativeStartY_units - ...
        edgeDelta_units(:, 2) .* relativeStartX_units) + ...
        (edgeStart_units(:, 1) .* relativeDeltaY_units - ...
        edgeStart_units(:, 2) .* relativeDeltaX_units);
    constant_units2 = edgeStart_units(:, 1) .* relativeStartY_units - ...
        edgeStart_units(:, 2) .* relativeStartX_units;
    regionScale_units     = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units      = max(max(abs(pointStart_units), [], 2), max(abs(pointEnd_units), [], 2));
    coordinateScale_units = max(1, max(regionScale_units, pointScale_units));
    residualTolerance_units2 = obstacleAvoidance.search.createResidualBound(coordinateScale_units);
    % Real roots of each residual inside the unit clock; NaN marks none.
    rootScale         = max(1, max(max(abs(quadratic_units2), abs(linear_units2)), abs(constant_units2)));
    rootTolerance     = 256 * eps(rootScale);
    isLinear          = abs(quadratic_units2) <= rootTolerance;
    discriminant      = linear_units2 .* linear_units2 - 4 * quadratic_units2 .* constant_units2;
    hasQuadraticRoots = ~isLinear & discriminant >= -rootTolerance;
    rootRadius        = sqrt(max(0, discriminant));
    firstRoot         = (-linear_units2 - rootRadius) ./ (2 * quadratic_units2);
    secondRoot        = (-linear_units2 + rootRadius) ./ (2 * quadratic_units2);
    linearRoot        = -constant_units2 ./ linear_units2;
    hasLinearRoot     = isLinear & abs(linear_units2) > rootTolerance;

    firstRoot(isLinear)                              = linearRoot(isLinear);
    firstRoot(~hasQuadraticRoots & ~hasLinearRoot)  = NaN;
    secondRoot(~hasQuadraticRoots)                  = NaN;
    firstRoot(~(firstRoot >= 0 & firstRoot <= 1))   = NaN;
    secondRoot(~(secondRoot >= 0 & secondRoot <= 1)) = NaN;
    cuts = sort([zeros(pointCount, 1), ones(pointCount, 1), firstRoot.', secondRoot.'], 2);
    probes     = [cuts, (cuts(:, 1:end - 1) + cuts(:, 2:end)) / 2];
    probeClock = reshape(probes, 1, pointCount, []);
    residual_units2 = quadratic_units2 .* probeClock.^2 + ...
        linear_units2 .* probeClock + constant_units2;

    residualTolerance_units2 = reshape(residualTolerance_units2, 1, pointCount);
    if isCounterclockwise
        inside = residual_units2 >= -residualTolerance_units2;
        strictInside = residual_units2 > 16 * residualTolerance_units2;
    else
        inside = residual_units2 <= residualTolerance_units2;
        strictInside = residual_units2 < -16 * residualTolerance_units2;
    end
    pointTouchesCell = reshape(any(all(inside, 1), 3), pointCount, 1);
    strictByPointAndProbe = reshape(all(strictInside, 1), pointCount, []);
    for pointIndex = reshape(find(pointTouchesCell), 1, [])
        strictProbeClocks = probes(pointIndex, strictByPointAndProbe(pointIndex, :));
        if ~isempty(strictProbeClocks)
            % For a later arrival the path point at a fixed physical time
            % moves toward its source. The latest strict collision time therefore
            % gives the longest conservative retry proof along the same
            % directed segment; the collision predicate itself is unchanged.
            lowerClock = max(strictProbeClocks);
            finiteCuts = cuts(pointIndex, isfinite(cuts(pointIndex, :)));
            nextCuts    = finiteCuts(finiteCuts > lowerClock);
            if ~isempty(nextCuts)
                upperClock = min(nextCuts);
                for refinementIndex = 1:12
                    trialClock = (lowerClock + upperClock) / 2;
                    trialResidual_units2 = quadratic_units2(:, pointIndex) * trialClock ^ 2 + ...
                        linear_units2(:, pointIndex) * trialClock + constant_units2(:, pointIndex);
                    if isCounterclockwise
                        trialIsStrict = all(trialResidual_units2 > ...
                            16 * residualTolerance_units2(pointIndex));
                    else
                        trialIsStrict = all(trialResidual_units2 < ...
                            -16 * residualTolerance_units2(pointIndex));
                    end
                    if trialIsStrict
                        lowerClock = trialClock;
                    else
                        upperClock = trialClock;
                    end
                end
            end
            strictCollisionClock(pointIndex) = lowerClock;
        end
    end
end

function [pointTouchesCell, strictCollisionClock] = affinePointTouchesConvexScalar( ...
        pointStart_units, pointEnd_units, regionStart_units, regionEnd_units, ...
        isCounterclockwise)
    % Keep the overwhelmingly common one-edge predicate in two dimensions;
    % the general batch path uses page arrays for several simultaneous edges.
    vertexCount       = size(regionStart_units, 1);
    following         = [2:vertexCount, 1];
    edgeStart_units   = regionStart_units(following, :) - regionStart_units;
    edgeDelta_units   = (regionEnd_units(following, :) - regionEnd_units) - edgeStart_units;
    regionDelta_units = regionEnd_units - regionStart_units;
    pointDelta_units  = pointEnd_units - pointStart_units;
    relativeStartX_units = pointStart_units(1) - regionStart_units(:, 1);
    relativeStartY_units = pointStart_units(2) - regionStart_units(:, 2);
    relativeDeltaX_units = pointDelta_units(1) - regionDelta_units(:, 1);
    relativeDeltaY_units = pointDelta_units(2) - regionDelta_units(:, 2);
    quadratic_units2 = edgeDelta_units(:, 1) .* relativeDeltaY_units - ...
        edgeDelta_units(:, 2) .* relativeDeltaX_units;
    linear_units2 = (edgeDelta_units(:, 1) .* relativeStartY_units - ...
        edgeDelta_units(:, 2) .* relativeStartX_units) + ...
        (edgeStart_units(:, 1) .* relativeDeltaY_units - ...
        edgeStart_units(:, 2) .* relativeDeltaX_units);
    constant_units2 = edgeStart_units(:, 1) .* relativeStartY_units - ...
        edgeStart_units(:, 2) .* relativeStartX_units;
    regionScale_units = max(abs([regionStart_units; regionEnd_units]), [], 'all');
    pointScale_units  = max(max(abs(pointStart_units), [], 2), ...
        max(abs(pointEnd_units), [], 2));
    coordinateScale_units = max(1, max(regionScale_units, pointScale_units));
    residualTolerance_units2 = obstacleAvoidance.search.createResidualBound(coordinateScale_units);
    if ~isCounterclockwise
        quadratic_units2 = -quadratic_units2;
        linear_units2    = -linear_units2;
        constant_units2  = -constant_units2;
    end
    pointTouchesCell   = false;
    strictCollisionClock = NaN;
    endResidual_units2 = quadratic_units2 + linear_units2 + constant_units2;
    if all(endResidual_units2 > 16 * residualTolerance_units2)
        pointTouchesCell   = true;
        strictCollisionClock = 1;
        return
    end

    % A half-space that excludes the point for the whole unit clock proves
    % clearance without enumerating residual roots. The extra full residual
    % reserve keeps this broad proof strictly inside the exact predicate.
    maximumResidual_units2 = max(constant_units2, endResidual_units2);
    isConcave               = quadratic_units2 < 0;
    stationaryClock         = -linear_units2 ./ (2 * quadratic_units2);
    stationaryIsInside      = isConcave & stationaryClock > 0 & stationaryClock < 1;
    if any(stationaryIsInside)
        stationaryResidual_units2 = quadratic_units2 .* stationaryClock.^2 + ...
            linear_units2 .* stationaryClock + constant_units2;
        maximumResidual_units2(stationaryIsInside) = max( ...
            maximumResidual_units2(stationaryIsInside), ...
            stationaryResidual_units2(stationaryIsInside));
    end
    if any(maximumResidual_units2 < -2 * residualTolerance_units2)
        return
    end

    rootScale         = max(1, max(max(abs(quadratic_units2), ...
        abs(linear_units2)), abs(constant_units2)));
    rootTolerance     = 256 * eps(rootScale);
    isLinear          = abs(quadratic_units2) <= rootTolerance;
    discriminant      = linear_units2 .* linear_units2 - ...
        4 * quadratic_units2 .* constant_units2;
    hasQuadraticRoots = ~isLinear & discriminant >= -rootTolerance;
    rootRadius        = sqrt(max(0, discriminant));
    firstRoot         = (-linear_units2 - rootRadius) ./ (2 * quadratic_units2);
    secondRoot        = (-linear_units2 + rootRadius) ./ (2 * quadratic_units2);
    linearRoot        = -constant_units2 ./ linear_units2;
    hasLinearRoot     = isLinear & abs(linear_units2) > rootTolerance;
    firstRoot(isLinear)                             = linearRoot(isLinear);
    firstRoot(~hasQuadraticRoots & ~hasLinearRoot) = NaN;
    secondRoot(~hasQuadraticRoots)                 = NaN;
    firstRoot(~(firstRoot >= 0 & firstRoot <= 1))  = NaN;
    secondRoot(~(secondRoot >= 0 & secondRoot <= 1)) = NaN;

    finiteRoots = [firstRoot(isfinite(firstRoot)); secondRoot(isfinite(secondRoot))];
    cuts   = sort([0, 1, finiteRoots.']);
    probes = [cuts, (cuts(1:end - 1) + cuts(2:end)) / 2];
    residual_units2 = quadratic_units2 .* probes.^2 + ...
        linear_units2 .* probes + constant_units2;
    inside      = residual_units2 >= -residualTolerance_units2;
    strictProbe = all(residual_units2 > 16 * residualTolerance_units2, 1);
    pointTouchesCell = any(all(inside, 1));
    if ~pointTouchesCell || ~any(strictProbe)
        return
    end

    lowerClock = max(probes(strictProbe));
    finiteCuts = cuts(isfinite(cuts));
    nextCuts   = finiteCuts(finiteCuts > lowerClock);
    if ~isempty(nextCuts)
        upperClock = min(nextCuts);
        for refinementIndex = 1:12
            trialClock = (lowerClock + upperClock) / 2;
            trialResidual_units2 = quadratic_units2 * trialClock ^ 2 + ...
                linear_units2 * trialClock + constant_units2;
            trialIsStrict = all(trialResidual_units2 > 16 * residualTolerance_units2);
            if trialIsStrict
                lowerClock = trialClock;
            else
                upperClock = trialClock;
            end
        end
    end
    strictCollisionClock = lowerClock;
end
