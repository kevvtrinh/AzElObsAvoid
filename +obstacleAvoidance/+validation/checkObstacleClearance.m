function [collisionFree, collisionResolved, seedCorridorCertified, planeCertificateCertified, minimumClearance_units, collisionCheckCount, unresolvedIntervalCount, certificateRejectionReason] = checkObstacleClearance(trajectory, obstacles, limits, options, canCheckCollision)
%% Section 0: Header & Readme
% SYNTAX
%   [collisionFree, collisionResolved, seedCorridorCertified, ...
%       planeCertificateCertified, minimumClearance_units, ...
%       collisionCheckCount, unresolvedIntervalCount, certificateRejectionReason] = ...
%       obstacleAvoidance.validation.checkObstacleClearance( ...
%       trajectory, obstacles, limits, options, canCheckCollision)
%
% PURPOSE
%   - Check a complete polynomial motion against protected obstacle histories.
%   - Prefer independently replayed certificates, then resolve ambiguity with
%     conservative adaptive curve-obstacle checks that fail closed.
%
% INPUTS
%   - trajectory (scalar motion struct)
%       Complete sampled and polynomial motion plus optional certificates.
%   - obstacles (prepared canonical obstacle struct array)
%       Original protected static or moving obstacle histories.
%   - limits (scalar struct)
%       Physical speed limits used by conservative interval bounds.
%   - options (resolved scalar struct)
%       Collision clearance and minimum-time-step controls.
%   - canCheckCollision (scalar logical)
%       True only when histories and polynomial bounds are structurally valid.
%
% OUTPUTS
%   - collisionFree, collisionResolved (scalar logicals)
%       Complete collision result and whether every interval was resolved.
%   - seedCorridorCertified, planeCertificateCertified (scalar logicals)
%       Which optional independent certificate, if any, proved separation.
%   - minimumClearance_units (scalar numeric)
%       Certified conservative clearance, exact checked clearance, or NaN.
%   - collisionCheckCount, unresolvedIntervalCount (nonnegative integers)
%       Exact polygon queries and intervals that reached the resolution floor.
%   - certificateRejectionReason (string scalar)
%       Failed timed-coverage check, or empty when no such check failed.
%
% UNITS
%   - Geometry and clearance are coordinate units; time is seconds.
%

%% Section 1: Check Complete Separation Evidence

% Check plane certificates, then seed corridors, then adaptive interval bounds.

[collisionFree, collisionResolved, seedCorridorCertified, ...
    planeCertificateCertified, minimumClearance_units, collisionCheckCount, ...
    unresolvedIntervalCount] = deal(false, false, false, false, NaN, 0, 0);
certificateRejectionReason = "";
if ~canCheckCollision
    return;
end
[planeCertificateCertified, planeClearance_units] = checkStaticObstacleClearance(trajectory, obstacles, options);
if ~planeCertificateCertified
    [planeCertificateCertified, planeClearance_units, certificateRejectionReason] = checkMovingObstacleClearance(trajectory, obstacles, options);
end
if planeCertificateCertified
    [collisionFree, collisionResolved, minimumClearance_units] = deal(true, true, planeClearance_units);
    return;
end
[seedCorridorCertified, seedClearance_units] = obstacleAvoidance.validation.certifySeedCorridor(trajectory, obstacles, options.CollisionClearanceTolerance_units);
if seedCorridorCertified
    [collisionFree, collisionResolved, minimumClearance_units] = deal(true, true, seedClearance_units);
    return;
end
[collisionFree, collisionResolved, minimumClearance_units, ...
    collisionCheckCount, unresolvedIntervalCount] = checkCurveObstacleSeparation(trajectory.Polynomial, obstacles, limits, options);
end

%% Section 2: Local Functions

function [certified, minimumClearance_units] = checkStaticObstacleClearance(trajectory, obstacles, options)
    % Verify separating planes against the static geometry and curve.
    certified            = false;
    minimumClearance_units = NaN;
    if ~isfield(trajectory, "PlaneCertificate") || isempty(obstacles)
        return;
    end
    certificate    = trajectory.PlaneCertificate;
    requiredFields = {'Kind', 'Planes'};
    if ~isstruct(certificate) || ~isscalar(certificate) || ~all(isfield(certificate, requiredFields))
        return;
    end
    kindIsText = (isstring(certificate.Kind) && isscalar(certificate.Kind)) || (ischar(certificate.Kind) && isrow(certificate.Kind));
    if ~kindIsText
        return;
    end
    certificateKind = string(certificate.Kind);
    if ismissing(certificateKind) || certificateKind ~= "staticDegreeOne"
        return;
    end
    [hasStaticHorizon, occupiedShape] = obstacleAvoidance.obstacles.queryStaticHorizon(obstacles, trajectory.time_s(1), trajectory.time_s(end));
    if ~hasStaticHorizon
        return;
    end
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        preparation = obstacles(obstacleIndex).InternalPreparation;
        if isempty(preparation.StaticShape.Vertices)
            return;
        end
    end
    [regionVertices, regionCoveragePassed] = reconstructCertificateRegions(certificate, occupiedShape);
    regionCount  = numel(regionVertices);
    segmentCount = trajectory.Polynomial.SegmentCount;
    if ~regionCoveragePassed || regionCount < 1 || ~isequal(size(certificate.Planes), [segmentCount regionCount])
        return;
    end
    activePairs = true(segmentCount, regionCount);
    [certified, minimumClearance_units] = verifyDegreeOneCertificate(trajectory, regionVertices, certificate.Planes, activePairs, options);
end

function [certified, minimumClearance_units, rejectionReason] = checkMovingObstacleClearance(trajectory, obstacles, options)
    % Rebuild timed cells and verify their separating planes.
    rejectionReason = "";
    [certified, minimumClearance_units] = deal(false, NaN);
    if ~isfield(trajectory, "PlaneCertificate") || isempty(obstacles)
        return;
    end
    certificate    = trajectory.PlaneCertificate;
    requiredFields = {'Kind', 'Planes', 'Regions_units', ...
        'RegionActiveBySegment', 'Coverage'};
    if ~isstruct(certificate) || ~isscalar(certificate) || ~all(isfield(certificate, requiredFields)) || string(certificate.Kind) ~= "timeCellDegreeOne"
        return;
    end
    coverage       = certificate.Coverage;
    coverageFields = {'Passed', 'RegionActiveTauInterval', ...
        'RegionSourceObstacleIndex', 'RegionSourceCellIndex', ...
        'BaseTimeCellCount'};
    if ~isstruct(coverage) || ~isscalar(coverage) || ~all(isfield(coverage, coverageFields)) || ~coverage.Passed
        return;
    end
    regions_units         = certificate.Regions_units;
    regionCount         = numel(regions_units);
    segmentCount        = trajectory.Polynomial.SegmentCount;
    activeTau           = double(coverage.RegionActiveTauInterval);
    sourceObstacleIndex = double(coverage.RegionSourceObstacleIndex(:));
    sourceCellIndex     = double(coverage.RegionSourceCellIndex(:));
    recordSizesMatch    = iscell(regions_units) && iscolumn(regions_units) && isequal(size(activeTau), [regionCount 2]) && numel(sourceObstacleIndex) == regionCount && numel(sourceCellIndex) == regionCount && isequal(size(certificate.Planes), [segmentCount regionCount]) && isequal(size(certificate.RegionActiveBySegment), [segmentCount regionCount]);
    if ~recordSizesMatch
        return;
    end
    segmentStartTau     = (0:segmentCount - 1).' / segmentCount;
    segmentFinishTau    = (1:segmentCount).' / segmentCount;
    expectedActivePairs = segmentStartTau < activeTau(:, 2).' & segmentFinishTau > activeTau(:, 1).';
    if ~isequal(logical(certificate.RegionActiveBySegment), expectedActivePairs)
        return;
    end
    startTime_s  = trajectory.time_s(1);
    finishTime_s = trajectory.time_s(end);
    [regionCoveragePassed, rejectionReason] = timedRegionCoverageMatches(regions_units, activeTau, sourceObstacleIndex, sourceCellIndex, obstacles, startTime_s, finishTime_s, coverage.BaseTimeCellCount);
    if ~regionCoveragePassed
        return;
    end
    [certified, minimumClearance_units] = verifyDegreeOneCertificate(trajectory, regions_units, certificate.Planes, expectedActivePairs, options);
end

function [passed, failure] = timedRegionCoverageMatches(regions_units, activeTau, sourceObstacleIndex, sourceCellIndex, obstacles, startTime_s, finishTime_s, baseTimeCellCount)
    % Prove stored convex cells cover each obstacle over their declared intervals.
    passed  = false;
    failure = "baseTimeCellCount";
    if ~isnumeric(baseTimeCellCount) || ~isscalar(baseTimeCellCount) || ~isfinite(baseTimeCellCount) || baseTimeCellCount < 1 || baseTimeCellCount ~= round(baseTimeCellCount)
        return;
    end
    baseEdges_s = linspace(startTime_s, finishTime_s, baseTimeCellCount + 1).';
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        obstacle = obstacles(obstacleIndex);
        [isStatic, staticShape] = obstacleAvoidance.obstacles.queryStaticHorizon(obstacle, startTime_s, finishTime_s);
        attributedRegion = sourceObstacleIndex == obstacleIndex;
        if isStatic
            staticRegion = attributedRegion & sourceCellIndex == 0;
            if ~any(staticRegion)
                failure = "missingStaticRegion";
                return;
            end
            unionShape = polyshape();
            % Process each geometric region while constructing or checking the region topology.
            for regionIndex = reshape(find(staticRegion), 1, [])
                unionShape = union(unionShape, polyshape(regions_units{regionIndex}(:, 1), regions_units{regionIndex}(:, 2)));
            end
            uncoveredArea_units2 = area(subtract(staticShape, unionShape));
            areaTolerance_units2 = 1e-10 * max(1, area(staticShape));
            if uncoveredArea_units2 > areaTolerance_units2
                failure = "staticCoverage";
                return;
            end
            continue;
        end
        obstacleTimes_s = double(obstacle.time_s(:));
        internalEdges_s = obstacleTimes_s(obstacleTimes_s > startTime_s & obstacleTimes_s < finishTime_s);
        cellEdges_s     = snapTimedCellEdges([baseEdges_s; internalEdges_s], obstacleTimes_s);
        % Process each geometric cell while constructing or checking the region topology.
        for cellIndex = 1:numel(cellEdges_s) - 1
            cellStart_s  = cellEdges_s(cellIndex);
            cellFinish_s = cellEdges_s(cellIndex + 1);
            queryTime_s  = [cellStart_s; ...
                0.5 * (cellStart_s + cellFinish_s); cellFinish_s];
            expectedVertices_units = zeros(0, 2);
            % Process each query in temporal order and accumulate its result.
            for queryIndex = 1:numel(queryTime_s)
                shape                = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle, queryTime_s(queryIndex));
                vertices_units         = double(shape.Vertices);
                expectedVertices_units = [expectedVertices_units; ...
                    vertices_units(all(isfinite(vertices_units), 2), :)]; %#ok<AGROW>
            end
            expectedVertices_units = unique(expectedVertices_units, "rows", "stable");
            matchingRegion       = attributedRegion & sourceCellIndex == cellIndex;
            if size(expectedVertices_units, 1) < 3
                if any(matchingRegion)
                    failure = "unexpectedInactiveRegion";
                    return;
                end
                continue;
            end
            if nnz(matchingRegion) ~= 1
                failure = "dynamicRegionCount";
                return;
            end
            regionIndex  = find(matchingRegion, 1);
            expectedTau  = ([cellStart_s cellFinish_s] - startTime_s) / (finishTime_s - startTime_s);
            tauScale     = max(1, max(abs([activeTau(regionIndex, :), expectedTau])));
            tauTolerance = 4096 * eps(tauScale);
            if max(abs(activeTau(regionIndex, :) - expectedTau)) > tauTolerance
                failure = "dynamicTau";
                return;
            end
            hullIndex             = convhull(expectedVertices_units(:, 1), expectedVertices_units(:, 2));
            expectedShape         = polyshape(expectedVertices_units(hullIndex(1:end - 1), 1), expectedVertices_units(hullIndex(1:end - 1), 2), "Simplify", false);
            certifiedVertices_units = unique(regions_units{regionIndex}, "rows", "stable");
            certifiedShape        = polyshape(certifiedVertices_units(:, 1), certifiedVertices_units(:, 2), "Simplify", false);
            uncoveredArea_units2    = area(subtract(expectedShape, certifiedShape));
            areaTolerance_units2    = 1e-10 * max(1, area(expectedShape));
            if uncoveredArea_units2 > areaTolerance_units2
                failure = "dynamicCoverage";
                return;
            end
        end
    end
    passed  = true;
    failure = "";
end

function cellEdges_s = snapTimedCellEdges(candidateEdges_s, obstacleTimes_s)
    % Merge certificate and obstacle event times that differ only by roundoff.
    timeScale_s     = max([1; abs(candidateEdges_s); abs(obstacleTimes_s)]);
    timeTolerance_s = 4096 * eps(timeScale_s);
    % Process each event in temporal order and accumulate its result.
    for eventIndex = 1:numel(obstacleTimes_s)
        nearEvent = abs(candidateEdges_s - obstacleTimes_s(eventIndex)) <= timeTolerance_s;
        candidateEdges_s(nearEvent) = obstacleTimes_s(eventIndex);
    end
    cellEdges_s = unique(candidateEdges_s, "sorted");
end

function [certified, minimumClearance_units] = verifyDegreeOneCertificate(trajectory, regionVertices, planes, activePairs, options)
    % Check polynomial controls against independently rebuilt regions.
    certified              = false;
    regionCount            = numel(regionVertices);
    segmentCount           = trajectory.Polynomial.SegmentCount;
    trajectoryControls_units = cell(segmentCount, 1);
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:segmentCount
        positionPower = reshape(trajectory.Polynomial.positionPower_units(segmentIndex, :, :), 2, []).';
        trajectoryControls_units{segmentIndex} = powerToBernstein(positionPower);
    end
    [~, ~, roundoffReserve_units] = bmtpEngine.createCoordinateTolerances(trajectoryControls_units, regionVertices);
    minimumClearance_units = Inf;
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:segmentCount
        trajectoryControl_units = trajectoryControls_units{segmentIndex};
        degree                = size(trajectoryControl_units, 1) - 1;
        fraction              = (0:degree + 1).' / (degree + 1);
        % Process each geometric region while constructing or checking the region topology.
        for regionIndex = 1:regionCount
            if ~activePairs(segmentIndex, regionIndex)
                continue;
            end
            plane = planes(segmentIndex, regionIndex);
            if ~validPlane(plane)
                minimumClearance_units = NaN;
                return;
            end
            planeNormal             = double(plane.Normal);
            planeOffset_units         = double(plane.Offset_units);
            obstacleSide_units        = regionVertices{regionIndex} * planeNormal.' + planeOffset_units;
            minimumObstacleSide_units = min(obstacleSide_units, [], "all");
            firstSide_units           = [trajectoryControl_units; zeros(1, 2)] * ...
                planeNormal(1, :).' + planeOffset_units(1);
            secondSide_units = [zeros(1, 2); trajectoryControl_units] * ...
                planeNormal(2, :).' + planeOffset_units(2);
            productControl_units        = (1 - fraction) .* firstSide_units + fraction .* secondSide_units;
            maximumTrajectorySide_units = max(productControl_units);
            signedGap_units             = minimumObstacleSide_units - maximumTrajectorySide_units;
            maximumNormalNorm         = max(vecnorm(planeNormal, 2, 2));
            certifiedClearance_units    = (signedGap_units - 2 * roundoffReserve_units) / max(maximumNormalNorm, realmin);
            pairPassed                = minimumObstacleSide_units >= roundoffReserve_units && maximumTrajectorySide_units <= -roundoffReserve_units && certifiedClearance_units >= options.CollisionClearanceTolerance_units && maximumNormalNorm <= 1 + 2 ^ 20 * eps;
            if ~pairPassed
                minimumClearance_units = NaN;
                return;
            end
            minimumClearance_units = min(minimumClearance_units, certifiedClearance_units);
        end
    end
    certified = isfinite(minimumClearance_units);
end

function [regions_units, passed] = reconstructCertificateRegions(certificate, occupiedShape)
    % Rebuild certified regions from the exact obstacle decomposition.
    exactRecords     = obstacleAvoidance.geometry.convexPolygonRegions(occupiedShape);
    exactRegionCount = numel(exactRecords);
    regions_units      = cell(exactRegionCount, 1);
    % Process each geometric region while constructing or checking the region topology.
    for regionIndex = 1:exactRegionCount
        vertices_units = exactRecords(regionIndex).Vertices;
        vertices_units = vertices_units(all(isfinite(vertices_units), 2), :);
        if size(vertices_units, 1) < 3
            regions_units = cell(0, 1);
            passed      = false;
            return;
        end
        regions_units{regionIndex} = vertices_units;
    end
    planeRegionCount = size(certificate.Planes, 2);
    if planeRegionCount == exactRegionCount
        passed = true;
        return;
    end
    passed      = false;
    regions_units = cell(0, 1);
    if ~isfield(certificate, "Coverage") || ~isstruct(certificate.Coverage) || ~isscalar(certificate.Coverage) || ~isfield(certificate.Coverage, "ConservativeGrouping")
        return;
    end
    grouping       = certificate.Coverage.ConservativeGrouping;
    requiredFields = {'Applied', 'ExactRegionCount', 'SolverRegionCount', ...
        'GroupMemberIndices'};
    countsAreValid = isstruct(grouping) && isscalar(grouping) && all(isfield(grouping, requiredFields)) && isnumeric(grouping.ExactRegionCount) && isreal(grouping.ExactRegionCount) && isscalar(grouping.ExactRegionCount) && isfinite(grouping.ExactRegionCount) && isnumeric(grouping.SolverRegionCount) && isreal(grouping.SolverRegionCount) && isscalar(grouping.SolverRegionCount) && isfinite(grouping.SolverRegionCount);
    if ~isstruct(grouping) || ~isscalar(grouping) || ~all(isfield(grouping, requiredFields)) || ~countsAreValid || ~isequal(grouping.Applied, true) || grouping.ExactRegionCount ~= exactRegionCount || grouping.SolverRegionCount ~= planeRegionCount || ~iscell(grouping.GroupMemberIndices) || numel(grouping.GroupMemberIndices) ~= planeRegionCount
        return;
    end
    memberCount = 0;
    % Process each geometric group while constructing or checking the region topology.
    for groupIndex = 1:planeRegionCount
        memberIndex     = grouping.GroupMemberIndices{groupIndex};
        membersAreValid = isnumeric(memberIndex) && isreal(memberIndex) && isvector(memberIndex) && ~isempty(memberIndex) && all(isfinite(memberIndex)) && all(memberIndex == fix(memberIndex)) && all(memberIndex >= 1 & memberIndex <= exactRegionCount);
        if ~membersAreValid
            return;
        end
        memberCount = memberCount + numel(memberIndex);
    end
    if memberCount ~= exactRegionCount
        return;
    end
    allMemberIndices = zeros(exactRegionCount, 1);
    regions_units      = cell(planeRegionCount, 1);
    nextMemberIndex  = 1;
    % Process each geometric group while constructing or checking the region topology.
    for groupIndex = 1:planeRegionCount
        memberIndex = reshape(grouping.GroupMemberIndices{groupIndex}, [], 1);
        targets     = nextMemberIndex:(nextMemberIndex + numel(memberIndex) - 1);
        allMemberIndices(targets) = memberIndex;
        nextMemberIndex   = targets(end) + 1;
        memberRegions_units = regions_unitsForMembers(exactRecords, memberIndex);
        vertices_units      = vertcat(memberRegions_units{:});
        hullIndex         = convhull(vertices_units(:, 1), vertices_units(:, 2));
        regions_units{groupIndex} = vertices_units(hullIndex(1:end - 1), :);
    end
    passed = isequal(sort(allMemberIndices), (1:exactRegionCount).');
    if ~passed
        regions_units = cell(0, 1);
    end
end

function regions_units = regions_unitsForMembers(exactRecords, memberIndex)
    % Extract finite vertices for the region's convex hull.
    regions_units = cell(numel(memberIndex), 1);
    % Process each local needed to complete regions units for members.
    for localIndex = 1:numel(memberIndex)
        vertices_units = exactRecords(memberIndex(localIndex)).Vertices;
        regions_units{localIndex} = vertices_units(all(isfinite(vertices_units), 2), :);
    end
end

function valid = validPlane(plane)
    % Reject malformed or nonfinite separating-plane records.
    valid = isstruct(plane) && isscalar(plane) && all(isfield(plane, {'Normal', 'Offset_units'})) && isnumeric(plane.Normal) && isnumeric(plane.Offset_units) && isreal(plane.Normal) && isreal(plane.Offset_units) && isequal(size(plane.Normal), [2 2]) && isequal(size(plane.Offset_units), [1 2]) && all(isfinite([plane.Normal(:); plane.Offset_units(:)]));
end

function bernstein = powerToBernstein(power)
    % Convert powers to Bernstein controls independently of the motion engine
    % so a shared conversion bug cannot make an invalid curve pass validation.
    degree    = size(power, 1) - 1;
    transform = zeros(degree + 1);
    % Process each bernstein needed to complete power to bernstein.
    for bernsteinIndex = 0:degree
        % Process each power needed to complete power to bernstein.
        for powerIndex = 0:bernsteinIndex
            transform(bernsteinIndex + 1, powerIndex + 1) = nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
        end
    end
    bernstein = transform * power;
end

function [collisionFree, resolved, minimumClearance_units, checkCount, unresolvedCount] = checkCurveObstacleSeparation(polynomial, obstacles, limits, options)
    % Check moving-obstacle clearance; reject unresolved minimum-step intervals.
    if isempty(obstacles)
        [collisionFree, resolved, minimumClearance_units, ...
            checkCount, unresolvedCount] = deal(true, true, Inf, 0, 0);
        return;
    end
    [collisionFree, resolved, minimumClearance_units, ...
        checkCount, unresolvedCount] = deal(true, true, Inf, 0, 0);
    pathSpeedBound_units_s = norm(limits.maxVelocity_units_s);
    historyBounds_units    = zeros(numel(obstacles), 4);
    obstacleEventTimes_s = zeros(0, 1);
    % Evaluate each obstacle against the current geometry or motion.
    for obstacleIndex = 1:numel(obstacles)
        historyBounds_units(obstacleIndex, :) = obstacles(obstacleIndex).InternalPreparation.HistoryBounds_units;
        obstacleEventTimes_s = [obstacleEventTimes_s; ...
            obstacles(obstacleIndex).time_s(:)]; %#ok<AGROW>
    end
    obstacleEventTimes_s = unique(obstacleEventTimes_s);
    % Process each segment while assembling the complete motion or interval result.
    for segmentIndex = 1:polynomial.SegmentCount
        segmentStart_s = polynomial.SegmentStartTime_s(segmentIndex);
        durationIndex  = min(segmentIndex, numel(polynomial.SegmentDuration_s));
        segmentEnd_s   = segmentStart_s + polynomial.SegmentDuration_s(durationIndex);
        % Merge event times differing by only a few floating-point steps.
        % Otherwise a near-zero interval can falsely imply infinite obstacle speed.
        eventScale_s     = max([1; abs(segmentStart_s); abs(segmentEnd_s); abs(obstacleEventTimes_s)]);
        eventTolerance_s = 1024 * eps(eventScale_s);
        segmentStart_s   = snapToEventTime(segmentStart_s, obstacleEventTimes_s, eventTolerance_s);
        segmentEnd_s     = snapToEventTime(segmentEnd_s, obstacleEventTimes_s, eventTolerance_s);
        splitTimes_s     = [segmentStart_s; segmentEnd_s];
        % Evaluate each obstacle against the current geometry or motion.
        for obstacleIndex = 1:numel(obstacles)
            obstacleTimes_s = obstacles(obstacleIndex).time_s(:);
            splitTimes_s    = [splitTimes_s; obstacleTimes_s(obstacleTimes_s > segmentStart_s & ...
                obstacleTimes_s < segmentEnd_s)]; %#ok<AGROW>
        end
        splitTimes_s = unique(splitTimes_s);
        [~, splitPoints_units] = bmtpEngine.evaluatePolynomial(polynomial, splitTimes_s, segmentIndex);
        % Process each split needed to verify curve obstacle separation.
        for splitIndex = 1:numel(splitTimes_s)
            % Evaluate each obstacle against the current geometry or motion.
            for obstacleIndex = 1:numel(obstacles)
                broadClearance_units = pointBoxClearance(splitPoints_units(splitIndex, :), historyBounds_units(obstacleIndex, :));
                if broadClearance_units > options.CollisionClearanceTolerance_units
                    minimumClearance_units = min(minimumClearance_units, broadClearance_units);
                    continue;
                end
                [shape, geometry]     = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacles(obstacleIndex), splitTimes_s(splitIndex));
                clearance_units        = obstacleAvoidance.geometry.pointPolygonClearance(shape, splitPoints_units(splitIndex, :), geometry);
                checkCount           = checkCount + 1;
                minimumClearance_units = min(minimumClearance_units, clearance_units);
                if clearance_units <= options.CollisionClearanceTolerance_units
                    collisionFree = false;
                    return;
                end
            end
        end
        stack_s = [splitTimes_s(1:end - 1), splitTimes_s(2:end)];
        % Continue iterating until the stopping condition for verify curve obstacle separation is satisfied.
        while ~isempty(stack_s)
            interval_s = stack_s(end, :);
            stack_s(end, :) = [];
            intervalMid_s = mean(interval_s);
            [~, point_units] = bmtpEngine.evaluatePolynomial(polynomial, intervalMid_s, segmentIndex);
            halfDuration_s        = diff(interval_s) / 2;
            pathDisplacement_units  = pathSpeedBound_units_s * halfDuration_s;
            intervalResolved      = true;
            intervalClearance_units = Inf;
            % Evaluate each obstacle against the current geometry or motion.
            for obstacleIndex = 1:numel(obstacles)
                broadClearance_units = pointBoxClearance(point_units, historyBounds_units(obstacleIndex, :)) - pathDisplacement_units;
                if broadClearance_units > options.CollisionClearanceTolerance_units
                    intervalClearance_units = min(intervalClearance_units, broadClearance_units);
                    continue;
                end
                [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacles(obstacleIndex), intervalMid_s);
                clearance_units = obstacleAvoidance.geometry.pointPolygonClearance(shape, point_units, geometry);
                checkCount    = checkCount + 1;
                if clearance_units <= options.CollisionClearanceTolerance_units
                    minimumClearance_units = min(minimumClearance_units, clearance_units);
                    collisionFree        = false;
                    return;
                end
                crossingBound_units     = (pathSpeedBound_units_s + geometry.VertexSpeedBound_units_s) * halfDuration_s;
                intervalClearance_units = min(intervalClearance_units, clearance_units - crossingBound_units);
                intervalResolved      = intervalResolved && isfinite(crossingBound_units) && clearance_units > crossingBound_units + options.CollisionClearanceTolerance_units;
            end
            if intervalResolved
                minimumClearance_units = min(minimumClearance_units, intervalClearance_units);
                continue;
            end
            if diff(interval_s) <= options.CollisionMinimumTimeStep_s
                collisionFree   = false;
                resolved        = false;
                unresolvedCount = unresolvedCount + 1;
                return;
            end
            stack_s = [stack_s; interval_s(1), intervalMid_s; ...
                intervalMid_s, interval_s(2)]; %#ok<AGROW>
        end
    end
end

function time_s = snapToEventTime(time_s, eventTimes_s, tolerance_s)
    % Merge roundoff-equivalent event times.
    if isempty(eventTimes_s)
        return;
    end
    [distance_s, eventIndex] = min(abs(eventTimes_s - time_s));
    if distance_s <= tolerance_s
        time_s = eventTimes_s(eventIndex);
    end
end

function clearance_units = pointBoxClearance(point_units, bounds_units)
    % Return Euclidean clearance from a point to an axis-aligned box.
    axisDistance_units = max([bounds_units([1 3]) - point_units; zeros(1, 2); point_units - bounds_units([2 4])], [], 1);
    clearance_units    = norm(axisDistance_units);
end
