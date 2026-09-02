function [collisionFree, collisionResolved, seedCorridorCertified, ...
        planeCertificateCertified, minimumClearance_deg, ...
        collisionCheckCount, unresolvedIntervalCount] = ...
        checkObstacleClearance( ...
        trajectory, obstacles, limits, options, canCheckCollision)
%% Section 0: Header & Readme
% SYNTAX
%   [collisionFree, collisionResolved, seedCorridorCertified, ...
%       planeCertificateCertified, minimumClearance_deg, ...
%       collisionCheckCount, unresolvedIntervalCount] = ...
%       obstacleAvoidance.validation.checkObstacleClearance( ...
%       trajectory, obstacles, limits, options, canCheckCollision)
%**************************************************************************
% PURPOSE
%   - Check a complete polynomial motion against protected obstacle histories.
%   - Prefer independently replayed certificates, then resolve ambiguity with
%     conservative adaptive curve-obstacle checks that fail closed.
%**************************************************************************
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
%**************************************************************************
% OUTPUTS
%   - collisionFree, collisionResolved (scalar logicals)
%       Complete collision result and whether every interval was resolved.
%   - seedCorridorCertified, planeCertificateCertified (scalar logicals)
%       Which optional independent certificate, if any, proved separation.
%   - minimumClearance_deg (scalar numeric)
%       Certified conservative clearance, exact checked clearance, or NaN.
%   - collisionCheckCount, unresolvedIntervalCount (nonnegative integers)
%       Exact polygon queries and intervals that reached the resolution floor.
%**************************************************************************
% UNITS
%   - Geometry and clearance are degrees; time is seconds.
%**************************************************************************

%% Section 1: Check Complete Separation Evidence

% Static and moving plane certificates are independently reconstructed before
% use. If neither passes, seed-side guidance is checked; remaining ambiguity
% falls through to conservative adaptive checks over every polynomial segment.

[collisionFree, collisionResolved, seedCorridorCertified, ...
    planeCertificateCertified, minimumClearance_deg, collisionCheckCount, ...
    unresolvedIntervalCount] = deal(false, false, false, false, NaN, 0, 0);
if ~canCheckCollision
    return;
end
[planeCertificateCertified, planeClearance_deg] = ...
    checkStaticObstacleClearance(trajectory, obstacles, options);
if ~planeCertificateCertified
    [planeCertificateCertified, planeClearance_deg] = ...
        checkMovingObstacleClearance(trajectory, obstacles, options);
end
if planeCertificateCertified
    [collisionFree, collisionResolved, minimumClearance_deg] = ...
        deal(true, true, planeClearance_deg);
    return;
end
[seedCorridorCertified, seedClearance_deg] = ...
    obstacleAvoidance.validation.checkSeedCorridor( ...
    trajectory, obstacles, options.CollisionClearanceTolerance_deg);
if seedCorridorCertified
    [collisionFree, collisionResolved, minimumClearance_deg] = ...
        deal(true, true, seedClearance_deg);
    return;
end
[collisionFree, collisionResolved, minimumClearance_deg, ...
    collisionCheckCount, unresolvedIntervalCount] = ...
    checkCurveObstacleSeparation( ...
    trajectory.Polynomial, obstacles, limits, options);
end

%% Section 2: Local Functions

function [certified, minimumClearance_deg] = ...
        checkStaticObstacleClearance(trajectory, obstacles, options)
% Independently verify complete static obstacle/curve plane separation.
certified = false;
minimumClearance_deg = NaN;
if ~isfield(trajectory, "PlaneCertificate") || isempty(obstacles)
    return;
end
certificate = trajectory.PlaneCertificate;
requiredFields = {'Kind', 'Planes'};
if ~isstruct(certificate) || ~isscalar(certificate) || ...
        ~all(isfield(certificate, requiredFields))
    return;
end
kindIsText = (isstring(certificate.Kind) && ...
    isscalar(certificate.Kind)) || ...
    (ischar(certificate.Kind) && isrow(certificate.Kind));
if ~kindIsText
    return;
end
certificateKind = string(certificate.Kind);
if ismissing(certificateKind) || ...
        certificateKind ~= "staticDegreeOne"
    return;
end
[hasStaticHorizon, occupiedShape] = ...
    obstacleAvoidance.obstacles.queryStaticHorizon( ...
    obstacles, trajectory.time_s(1), trajectory.time_s(end));
if ~hasStaticHorizon
    return;
end
for obstacleIndex = 1:numel(obstacles)
    preparation = obstacles(obstacleIndex).InternalPreparation;
    if isempty(preparation.StaticShape.Vertices)
        return;
    end
end
[regionVertices, regionCoveragePassed] = ...
    reconstructCertificateRegions(certificate, occupiedShape);
regionCount = numel(regionVertices);
segmentCount = trajectory.Polynomial.SegmentCount;
if ~regionCoveragePassed || regionCount < 1 || ...
        ~isequal(size(certificate.Planes), [segmentCount regionCount])
    return;
end
activePairs = true(segmentCount, regionCount);
[certified, minimumClearance_deg] = verifyDegreeOneCertificate( ...
    trajectory, regionVertices, certificate.Planes, activePairs, options);
end

function [certified, minimumClearance_deg] = ...
        checkMovingObstacleClearance(trajectory, obstacles, options)
% Reconstruct timed cells and independently verify every applicable plane.
[certified, minimumClearance_deg] = deal(false, NaN);
if ~isfield(trajectory, "PlaneCertificate") || isempty(obstacles)
    return;
end
certificate = trajectory.PlaneCertificate;
requiredFields = {'Kind', 'Planes', 'Regions_deg', ...
    'RegionActiveBySegment', 'Coverage'};
if ~isstruct(certificate) || ~isscalar(certificate) || ...
        ~all(isfield(certificate, requiredFields)) || ...
        string(certificate.Kind) ~= "timeCellDegreeOne"
    return;
end
coverage = certificate.Coverage;
coverageFields = {'Passed', 'RegionActiveTauInterval', ...
    'RegionSourceObstacleIndex', 'RegionSourceCellIndex', ...
    'BaseTimeCellCount'};
if ~isstruct(coverage) || ~isscalar(coverage) || ...
        ~all(isfield(coverage, coverageFields)) || ~coverage.Passed
    return;
end
regions_deg = certificate.Regions_deg;
regionCount = numel(regions_deg);
segmentCount = trajectory.Polynomial.SegmentCount;
activeTau = double(coverage.RegionActiveTauInterval);
sourceObstacleIndex = double(coverage.RegionSourceObstacleIndex(:));
sourceCellIndex = double(coverage.RegionSourceCellIndex(:));
recordSizesMatch = iscell(regions_deg) && iscolumn(regions_deg) && ...
    isequal(size(activeTau), [regionCount 2]) && ...
    numel(sourceObstacleIndex) == regionCount && ...
    numel(sourceCellIndex) == regionCount && ...
    isequal(size(certificate.Planes), [segmentCount regionCount]) && ...
    isequal(size(certificate.RegionActiveBySegment), ...
    [segmentCount regionCount]);
if ~recordSizesMatch
    return;
end
segmentStartTau = (0:segmentCount - 1).' / segmentCount;
segmentFinishTau = (1:segmentCount).' / segmentCount;
expectedActivePairs = segmentStartTau < activeTau(:, 2).' & ...
    segmentFinishTau > activeTau(:, 1).';
if ~isequal(logical(certificate.RegionActiveBySegment), expectedActivePairs)
    return;
end
startTime_s = trajectory.time_s(1);
finishTime_s = trajectory.time_s(end);
[regionCoveragePassed, ~] = timedRegionCoverageMatches( ...
        regions_deg, activeTau, sourceObstacleIndex, sourceCellIndex, ...
        obstacles, startTime_s, finishTime_s, coverage.BaseTimeCellCount);
if ~regionCoveragePassed
    return;
end
[certified, minimumClearance_deg] = verifyDegreeOneCertificate( ...
    trajectory, regions_deg, certificate.Planes, expectedActivePairs, options);
end

function [passed, failure] = timedRegionCoverageMatches( ...
        regions_deg, activeTau, sourceObstacleIndex, sourceCellIndex, ...
        obstacles, startTime_s, finishTime_s, baseTimeCellCount)
% Prove stored convex cells cover each obstacle over their declared intervals.
passed = false;
failure = "baseTimeCellCount";
if ~isnumeric(baseTimeCellCount) || ~isscalar(baseTimeCellCount) || ...
        ~isfinite(baseTimeCellCount) || baseTimeCellCount < 1 || ...
        baseTimeCellCount ~= round(baseTimeCellCount)
    return;
end
baseEdges_s = linspace( ...
    startTime_s, finishTime_s, baseTimeCellCount + 1).';
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    [isStatic, staticShape] = ...
        obstacleAvoidance.obstacles.queryStaticHorizon( ...
        obstacle, startTime_s, finishTime_s);
    attributedRegion = sourceObstacleIndex == obstacleIndex;
    if isStatic
        staticRegion = attributedRegion & sourceCellIndex == 0;
        if ~any(staticRegion)
            failure = "missingStaticRegion";
            return;
        end
        unionShape = polyshape();
        for regionIndex = reshape(find(staticRegion), 1, [])
            unionShape = union(unionShape, ...
                polyshape(regions_deg{regionIndex}(:, 1), ...
                regions_deg{regionIndex}(:, 2)));
        end
        uncoveredArea_deg2 = area(subtract(staticShape, unionShape));
        areaTolerance_deg2 = 1e-10 * max(1, area(staticShape));
        if uncoveredArea_deg2 > areaTolerance_deg2
            failure = "staticCoverage";
            return;
        end
        continue;
    end
    obstacleTimes_s = double(obstacle.time_s(:));
    internalEdges_s = obstacleTimes_s(obstacleTimes_s > startTime_s & ...
        obstacleTimes_s < finishTime_s);
    cellEdges_s = snapTimedCellEdges( ...
        [baseEdges_s; internalEdges_s], obstacleTimes_s);
    for cellIndex = 1:numel(cellEdges_s) - 1
        cellStart_s = cellEdges_s(cellIndex);
        cellFinish_s = cellEdges_s(cellIndex + 1);
        queryTime_s = [cellStart_s; ...
            0.5 * (cellStart_s + cellFinish_s); cellFinish_s];
        expectedVertices_deg = zeros(0, 2);
        for queryIndex = 1:numel(queryTime_s)
            shape = obstacleAvoidance.obstacles.shapeAtTime( ...
                obstacle, queryTime_s(queryIndex));
            vertices_deg = double(shape.Vertices);
            expectedVertices_deg = [expectedVertices_deg; ...
                vertices_deg(all(isfinite(vertices_deg), 2), :)]; %#ok<AGROW>
        end
        expectedVertices_deg = unique(expectedVertices_deg, "rows", "stable");
        matchingRegion = attributedRegion & sourceCellIndex == cellIndex;
        if size(expectedVertices_deg, 1) < 3
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
        regionIndex = find(matchingRegion, 1);
        expectedTau = ([cellStart_s cellFinish_s] - startTime_s) / ...
            (finishTime_s - startTime_s);
        tauScale = max(1, max(abs( ...
            [activeTau(regionIndex, :), expectedTau])));
        tauTolerance = 4096 * eps(tauScale);
        if max(abs(activeTau(regionIndex, :) - expectedTau)) > tauTolerance
            failure = "dynamicTau";
            return;
        end
        hullIndex = convhull( ...
            expectedVertices_deg(:, 1), expectedVertices_deg(:, 2));
        expectedShape = polyshape( ...
            expectedVertices_deg(hullIndex(1:end - 1), 1), ...
            expectedVertices_deg(hullIndex(1:end - 1), 2), ...
            "Simplify", false);
        certifiedVertices_deg = unique( ...
            regions_deg{regionIndex}, "rows", "stable");
        certifiedShape = polyshape( ...
            certifiedVertices_deg(:, 1), certifiedVertices_deg(:, 2), ...
            "Simplify", false);
        uncoveredArea_deg2 = area(subtract(expectedShape, certifiedShape));
        areaTolerance_deg2 = 1e-10 * max(1, area(expectedShape));
        if uncoveredArea_deg2 > areaTolerance_deg2
            failure = "dynamicCoverage";
            return;
        end
    end
end
passed = true;
failure = "";
end

function cellEdges_s = snapTimedCellEdges(candidateEdges_s, obstacleTimes_s)
% Coalesce roundoff-equivalent certificate-grid and obstacle-event times.
timeScale_s = max([1; abs(candidateEdges_s); abs(obstacleTimes_s)]);
timeTolerance_s = 4096 * eps(timeScale_s);
for eventIndex = 1:numel(obstacleTimes_s)
    nearEvent = abs(candidateEdges_s - obstacleTimes_s(eventIndex)) <= ...
        timeTolerance_s;
    candidateEdges_s(nearEvent) = obstacleTimes_s(eventIndex);
end
cellEdges_s = unique(candidateEdges_s, "sorted");
end

function [certified, minimumClearance_deg] = verifyDegreeOneCertificate( ...
        trajectory, regionVertices, planes, activePairs, options)
% Verify polynomial controls against caller-reconstructed convex regions.
certified = false;
regionCount = numel(regionVertices);
segmentCount = trajectory.Polynomial.SegmentCount;
trajectoryControls_deg = cell(segmentCount, 1);
for segmentIndex = 1:segmentCount
    positionPower = reshape(trajectory.Polynomial.positionPower_deg( ...
        segmentIndex, :, :), 2, []).';
    trajectoryControls_deg{segmentIndex} = powerToBernstein(positionPower);
end
[~, ~, roundoffReserve_deg] = bmtpEngine.createCoordinateTolerances( ...
    trajectoryControls_deg, regionVertices);
minimumClearance_deg = Inf;
for segmentIndex = 1:segmentCount
    trajectoryControl_deg = trajectoryControls_deg{segmentIndex};
    degree = size(trajectoryControl_deg, 1) - 1;
    fraction = (0:degree + 1).' / (degree + 1);
    for regionIndex = 1:regionCount
        if ~activePairs(segmentIndex, regionIndex)
            continue;
        end
        plane = planes(segmentIndex, regionIndex);
        if ~validPlane(plane)
            minimumClearance_deg = NaN;
            return;
        end
        planeNormal = double(plane.Normal);
        planeOffset_deg = double(plane.Offset_deg);
        obstacleSide_deg = regionVertices{regionIndex} * ...
            planeNormal.' + planeOffset_deg;
        minimumObstacleSide_deg = min(obstacleSide_deg, [], "all");
        firstSide_deg = [trajectoryControl_deg; zeros(1, 2)] * ...
            planeNormal(1, :).' + planeOffset_deg(1);
        secondSide_deg = [zeros(1, 2); trajectoryControl_deg] * ...
            planeNormal(2, :).' + planeOffset_deg(2);
        productControl_deg = (1 - fraction) .* firstSide_deg + ...
            fraction .* secondSide_deg;
        maximumTrajectorySide_deg = max(productControl_deg);
        signedGap_deg = minimumObstacleSide_deg - maximumTrajectorySide_deg;
        maximumNormalNorm = max(vecnorm(planeNormal, 2, 2));
        certifiedClearance_deg = ...
            (signedGap_deg - 2 * roundoffReserve_deg) / ...
            max(maximumNormalNorm, realmin);
        pairPassed = minimumObstacleSide_deg >= roundoffReserve_deg && ...
            maximumTrajectorySide_deg <= -roundoffReserve_deg && ...
            certifiedClearance_deg >= ...
            options.CollisionClearanceTolerance_deg && ...
            maximumNormalNorm <= 1 + 2 ^ 20 * eps;
        if ~pairPassed
            minimumClearance_deg = NaN;
            return;
        end
        minimumClearance_deg = min( ...
            minimumClearance_deg, certifiedClearance_deg);
    end
end
certified = isfinite(minimumClearance_deg);
end

function [regions_deg, passed] = ...
        reconstructCertificateRegions(certificate, occupiedShape)
% Rebuild every certified region from the authoritative exact decomposition.
exactRecords = obstacleAvoidance.geometry.convexPolygonRegions(occupiedShape);
exactRegionCount = numel(exactRecords);
regions_deg = cell(exactRegionCount, 1);
for regionIndex = 1:exactRegionCount
    vertices_deg = exactRecords(regionIndex).Vertices;
    vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
    if size(vertices_deg, 1) < 3
        regions_deg = cell(0, 1);
        passed = false;
        return;
    end
    regions_deg{regionIndex} = vertices_deg;
end
planeRegionCount = size(certificate.Planes, 2);
if planeRegionCount == exactRegionCount
    passed = true;
    return;
end
passed = false;
regions_deg = cell(0, 1);
if ~isfield(certificate, "Coverage") || ...
        ~isstruct(certificate.Coverage) || ...
        ~isscalar(certificate.Coverage) || ...
        ~isfield(certificate.Coverage, "ConservativeGrouping")
    return;
end
grouping = certificate.Coverage.ConservativeGrouping;
requiredFields = {'Applied', 'ExactRegionCount', 'SolverRegionCount', ...
    'GroupMemberIndices'};
countsAreValid = isstruct(grouping) && isscalar(grouping) && ...
    all(isfield(grouping, requiredFields)) && ...
    isnumeric(grouping.ExactRegionCount) && ...
    isreal(grouping.ExactRegionCount) && ...
    isscalar(grouping.ExactRegionCount) && ...
    isfinite(grouping.ExactRegionCount) && ...
    isnumeric(grouping.SolverRegionCount) && ...
    isreal(grouping.SolverRegionCount) && ...
    isscalar(grouping.SolverRegionCount) && ...
    isfinite(grouping.SolverRegionCount);
if ~isstruct(grouping) || ~isscalar(grouping) || ...
        ~all(isfield(grouping, requiredFields)) || ...
        ~countsAreValid || ...
        ~isequal(grouping.Applied, true) || ...
        grouping.ExactRegionCount ~= exactRegionCount || ...
        grouping.SolverRegionCount ~= planeRegionCount || ...
        ~iscell(grouping.GroupMemberIndices) || ...
        numel(grouping.GroupMemberIndices) ~= planeRegionCount
    return;
end
memberCount = 0;
for groupIndex = 1:planeRegionCount
    memberIndex = grouping.GroupMemberIndices{groupIndex};
    membersAreValid = isnumeric(memberIndex) && isreal(memberIndex) && ...
        isvector(memberIndex) && ...
        ~isempty(memberIndex) && all(isfinite(memberIndex)) && ...
        all(memberIndex == fix(memberIndex)) && ...
        all(memberIndex >= 1 & memberIndex <= exactRegionCount);
    if ~membersAreValid
        return;
    end
    memberCount = memberCount + numel(memberIndex);
end
if memberCount ~= exactRegionCount
    return;
end
allMemberIndices = zeros(exactRegionCount, 1);
regions_deg = cell(planeRegionCount, 1);
nextMemberIndex = 1;
for groupIndex = 1:planeRegionCount
    memberIndex = reshape( ...
        grouping.GroupMemberIndices{groupIndex}, [], 1);
    targets = nextMemberIndex:(nextMemberIndex + numel(memberIndex) - 1);
    allMemberIndices(targets) = memberIndex;
    nextMemberIndex = targets(end) + 1;
    memberRegions_deg = ...
        regions_degForMembers(exactRecords, memberIndex);
    vertices_deg = vertcat(memberRegions_deg{:});
    hullIndex = convhull(vertices_deg(:, 1), vertices_deg(:, 2));
    regions_deg{groupIndex} = vertices_deg(hullIndex(1:end - 1), :);
end
passed = isequal(sort(allMemberIndices), (1:exactRegionCount).');
if ~passed
    regions_deg = cell(0, 1);
end
end

function regions_deg = regions_degForMembers(exactRecords, memberIndex)
% Extract finite exact-cell vertices for one independently replayed hull.
regions_deg = cell(numel(memberIndex), 1);
for localIndex = 1:numel(memberIndex)
    vertices_deg = exactRecords(memberIndex(localIndex)).Vertices;
    regions_deg{localIndex} = ...
        vertices_deg(all(isfinite(vertices_deg), 2), :);
end
end

function valid = validPlane(plane)
% Reject malformed or nonfinite separating-plane records.
valid = isstruct(plane) && isscalar(plane) && ...
    all(isfield(plane, {'Normal', 'Offset_deg'})) && ...
    isnumeric(plane.Normal) && isnumeric(plane.Offset_deg) && ...
    isreal(plane.Normal) && isreal(plane.Offset_deg) && ...
    isequal(size(plane.Normal), [2 2]) && ...
    isequal(size(plane.Offset_deg), [1 2]) && ...
    all(isfinite([plane.Normal(:); plane.Offset_deg(:)]));
end

function bernstein = powerToBernstein(power)
% Deliberately independent from motion construction so a shared arithmetic
% error cannot make generated motions and independent checking agree.
% Convert ascending power coefficients to same-degree Bernstein controls.
degree = size(power, 1) - 1;
transform = zeros(degree + 1);
for bernsteinIndex = 0:degree
    for powerIndex = 0:bernsteinIndex
        transform(bernsteinIndex + 1, powerIndex + 1) = ...
            nchoosek(bernsteinIndex, powerIndex) / ...
            nchoosek(degree, powerIndex);
    end
end
bernstein = transform * power;
end

function [collisionFree, resolved, minimumClearance_deg, ...
        checkCount, unresolvedCount] = ...
        checkCurveObstacleSeparation(polynomial, obstacles, limits, options)
% Prove moving-obstacle clearance or fail closed at the minimum time step.
if isempty(obstacles)
    [collisionFree, resolved, minimumClearance_deg, ...
        checkCount, unresolvedCount] = deal(true, true, Inf, 0, 0);
    return;
end
[collisionFree, resolved, minimumClearance_deg, ...
    checkCount, unresolvedCount] = deal(true, true, Inf, 0, 0);
pathSpeedBound_deg_s = norm(limits.maxVelocity_deg_s);
historyBounds_deg = zeros(numel(obstacles), 4);
obstacleEventTimes_s = zeros(0, 1);
for obstacleIndex = 1:numel(obstacles)
    historyBounds_deg(obstacleIndex, :) = ...
        obstacles(obstacleIndex).InternalPreparation.HistoryBounds_deg;
    obstacleEventTimes_s = [obstacleEventTimes_s; ...
        obstacles(obstacleIndex).time_s(:)]; %#ok<AGROW>
end
obstacleEventTimes_s = unique(obstacleEventTimes_s);
for segmentIndex = 1:polynomial.SegmentCount
    segmentStart_s = polynomial.SegmentStartTime_s(segmentIndex);
    durationIndex = min(segmentIndex, numel(polynomial.SegmentDuration_s));
    segmentEnd_s = segmentStart_s + ...
        polynomial.SegmentDuration_s(durationIndex);
    % Independently computed switching laws and obstacle histories can encode
    % the same physical event a few ULPs apart. Snap only that roundoff-sized
    % disagreement so a zero-width interval is not assigned an infinite
    % topology-change speed and rejected without evaluating its clearance.
    eventScale_s = max([1; abs(segmentStart_s); abs(segmentEnd_s); ...
        abs(obstacleEventTimes_s)]);
    eventTolerance_s = 1024 * eps(eventScale_s);
    segmentStart_s = snapToEventTime( ...
        segmentStart_s, obstacleEventTimes_s, eventTolerance_s);
    segmentEnd_s = snapToEventTime( ...
        segmentEnd_s, obstacleEventTimes_s, eventTolerance_s);
    splitTimes_s = [segmentStart_s; segmentEnd_s];
    for obstacleIndex = 1:numel(obstacles)
        obstacleTimes_s = obstacles(obstacleIndex).time_s(:);
        splitTimes_s = [splitTimes_s; obstacleTimes_s( ...
            obstacleTimes_s > segmentStart_s & ...
            obstacleTimes_s < segmentEnd_s)]; %#ok<AGROW>
    end
    splitTimes_s = unique(splitTimes_s);
    [~, splitPoints_deg] = ...
        bmtpEngine.evaluatePolynomial( ...
        polynomial, splitTimes_s, segmentIndex);
    for splitIndex = 1:numel(splitTimes_s)
        for obstacleIndex = 1:numel(obstacles)
            broadClearance_deg = pointBoxClearance( ...
                splitPoints_deg(splitIndex, :), ...
                historyBounds_deg(obstacleIndex, :));
            if broadClearance_deg > options.CollisionClearanceTolerance_deg
                minimumClearance_deg = min(minimumClearance_deg, ...
                    broadClearance_deg);
                continue;
            end
            shape = obstacleAvoidance.obstacles.shapeAtTime( ...
                obstacles(obstacleIndex), splitTimes_s(splitIndex));
            clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance( ...
                shape, splitPoints_deg(splitIndex, :));
            checkCount = checkCount + 1;
            minimumClearance_deg = min(minimumClearance_deg, clearance_deg);
            if clearance_deg <= options.CollisionClearanceTolerance_deg
                collisionFree = false;
                return;
            end
        end
    end
    stack_s = [splitTimes_s(1:end - 1), splitTimes_s(2:end)];
    while ~isempty(stack_s)
        interval_s = stack_s(end, :);
        stack_s(end, :) = [];
        intervalMid_s = mean(interval_s);
        [~, point_deg] = ...
        bmtpEngine.evaluatePolynomial( ...
            polynomial, intervalMid_s, segmentIndex);
        halfDuration_s = diff(interval_s) / 2;
        pathDisplacement_deg = pathSpeedBound_deg_s * halfDuration_s;
        intervalResolved = true;
        intervalClearance_deg = Inf;
        for obstacleIndex = 1:numel(obstacles)
            broadClearance_deg = pointBoxClearance(point_deg, ...
                historyBounds_deg(obstacleIndex, :)) - pathDisplacement_deg;
            if broadClearance_deg > options.CollisionClearanceTolerance_deg
                intervalClearance_deg = min(intervalClearance_deg, ...
                    broadClearance_deg);
                continue;
            end
            [shape, geometry] = obstacleAvoidance.obstacles.shapeAtTime( ...
                obstacles(obstacleIndex), intervalMid_s);
            clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance( ...
                shape, point_deg);
            checkCount = checkCount + 1;
            if clearance_deg <= options.CollisionClearanceTolerance_deg
                minimumClearance_deg = min(minimumClearance_deg, clearance_deg);
                collisionFree = false;
                return;
            end
            crossingBound_deg = (pathSpeedBound_deg_s + ...
                geometry.VertexSpeedBound_deg_s) * halfDuration_s;
            intervalClearance_deg = min(intervalClearance_deg, ...
                clearance_deg - crossingBound_deg);
            intervalResolved = intervalResolved && ...
                isfinite(crossingBound_deg) && clearance_deg > ...
                crossingBound_deg + options.CollisionClearanceTolerance_deg;
        end
        if intervalResolved
            minimumClearance_deg = min(minimumClearance_deg, ...
                intervalClearance_deg);
            continue;
        end
        if diff(interval_s) <= options.CollisionMinimumTimeStep_s
            collisionFree = false;
            resolved = false;
            unresolvedCount = unresolvedCount + 1;
            return;
        end
        stack_s = [stack_s; interval_s(1), intervalMid_s; ...
            intervalMid_s, interval_s(2)]; %#ok<AGROW>
    end
end
end

function time_s = snapToEventTime(time_s, eventTimes_s, tolerance_s)
% Coalesce only floating representations of the same physical event time.
if isempty(eventTimes_s)
    return;
end
[distance_s, eventIndex] = min(abs(eventTimes_s - time_s));
if distance_s <= tolerance_s
    time_s = eventTimes_s(eventIndex);
end
end

function clearance_deg = pointBoxClearance(point_deg, bounds_deg)
% Return Euclidean clearance from a point to an axis-aligned box.
axisDistance_deg = max([bounds_deg([1 3]) - point_deg; zeros(1, 2); ...
    point_deg - bounds_deg([2 4])], [], 1);
clearance_deg = norm(axisDistance_deg);
end
