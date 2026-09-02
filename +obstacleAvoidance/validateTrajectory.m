function validation = validateTrajectory( ...
        trajectory, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   validation = obstacleAvoidance.validateTrajectory()
%   validation = obstacleAvoidance.validateTrajectory(result)
%   validation = obstacleAvoidance.validateTrajectory( ...
%       trajectory, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Validate one complete polynomial motion independently of planner status.
%   - Fail closed when a continuous collision interval cannot be resolved.
%**************************************************************************
% INPUTS
%   - trajectory (scalar candidate or planner-result struct)
%       Must contain sampled histories and Polynomial segment coefficients.
%   - obstacles (canonical protected obstacle array, optional with result)
%   - initialState, goalState (normalized state structs, optional with result)
%   - limits (normalized physical limits struct, optional with result)
%   - options (resolved planner options, optional with result)
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%       Stable checks, clearance, message, collision counts, interval-proof
%       diagnostics, and timings.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees. Derivatives use deg/s, deg/s^2,
%     and deg/s^3. Time is seconds.
%**************************************************************************
if nargin == 0
    validation = createEmptyValidation();
    return;
end
validationTimer = tic;

%% Section 1: Resolve Inputs And Validate Histories

if nargin == 1
    requiredFields = {'Inputs', 'Options', 'Polynomial'};
    if ~isstruct(trajectory) || ~isscalar(trajectory) || ...
            ~all(isfield(trajectory, requiredFields))
        error("validateTrajectory:InvalidResult", ...
            "A one-input call requires a planner result with Inputs, " + ...
            "Options, and Polynomial fields.");
    end
    obstacles = trajectory.Inputs.obstacles;
    initialState = trajectory.Inputs.initialState;
    goalState = trajectory.Inputs.goalState;
    limits = trajectory.Inputs.limits;
    options = trajectory.Options;
elseif nargin ~= 6
    error("validateTrajectory:InvalidCall", ...
        "Use one planner result or all six explicit validation inputs.");
end
if isempty(obstacles) || ~isfield(obstacles, "InternalPreparation")
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacles);
end
obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles);
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
if options.AllowAzimuthWrapping && (~isempty(obstacles) || hasMovingGoal)
    error("validateTrajectory:UnsupportedWrappedGeometry", ...
        "Wrapped validation is supported only for obstacle-free " + ...
        "fixed-position goals.");
end
[time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3, historySizesMatch, timeIsFinite, ...
    timeIsStrictlyIncreasing, historyIsFinite] = ...
    validateSampledHistory(trajectory);

%% Section 2: Validate Endpoints And Continuous Polynomial Bounds

stateTolerance = max(10 * options.ConstraintTolerance, 1e-7);
[initialStateMatched, terminalStateMatched, goalTimeSatisfied] = ...
    validateEndpointAgreement(time_s, position_deg, velocity_deg_s, ...
    acceleration_deg_s2, historyIsFinite, timeIsFinite, initialState, ...
    goalState, options, stateTolerance);
[continuousBounds, dynamics, polynomialChecks] = ...
        obstacleAvoidance.validation.validatePolynomialTrajectory( ...
    trajectory.Polynomial, time_s, position_deg, velocity_deg_s, ...
    acceleration_deg_s2, jerk_deg_s3, initialState, goalState, limits, ...
    options, stateTolerance, @withinBounds);

%% Section 3: Certify Continuous Collision Freedom

collisionTimer = tic;
[collisionFree, collisionResolved, seedCorridorCertified, ...
    planeCertificateCertified, minimumClearance_deg, ...
    collisionCheckCount, collisionIntervalCount, unresolvedIntervalCount, ...
    collisionDiagnostics] = ...
    certifyCompleteCollision(trajectory, obstacles, options, ...
    timeIsStrictlyIncreasing && historyIsFinite && continuousBounds.Valid);
collisionCheckingElapsedTime_s = toc(collisionTimer);
safetyMarginPolicySatisfied = safetyMarginProvenanceSatisfied(obstacles);
azimuthWrapPolicySatisfied = options.AllowAzimuthWrapping || ...
    continuousBounds.PositionWithinLimits;

%% Section 4: Assemble The Stable Validation Record

checkNames = ["history shape", "finite increasing time", ...
    "finite histories", "initial state", "terminal state", "goal time", ...
    "polynomial format", "polynomial initial time", ...
    "polynomial time base", "polynomial segment continuity", ...
    "polynomial endpoint states", "polynomial sampled histories", ...
    "continuous limits", "polynomial dynamics", "collision freedom", ...
    "collision resolution", "safety-margin provenance", ...
    "azimuth-wrap policy"];
checkValues = [historySizesMatch, timeIsStrictlyIncreasing, ...
    historyIsFinite, initialStateMatched, terminalStateMatched, ...
    goalTimeSatisfied, polynomialChecks.FormatValid, ...
    polynomialChecks.InitialTimeMatched, ...
    polynomialChecks.TimeBaseConsistent, ...
    polynomialChecks.SegmentContinuity, ...
    polynomialChecks.EndpointStatesMatched, ...
    polynomialChecks.HistoryConsistent, continuousBounds.Valid, ...
    dynamics.Consistent, collisionFree, collisionResolved, ...
    safetyMarginPolicySatisfied, azimuthWrapPolicySatisfied];
passed = all(checkValues);
issues = checkNames(~checkValues).';
if passed
    message = "Independent continuous validation passed.";
else
    message = "Independent validation failed: " + ...
        strjoin(issues, ", ") + ".";
end
values = {passed, message, historySizesMatch, timeIsFinite, ...
    timeIsStrictlyIncreasing, historyIsFinite, initialStateMatched, ...
    terminalStateMatched, goalTimeSatisfied, polynomialChecks.FormatValid, ...
    polynomialChecks.InitialTimeMatched, polynomialChecks.TimeBaseConsistent, ...
    polynomialChecks.SegmentContinuity, ...
    polynomialChecks.EndpointStatesMatched, ...
    polynomialChecks.HistoryConsistent, ...
    polynomialChecks.MaximumSegmentContinuityResidual, ...
    polynomialChecks.MaximumHistoryResidual, ...
    continuousBounds.PositionWithinLimits, ...
    continuousBounds.VelocityWithinLimits, ...
    continuousBounds.AccelerationWithinLimits, ...
    continuousBounds.JerkWithinLimits, dynamics.Consistent, ...
    dynamics.MaximumResidual, collisionFree, collisionResolved, ...
    seedCorridorCertified, planeCertificateCertified, minimumClearance_deg, ...
    collisionCheckCount, collisionIntervalCount, unresolvedIntervalCount, ...
    collisionDiagnostics, ...
    safetyMarginPolicySatisfied, azimuthWrapPolicySatisfied, ...
    maximumAbsolute(velocity_deg_s), maximumAbsolute(acceleration_deg_s2), ...
    maximumAbsolute(jerk_deg_s3), issues, collisionCheckingElapsedTime_s, ...
    toc(validationTimer)};
validation = createValidationRecord(values);
end

%% Section 5: Local Functions

function [time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
        jerk_deg_s3, historySizesMatch, timeIsFinite, ...
        timeIsStrictlyIncreasing, historyIsFinite] = ...
        validateSampledHistory(trajectory)
% Normalize sampled histories and check their shared shape and finiteness.
requiredFields = {'time_s', 'position_deg', 'velocity_deg_s', ...
    'acceleration_deg_s2', 'jerk_deg_s3', 'Polynomial'};
if ~isstruct(trajectory) || ~isscalar(trajectory) || ...
        ~all(isfield(trajectory, requiredFields))
    error("validateTrajectory:InvalidTrajectory", ...
        "trajectory must contain sampled histories and Polynomial data.");
end
time_s = double(trajectory.time_s(:));
position_deg = double(trajectory.position_deg);
velocity_deg_s = double(trajectory.velocity_deg_s);
acceleration_deg_s2 = double(trajectory.acceleration_deg_s2);
jerk_deg_s3 = double(trajectory.jerk_deg_s3);
sampleCount = numel(time_s);
historySizesMatch = isequal(size(position_deg), [sampleCount 2]) && ...
    isequal(size(velocity_deg_s), [sampleCount 2]) && ...
    isequal(size(acceleration_deg_s2), [sampleCount 2]) && ...
    isequal(size(jerk_deg_s3), [sampleCount 2]);
timeIsFinite = ~isempty(time_s) && all(isfinite(time_s));
timeIsStrictlyIncreasing = timeIsFinite && all(diff(time_s) > 0);
historyIsFinite = sampleCount > 0 && historySizesMatch && ...
    all(isfinite(position_deg), "all") && ...
    all(isfinite(velocity_deg_s), "all") && ...
    all(isfinite(acceleration_deg_s2), "all") && ...
    all(isfinite(jerk_deg_s3), "all");
end

function [initialStateMatched, terminalStateMatched, goalTimeSatisfied] = ...
        validateEndpointAgreement(time_s, position_deg, velocity_deg_s, ...
        acceleration_deg_s2, historyIsFinite, timeIsFinite, initialState, ...
        goalState, options, stateTolerance)
% Compare sampled endpoints with the requested states and arrival policy.
initialStateMatched = historyIsFinite && max(abs([ ...
    position_deg(1, :) - initialState.position_deg, ...
    velocity_deg_s(1, :) - initialState.velocity_deg_s, ...
    acceleration_deg_s2(1, :) - initialState.acceleration_deg_s2])) <= ...
    stateTolerance;
if timeIsFinite
    goalPosition_deg = obstacleAvoidance.input.goalPositionAtTime( ...
        goalState, time_s(end));
else
    goalPosition_deg = [NaN NaN];
end
terminalStateMatched = historyIsFinite && max(abs([ ...
    position_deg(end, :) - goalPosition_deg, ...
    velocity_deg_s(end, :) - goalState.velocity_deg_s, ...
    acceleration_deg_s2(end, :) - goalState.acceleration_deg_s2])) <= ...
    stateTolerance;
if options.GoalTimeMode == "fixedArrival"
    goalTimeSatisfied = timeIsFinite && ...
        abs(time_s(end) - goalState.time_s) <= stateTolerance;
else
    goalTimeSatisfied = timeIsFinite && ...
        time_s(end) <= goalState.time_s + stateTolerance && ...
        time_s(end) > initialState.time_s;
end
end

function [collisionFree, collisionResolved, seedCorridorCertified, ...
        planeCertificateCertified, minimumClearance_deg, ...
        collisionCheckCount, collisionIntervalCount, ...
        unresolvedIntervalCount, collisionDiagnostics] = ...
        certifyCompleteCollision( ...
        trajectory, obstacles, options, canCertifyCollision)
% Prefer independently checked complete certificates, then adaptive checks.
[collisionFree, collisionResolved, seedCorridorCertified, ...
    planeCertificateCertified, minimumClearance_deg, collisionCheckCount, ...
    collisionIntervalCount, unresolvedIntervalCount] = ...
    deal(false, false, false, false, NaN, 0, 0, 0);
collisionDiagnostics = createCollisionDiagnostics( ...
    options.CollisionMinimumTimeStep_s);
if ~canCertifyCollision
    return;
end
[planeCertificateCertified, planeClearance_deg] = ...
    certifyStaticPlaneCertificate(trajectory, obstacles, options);
if ~planeCertificateCertified
    [planeCertificateCertified, planeClearance_deg] = ...
        certifyTimedPlaneCertificate(trajectory, obstacles, options);
end
if planeCertificateCertified
    [collisionFree, collisionResolved, minimumClearance_deg] = ...
        deal(true, true, planeClearance_deg);
    collisionDiagnostics.Method = "planeCertificate";
    collisionDiagnostics.TerminationReason = "certified";
    return;
end
[seedCorridorCertified, seedClearance_deg] = ...
    obstacleAvoidance.validation.certifySeedCorridor( ...
    trajectory, obstacles, options.CollisionClearanceTolerance_deg);
if seedCorridorCertified
    [collisionFree, collisionResolved, minimumClearance_deg] = ...
        deal(true, true, seedClearance_deg);
    collisionDiagnostics.Method = "seedCorridorCertificate";
    collisionDiagnostics.TerminationReason = "certified";
    return;
end
[collisionFree, collisionResolved, minimumClearance_deg, ...
    collisionCheckCount, collisionIntervalCount, unresolvedIntervalCount, ...
    collisionDiagnostics] = ...
    certifyCollision(trajectory.Polynomial, obstacles, options);
end

function [certified, minimumClearance_deg] = ...
        certifyStaticPlaneCertificate(trajectory, obstacles, options)
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
        certifyTimedPlaneCertificate(trajectory, obstacles, options)
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

function satisfied = safetyMarginProvenanceSatisfied(obstacles)
% Require original geometry and one finite nonnegative applied margin.
if isempty(obstacles)
    satisfied = true;
    return;
end
hasFields = all(isfield(obstacles, ...
    {'originalAz_deg', 'originalEl_deg', 'safetyMargin_deg'}));
satisfied = hasFields && all(isfinite([obstacles.safetyMargin_deg])) && ...
    all([obstacles.safetyMargin_deg] >= 0);
end

function within = withinBounds(powerCoefficient, lower, upper, tolerance)
% Check endpoints and every finite stationary point on normalized time [0, 1].
derivativeCoefficient = (1:numel(powerCoefficient) - 1).' .* ...
    powerCoefficient(2:end);
lastDerivativeIndex = find(derivativeCoefficient ~= 0, 1, "last");
candidateTau = [0; 1];
if ~isempty(lastDerivativeIndex)
    stationaryTau = real(roots(flip( ...
        derivativeCoefficient(1:lastDerivativeIndex))));
    rootTolerance = 1e-9;
    stationaryTau = stationaryTau(stationaryTau >= -rootTolerance & ...
        stationaryTau <= 1 + rootTolerance);
    candidateTau = [candidateTau; min(max(stationaryTau, 0), 1)];
end
candidateValue = polyval(flip(powerCoefficient), candidateTau);
within = all(candidateValue >= lower - tolerance & ...
    candidateValue <= upper + tolerance);
end

function [collisionFree, resolved, minimumClearance_deg, ...
        checkCount, intervalCount, unresolvedCount, diagnostics] = ...
        certifyCollision(polynomial, obstacles, options)
% Prove moving-obstacle clearance or fail closed at the minimum time step.
diagnostics = createCollisionDiagnostics(options.CollisionMinimumTimeStep_s);
diagnostics.Method = "adaptiveIntervalCertificate";
if isempty(obstacles)
    [collisionFree, resolved, minimumClearance_deg, ...
        checkCount, intervalCount, unresolvedCount] = ...
        deal(true, true, Inf, 0, 0, 0);
    diagnostics.TerminationReason = "noObstacles";
    return;
end
[collisionFree, resolved, minimumClearance_deg, ...
    checkCount, intervalCount, unresolvedCount] = ...
    deal(true, true, Inf, 0, 0, 0);
obstacleEventTimes_s = zeros(0, 1);
trajectoryStart_s = polynomial.SegmentStartTime_s(1);
lastDurationIndex = min(polynomial.SegmentCount, ...
    numel(polynomial.SegmentDuration_s));
trajectoryEnd_s = polynomial.SegmentStartTime_s(end) + ...
    polynomial.SegmentDuration_s(lastDurationIndex);
for obstacleIndex = 1:numel(obstacles)
    obstacleEventTimes_s = [obstacleEventTimes_s; ...
        obstacles(obstacleIndex).time_s( ...
        obstacles(obstacleIndex).time_s >= trajectoryStart_s & ...
        obstacles(obstacleIndex).time_s <= trajectoryEnd_s)]; %#ok<AGROW>
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
    velocityPower = squeeze( ...
        polynomial.velocityPower_deg_s(segmentIndex, :, :)).';
    segmentVelocityControls_deg_s = powerToBernstein(velocityPower);
    [~, splitPoints_deg] = ...
        bmtpEngine.evaluatePolynomial( ...
        polynomial, splitTimes_s, segmentIndex);
    for splitIndex = 1:numel(splitTimes_s)
        for obstacleIndex = 1:numel(obstacles)
            pointTime_s = splitTimes_s(splitIndex);
            [pointBounds_deg, ~, obstacleIsActive] = ...
                obstacleIntervalBounds( ...
                obstacles(obstacleIndex), [pointTime_s pointTime_s]);
            if ~obstacleIsActive
                continue;
            end
            broadClearance_deg = pointBoxClearance( ...
                splitPoints_deg(splitIndex, :), ...
                pointBounds_deg);
            if broadClearance_deg > options.CollisionClearanceTolerance_deg
                diagnostics.PointBroadPhaseRejectCount = ...
                    diagnostics.PointBroadPhaseRejectCount + 1;
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
                diagnostics.TerminationReason = "collisionAtSplitTime";
                return;
            end
        end
    end
    stack_s = [splitTimes_s(1:end - 1), splitTimes_s(2:end)];
    while ~isempty(stack_s)
        intervalCount = intervalCount + 1;
        interval_s = stack_s(end, :);
        stack_s(end, :) = [];
        intervalMid_s = mean(interval_s);
        [~, point_deg] = ...
        bmtpEngine.evaluatePolynomial( ...
            polynomial, intervalMid_s, segmentIndex);
        halfDuration_s = diff(interval_s) / 2;
        pathSpeedBound_deg_s = polynomialVelocityBound( ...
            segmentVelocityControls_deg_s, segmentStart_s, ...
            polynomial.SegmentDuration_s(durationIndex), interval_s);
        diagnostics.MaximumPathSpeedBound_deg_s = max( ...
            diagnostics.MaximumPathSpeedBound_deg_s, ...
            pathSpeedBound_deg_s);
        pathDisplacement_deg = pathSpeedBound_deg_s * halfDuration_s;
        intervalResolved = true;
        intervalClearance_deg = Inf;
        leastSlack_deg = Inf;
        limitingObstacleIndex = NaN;
        limitingObstacleSpeed_deg_s = NaN;
        limitingRequiredClearance_deg = NaN;
        limitingObservedClearance_deg = NaN;
        for obstacleIndex = 1:numel(obstacles)
            [sweptBounds_deg, obstacleSpeedBound_deg_s, ...
                obstacleIsActive] = obstacleIntervalBounds( ...
                obstacles(obstacleIndex), interval_s);
            if ~obstacleIsActive
                continue;
            end
            diagnostics.MaximumObstacleSpeedBound_deg_s = max( ...
                diagnostics.MaximumObstacleSpeedBound_deg_s, ...
                obstacleSpeedBound_deg_s);
            broadClearance_deg = pointBoxClearance(point_deg, ...
                sweptBounds_deg) - pathDisplacement_deg;
            if broadClearance_deg > options.CollisionClearanceTolerance_deg
                diagnostics.IntervalBroadPhaseRejectCount = ...
                    diagnostics.IntervalBroadPhaseRejectCount + 1;
                intervalClearance_deg = min(intervalClearance_deg, ...
                    broadClearance_deg);
                continue;
            end
            shape = obstacleAvoidance.obstacles.shapeAtTime( ...
                obstacles(obstacleIndex), intervalMid_s);
            clearance_deg = obstacleAvoidance.geometry.pointPolygonClearance( ...
                shape, point_deg);
            checkCount = checkCount + 1;
            if clearance_deg <= options.CollisionClearanceTolerance_deg
                minimumClearance_deg = min(minimumClearance_deg, clearance_deg);
                collisionFree = false;
                diagnostics.TerminationReason = "collisionAtIntervalMidpoint";
                return;
            end
            crossingBound_deg = (pathSpeedBound_deg_s + ...
                obstacleSpeedBound_deg_s) * halfDuration_s;
            requiredClearance_deg = crossingBound_deg + ...
                options.CollisionClearanceTolerance_deg;
            intervalClearance_deg = min(intervalClearance_deg, ...
                clearance_deg - crossingBound_deg);
            obstacleResolved = isfinite(crossingBound_deg) && ...
                clearance_deg > requiredClearance_deg;
            intervalResolved = intervalResolved && obstacleResolved;
            clearanceSlack_deg = clearance_deg - requiredClearance_deg;
            if ~obstacleResolved && clearanceSlack_deg < leastSlack_deg
                leastSlack_deg = clearanceSlack_deg;
                limitingObstacleIndex = obstacleIndex;
                limitingObstacleSpeed_deg_s = obstacleSpeedBound_deg_s;
                limitingRequiredClearance_deg = requiredClearance_deg;
                limitingObservedClearance_deg = clearance_deg;
            end
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
            diagnostics.LastUnresolvedInterval_s = interval_s;
            diagnostics.LastUnresolvedObstacleIndex = limitingObstacleIndex;
            diagnostics.LastPathSpeedBound_deg_s = pathSpeedBound_deg_s;
            diagnostics.LastObstacleSpeedBound_deg_s = ...
                limitingObstacleSpeed_deg_s;
            diagnostics.LastRequiredClearance_deg = ...
                limitingRequiredClearance_deg;
            diagnostics.LastObservedClearance_deg = ...
                limitingObservedClearance_deg;
            diagnostics.TerminationReason = "minimumTimeStepUnresolved";
            return;
        end
        stack_s = [stack_s; interval_s(1), intervalMid_s; ...
            intervalMid_s, interval_s(2)]; %#ok<AGROW>
    end
end
diagnostics.TerminationReason = "certified";
end

function speedBound_deg_s = polynomialVelocityBound( ...
        segmentControls_deg_s, segmentStart_s, duration_s, interval_s)
% Bound vector speed from Bernstein envelopes over this exact subinterval.
intervalTau = (double(interval_s(:)) - segmentStart_s) / duration_s;
intervalTau = min(1, max(0, intervalTau));
velocityControls_deg_s = restrictBernsteinControls( ...
    segmentControls_deg_s, intervalTau(1), intervalTau(2));
coefficientScale_deg_s = max(1, max(abs(velocityControls_deg_s), [], "all"));
roundoffReserve_deg_s = 256 * eps(coefficientScale_deg_s);
axisSpeedBound_deg_s = max(abs(velocityControls_deg_s), [], 1) + ...
    roundoffReserve_deg_s;
speedBound_deg_s = norm(axisSpeedBound_deg_s);
end

function controls = restrictBernsteinControls( ...
        controls, lowerTau, upperTau)
% Restrict one Bernstein curve through stable de Casteljau subdivisions.
if lowerTau > 0
    [~, controls] = subdivideBernsteinControls(controls, lowerTau);
    upperTau = (upperTau - lowerTau) / (1 - lowerTau);
end
if upperTau < 1
    [controls, ~] = subdivideBernsteinControls(controls, upperTau);
end
end

function [leftControls, rightControls] = ...
        subdivideBernsteinControls(controls, fraction)
% Return both exact Bernstein control polygons at one normalized fraction.
degree = size(controls, 1) - 1;
leftControls = zeros(size(controls));
rightControls = zeros(size(controls));
leftControls(1, :) = controls(1, :);
rightControls(end, :) = controls(end, :);
work = controls;
for level = 1:degree
    remainingCount = degree - level + 1;
    work(1:remainingCount, :) = ...
        (1 - fraction) * work(1:remainingCount, :) + ...
        fraction * work(2:remainingCount + 1, :);
    leftControls(level + 1, :) = work(1, :);
    rightControls(end - level, :) = work(remainingCount, :);
end
end

function [bounds_deg, speedBound_deg_s, active] = ...
        obstacleIntervalBounds(obstacle, interval_s)
% Bound occupied geometry over one interval already split at source events.
startTime_s = interval_s(1);
endTime_s = interval_s(2);
midTime_s = mean(interval_s);
preparation = obstacle.InternalPreparation;
sourceTime_s = double(obstacle.time_s(:));
isOutsideHistory = numel(sourceTime_s) > 1 && ...
    (midTime_s < sourceTime_s(1) || midTime_s > sourceTime_s(end));
if isempty(sourceTime_s) || isOutsideHistory
    bounds_deg = [Inf -Inf Inf -Inf];
    speedBound_deg_s = 0;
    active = false;
    return;
end
if isscalar(sourceTime_s)
    bounds_deg = preparation.SampleBounds_deg(1, :);
    speedBound_deg_s = 0;
elseif endTime_s == startTime_s
    [bounds_deg, speedBound_deg_s] = obstaclePointBounds( ...
        obstacle, startTime_s);
else
    intervalIndex = find(sourceTime_s < midTime_s, 1, "last");
    if isempty(intervalIndex)
        intervalIndex = 1;
    end
    intervalIndex = min(intervalIndex, numel(sourceTime_s) - 1);
    speedBound_deg_s = ...
        preparation.IntervalSpeedBound_deg_s(intervalIndex);
    if preparation.MatchingTopology(intervalIndex)
        startBounds_deg = interpolatedIntervalBounds( ...
            obstacle, intervalIndex, startTime_s);
        endBounds_deg = interpolatedIntervalBounds( ...
            obstacle, intervalIndex, endTime_s);
        bounds_deg = [min(startBounds_deg(1), endBounds_deg(1)), ...
            max(startBounds_deg(2), endBounds_deg(2)), ...
            min(startBounds_deg(3), endBounds_deg(3)), ...
            max(startBounds_deg(4), endBounds_deg(4))];
    else
        bounds_deg = preparation.IntervalBounds_deg(intervalIndex, :);
    end
end
active = bounds_deg(1) <= bounds_deg(2) && bounds_deg(3) <= bounds_deg(4);
end

function [bounds_deg, speedBound_deg_s] = ...
        obstaclePointBounds(obstacle, queryTime_s)
% Evaluate one cached sample or verified linear boundary without a polyshape.
sourceTime_s = double(obstacle.time_s(:));
preparation = obstacle.InternalPreparation;
lowerIndex = find(sourceTime_s <= queryTime_s, 1, "last");
upperIndex = find(sourceTime_s >= queryTime_s, 1, "first");
if lowerIndex == upperIndex
    bounds_deg = preparation.SampleBounds_deg(lowerIndex, :);
    speedBound_deg_s = preparation.SampleSpeedBound_deg_s(lowerIndex);
elseif preparation.MatchingTopology(lowerIndex)
    bounds_deg = interpolatedIntervalBounds( ...
        obstacle, lowerIndex, queryTime_s);
    speedBound_deg_s = ...
        preparation.IntervalSpeedBound_deg_s(lowerIndex);
else
    bounds_deg = preparation.IntervalBounds_deg(lowerIndex, :);
    speedBound_deg_s = 0;
end
end

function bounds_deg = interpolatedIntervalBounds( ...
        obstacle, intervalIndex, queryTime_s)
% Bound the verified linearly corresponding vertices at one exact time.
sourceTime_s = double(obstacle.time_s(:));
preparation = obstacle.InternalPreparation;
fraction = (queryTime_s - sourceTime_s(intervalIndex)) / ...
    (sourceTime_s(intervalIndex + 1) - sourceTime_s(intervalIndex));
azimuth_deg = double(obstacle.az_deg{intervalIndex}(:)) + ...
    fraction * preparation.DeltaAzimuth_deg{intervalIndex};
elevation_deg = double(obstacle.el_deg{intervalIndex}(:)) + ...
    fraction * preparation.DeltaElevation_deg{intervalIndex};
bounds_deg = finiteBoundaryBounds([azimuth_deg, elevation_deg]);
end

function bounds_deg = finiteBoundaryBounds(vertices_deg)
% Return an empty or finite [minAz maxAz minEl maxEl] boundary box.
vertices_deg = vertices_deg(all(isfinite(vertices_deg), 2), :);
if isempty(vertices_deg)
    bounds_deg = [Inf -Inf Inf -Inf];
else
    bounds_deg = [min(vertices_deg(:, 1)), max(vertices_deg(:, 1)), ...
        min(vertices_deg(:, 2)), max(vertices_deg(:, 2))];
end
end

function diagnostics = createCollisionDiagnostics(minimumTimeStep_s)
% Define stable proof evidence for certified, colliding, and unresolved exits.
diagnostics = struct( ...
    "Method", "notRun", ...
    "TerminationReason", "notRun", ...
    "LastUnresolvedInterval_s", [NaN NaN], ...
    "LastUnresolvedObstacleIndex", NaN, ...
    "LastPathSpeedBound_deg_s", NaN, ...
    "LastObstacleSpeedBound_deg_s", NaN, ...
    "LastRequiredClearance_deg", NaN, ...
    "LastObservedClearance_deg", NaN, ...
    "MinimumTimeStep_s", minimumTimeStep_s, ...
    "MaximumPathSpeedBound_deg_s", 0, ...
    "MaximumObstacleSpeedBound_deg_s", 0, ...
    "PointBroadPhaseRejectCount", 0, ...
    "IntervalBroadPhaseRejectCount", 0);
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

function peak = maximumAbsolute(values)
% Return per-coordinate sampled peaks or documented NaN values.
if isempty(values)
    peak = [NaN NaN];
else
    peak = max(abs(values), [], 1);
end
end

function validation = createEmptyValidation()
% Define every stable validation field before checks are available.
collisionDiagnostics = createCollisionDiagnostics(NaN);
values = {false, "No trajectory was validated.", false, false, false, ...
    false, false, false, false, false, false, false, false, false, false, ...
    NaN, NaN, false, false, false, false, false, NaN, false, false, false, ...
    false, NaN, 0, 0, 0, collisionDiagnostics, false, false, ...
    [NaN NaN], [NaN NaN], [NaN NaN], ...
    strings(0, 1), 0, 0};
validation = createValidationRecord(values);
end

function validation = createValidationRecord(values)
% Assemble values once in stable public field order.
names = ["Passed", "Message", "HistorySizesMatch", "TimeIsFinite", ...
    "TimeIsStrictlyIncreasing", "HistoryIsFinite", "InitialStateMatched", ...
    "TerminalStateMatched", "GoalTimeSatisfied", "PolynomialFormatValid", ...
    "PolynomialInitialTimeMatched", "PolynomialTimeBaseConsistent", ...
    "PolynomialSegmentContinuity", "PolynomialEndpointStatesMatched", ...
    "PolynomialHistoryConsistent", "MaximumSegmentContinuityResidual", ...
    "MaximumPolynomialHistoryResidual", "PositionWithinLimits", ...
    "VelocityWithinLimits", "AccelerationWithinLimits", "JerkWithinLimits", ...
    "DynamicsConsistent", "MaximumDynamicsResidual", "CollisionFree", ...
    "CollisionResolved", "SeedCorridorCertified", ...
    "PlaneCertificateCertified", "MinimumClearance_deg", ...
    "CollisionCheckCount", "CollisionIntervalCount", ...
    "UnresolvedIntervalCount", "CollisionDiagnostics", ...
    "SafetyMarginPolicySatisfied", "AzimuthWrapPolicySatisfied", ...
    "PeakVelocity_deg_s", "PeakAcceleration_deg_s2", "PeakJerk_deg_s3", ...
    "Issues", "CollisionCheckingElapsedTime_s", "ElapsedTime_s"];
validation = cell2struct(values, cellstr(names), 2);
end
