function [candidate, diagnostics] = createFixedClockLateralExcursion( ...
        directCandidate, obstacles, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createFixedClockLateralExcursion()
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createFixedClockLateralExcursion( ...
%       directCandidate, obstacles, initialState, goalState, limits, options)
%**************************************************************************
% PURPOSE
%   - Preserve a certified direct physical-minimum clock while enumerating
%     additive coordinate-axis rest-to-rest collision detours.
%   - On a static sequence of boundary-attached barriers, construct an
%     input-driven minimum-jerk lateral spline on that same exact clock.
%**************************************************************************
% INPUTS
%   - directCandidate (scalar planner candidate struct)
%       A successful exact direct motion whose duration equals its reported
%       maximum componentwise minimum duration.
%   - obstacles (canonical or prepared obstacle array)
%       Original protected geometry and complete static or moving histories.
%   - initialState, goalState (normalized scalar structs)
%       Fixed two-coordinate rest-to-rest request used by the direct motion.
%   - limits (normalized scalar struct)
%       Per-axis physical limits and azimuth/elevation workspace intervals.
%   - options (resolved scalar planner-options struct)
%       Supplies sampling, collision, constraint, and goal-time policies.
%**************************************************************************
% OUTPUTS
%   - candidate (scalar planner candidate struct)
%       An independently validated fixed-clock motion, or the unchanged
%       direct record when no supported construction passes.
%   - diagnostics (scalar struct)
%       Stable clock, enumeration, feasible-side boundary, validation, and
%       explicit unsupported-or-failure evidence.
%**************************************************************************
% UNITS
%   - Position and path length are degrees. Time is seconds. Derivatives use
%     degrees per second and its second and third powers.
%**************************************************************************

%% Section 1: Verify The Direct Physical Clock

if nargin == 0
    candidate = struct();
    diagnostics = createDiagnostics();
    return;
end
if nargin ~= 6
    error("createFixedClockLateralExcursion:InvalidCall", ...
        "directCandidate, obstacles, states, limits, and options are required.");
end
timer = tic;
candidate = directCandidate;
diagnostics = createDiagnostics();
if ~isstruct(directCandidate) || ~isscalar(directCandidate) || ...
        ~all(isfield(directCandidate, {'Success', 'MotionDuration_s', ...
        'MinimumAxisDuration_s', 'Polynomial', 'position_deg'}))
    diagnostics = finishFailure(diagnostics, "invalidDirectCandidate", ...
        "The direct candidate lacks the required stable motion fields.", timer);
    return;
end
if ~directCandidate.Success || isempty(directCandidate.position_deg)
    diagnostics = finishFailure(diagnostics, "directMotionUnavailable", ...
        "A successful direct motion is required before an excursion is tried.", timer);
    return;
end
dimensionCount = size(directCandidate.position_deg, 2);
if dimensionCount ~= 2
    diagnostics = finishFailure(diagnostics, "unsupportedDimension", ...
        "The independent validator currently requires two coordinates.", timer);
    return;
end
% A moving-target adapter may retain its sampled history after evaluating the
% fixed capture position and time. That metadata does not alter this motion.
duration_s = double(directCandidate.MotionDuration_s);
lowerBound_s = max(double(directCandidate.MinimumAxisDuration_s));
clockTolerance_s = max(double(options.ConstraintTolerance), ...
    256 * eps(max(1, duration_s)));
diagnostics.Attempted = true;
diagnostics.DirectDuration_s = duration_s;
diagnostics.CertifiedLowerBound_s = lowerBound_s;
diagnostics.ClockTolerance_s = clockTolerance_s;
diagnostics.ClockMatched = isfinite(duration_s) && isfinite(lowerBound_s) && ...
    abs(duration_s - lowerBound_s) <= clockTolerance_s;
if ~diagnostics.ClockMatched
    diagnostics = finishFailure(diagnostics, "directClockNotCertified", ...
        "The direct duration does not equal its componentwise physical lower bound.", timer);
    return;
end
directValidation = obstacleAvoidance.validateTrajectory( ...
    directCandidate, obstacles, initialState, goalState, limits, options);
diagnostics.DirectValidation = directValidation;
diagnostics.ValidationCount = 1;
if directValidation.Passed
    candidate.Validation = directValidation;
    diagnostics.Success = true;
    diagnostics.SelectedMode = "direct";
    diagnostics.Message = "The zero-amplitude direct motion already passes validation.";
    diagnostics.TerminationReason = "directAlreadyValid";
    diagnostics.SelectedValidation = directValidation;
    diagnostics.ElapsedTime_s = toc(timer);
    return;
end
if ~directMotionIsPhysical(directValidation)
    diagnostics = finishFailure(diagnostics, "directMotionInvalid", ...
        "The direct motion failed a non-collision invariant.", timer);
    return;
end

%% Section 2: Try A Static Boundary-Attached Barrier Sequence

[sequenceCandidate, sequenceReport] = createBarrierSequence( ...
    directCandidate, obstacles, initialState, goalState, limits, options, clockTolerance_s);
diagnostics.BarrierSequence = sequenceReport;
if sequenceReport.CandidateCreated
    sequenceValidation = obstacleAvoidance.validateTrajectory( ...
        sequenceCandidate, obstacles, initialState, goalState, limits, options);
    diagnostics.ValidationCount = diagnostics.ValidationCount + 1;
    diagnostics.BarrierSequence.Validation = sequenceValidation;
    if sequenceValidation.Passed
        sequenceCandidate.Validation = sequenceValidation;
        candidate = sequenceCandidate;
        diagnostics.Success = true;
        diagnostics.SelectedMode = "barrierSequence";
        diagnostics.Message = "A barrier-sequence motion attained the physical time floor.";
        diagnostics.TerminationReason = "goalReached";
        diagnostics.SelectedAxisIndex = sequenceReport.LateralAxisIndex;
        diagnostics.MotionLength_deg = candidate.MotionLength_deg;
        diagnostics.SelectedValidation = sequenceValidation;
        diagnostics.ElapsedTime_s = toc(timer);
        return;
    end
    diagnostics.BarrierSequence.TerminationReason = "independentValidationFailed";
    diagnostics.BarrierSequence.Message = sequenceValidation.Message;
end

%% Section 3: Enumerate Input-Scaled Axis Excursions

workspaceInterval_deg = [double(limits.azimuthInterval_deg(:).'); ...
    double(limits.elevationInterval_deg(:).')];
coarseLevelCount = 8;
boundaryResolution_deg = max(8 * double(options.CollisionClearanceTolerance_deg), ...
    sqrt(eps) * max(1, max(abs(directCandidate.position_deg), [], "all")));
peakTime_s = createPeakTimeCandidates(directCandidate, obstacles, options);
axisReports = repmat( ...
    createAxisReport(), 2 * dimensionCount * numel(peakTime_s), 1);
screenedCandidates = cell(size(axisReports));
bestReportIndex = 0;
reportIndex = 0;
for axisIndex = 1:dimensionCount
    axisMinimum_s = directCandidate.MinimumAxisDuration_s(axisIndex);
    axisGovernsClock = axisMinimum_s >= duration_s - clockTolerance_s;
    for direction = [-1, 1]
        for peakIndex = 1:numel(peakTime_s)
            reportIndex = reportIndex + 1;
            report = createAxisReport();
            report.AxisIndex = axisIndex;
            report.Direction = direction;
            report.PeakTime_s = peakTime_s(peakIndex);
            report.AxisGovernsClock = axisGovernsClock;
            if axisGovernsClock
                report.TerminationReason = "governingAxisHasNoCertifiedSlack";
                axisReports(reportIndex) = report;
                continue;
            end
            if direction > 0
                workspaceRoom_deg = workspaceInterval_deg(axisIndex, 2) - ...
                    max(directCandidate.position_deg(:, axisIndex));
            else
                workspaceRoom_deg = min(directCandidate.position_deg(:, axisIndex)) - ...
                    workspaceInterval_deg(axisIndex, 1);
            end
            phaseDuration_s = [peakTime_s(peakIndex) - initialState.time_s, ...
                directCandidate.FinalTime_s - peakTime_s(peakIndex)];
            physicalRoom_deg = Inf;
            for phaseIndex = 1:2
                phaseRoom_deg = bmtpEngine.maximumRestToRestDistance( ...
                    phaseDuration_s(phaseIndex), ...
                    limits.maxVelocity_deg_s(axisIndex), ...
                    limits.maxAcceleration_deg_s2(axisIndex), ...
                    limits.maxJerk_deg_s3(axisIndex));
                physicalRoom_deg = min(physicalRoom_deg, phaseRoom_deg);
            end
            maximumMagnitude_deg = max(0, min(workspaceRoom_deg, physicalRoom_deg));
            report.MaximumMagnitude_deg = maximumMagnitude_deg;
            report.Eligible = maximumMagnitude_deg > boundaryResolution_deg;
            if ~report.Eligible
                report.TerminationReason = "noExcursionRoom";
                axisReports(reportIndex) = report;
                continue;
            end
            lowerMagnitude_deg = 0;
            upperMagnitude_deg = NaN;
            upperCandidate = struct();
            trialCandidates = cell(coarseLevelCount, 1);
            trialMagnitudes_deg = zeros(coarseLevelCount, 1);
            for levelIndex = 1:coarseLevelCount
                magnitude_deg = maximumMagnitude_deg * levelIndex / coarseLevelCount;
                trialCandidates{levelIndex} = createExcursion( ...
                    directCandidate, direction * magnitude_deg, ...
                    axisIndex, peakTime_s(peakIndex), initialState, options);
                trialMagnitudes_deg(levelIndex) = magnitude_deg;
            end
            sampledClear = sampledCandidatesAreClear( ...
                trialCandidates, obstacles, options);
            diagnostics.ScreeningCount = diagnostics.ScreeningCount + ...
                coarseLevelCount;
            passingLevel = find(sampledClear, 1, "first");
            if ~isempty(passingLevel)
                upperMagnitude_deg = trialMagnitudes_deg(passingLevel);
                upperCandidate = trialCandidates{passingLevel};
                if passingLevel > 1
                    lowerMagnitude_deg = trialMagnitudes_deg(passingLevel - 1);
                end
            end
            if ~isfinite(upperMagnitude_deg)
                report.TerminationReason = "noPassingAmplitudeBracket";
                axisReports(reportIndex) = report;
                continue;
            end
            report.InvalidBoundaryMagnitude_deg = lowerMagnitude_deg;
            report.ValidBoundaryMagnitude_deg = upperMagnitude_deg;
            report.BoundaryResolutionReserve_deg = ...
                upperMagnitude_deg - lowerMagnitude_deg;
            report.RetainedAmplitude_deg = direction * upperMagnitude_deg;
            report.MotionLength_deg = upperCandidate.MotionLength_deg;
            report.TerminationReason = "sampledClearancePassed";
            screenedCandidates{reportIndex} = upperCandidate;
            axisReports(reportIndex) = report;
        end
    end
end

%% Section 4: Return The Shortest Independently Passing Excursion

diagnostics.AxisReports = axisReports;
hasScreenedCandidate = false(size(screenedCandidates));
for candidateIndex = 1:numel(screenedCandidates)
    hasScreenedCandidate(candidateIndex) = ...
        ~isempty(screenedCandidates{candidateIndex});
end
screenedReportIndex = find(hasScreenedCandidate);
if ~isempty(screenedReportIndex)
    [~, order] = sort([axisReports(screenedReportIndex).MotionLength_deg]);
    for candidateIndex = reshape(screenedReportIndex(order), 1, [])
        trial = screenedCandidates{candidateIndex};
        validation = obstacleAvoidance.validateTrajectory( ...
            trial, obstacles, initialState, goalState, limits, options);
        diagnostics.ValidationCount = diagnostics.ValidationCount + 1;
        axisReports(candidateIndex).Validation = validation;
        if validation.Passed
            trial.Validation = validation;
            candidate = trial;
            bestReportIndex = candidateIndex;
            axisReports(candidateIndex).TerminationReason = ...
                "validatedFeasibleBoundary";
            break;
        end
        axisReports(candidateIndex).TerminationReason = ...
            "independentValidationFailed";
    end
end
diagnostics.AxisReports = axisReports;
if bestReportIndex == 0
    diagnostics = finishFailure(diagnostics, "noValidatedExcursion", ...
        "No enumerated fixed-clock excursion passed independent validation.", timer);
    diagnostics.AxisReports = axisReports;
    return;
end
selectedReport = axisReports(bestReportIndex);
diagnostics.Success = true;
diagnostics.SelectedMode = "singleAmplitude";
diagnostics.Message = "A shortest enumerated fixed-clock excursion passed independent validation.";
diagnostics.TerminationReason = "goalReached";
diagnostics.SelectedAxisIndex = selectedReport.AxisIndex;
diagnostics.SelectedDirection = selectedReport.Direction;
diagnostics.InvalidBoundaryMagnitude_deg = selectedReport.InvalidBoundaryMagnitude_deg;
diagnostics.ValidBoundaryMagnitude_deg = selectedReport.ValidBoundaryMagnitude_deg;
diagnostics.BoundaryResolutionReserve_deg = selectedReport.BoundaryResolutionReserve_deg;
diagnostics.RetainedAmplitude_deg = selectedReport.RetainedAmplitude_deg;
diagnostics.MotionLength_deg = candidate.MotionLength_deg;
diagnostics.SelectedValidation = candidate.Validation;
diagnostics.ElapsedTime_s = toc(timer);
end

%% Section 5: Local Functions

function [candidate, report] = createBarrierSequence(directCandidate, obstacles, ...
        initialState, goalState, limits, options, clockTolerance_s)
% Create an exact-clock lateral offset through a static barrier sequence.
candidate = directCandidate;
report = createBarrierReport();
if isempty(obstacles)
    report = barrierFailure(report, "noBarriers", ...
        "A barrier sequence requires at least one obstacle.");
    return;
end
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
if hasMovingGoal || obstacleAvoidance.obstacles.hasChangingHistory( ...
        obstacles, initialState.time_s, directCandidate.FinalTime_s)
    report = barrierFailure(report, "unsupportedChangingRequest", ...
        "The barrier sequence requires a fixed goal and static obstacles.");
    return;
end
axisMinimum_s = double(directCandidate.MinimumAxisDuration_s(:).');
governingAxes = find(abs(axisMinimum_s - directCandidate.MotionDuration_s) <= clockTolerance_s);
if numel(governingAxes) ~= 1
    report = barrierFailure(report, "unsupportedClockOwnership", ...
        "Exactly one coordinate must own the physical clock.");
    return;
end
progressAxisIndex = governingAxes(1);
lateralAxisIndex = 3 - progressAxisIndex;
progressDelta_deg = goalState.position_deg(progressAxisIndex) - ...
    initialState.position_deg(progressAxisIndex);
if progressDelta_deg == 0
    report = barrierFailure(report, "zeroProgress", ...
        "The clock-owning coordinate requires nonzero displacement.");
    return;
end
progressDirection = sign(progressDelta_deg);
workspaceInterval_deg = [double(limits.azimuthInterval_deg(:).'); ...
    double(limits.elevationInterval_deg(:).')];
lateralInterval_deg = workspaceInterval_deg(lateralAxisIndex, :);
workspaceScale_deg = diff(lateralInterval_deg);
clearanceTolerance_deg = double(options.CollisionClearanceTolerance_deg);
% The relative reserve is the cube root of the relative validation tolerance:
% guard^3 = tolerance * workspaceScale^2. It tightens with the validator,
% has position units, and remains separated from a floating tangency. One
% scale-sized ULP makes the validator's strict clearance test representable.
strictClearance_deg = clearanceTolerance_deg + eps(max(1, workspaceScale_deg));
guard_deg = max(strictClearance_deg, ...
    nthroot(clearanceTolerance_deg * workspaceScale_deg ^ 2, 3));
barrierCount = numel(obstacles);
projection_deg = zeros(barrierCount, 2);
targetLateral_deg = zeros(barrierCount, 1);
for obstacleIndex = 1:barrierCount
    obstacle = obstacles(obstacleIndex);
    boundary_deg = [obstacle.az_deg{1}, obstacle.el_deg{1}];
    if size(boundary_deg, 1) < 3 || any(~isfinite(boundary_deg), "all") || ...
            any(string(obstacle.status) ~= "visible")
        report = barrierFailure(report, "unsupportedBoundary", ...
            "Every barrier requires one finite, fully active boundary ring.");
        return;
    end
    progress_deg = boundary_deg(:, progressAxisIndex);
    lateral_deg = boundary_deg(:, lateralAxisIndex);
    projection_deg(obstacleIndex, :) = [min(progress_deg), max(progress_deg)];
    coordinateTolerance_deg = sqrt(eps) * max(1, max(abs(boundary_deg), [], "all"));
    touchesLower = min(lateral_deg) <= lateralInterval_deg(1) + coordinateTolerance_deg;
    touchesUpper = max(lateral_deg) >= lateralInterval_deg(2) - coordinateTolerance_deg;
    if touchesUpper && ~touchesLower
        targetLateral_deg(obstacleIndex) = min(lateral_deg) - guard_deg;
    elseif touchesLower && ~touchesUpper
        targetLateral_deg(obstacleIndex) = max(lateral_deg) + guard_deg;
    else
        report = barrierFailure(report, "unsupportedBarrierAttachment", ...
            "Each barrier must attach to exactly one lateral workspace side.");
        return;
    end
    if targetLateral_deg(obstacleIndex) <= lateralInterval_deg(1) || ...
            targetLateral_deg(obstacleIndex) >= lateralInterval_deg(2)
        report = barrierFailure(report, "insufficientLateralRoom", ...
            "A protected side target lies outside the lateral workspace.");
        return;
    end
end
[~, order] = sort(progressDirection * mean(projection_deg, 2), "ascend");
projection_deg = projection_deg(order, :);
targetLateral_deg = targetLateral_deg(order);
if progressDirection > 0
    projectionsOverlap = any(projection_deg(2:end, 1) <= projection_deg(1:end - 1, 2));
else
    projectionsOverlap = any(projection_deg(2:end, 2) >= projection_deg(1:end - 1, 1));
end
progressRange_deg = sort([initialState.position_deg(progressAxisIndex), ...
    goalState.position_deg(progressAxisIndex)]);
if projectionsOverlap || any(projection_deg(:, 1) <= progressRange_deg(1)) || ...
        any(projection_deg(:, 2) >= progressRange_deg(2))
    report = barrierFailure(report, "unsupportedProjectionOrder", ...
        "Barrier projections must be interior, disjoint, and progress ordered.");
    return;
end
knotTime_s = zeros(2 * barrierCount + 2, 1);
knotOffset_deg = zeros(size(knotTime_s));
knotTime_s([1, end]) = [initialState.time_s; directCandidate.FinalTime_s];
for barrierIndex = 1:barrierCount
    face_deg = projection_deg(barrierIndex, :);
    if progressDirection < 0
        face_deg = fliplr(face_deg);
    end
    rows = 2 * barrierIndex:2 * barrierIndex + 1;
    for faceIndex = 1:2
        knotTime_s(rows(faceIndex)) = invertProgress( ...
            directCandidate.Polynomial, progressAxisIndex, ...
            face_deg(faceIndex), progressDirection);
    end
    [~, directPosition_deg] = ...
        bmtpEngine.evaluatePolynomial( ...
        directCandidate.Polynomial, knotTime_s(rows));
    knotOffset_deg(rows) = targetLateral_deg(barrierIndex) - ...
        directPosition_deg(:, lateralAxisIndex);
end
if any(diff(knotTime_s) <= clockTolerance_s)
    report = barrierFailure(report, "nonIncreasingKnotTime", ...
        "Progress inversion did not produce increasing barrier times.");
    return;
end
candidate = bmtpEngine.createOffsetSplineMotion( ...
    directCandidate, knotTime_s, knotOffset_deg, lateralAxisIndex, ...
    initialState, options.SampleTime_s, "fixedClockBarrierSequence");
report.CandidateCreated = true;
report.Message = "An input-driven fixed-clock barrier sequence was created.";
report.TerminationReason = "candidateCreated";
report.ProgressAxisIndex = progressAxisIndex;
report.LateralAxisIndex = lateralAxisIndex;
report.BarrierCount = barrierCount;
report.Guard_deg = guard_deg;
report.Projection_deg = projection_deg;
report.TargetLateral_deg = targetLateral_deg;
report.KnotTime_s = knotTime_s;
report.KnotOffset_deg = knotOffset_deg;
end

function crossingTime_s = invertProgress( ...
        polynomial, axisIndex, target_deg, direction)
% Invert one monotone direct coordinate by deterministic bisection.
lowerTime_s = polynomial.SegmentStartTime_s(1);
upperTime_s = polynomial.FinalTime_s;
for iterationIndex = 1:80
    crossingTime_s = 0.5 * (lowerTime_s + upperTime_s);
[~, position_deg] = bmtpEngine.evaluatePolynomial( ...
        polynomial, crossingTime_s);
    if direction * (position_deg(axisIndex) - target_deg) >= 0
        upperTime_s = crossingTime_s;
    else
        lowerTime_s = crossingTime_s;
    end
end
crossingTime_s = 0.5 * (lowerTime_s + upperTime_s);
end

function candidate = createExcursion( ...
        directCandidate, amplitude_deg, axisIndex, peakTime_s, ...
        initialState, options)
% Add a minimum-jerk lobe whose interior velocity and acceleration stay free.
% Forcing both derivatives to zero at the peak creates an artificial dwell
% that can intersect a broad obstacle even when a smoother fixed-clock lobe
% has ample physical margin. The unchanged validator remains authoritative.
startTime_s = initialState.time_s;
endTime_s = directCandidate.FinalTime_s;
candidate = bmtpEngine.createOffsetSplineMotion( ...
    directCandidate, [startTime_s; peakTime_s; endTime_s], ...
    [0; amplitude_deg; 0], axisIndex, initialState, ...
    options.SampleTime_s, "fixedClockLateralExcursion");
end

function peakTime_s = createPeakTimeCandidates(directCandidate, obstacles, options)
% Derive shifted excursion peaks from direct-path collision intervals.
startTime_s = directCandidate.time_s(1);
endTime_s = directCandidate.time_s(end);
midpointTime_s = 0.5 * (startTime_s + endTime_s);
peakTime_s = midpointTime_s;
if ~obstacleAvoidance.obstacles.hasChangingHistory( ...
        obstacles, startTime_s, endTime_s)
    return;
end
queryOptions = struct( ...
    "BoundaryIsOccupied", true, ...
    "ClearanceTolerance_deg", options.CollisionClearanceTolerance_deg);
[isOccupied, ~, details] = ...
    obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacles, directCandidate.position_deg(:, 1), ...
    directCandidate.position_deg(:, 2), directCandidate.time_s, queryOptions);
isOccupied = logical(isOccupied(:));
runChange = diff([false; isOccupied; false]);
runStart = find(runChange == 1);
runEnd = find(runChange == -1) - 1;
collisionPeak_s = zeros(2 * numel(runStart), 1);
for runIndex = 1:numel(runStart)
    indices = runStart(runIndex):runEnd(runIndex);
    [~, localIndex] = min(details.MinimumClearance_deg(indices));
    collisionPeak_s(2 * runIndex - 1) = ...
        directCandidate.time_s(indices(localIndex));
    collisionPeak_s(2 * runIndex) = 0.5 * sum( ...
        directCandidate.time_s([indices(1), indices(end)]));
end
peakTime_s = unique([collisionPeak_s; midpointTime_s], "stable");
endpointReserve_s = 256 * eps(max(1, endTime_s - startTime_s));
peakTime_s = peakTime_s(peakTime_s > startTime_s + endpointReserve_s & ...
    peakTime_s < endTime_s - endpointReserve_s);
minimumPeakSeparation_s = max(endpointReserve_s, 0.5 * options.SampleTime_s);
retainedPeak = false(size(peakTime_s));
for peakIndex = 1:numel(peakTime_s)
    retainedPeak(peakIndex) = ~any(abs(peakTime_s(1:peakIndex - 1) - ...
        peakTime_s(peakIndex)) < minimumPeakSeparation_s & ...
        retainedPeak(1:peakIndex - 1));
end
peakTime_s = peakTime_s(retainedPeak);
end

function valid = directMotionIsPhysical(validation)
% Exclude collision fields while requiring every other authoritative gate.
allowedIssues = ["collision freedom", "collision resolution"];
valid = all(ismember(validation.Issues, allowedIssues));
end

function isClear = sampledCandidatesAreClear(candidates, obstacles, options)
% Batch equal-time sampled histories so each moving shape is evaluated once.
candidateCount = numel(candidates);
sampleCount = numel(candidates{1}.time_s);
azimuth_deg = zeros(sampleCount, candidateCount);
elevation_deg = zeros(sampleCount, candidateCount);
time_s = repmat(candidates{1}.time_s, 1, candidateCount);
for candidateIndex = 1:candidateCount
    azimuth_deg(:, candidateIndex) = candidates{candidateIndex}.position_deg(:, 1);
    elevation_deg(:, candidateIndex) = candidates{candidateIndex}.position_deg(:, 2);
end
queryOptions = struct( ...
    "BoundaryIsOccupied", true, ...
    "ClearanceTolerance_deg", options.CollisionClearanceTolerance_deg);
isOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
    obstacles, azimuth_deg, elevation_deg, time_s, queryOptions);
isClear = ~any(isOccupied, 1);
end

function report = createBarrierReport()
% Define stable barrier eligibility, construction, and validation evidence.
report = struct( ...
    "CandidateCreated", false, ...
    "Message", "The fixed-clock barrier sequence was not attempted.", ...
    "TerminationReason", "notAttempted", ...
    "ProgressAxisIndex", 0, "LateralAxisIndex", 0, ...
    "BarrierCount", 0, "Guard_deg", NaN, ...
    "Projection_deg", zeros(0, 2), ...
    "TargetLateral_deg", zeros(0, 1), ...
    "KnotTime_s", zeros(0, 1), "KnotOffset_deg", zeros(0, 1), ...
    "Validation", obstacleAvoidance.validateTrajectory());
end

function report = barrierFailure(report, reason, message)
% Preserve an explicit fail-closed barrier classifier or construction result.
report.TerminationReason = reason;
report.Message = message;
end

function report = createAxisReport()
% Define one stable record for every coordinate and signed direction.
report = struct( ...
    "AxisIndex", 0, "Direction", 0, "AxisGovernsClock", false, ...
    "PeakTime_s", NaN, "Eligible", false, "MaximumMagnitude_deg", 0, ...
    "InvalidBoundaryMagnitude_deg", NaN, ...
    "ValidBoundaryMagnitude_deg", NaN, ...
    "BoundaryResolutionReserve_deg", NaN, ...
    "RetainedAmplitude_deg", NaN, "MotionLength_deg", NaN, ...
    "Validation", obstacleAvoidance.validateTrajectory(), ...
    "TerminationReason", "notAttempted");
end

function diagnostics = createDiagnostics()
% Define stable success, failure, selection, and enumeration evidence.
diagnostics = struct( ...
    "Attempted", false, "Success", false, ...
    "Message", "The fixed-clock excursion was not attempted.", ...
    "TerminationReason", "notRun", "SelectedMode", "", ...
    "ClockMatched", false, "DirectDuration_s", NaN, ...
    "CertifiedLowerBound_s", NaN, "ClockTolerance_s", NaN, ...
    "ValidationCount", 0, "ScreeningCount", 0, "SelectedAxisIndex", 0, ...
    "SelectedDirection", 0, "InvalidBoundaryMagnitude_deg", NaN, ...
    "ValidBoundaryMagnitude_deg", NaN, "BoundaryResolutionReserve_deg", NaN, ...
    "RetainedAmplitude_deg", NaN, "MotionLength_deg", NaN, ...
    "DirectValidation", obstacleAvoidance.validateTrajectory(), ...
    "SelectedValidation", obstacleAvoidance.validateTrajectory(), ...
    "BarrierSequence", createBarrierReport(), ...
    "AxisReports", repmat(createAxisReport(), 0, 1), ...
    "ElapsedTime_s", 0);
end

function diagnostics = finishFailure(diagnostics, reason, message, timer)
% Preserve an explicit terminal reason and elapsed work on every failure.
diagnostics.Success = false;
diagnostics.TerminationReason = reason;
diagnostics.Message = message;
diagnostics.ElapsedTime_s = toc(timer);
end
