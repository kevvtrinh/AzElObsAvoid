function [candidate, diagnostics] = createFixedClockLateralExcursion( ...
        directCandidate, obstacles, initialState, goalState, limits, options, ...
        directValidation)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createFixedClockLateralExcursion()
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.createFixedClockLateralExcursion( ...
%       directCandidate, obstacles, initialState, goalState, limits, options, ...
%       directValidation)
%**************************************************************************
% PURPOSE
%   - Preserve a certified direct physical-minimum clock while enumerating
%     additive coordinate-axis rest-to-rest collision detours.
%   - Return the first independently validated axis excursion on that clock.
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
%   - directValidation (scalar validation record)
%       The caller's authoritative validation of directCandidate avoids
%       repeating the identical full-trajectory validation.
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
if nargin ~= 7
    error("createFixedClockLateralExcursion:InvalidCall", ...
        "Use zero inputs or all seven documented inputs.");
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
if ~isstruct(directValidation) || ~isscalar(directValidation) || ...
        ~isfield(directValidation, "Passed")
    error("createFixedClockLateralExcursion:InvalidDirectValidation", ...
        "directValidation must be the scalar public validation record.");
end
diagnostics.DirectValidation = directValidation;
if ~directMotionIsPhysical(directValidation)
    diagnostics = finishFailure(diagnostics, "directMotionInvalid", ...
        "The direct motion failed a non-collision invariant.", timer);
    return;
end

%% Section 2: Enumerate Input-Scaled Axis Excursions

workspaceInterval_deg = [double(limits.azimuthInterval_deg(:).'); ...
    double(limits.elevationInterval_deg(:).')];
coarseLevelCount = 8;
boundaryResolution_deg = max(8 * double(options.CollisionClearanceTolerance_deg), ...
    sqrt(eps) * max(1, max(abs(directCandidate.position_deg), [], "all")));
peakTime_s = createPeakTimeCandidates(directCandidate, obstacles, options);
axisReports = repmat( ...
    createAxisReport(), 2 * dimensionCount * numel(peakTime_s), 1);
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
            upperValidation = obstacleAvoidance.validateTrajectory();
            if ~isempty(passingLevel)
                if passingLevel > 1
                    lowerMagnitude_deg = trialMagnitudes_deg(passingLevel - 1);
                end
                for levelIndex = passingLevel:coarseLevelCount
                    validationTimer = tic;
                    trialValidation = obstacleAvoidance.validateTrajectory( ...
                        trialCandidates{levelIndex}, obstacles, initialState, ...
                        goalState, limits, options);
                    diagnostics = addValidationTiming( ...
                        diagnostics, trialValidation, toc(validationTimer));
                    if trialValidation.Passed
                        upperMagnitude_deg = trialMagnitudes_deg(levelIndex);
                        upperCandidate = trialCandidates{levelIndex};
                        upperValidation = trialValidation;
                        break;
                    end
                    lowerMagnitude_deg = trialMagnitudes_deg(levelIndex);
                end
            end
            if ~isfinite(upperMagnitude_deg)
                report.TerminationReason = "noPassingAmplitudeBracket";
                axisReports(reportIndex) = report;
                continue;
            end

            % Six bisections reduce one coarse interval to less than 1/512
            % of the available amplitude while avoiding near-identical full
            % validations. The retained upper endpoint is always validated.
            refinementCount = 0;
            while upperMagnitude_deg - lowerMagnitude_deg > ...
                    boundaryResolution_deg && refinementCount < 6
                midpointMagnitude_deg = ...
                    0.5 * (lowerMagnitude_deg + upperMagnitude_deg);
                midpointCandidate = createExcursion( ...
                    directCandidate, direction * midpointMagnitude_deg, ...
                    axisIndex, peakTime_s(peakIndex), initialState, options);
                validationTimer = tic;
                midpointValidation = obstacleAvoidance.validateTrajectory( ...
                    midpointCandidate, obstacles, initialState, goalState, ...
                    limits, options);
                diagnostics = addValidationTiming( ...
                    diagnostics, midpointValidation, toc(validationTimer));
                refinementCount = refinementCount + 1;
                if midpointValidation.Passed
                    upperMagnitude_deg = midpointMagnitude_deg;
                    upperCandidate = midpointCandidate;
                    upperValidation = midpointValidation;
                else
                    lowerMagnitude_deg = midpointMagnitude_deg;
                end
            end
            upperCandidate.Validation = upperValidation;
            report.InvalidBoundaryMagnitude_deg = lowerMagnitude_deg;
            report.ValidBoundaryMagnitude_deg = upperMagnitude_deg;
            report.BoundaryResolutionReserve_deg = ...
                upperMagnitude_deg - lowerMagnitude_deg;
            report.BoundaryRefinementCount = refinementCount;
            report.RetainedAmplitude_deg = direction * upperMagnitude_deg;
            report.MotionLength_deg = upperCandidate.MotionLength_deg;
            report.Validation = upperValidation;
            report.TerminationReason = "validatedFeasibleBoundary";
            axisReports(reportIndex) = report;

            candidate = upperCandidate;
            diagnostics.AxisReports = axisReports;
            diagnostics.Success = true;
            diagnostics.TerminationReason = "goalReached";
            diagnostics.SelectedMode = "singleAmplitude";
            diagnostics.Message = ...
                "A one-sided fixed-clock excursion passed independent validation.";
            diagnostics.SelectedAxisIndex = report.AxisIndex;
            diagnostics.SelectedDirection = report.Direction;
            diagnostics.InvalidBoundaryMagnitude_deg = ...
                report.InvalidBoundaryMagnitude_deg;
            diagnostics.ValidBoundaryMagnitude_deg = ...
                report.ValidBoundaryMagnitude_deg;
            diagnostics.BoundaryResolutionReserve_deg = ...
                report.BoundaryResolutionReserve_deg;
            diagnostics.RetainedAmplitude_deg = report.RetainedAmplitude_deg;
            diagnostics.MotionLength_deg = candidate.MotionLength_deg;
            diagnostics.SelectedValidation = candidate.Validation;
            diagnostics.ElapsedTime_s = toc(timer);
            return;
        end
    end
end

%% Section 3: Return Failure After Exhaustive Enumeration

diagnostics.AxisReports = axisReports;
diagnostics = finishFailure(diagnostics, "noValidatedExcursion", ...
    "No enumerated fixed-clock excursion passed independent validation.", timer);
end

%% Section 4: Local Functions

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
% Preserve the retired barrier method's diagnostic shape for compatibility.
report = struct( ...
    "CandidateCreated", false, ...
    "Message", "The retired fixed-clock barrier method is unavailable.", ...
    "TerminationReason", "retiredMethod", ...
    "ProgressAxisIndex", 0, "LateralAxisIndex", 0, ...
    "BarrierCount", 0, "Guard_deg", NaN, ...
    "Projection_deg", zeros(0, 2), ...
    "TargetLateral_deg", zeros(0, 1), ...
    "KnotTime_s", zeros(0, 1), "KnotOffset_deg", zeros(0, 1), ...
    "Validation", obstacleAvoidance.validateTrajectory());
end

function report = createProgressReport()
% Preserve the retired progress method's diagnostic shape for compatibility.
report = struct( ...
    "Attempted", false, "Eligible", false, "Success", false, ...
    "Message", "The retired progress-polynomial method is unavailable.", ...
    "TerminationReason", "retiredMethod", ...
    "ProgressAxisIndex", 0, "LateralAxisIndex", 0, ...
    "LowerAmplitude_deg", NaN, "UpperAmplitude_deg", NaN, ...
    "SelectedBasis", "", ...
    "BasisReports", repmat(createProgressBasisReport(), 0, 1), ...
    "CandidateAmplitude_deg", zeros(0, 1), ...
    "CandidateBasisIndex", zeros(0, 1), ...
    "MotionLength_deg", zeros(0, 1), ...
    "SampledClear", false(0, 1), ...
    "ValidationPassed", false(0, 1), ...
    "ValidationMessage", strings(0, 1), ...
    "ScreeningCount", 0, "ValidationCount", 0, ...
    "ValidationElapsedTime_s", 0, "CollisionCheckingElapsedTime_s", 0, ...
    "SelectedAmplitude_deg", NaN, "SelectedMotionLength_deg", NaN, ...
    "Validation", obstacleAvoidance.validateTrajectory());
end

function report = createProgressBasisReport()
% Preserve physical bounds and the best validated result for one basis.
report = struct( ...
    "Name", "", "Power", zeros(1, 0), "PeakProgress", NaN, ...
    "LowerAmplitude_deg", NaN, "UpperAmplitude_deg", NaN, ...
    "SelectedAmplitude_deg", NaN, "SelectedMotionLength_deg", NaN, ...
    "BoundaryRefinementCount", 0, "TerminationReason", "notAttempted");
end

function report = createAxisReport()
% Define one stable record for every coordinate and signed direction.
report = struct( ...
    "AxisIndex", 0, "Direction", 0, "AxisGovernsClock", false, ...
    "PeakTime_s", NaN, "Eligible", false, "MaximumMagnitude_deg", 0, ...
    "InvalidBoundaryMagnitude_deg", NaN, ...
    "ValidBoundaryMagnitude_deg", NaN, ...
    "BoundaryResolutionReserve_deg", NaN, "BoundaryRefinementCount", 0, ...
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
    "ProgressPolynomial", createProgressReport(), ...
    "AxisReports", repmat(createAxisReport(), 0, 1), ...
    "ValidationElapsedTime_s", 0, "CollisionCheckingElapsedTime_s", 0, ...
    "ElapsedTime_s", 0);
end

function diagnostics = addValidationTiming(diagnostics, validation, elapsedTime_s)
% Keep nested public-validation work separable from construction time.
diagnostics.ValidationCount = diagnostics.ValidationCount + 1;
diagnostics.ValidationElapsedTime_s = diagnostics.ValidationElapsedTime_s + ...
    elapsedTime_s;
diagnostics.CollisionCheckingElapsedTime_s = ...
    diagnostics.CollisionCheckingElapsedTime_s + ...
    validation.CollisionCheckingElapsedTime_s;
end

function diagnostics = finishFailure(diagnostics, reason, message, timer)
% Preserve an explicit terminal reason and elapsed work on every failure.
diagnostics.Success = false;
diagnostics.TerminationReason = reason;
diagnostics.Message = message;
diagnostics.ElapsedTime_s = toc(timer);
end
