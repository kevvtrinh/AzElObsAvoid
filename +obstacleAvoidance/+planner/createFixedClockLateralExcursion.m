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
%   - On a static sequence of boundary-attached barriers, construct an
%     input-driven minimum-jerk lateral spline on that same exact clock.
%   - Compare an endpoint-flat alternating progress polynomial with one-sided
%     input-derived excursions and retain the shortest validated exact-clock
%     motion.
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

%% Section 2: Try A Static Boundary-Attached Barrier Sequence

[sequenceCandidate, sequenceReport] = createBarrierSequence( ...
    directCandidate, obstacles, initialState, goalState, limits, options, clockTolerance_s);
diagnostics.BarrierSequence = sequenceReport;
if sequenceReport.CandidateCreated
    validationTimer = tic;
    sequenceValidation = obstacleAvoidance.validateTrajectory( ...
        sequenceCandidate, obstacles, initialState, goalState, limits, options);
    diagnostics = addValidationTiming(diagnostics, sequenceValidation, ...
        toc(validationTimer));
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

%% Section 3: Try Input-Derived Progress Polynomials

[progressCandidate, progressReport] = createAlternatingProgressExcursion( ...
    directCandidate, obstacles, initialState, goalState, limits, ...
    options, clockTolerance_s);
diagnostics.ProgressPolynomial = progressReport;
diagnostics.ScreeningCount = diagnostics.ScreeningCount + ...
    progressReport.ScreeningCount;
diagnostics.ValidationCount = diagnostics.ValidationCount + ...
    progressReport.ValidationCount;
diagnostics.ValidationElapsedTime_s = diagnostics.ValidationElapsedTime_s + ...
    progressReport.ValidationElapsedTime_s;
diagnostics.CollisionCheckingElapsedTime_s = ...
    diagnostics.CollisionCheckingElapsedTime_s + ...
    progressReport.CollisionCheckingElapsedTime_s;

%% Section 4: Enumerate Input-Scaled Axis Excursions

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

            % The coarse sweep only proves a failing/passing bracket. Using
            % its first passing level directly can retain almost one eighth
            % of the available lateral range as needless joint travel. Refine
            % the same input-derived bracket until geometry-scale resolution.
            % Full validation owns each boundary decision because sampled
            % clearance can miss a collision between adjacent history points.
            refinementCount = 0;
            while upperMagnitude_deg - lowerMagnitude_deg > ...
                    boundaryResolution_deg
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
            report.TerminationReason = "validatedClearanceBoundary";
            screenedCandidates{reportIndex} = upperCandidate;
            axisReports(reportIndex) = report;
        end
    end
end

%% Section 5: Return The Shortest Independently Passing Excursion

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
        validationTimer = tic;
        validation = obstacleAvoidance.validateTrajectory( ...
            trial, obstacles, initialState, goalState, limits, options);
        diagnostics = addValidationTiming(diagnostics, validation, ...
            toc(validationTimer));
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
if bestReportIndex == 0 && ~progressReport.Success
    diagnostics = finishFailure(diagnostics, "noValidatedExcursion", ...
        "No enumerated fixed-clock excursion passed independent validation.", timer);
    diagnostics.AxisReports = axisReports;
    return;
end
diagnostics.Success = true;
diagnostics.TerminationReason = "goalReached";
selectProgress = progressReport.Success && (bestReportIndex == 0 || ...
    progressCandidate.MotionLength_deg <= candidate.MotionLength_deg);
if selectProgress
    candidate = progressCandidate;
    diagnostics.SelectedMode = "progressPolynomial";
    diagnostics.Message = "The shortest validated fixed-clock family was " + ...
        "the " + progressReport.SelectedBasis + " progress polynomial.";
    diagnostics.SelectedAxisIndex = progressReport.LateralAxisIndex;
    diagnostics.SelectedDirection = sign(progressReport.SelectedAmplitude_deg);
    diagnostics.RetainedAmplitude_deg = progressReport.SelectedAmplitude_deg;
else
    selectedReport = axisReports(bestReportIndex);
    diagnostics.SelectedMode = "singleAmplitude";
    diagnostics.Message = ...
        "The shortest validated fixed-clock family was a one-sided excursion.";
    diagnostics.SelectedAxisIndex = selectedReport.AxisIndex;
    diagnostics.SelectedDirection = selectedReport.Direction;
    diagnostics.InvalidBoundaryMagnitude_deg = ...
        selectedReport.InvalidBoundaryMagnitude_deg;
    diagnostics.ValidBoundaryMagnitude_deg = ...
        selectedReport.ValidBoundaryMagnitude_deg;
    diagnostics.BoundaryResolutionReserve_deg = ...
        selectedReport.BoundaryResolutionReserve_deg;
    diagnostics.RetainedAmplitude_deg = selectedReport.RetainedAmplitude_deg;
end
diagnostics.MotionLength_deg = candidate.MotionLength_deg;
diagnostics.SelectedValidation = candidate.Validation;
diagnostics.ElapsedTime_s = toc(timer);
end

%% Section 6: Local Functions

function [candidate, report] = createAlternatingProgressExcursion( ...
        directCandidate, obstacles, initialState, goalState, limits, ...
        options, clockTolerance_s)
% Search a bounded endpoint-flat family on an exact direct progress clock.
candidate = directCandidate;
report = createProgressReport();
report.Attempted = true;
hasMovingGoal = isfield(goalState, "targetTime_s") && ...
    ~isempty(goalState.targetTime_s);
hasStraightProgress = isfield(directCandidate, "UsedStraightProgress") && ...
    directCandidate.UsedStraightProgress;
hasRestEndpoints = max(abs([initialState.velocity_deg_s, ...
    goalState.velocity_deg_s, initialState.acceleration_deg_s2, ...
    goalState.acceleration_deg_s2])) <= clockTolerance_s;
if hasMovingGoal || obstacleAvoidance.obstacles.hasChangingHistory( ...
        obstacles, initialState.time_s, directCandidate.FinalTime_s)
    report = progressFailure(report, "unsupportedChangingRequest", ...
        "The progress polynomial requires a fixed goal and static obstacles.");
    return;
end
if ~hasStraightProgress || ~hasRestEndpoints
    report = progressFailure(report, "unsupportedBaseMotion", ...
        "A straight-progress motion with rest endpoints is required.");
    return;
end
if isfield(options, "AllowAzimuthWrapping") && options.AllowAzimuthWrapping
    report = progressFailure(report, "unsupportedAzimuthWrapping", ...
        "The progress polynomial requires an unwrapped workspace.");
    return;
end
axisMinimum_s = double(directCandidate.MinimumAxisDuration_s(:).');
governingAxes = find(abs(axisMinimum_s - ...
    directCandidate.MotionDuration_s) <= clockTolerance_s);
if numel(governingAxes) ~= 1
    report = progressFailure(report, "unsupportedClockOwnership", ...
        "Exactly one coordinate must own the physical clock.");
    return;
end
progressAxisIndex = governingAxes(1);
lateralAxisIndex = 3 - progressAxisIndex;
progressDelta_deg = goalState.position_deg(progressAxisIndex) - ...
    initialState.position_deg(progressAxisIndex);
if progressDelta_deg == 0
    report = progressFailure(report, "zeroProgress", ...
        "The clock-owning coordinate requires nonzero displacement.");
    return;
end
report.Eligible = true;
report.ProgressAxisIndex = progressAxisIndex;
report.LateralAxisIndex = lateralAxisIndex;

progressBases = createProgressBases( ...
    directCandidate, obstacles, progressAxisIndex, options);
basisCount = numel(progressBases);
basisReports = repmat(createProgressBasisReport(), basisCount, 1);
levelCount = 8;
levelFraction = (1:levelCount).' / levelCount;
trialCandidates = cell(0, 1);
candidateAmplitude_deg = zeros(0, 1);
candidateBasisIndex = zeros(0, 1);
motionLength_deg = zeros(0, 1);
for basisIndex = 1:basisCount
    basisPower = progressBases(basisIndex).Power;
    unitCandidate = bmtpEngine.createProgressPolynomialMotion( ...
        directCandidate, initialState, goalState, progressAxisIndex, 1, ...
        options.SampleTime_s, "fixedClockProgressPolynomialUnit", ...
        basisPower);
    [lowerAmplitude_deg, upperAmplitude_deg] = ...
        createContinuousAmplitudeInterval( ...
        directCandidate, unitCandidate, lateralAxisIndex, limits);
    basisReports(basisIndex).Name = progressBases(basisIndex).Name;
    basisReports(basisIndex).Power = basisPower;
    basisReports(basisIndex).PeakProgress = ...
        progressBases(basisIndex).PeakProgress;
    basisReports(basisIndex).LowerAmplitude_deg = lowerAmplitude_deg;
    basisReports(basisIndex).UpperAmplitude_deg = upperAmplitude_deg;
    intervalIsBounded = isfinite(lowerAmplitude_deg) && ...
        isfinite(upperAmplitude_deg) && lowerAmplitude_deg < 0 && ...
        upperAmplitude_deg > 0;
    if ~intervalIsBounded
        basisReports(basisIndex).TerminationReason = ...
            "noBoundedPhysicalAmplitude";
        continue;
    end
    amplitudes_deg = [ ...
        -(-lowerAmplitude_deg) * levelFraction; ...
        upperAmplitude_deg * levelFraction];
    for levelIndex = 1:numel(amplitudes_deg)
        trial = bmtpEngine.createProgressPolynomialMotion( ...
            directCandidate, initialState, goalState, progressAxisIndex, ...
            amplitudes_deg(levelIndex), options.SampleTime_s, ...
            "fixedClockProgressPolynomial", basisPower);
        trialCandidates{end + 1, 1} = trial; %#ok<AGROW>
        candidateAmplitude_deg(end + 1, 1) = ...
            amplitudes_deg(levelIndex); %#ok<AGROW>
        candidateBasisIndex(end + 1, 1) = basisIndex; %#ok<AGROW>
        motionLength_deg(end + 1, 1) = trial.MotionLength_deg; %#ok<AGROW>
    end
end
workspaceInterval_deg = [double(limits.azimuthInterval_deg(:).'); ...
    double(limits.elevationInterval_deg(:).')];
workspaceScale_deg = diff(workspaceInterval_deg(lateralAxisIndex, :));
amplitudeResolution_deg = max( ...
    8 * double(options.CollisionClearanceTolerance_deg), ...
    sqrt(eps) * max(1, workspaceScale_deg));
candidateCount = numel(trialCandidates);
report.BasisReports = basisReports;
report.CandidateAmplitude_deg = candidateAmplitude_deg;
report.CandidateBasisIndex = candidateBasisIndex;
if candidateCount == 0
    report = progressFailure(report, "noResolvedAmplitude", ...
        "No progress basis retained a bounded physical amplitude interval.");
    return;
end
sampledClear = sampledCandidatesAreClear( ...
    trialCandidates, obstacles, options);
sampledClear = logical(sampledClear(:));
report.MotionLength_deg = motionLength_deg;
report.SampledClear = sampledClear;
report.ScreeningCount = candidateCount;
report.ValidationPassed = false(candidateCount, 1);
report.ValidationMessage = strings(candidateCount, 1);

bestCandidate = struct();
bestValidation = obstacleAvoidance.validateTrajectory();
bestAmplitude_deg = NaN;
bestBasisIndex = 0;
for basisIndex = 1:basisCount
    screenedIndex = find(sampledClear & candidateBasisIndex == basisIndex);
    if isempty(screenedIndex)
        basisReports(basisIndex).TerminationReason = ...
            "noSampledClearAmplitude";
        continue;
    end
    [~, order] = sort(motionLength_deg(screenedIndex));
    selectedIndex = 0;
    for candidateIndex = reshape(screenedIndex(order), 1, [])
        trial = trialCandidates{candidateIndex};
        validationTimer = tic;
        validation = obstacleAvoidance.validateTrajectory( ...
            trial, obstacles, initialState, goalState, limits, options);
        report = addProgressValidationTiming(report, validation, ...
            toc(validationTimer));
        report.ValidationPassed(candidateIndex) = validation.Passed;
        report.ValidationMessage(candidateIndex) = validation.Message;
        if validation.Passed
            selectedIndex = candidateIndex;
            break;
        end
    end
    if selectedIndex == 0
        basisReports(basisIndex).TerminationReason = ...
            "noValidatedAmplitude";
        continue;
    end
    upperMagnitude_deg = abs(candidateAmplitude_deg(selectedIndex));
    direction = sign(candidateAmplitude_deg(selectedIndex));
    sameSide = candidateBasisIndex == basisIndex & ...
        sign(candidateAmplitude_deg) == direction & ...
        abs(candidateAmplitude_deg) < upperMagnitude_deg;
    lowerMagnitude_deg = max([0; abs(candidateAmplitude_deg( ...
        sameSide & ~sampledClear))]);
    refinedCandidate = trialCandidates{selectedIndex};
    refinedValidation = validation;
    refinementCount = 0;
    while upperMagnitude_deg - lowerMagnitude_deg > ...
            amplitudeResolution_deg && refinementCount < 16
        midpointMagnitude_deg = ...
            0.5 * (lowerMagnitude_deg + upperMagnitude_deg);
        midpointCandidate = bmtpEngine.createProgressPolynomialMotion( ...
            directCandidate, initialState, goalState, progressAxisIndex, ...
            direction * midpointMagnitude_deg, options.SampleTime_s, ...
            "fixedClockProgressPolynomial", ...
            progressBases(basisIndex).Power);
        validationTimer = tic;
        midpointValidation = obstacleAvoidance.validateTrajectory( ...
            midpointCandidate, obstacles, initialState, goalState, ...
            limits, options);
        report = addProgressValidationTiming(report, midpointValidation, ...
            toc(validationTimer));
        refinementCount = refinementCount + 1;
        if midpointValidation.Passed
            upperMagnitude_deg = midpointMagnitude_deg;
            refinedCandidate = midpointCandidate;
            refinedValidation = midpointValidation;
        else
            lowerMagnitude_deg = midpointMagnitude_deg;
        end
    end
    basisReports(basisIndex).SelectedAmplitude_deg = ...
        direction * upperMagnitude_deg;
    basisReports(basisIndex).SelectedMotionLength_deg = ...
        refinedCandidate.MotionLength_deg;
    basisReports(basisIndex).BoundaryRefinementCount = refinementCount;
    basisReports(basisIndex).TerminationReason = "validatedClearanceBoundary";
    if bestBasisIndex == 0 || refinedCandidate.MotionLength_deg < ...
            bestCandidate.MotionLength_deg
        bestCandidate = refinedCandidate;
        bestValidation = refinedValidation;
        bestAmplitude_deg = direction * upperMagnitude_deg;
        bestBasisIndex = basisIndex;
    end
end
report.BasisReports = basisReports;
if bestBasisIndex == 0
    report = progressFailure(report, "noValidatedProgressPolynomial", ...
        "No sampled-clear progress polynomial passed independent validation.");
    return;
end
bestCandidate.Validation = bestValidation;
candidate = bestCandidate;
report.Success = true;
report.Message = "A bounded progress polynomial passed independent validation.";
report.TerminationReason = "goalReached";
report.SelectedBasis = progressBases(bestBasisIndex).Name;
report.LowerAmplitude_deg = ...
    basisReports(bestBasisIndex).LowerAmplitude_deg;
report.UpperAmplitude_deg = ...
    basisReports(bestBasisIndex).UpperAmplitude_deg;
report.SelectedAmplitude_deg = bestAmplitude_deg;
report.SelectedMotionLength_deg = bestCandidate.MotionLength_deg;
report.Validation = bestValidation;
end

function progressBases = createProgressBases( ...
        directCandidate, obstacles, progressAxisIndex, options)
% Derive asymmetric one-sided bases from direct collision progress.
progressBases = struct( ...
    "Name", "alternatingDegreeFive", ...
    "Power", [0, 4, 4, -56, 80, -32], ...
    "PeakProgress", NaN);
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
progressDelta_deg = directCandidate.position_deg(end, progressAxisIndex) - ...
    directCandidate.position_deg(1, progressAxisIndex);
collisionProgress = zeros(numel(runStart), 1);
for runIndex = 1:numel(runStart)
    indices = runStart(runIndex):runEnd(runIndex);
    [~, localIndex] = min(details.MinimumClearance_deg(indices));
    sampleIndex = indices(localIndex);
    collisionProgress(runIndex) = ( ...
        directCandidate.position_deg(sampleIndex, progressAxisIndex) - ...
        directCandidate.position_deg(1, progressAxisIndex)) / ...
        progressDelta_deg;
end
if isempty(collisionProgress)
    % A continuous collision can lie between history samples. Symmetric
    % progress keeps the family useful without inventing obstacle geometry.
    collisionProgress = 0.5;
end
collisionProgress = min(1 - eps, max(eps, collisionProgress));
for collisionIndex = 1:numel(collisionProgress)
    for degree = 3:6
        firstExponent = min(degree - 1, max(1, ...
            round(degree * collisionProgress(collisionIndex))));
        secondExponent = degree - firstExponent;
        basisName = "oneSidedBeta_" + firstExponent + "_" + secondExponent;
        if any([progressBases.Name] == basisName)
            continue;
        end
        basisPower = zeros(1, degree + 1);
        for secondPower = 0:secondExponent
            basisPower(firstExponent + secondPower + 1) = ...
                (-1) ^ secondPower * nchoosek(secondExponent, secondPower);
        end
        peakProgress = firstExponent / degree;
        peakValue = peakProgress ^ firstExponent * ...
            (1 - peakProgress) ^ secondExponent;
        basisPower = basisPower / peakValue;
        progressBases(end + 1) = struct( ...
            "Name", basisName, "Power", basisPower, ...
            "PeakProgress", peakProgress); %#ok<AGROW>
    end
end
end

function [lowerAmplitude_deg, upperAmplitude_deg] = ...
        createContinuousAmplitudeInterval( ...
        baseCandidate, unitCandidate, lateralAxisIndex, limits)
% Intersect sufficient Bernstein workspace and P/V/A/J amplitude bounds.
fieldNames = {'positionPower_deg', 'velocityPower_deg_s', ...
    'accelerationPower_deg_s2', 'jerkPower_deg_s3'};
workspaceInterval_deg = [double(limits.azimuthInterval_deg(:).'); ...
    double(limits.elevationInterval_deg(:).')];
lowerBound = [workspaceInterval_deg(lateralAxisIndex, 1), ...
    -double(limits.maxVelocity_deg_s(lateralAxisIndex)), ...
    -double(limits.maxAcceleration_deg_s2(lateralAxisIndex)), ...
    -double(limits.maxJerk_deg_s3(lateralAxisIndex))];
upperBound = [workspaceInterval_deg(lateralAxisIndex, 2), ...
    double(limits.maxVelocity_deg_s(lateralAxisIndex)), ...
    double(limits.maxAcceleration_deg_s2(lateralAxisIndex)), ...
    double(limits.maxJerk_deg_s3(lateralAxisIndex))];
lowerAmplitude_deg = -Inf;
upperAmplitude_deg = Inf;
segmentCount = unitCandidate.Polynomial.SegmentCount;
for fieldIndex = 1:numel(fieldNames)
    fieldName = fieldNames{fieldIndex};
    unitRecords = unitCandidate.Polynomial.(fieldName);
    baseRecords = baseCandidate.Polynomial.(fieldName);
    coefficientCount = size(unitRecords, 3);
    for segmentIndex = 1:segmentCount
        unitPower = reshape( ...
            unitRecords(segmentIndex, lateralAxisIndex, :), 1, []);
        basePower = zeros(1, coefficientCount);
        originalBasePower = reshape( ...
            baseRecords(segmentIndex, lateralAxisIndex, :), 1, []);
        basePower(1:numel(originalBasePower)) = originalBasePower;
        baseControl = createMidpointBernsteinControls(basePower);
        unitControl = createMidpointBernsteinControls(unitPower);
        amplitudeControl = unitControl - baseControl;
        [lowerAmplitude_deg, upperAmplitude_deg] = intersectAmplitudeBounds( ...
            lowerAmplitude_deg, upperAmplitude_deg, baseControl, ...
            amplitudeControl, lowerBound(fieldIndex), upperBound(fieldIndex));
        if lowerAmplitude_deg >= upperAmplitude_deg
            return;
        end
    end
end
% Pull the proposal cap a few ULPs into the sufficient control-point set.
intervalScale_deg = max([1, abs(lowerAmplitude_deg), ...
    abs(upperAmplitude_deg)]);
amplitudeReserve_deg = 512 * eps(intervalScale_deg);
lowerAmplitude_deg = lowerAmplitude_deg + amplitudeReserve_deg;
upperAmplitude_deg = upperAmplitude_deg - amplitudeReserve_deg;
end

function control = createMidpointBernsteinControls(power)
% Convert ascending powers and exactly subdivide the control polygon at 1/2.
degree = numel(power) - 1;
bernstein = zeros(1, degree + 1);
for controlIndex = 0:degree
    for powerIndex = 0:controlIndex
        bernstein(controlIndex + 1) = bernstein(controlIndex + 1) + ...
            nchoosek(controlIndex, powerIndex) / ...
            nchoosek(degree, powerIndex) * power(powerIndex + 1);
    end
end
leftControl = zeros(size(bernstein));
rightControl = zeros(size(bernstein));
leftControl(1) = bernstein(1);
rightControl(end) = bernstein(end);
work = bernstein;
for levelIndex = 1:degree
    work = 0.5 * (work(1:end - 1) + work(2:end));
    leftControl(levelIndex + 1) = work(1);
    rightControl(end - levelIndex) = work(end);
end
control = [leftControl, rightControl];
end

function [lowerAmplitude_deg, upperAmplitude_deg] = intersectAmplitudeBounds( ...
        lowerAmplitude_deg, upperAmplitude_deg, baseControl, ...
        amplitudeControl, lowerBound, upperBound)
% Intersect scalar affine control-point inequalities without sign assumptions.
controlScale = max(1, max(abs([baseControl, amplitudeControl])));
zeroTolerance = 256 * eps(controlScale);
for controlIndex = 1:numel(baseControl)
    slope = amplitudeControl(controlIndex);
    base = baseControl(controlIndex);
    if abs(slope) <= zeroTolerance
        if base < lowerBound || base > upperBound
            lowerAmplitude_deg = Inf;
            upperAmplitude_deg = -Inf;
            return;
        end
        continue;
    end
    firstIntersection = (lowerBound - base) / slope;
    secondIntersection = (upperBound - base) / slope;
    lowerAmplitude_deg = max(lowerAmplitude_deg, ...
        min(firstIntersection, secondIntersection));
    upperAmplitude_deg = min(upperAmplitude_deg, ...
        max(firstIntersection, secondIntersection));
end
end

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

function report = createProgressReport()
% Define stable bounded-family proposal and independent validation evidence.
report = struct( ...
    "Attempted", false, "Eligible", false, "Success", false, ...
    "Message", "The progress polynomial was not attempted.", ...
    "TerminationReason", "notAttempted", ...
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

function report = progressFailure(report, reason, message)
% Preserve an explicit fail-closed eligibility, bound, or validation result.
report.Success = false;
report.TerminationReason = reason;
report.Message = message;
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

function report = addProgressValidationTiming(report, validation, elapsedTime_s)
% Record validation timing inside the local bounded progress family.
report.ValidationCount = report.ValidationCount + 1;
report.ValidationElapsedTime_s = report.ValidationElapsedTime_s + elapsedTime_s;
report.CollisionCheckingElapsedTime_s = ...
    report.CollisionCheckingElapsedTime_s + ...
    validation.CollisionCheckingElapsedTime_s;
end

function diagnostics = finishFailure(diagnostics, reason, message, timer)
% Preserve an explicit terminal reason and elapsed work on every failure.
diagnostics.Success = false;
diagnostics.TerminationReason = reason;
diagnostics.Message = message;
diagnostics.ElapsedTime_s = toc(timer);
end
