function [candidate, diagnostics] = tryFixedTimeDetour(directCandidate, obstacles, initialState, goalState, limits, options,  directValidation)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.tryFixedTimeDetour()
%   [candidate, diagnostics] = ...
%       obstacleAvoidance.planner.tryFixedTimeDetour( ...
%       directCandidate, obstacles, initialState, goalState, limits, options, ...
%       directValidation)
%
% PURPOSE
%   - Try detours around obstacles while keeping the
%     direct move's minimum travel time and starting and ending at rest.
%   - Return the shortest detour among those tried that passes all safety
%     and motion checks.
%
% INPUTS
%   - directCandidate (scalar planner candidate struct)
%       A successful exact direct motion whose duration equals its reported
%       maximum componentwise minimum duration.
%   - obstacles (canonical or prepared obstacle array)
%       Original protected geometry and complete static or moving histories.
%   - initialState, goalState (normalized scalar structs)
%       Fixed two-coordinate rest-to-rest request used by the direct motion.
%   - limits (normalized scalar struct)
%       Per-axis physical limits and x/y workspace intervals.
%   - options (resolved scalar planner-options struct)
%       Supplies sampling, collision, constraint, and goal-time policies.
%   - directValidation (scalar validation record)
%       The caller's authoritative validation of directCandidate avoids
%       repeating the identical full-trajectory validation.
%
% OUTPUTS
%   - candidate (scalar planner candidate struct)
%       An independently validated fixed-clock motion, or the unchanged
%       direct record when no supported construction passes.
%   - diagnostics (scalar struct)
%       Stable clock, enumeration, feasible-side boundary, validation, and
%       explicit unsupported-or-failure evidence.
%
% UNITS
%   - Position and path length are coordinate units. Time is seconds. Derivatives use
%     coordinate units per second and its second and third powers.
%

%% Section 1: Check That The Direct Move Uses The Minimum Travel Time

if nargin == 0
    % Let callers obtain empty outputs without attempting a motion.
    candidate   = struct();
    diagnostics = createDiagnostics();
    return;
end
if nargin ~= 7
    error("tryFixedTimeDetour:InvalidCall",  "Use zero inputs or all seven documented inputs.");
end
timer = tic;
% Keep the original motion unless a detour passes every required check.
% Returning this original record after failure does not make it collision-free;
% diagnostics.Success tells the caller whether this function found a detour.
candidate   = directCandidate;
diagnostics = createDiagnostics();
if ~isstruct(directCandidate) || ~isscalar(directCandidate) ||  ~all(isfield(directCandidate, {'Success', 'TrajectoryDuration_s',  'MinimumAxisDuration_s', 'Polynomial', 'position_units'}))
    diagnostics = finishFailure(diagnostics, "invalidDirectCandidate",  "The direct candidate lacks the required stable motion fields.", timer);
    return;
end
% Promote the successful candidate; otherwise continue the configured fallback or search path.
if ~directCandidate.Success || isempty(directCandidate.position_units)
    diagnostics = finishFailure(diagnostics, "directMotionUnavailable",  "A successful direct motion is required before an excursion is tried.", timer);
    return;
end
dimensionCount = size(directCandidate.position_units, 2);
if dimensionCount ~= 2
    diagnostics = finishFailure(diagnostics, "unsupportedDimension",  "The independent validator currently requires two coordinates.", timer);
    return;
end

% Only try these detours when the direct move already takes the shortest
% possible time allowed by the axis limits. Both axes must finish, so the
% larger of their minimum travel times sets the overall minimum. Allow a
% small numerical difference when comparing it with the direct duration.
duration_s       = double(directCandidate.TrajectoryDuration_s);
lowerBound_s     = max(double(directCandidate.MinimumAxisDuration_s));
clockTolerance_s = max(double(options.ConstraintTolerance),  256 * eps(max(1, duration_s)));
diagnostics.Attempted             = true;
diagnostics.DirectDuration_s      = duration_s;
diagnostics.CertifiedLowerBound_s = lowerBound_s;
diagnostics.ClockTolerance_s      = clockTolerance_s;
diagnostics.ClockMatched          = isfinite(duration_s) && isfinite(lowerBound_s) &&  abs(duration_s - lowerBound_s) <= clockTolerance_s;

if ~diagnostics.ClockMatched
    diagnostics = finishFailure(diagnostics, "directClockNotCertified",  "The direct duration does not equal its componentwise physical lower bound.", timer);
    return;
end
if ~isstruct(directValidation) || ~isscalar(directValidation) ||  ~isfield(directValidation, "Passed")
    error("tryFixedTimeDetour:InvalidDirectValidation",  "directValidation must be the scalar public validation record.");
end
diagnostics.DirectValidation = directValidation;
% A detour may fix a collision, but this method does not repair an already
% invalid start/goal state, timing, or violation of the motion limits.
if ~directMotionIsPhysical(directValidation)
    diagnostics = finishFailure(diagnostics, "directMotionInvalid",  "The direct motion failed a non-collision invariant.", timer);
    return;
end

%% Section 2: Try Sideways Detours Without Extending The Motion

workspaceInterval_units = [double(limits.xInterval_units(:).'); ...
                        double(limits.yInterval_units(:).')];
coarseLevelCount      = 8;
% Stop shrinking the detour once changes are small relative to the collision
% tolerance or coordinate precision.
boundaryResolution_units = max(8 * double(options.CollisionClearanceTolerance_units),  sqrt(eps) * max(1, max(abs(directCandidate.position_units), [], "all")));

% Choose times to try the largest sideways offset, based on when the
% direct move encounters obstacles. Also try the middle of the move.
peakTime_s  = createPeakTimeCandidates(directCandidate, obstacles, options);
axisReports = repmat(createAxisReport(), 4 * dimensionCount * numel(peakTime_s), 1);
reportIndex = 0;

% Try each axis, both directions, and several times for the largest offset.
% Only one axis is changed in each trial; the other follows the direct move.
for axisIndex = 1:dimensionCount
    axisMinimum_s    = directCandidate.MinimumAxisDuration_s(axisIndex);
    axisGovernsClock = axisMinimum_s >= duration_s - clockTolerance_s;
    % Repeat the direction alternatives needed to refine the current solution.
    for direction = [-1, 1]
        % Process each peak needed to find fixed time detour.
        for peakChoiceIndex = 1:2 * numel(peakTime_s)
            % Preserve through-point proposals and also try an actual axis turn.
            % A through-point spline may continue rising well past this time.
            peakIndex = 1 + mod(peakChoiceIndex - 1, numel(peakTime_s));
            constrainPeakVelocity = peakChoiceIndex > numel(peakTime_s);
            reportIndex = reportIndex + 1;
            report      = createAxisReport();
            report.AxisIndex        = axisIndex;
            report.Direction        = direction;
            report.PeakTime_s       = peakTime_s(peakIndex);
            report.ConstrainedPeakVelocity = constrainPeakVelocity;
            report.AxisGovernsClock = axisGovernsClock;
            if axisGovernsClock
                % This axis already needs the full travel time. This method
                % has no established spare capacity for adding a detour to it.
                report.TerminationReason = "governingAxisHasNoCertifiedSlack";
                axisReports(reportIndex) = report;
                continue;
            end
            if direction > 0
                % Limit the offset using the room between the sampled direct
                % motion and the workspace edge on the chosen side.
                workspaceRoom_units = workspaceInterval_units(axisIndex, 2) -  max(directCandidate.position_units(:, axisIndex));
            else
                workspaceRoom_units = min(directCandidate.position_units(:, axisIndex)) -  workspaceInterval_units(axisIndex, 1);
            end
            phaseDuration_s  = [peakTime_s(peakIndex) - initialState.time_s, ...
                directCandidate.ArrivalTime_s - peakTime_s(peakIndex)];
            physicalRoom_units = Inf;
            % Estimate how far the offset can move out and return in the time
            % on either side of the peak. Use the smaller distance as a search
            % bound, not as proof that the combined motion obeys the limits.
            for phaseIndex = 1:2
                phaseRoom_units    = bmtpEngine.maximumRestToRestDistance(phaseDuration_s(phaseIndex),  limits.maxVelocity_units_s(axisIndex),  limits.maxAcceleration_units_s2(axisIndex),  limits.maxJerk_units_s3(axisIndex));
                physicalRoom_units = min(physicalRoom_units, phaseRoom_units);
            end
            maximumMagnitude_units = max(0, min(workspaceRoom_units, physicalRoom_units));
            report.MaximumMagnitude_units = maximumMagnitude_units;
            report.Eligible             = maximumMagnitude_units > boundaryResolution_units;
            if ~report.Eligible
                report.TerminationReason = "noExcursionRoom";
                axisReports(reportIndex) = report;
                continue;
            end
            lowerMagnitude_units  = 0;
            upperMagnitude_units  = NaN;
            upperCandidate      = struct();
            trialCandidates     = cell(coarseLevelCount, 1);
            trialMagnitudes_units = zeros(coarseLevelCount, 1);
            % Start with eight evenly spaced offset sizes, from small to large.
            % Each trial adds a smooth offset that is zero at both endpoints.
            for levelIndex = 1:coarseLevelCount
                magnitude_units                   = maximumMagnitude_units * levelIndex / coarseLevelCount;
                trialCandidates{levelIndex}     = createExcursion(directCandidate, direction * magnitude_units, axisIndex, peakTime_s(peakIndex), initialState, options, constrainPeakVelocity);
                trialMagnitudes_units(levelIndex) = magnitude_units;
            end
            % Quickly reject collisions at the stored sample times. A clear
            % sample check can still miss collisions between samples, so only
            % full validation below can make a trial acceptable.
            sampledClear = sampledCandidatesAreClear(trialCandidates, obstacles, options);
            diagnostics.ScreeningCount = diagnostics.ScreeningCount +  coarseLevelCount;
            passingLevel    = find(sampledClear, 1, "first");
            upperValidation = obstacleAvoidance.validation.validatePreparedTrajectory();
            if ~isempty(passingLevel)
                if passingLevel > 1
                    lowerMagnitude_units = trialMagnitudes_units(passingLevel - 1);
                end
                % Repeat the level alternatives needed to refine the current solution.
                for levelIndex = passingLevel:coarseLevelCount
                    validationTimer = tic;
                    trialValidation = obstacleAvoidance.validation.validatePreparedTrajectory(trialCandidates{levelIndex}, obstacles, initialState,  goalState, limits, options);
                    diagnostics     = addValidationTiming(diagnostics, trialValidation, toc(validationTimer));
                    % Use the first coarse detour magnitude that validates as the upper passing bound for refinement.
                    if trialValidation.Passed
                        upperMagnitude_units = trialMagnitudes_units(levelIndex);
                        upperCandidate     = trialCandidates{levelIndex};
                        upperValidation    = trialValidation;
                        break;
                    end
                    lowerMagnitude_units = trialMagnitudes_units(levelIndex);
                end
            end
            if ~isfinite(upperMagnitude_units)
                % None of the tested sizes passed. This is a limited search
                % failure, not proof that no detour exists.
                report.TerminationReason = "noPassingAmplitudeBracket";
                axisReports(reportIndex) = report;
                continue;
            end

            % Try smaller offsets between the last rejected size and the
            % passing size, halving the interval up to six times. Always keep
            % a fully validated candidate at the upper end. Other valid sizes
            % may exist outside this interval; this is not a global search.
            refinementCount = 0;
            % Refine the passing detour magnitude until resolution or iteration limits stop the search.
            while upperMagnitude_units - lowerMagnitude_units > boundaryResolution_units && refinementCount < 6
                midpointMagnitude_units = 0.5 * (lowerMagnitude_units + upperMagnitude_units);
                midpointCandidate     = createExcursion(directCandidate, direction * midpointMagnitude_units, axisIndex, peakTime_s(peakIndex), initialState, options, constrainPeakVelocity);
                validationTimer       = tic;
                midpointValidation    = obstacleAvoidance.validation.validatePreparedTrajectory(midpointCandidate, obstacles, initialState, goalState, limits, options);
                diagnostics           = addValidationTiming(diagnostics, midpointValidation, toc(validationTimer));
                refinementCount       = refinementCount + 1;
                % Move the passing bound inward when the midpoint validates; otherwise raise the failing lower bound.
                if midpointValidation.Passed
                    upperMagnitude_units = midpointMagnitude_units;
                    upperCandidate     = midpointCandidate;
                    upperValidation    = midpointValidation;
                else
                    lowerMagnitude_units = midpointMagnitude_units;
                end
            end
            upperCandidate.Validation = upperValidation;
            report.InvalidBoundaryMagnitude_units  = lowerMagnitude_units;
            report.ValidBoundaryMagnitude_units    = upperMagnitude_units;
            report.BoundaryResolutionReserve_units =  upperMagnitude_units - lowerMagnitude_units;
            report.BoundaryRefinementCount       = refinementCount;
            report.RetainedAmplitude_units         = direction * upperMagnitude_units;
            report.MotionLength_units              = upperCandidate.MotionLength_units;
            report.Validation                    = upperValidation;
            report.TerminationReason             = "validatedFeasibleBoundary";
            axisReports(reportIndex) = report;

            % All candidates have the same arrival time; keep the shortest.
            % Later route search may still find a better motion.
            if diagnostics.Success &&  upperCandidate.MotionLength_units >= candidate.MotionLength_units
                continue;
            end
            candidate = upperCandidate;
            diagnostics.AxisReports                   = axisReports;
            diagnostics.Success                       = true;
            diagnostics.TerminationReason             = "goalReached";
            diagnostics.SelectedMode                  = "singleAmplitude";
            diagnostics.Message                       =  "A one-sided fixed-clock excursion passed independent validation.";
            diagnostics.SelectedAxisIndex             = report.AxisIndex;
            diagnostics.SelectedDirection             = report.Direction;
            diagnostics.SelectedPeakVelocityConstrained = report.ConstrainedPeakVelocity;
            diagnostics.InvalidBoundaryMagnitude_units  =  report.InvalidBoundaryMagnitude_units;
            diagnostics.ValidBoundaryMagnitude_units    =  report.ValidBoundaryMagnitude_units;
            diagnostics.BoundaryResolutionReserve_units =  report.BoundaryResolutionReserve_units;
            diagnostics.RetainedAmplitude_units         = report.RetainedAmplitude_units;
            diagnostics.MotionLength_units              = candidate.MotionLength_units;
            diagnostics.SelectedValidation            = candidate.Validation;
            diagnostics.ElapsedTime_s                 = toc(timer);
        end
    end
end

%% Section 3: Shorten A Passing Detour Or Report That None Passed

diagnostics.AxisReports = axisReports;
% Retain a locally improved detour only when reconstruction succeeds; failed perturbations leave the incumbent unchanged.
if diagnostics.Success
    [candidate, diagnostics] = refineOffsetTravel(candidate, directCandidate,  diagnostics, obstacles, initialState, goalState, limits, options);
    diagnostics.ElapsedTime_s = toc(timer);
    return;
end
diagnostics = finishFailure(diagnostics, "noValidatedExcursion",  "No enumerated fixed-clock excursion passed independent validation.", timer);
end

%% Section 4: Local Functions

function [candidate, diagnostics] = refineOffsetTravel(candidate, direct,  diagnostics, obstacles, initialState, goalState, limits, options)
    % Reshape the selected detour while keeping the other axis and arrival time.
    % Keep the existing safe motion whenever a proposed change fails.
    axisIndex      = diagnostics.SelectedAxisIndex;
    reports        = diagnostics.AxisReports;
    selectedReport = find([reports.AxisIndex] == axisIndex &  [reports.Direction] == diagnostics.SelectedDirection &  [reports.MotionLength_units] == candidate.MotionLength_units, 1);
    % Use nine evenly spaced times plus the original peak time as adjustment
    % points (knots). Their offsets describe how far to depart from the direct move.
    knotTime_s            = unique([linspace(initialState.time_s, direct.ArrivalTime_s, 9).';  reports(selectedReport).PeakTime_s]);
    [~, basePosition_units] = bmtpEngine.evaluatePolynomial(direct.Polynomial, knotTime_s);
    [~, position_units]     = bmtpEngine.evaluatePolynomial(candidate.Polynomial, knotTime_s);
    offset_units            = position_units(:, axisIndex) - basePosition_units(:, axisIndex);
    % The detour must still start and end at the requested positions.
    offset_units([1 end]) = 0;
    record              = createTravelRefinement();
    record.Attempted         = true;
    record.InitialLength_units = candidate.MotionLength_units;
    record.InitialIntegratedSquaredJerk_units2_s5 = candidate.IntegratedSquaredJerk_units2_s5;
    record.KnotTime_s        = knotTime_s;
    initialStep_units = max(abs(offset_units)) / 2;
    % Nudge one interior offset at a time in both directions. Start with larger
    % changes, then halve the step for finer adjustments. Two passes at each
    % step size let later improvements influence earlier adjustment points.
    for level = 0:7
        step_units = initialStep_units / 2^level;
        % Repeat the sweep alternatives needed to refine the current solution.
        for sweep = 1:2
            % Process each knot needed to find offset travel.
            for knotIndex = 2:numel(knotTime_s)-1
                % Repeat the direction alternatives needed to refine the current solution.
                for direction = [-1 1]
                    trialOffset_units            = offset_units;
                    trialOffset_units(knotIndex) = trialOffset_units(knotIndex) + direction * step_units;
                    trial                      = bmtpEngine.createOffsetSplineMotion(direct, knotTime_s,  trialOffset_units, axisIndex, initialState, options.SampleTime_s,  "fixedClockLateralExcursion");
                    record.TrialCount = record.TrialCount + 1;
                    % Arrival is unchanged, so shorter travel is the objective.
                    % The full validator below enforces all derivative limits;
                    % integrated squared jerk is diagnostic, not a ranking gate.
                    if trial.MotionLength_units >= candidate.MotionLength_units - 1e-8
                        continue;
                    end
                    validationTimer = tic;
                    validation      = obstacleAvoidance.validation.validatePreparedTrajectory(trial, obstacles,  initialState, goalState, limits, options);
                    diagnostics     = addValidationTiming(diagnostics, validation, toc(validationTimer));
                    % Accept only validated knot perturbations as detour candidates; invalid perturbations are discarded.
                    if validation.Passed
                        trial.Validation = validation;
                        candidate  = trial;
                        offset_units = trialOffset_units;
                        record.AcceptedCount = record.AcceptedCount + 1;
                    end
                end
            end
        end
    end
    record.FinalLength_units = candidate.MotionLength_units;
    record.FinalIntegratedSquaredJerk_units2_s5 = candidate.IntegratedSquaredJerk_units2_s5;
    record.KnotOffset_units  = offset_units;
    diagnostics.TravelRefinement   = record;
    diagnostics.MotionLength_units   = candidate.MotionLength_units;
    diagnostics.SelectedValidation = candidate.Validation;
    % Replace the original detour only when at least one perturbation was accepted.
    if record.AcceptedCount > 0
        diagnostics.SelectedMode = "refinedOffsetSpline";
        diagnostics.Message      = "A refined fixed-clock offset spline passed independent validation.";
    end
end

function record = createTravelRefinement()
    % Record how much the later reshaping improved the initial passing detour.
    record = struct();
    record.Attempted         = false;
    record.AcceptedCount     = 0;
    record.TrialCount        = 0;
    record.InitialLength_units = NaN;
    record.FinalLength_units   = NaN;
    record.InitialIntegratedSquaredJerk_units2_s5 = NaN;
    record.FinalIntegratedSquaredJerk_units2_s5   = NaN;
    record.KnotTime_s        = zeros(0, 1);
    record.KnotOffset_units    = zeros(0, 1);
end

function candidate = createExcursion(directCandidate, amplitude_units, axisIndex, peakTime_s, initialState, options, constrainPeakVelocity)
    % Add an offset of zero at the start, amplitude_units at the chosen interior
    % time, and zero at arrival. Try either a free through-point velocity or
    % zero complete-axis velocity there; acceleration is free in both families.
    startTime_s = initialState.time_s;
    endTime_s   = directCandidate.ArrivalTime_s;
    knotVelocity_units_s = NaN(3, 1);
    if constrainPeakVelocity
        % Cancel the base velocity so the complete axis turns at the waypoint.
        [~, ~, baseVelocity_units_s] = bmtpEngine.evaluatePolynomial(directCandidate.Polynomial, peakTime_s);
        knotVelocity_units_s(2) = -baseVelocity_units_s(axisIndex);
    end
    candidate = bmtpEngine.createOffsetSplineMotion(directCandidate, [startTime_s; peakTime_s; endTime_s], [0; amplitude_units; 0], axisIndex, initialState, options.SampleTime_s, "fixedClockLateralExcursion", knotVelocity_units_s);
end

function peakTime_s = createPeakTimeCandidates(directCandidate, obstacles, options)
    % Choose times to try the largest offset, using where the direct move collides.
    % These sampled observations guide the search; they do not certify safety.
    startTime_s    = directCandidate.time_s(1);
    endTime_s      = directCandidate.time_s(end);
    midpointTime_s = 0.5 * (startTime_s + endTime_s);
    queryOptions   = struct();
    queryOptions.BoundaryIsOccupied     = true;
    queryOptions.ClearanceTolerance_units = options.CollisionClearanceTolerance_units;
    [isOccupied, ~, details] =  obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, directCandidate.position_units(:, 1),  directCandidate.position_units(:, 2), directCandidate.time_s, queryOptions);
    isOccupied               = logical(isOccupied(:));
    % Find the start and end of each consecutive group of colliding samples.
    runChange       = diff([false; isOccupied; false]);
    runStart        = find(runChange == 1);
    runEnd          = find(runChange == -1) - 1;
    collisionPeak_s = zeros(2 * numel(runStart), 1);
    for runIndex = 1:numel(runStart)
        % Try both the worst-clearance sample and the middle of this collision
        % interval, so the detour can be strongest near the obstruction.
        indices                           = runStart(runIndex):runEnd(runIndex);
        [~, localIndex]                   = min(details.MinimumClearance_units(indices));
        collisionPeak_s(2 * runIndex - 1) =  directCandidate.time_s(indices(localIndex));
        collisionPeak_s(2 * runIndex)     = 0.5 * sum(directCandidate.time_s([indices(1), indices(end)]));
    end
    % Always include the middle of the whole move as another timing choice.
    peakTime_s = unique([collisionPeak_s; midpointTime_s], "stable");
    % Leave time to move out and return, and avoid nearly duplicate trials.
    endpointReserve_s       = 256 * eps(max(1, endTime_s - startTime_s));
    peakTime_s              = peakTime_s(peakTime_s > startTime_s + endpointReserve_s &  peakTime_s < endTime_s - endpointReserve_s);
    minimumPeakSeparation_s = max(endpointReserve_s, 0.5 * options.SampleTime_s);
    retainedPeak            = false(size(peakTime_s));
    % Process each peak needed to build peak time candidates.
    for peakIndex = 1:numel(peakTime_s)
        retainedPeak(peakIndex) = ~any(abs(peakTime_s(1:peakIndex - 1) -  peakTime_s(peakIndex)) < minimumPeakSeparation_s &  retainedPeak(1:peakIndex - 1));
    end
    peakTime_s = peakTime_s(retainedPeak);
end



function valid = directMotionIsPhysical(validation)
    % Require all checks except collision checks to pass.
    allowedIssues = ["collision freedom", "collision resolution"];
    valid         = all(ismember(validation.Issues, allowedIssues));
end

function isClear = sampledCandidatesAreClear(candidates, obstacles, options)
    % Check all trial motions together because they share the same sample times.
    % A true result means only that these samples are clear, not the entire motion.
    candidateCount = numel(candidates);
    sampleCount    = numel(candidates{1}.time_s);
    x_units    = zeros(sampleCount, candidateCount);
    y_units  = zeros(sampleCount, candidateCount);
    time_s         = repmat(candidates{1}.time_s, 1, candidateCount);
    % Evaluate each candidate before retaining the best admissible candidate.
    for candidateIndex = 1:candidateCount
        x_units(:, candidateIndex)   = candidates{candidateIndex}.position_units(:, 1);
        y_units(:, candidateIndex) = candidates{candidateIndex}.position_units(:, 2);
    end
    queryOptions = struct();
    queryOptions.BoundaryIsOccupied     = true;
    queryOptions.ClearanceTolerance_units = options.CollisionClearanceTolerance_units;
    isOccupied = obstacleAvoidance.obstacles.queryPreparedObstacles(obstacles, x_units, y_units, time_s, queryOptions);
    isClear    = ~any(isOccupied, 1);
end

function report = createAxisReport()
    % Store one trial family's axis, direction, peak time, and rejection or result.
    report = struct();
    report.AxisIndex                     = 0;
    report.Direction                     = 0;
    report.AxisGovernsClock              = false;
    report.PeakTime_s                    = NaN;
    report.ConstrainedPeakVelocity       = false;
    report.Eligible                      = false;
    report.MaximumMagnitude_units          = 0;
    report.InvalidBoundaryMagnitude_units  = NaN;
    report.ValidBoundaryMagnitude_units    = NaN;
    report.BoundaryResolutionReserve_units = NaN;
    report.BoundaryRefinementCount       = 0;
    report.RetainedAmplitude_units         = NaN;
    report.MotionLength_units              = NaN;
    report.Validation                    = obstacleAvoidance.validation.validatePreparedTrajectory();
    report.TerminationReason             = "notAttempted";
end

function diagnostics = createDiagnostics()
    % Keep the same diagnostic fields on success, failure, and zero-input calls.
    % NaN marks numeric results that are not available because no trial supplied them.
    diagnostics = struct();
    diagnostics.Attempted                      = false;
    diagnostics.Success                        = false;
    diagnostics.Message                        = "The fixed-clock excursion was not attempted.";
    diagnostics.TerminationReason              = "notRun";
    diagnostics.SelectedMode                   = "";
    diagnostics.ClockMatched                   = false;
    diagnostics.DirectDuration_s               = NaN;
    diagnostics.CertifiedLowerBound_s          = NaN;
    diagnostics.ClockTolerance_s               = NaN;
    diagnostics.ValidationCount                = 0;
    diagnostics.ScreeningCount                 = 0;
    diagnostics.SelectedAxisIndex              = 0;
    diagnostics.SelectedDirection              = 0;
    diagnostics.SelectedPeakVelocityConstrained = false;
    diagnostics.InvalidBoundaryMagnitude_units   = NaN;
    diagnostics.ValidBoundaryMagnitude_units     = NaN;
    diagnostics.BoundaryResolutionReserve_units  = NaN;
    diagnostics.RetainedAmplitude_units          = NaN;
    diagnostics.MotionLength_units               = NaN;
    diagnostics.DirectValidation               = obstacleAvoidance.validation.validatePreparedTrajectory();
    diagnostics.SelectedValidation             = obstacleAvoidance.validation.validatePreparedTrajectory();
    diagnostics.AxisReports                    = repmat(createAxisReport(), 0, 1);
    diagnostics.TravelRefinement               = createTravelRefinement();
    diagnostics.ValidationElapsedTime_s        = 0;
    diagnostics.CollisionCheckingElapsedTime_s = 0;
    diagnostics.ElapsedTime_s                  = 0;
end

function diagnostics = addValidationTiming(diagnostics, validation, elapsedTime_s)
    % Count validation time separately from motion construction.
    diagnostics.ValidationCount                = diagnostics.ValidationCount + 1;
    diagnostics.ValidationElapsedTime_s        = diagnostics.ValidationElapsedTime_s +  elapsedTime_s;
    diagnostics.CollisionCheckingElapsedTime_s =  diagnostics.CollisionCheckingElapsedTime_s +  validation.CollisionCheckingElapsedTime_s;
end

function diagnostics = finishFailure(diagnostics, reason, message, timer)
    % Record the failure and elapsed time.
    diagnostics.Success           = false;
    diagnostics.TerminationReason = reason;
    diagnostics.Message           = message;
    diagnostics.ElapsedTime_s     = toc(timer);
end
