function [result, diagnostics] = solveTimedAlternatingTrajectory( ...
    request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.solveTimedAlternatingTrajectory( ...
%       request, warmStart, diagnostics, obstacleTarget_units, ...
%       roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Minimize arrival while rebuilding every moving-cell constraint on the
%     current physical clock and the exact timed visibility-guide homotopy.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked solve request with regions, coverage, limits, and options.
%   - warmStart (scalar struct)
%       Timed warm start supplying controls, span durations, and ratios.
%   - diagnostics (scalar struct)
%       Solver diagnostics accumulated so far in this solve.
%   - obstacleTarget_units (finite scalar)
%       Required obstacle-side separation target.
%   - roundoffReserve_units (finite scalar)
%       Numerical separation reserve.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Earliest exactly time-scoped candidate. An expected infeasible solve
%       returns Success = false with empty controls. Invalid input throws.
%   - diagnostics (scalar struct)
%       Solver counts, residual pair counts, and termination data.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Initialize The Variable-Clock Formulation
segmentCount = warmStart.SegmentCount;
segmentRatio = warmStart.SegmentRatio(:);
emptyPlane   = bmtpEngine.createEmptyPlane();

selectedControl_units      = zeros(0, request.Degree + 1, 2);
selectedSegmentTime_s      = NaN;
selectedPlanes             = repmat(emptyPlane, 0, 0);
selectedPairs              = [];
selectedPairCount          = 0;
selectedCollisionPairCount = 0;
selectedSolverMessage      = "";
selectedStepOutput         = struct();
lastStepOutput             = struct();
previousFailedPairs        = false(segmentCount, numel(request.Regions_units));
lastAttemptMessage         = "The time-scoped alternating iteration limit was reached.";
diagnostics.ConicSolver    = bmtpEngine.accumulateConicDiagnostics();

% Establish the exact moving corridor at the timed guide's physical clock.
% An unconstrained first solve would collapse to a straight collision path
% before the alternating method had any obstacle planes to retain.
[planes, ~, complete] = ...
    bmtpEngine.createTimeScopedPlanes(warmStart.ControlPoint_units, ...
    warmStart.SegmentTime_s, request, obstacleTarget_units, roundoffReserve_units);
if ~complete
    diagnostics.ApplicablePairCount     = 0;
    diagnostics.FinalCollisionPairCount = 0;
    diagnostics.TaggedPairCount         = 0;
    diagnostics.SolverMessage           = "The timed visibility guide could not initialize its exact corridor.";
    diagnostics.LastAttemptMessage      = diagnostics.SolverMessage;
    result = struct('Success', false, 'SolverMessage', diagnostics.SolverMessage, ...
        'ControlPoint_units', selectedControl_units, ...
        'SegmentTime_s',      selectedSegmentTime_s, ...
        'Planes',             selectedPlanes, ...
        'TaggedPairs',        selectedPairs);
    return
end
previousDuration_s = warmStart.Duration_s;

%% Section 2: Solve And Rebuild Constraints On Every Returned Clock
for iterationIndex = 1:request.MaximumAlternatingIterations
    diagnostics.IterationCount = iterationIndex;
    trajectoryGoalTimeMode     = request.Options.GoalTimeMode;
    [trialControl_units, trialSegmentTime_s, exitFlag, output] = ...
        bmtpEngine.solveTimedTrajectoryStep(segmentCount, request.Degree, ...
        request.InitialState.position_units, request.GoalState.position_units, ...
        request.Limits, planes, roundoffReserve_units, request.MotionHorizon_s, ...
        trajectoryGoalTimeMode, request.TimedTrajectoryOptions, request.MinimumMotionDuration_s, ...
        segmentRatio);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + output.SolveCount;
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, output);
    lastStepOutput = output;
    if ~bmtpEngine.hasUsableConicIterate(trialControl_units, exitFlag)
        lastAttemptMessage = "Trajectory SOCP failed: " + string(output.message);
        break
    end

    duration_s = sum(trialSegmentTime_s);
    trialMotion = struct('CertifiedControlPoint_units', trialControl_units, ...
        'ControlPoint_units',    trialControl_units, ...
        'SegmentTime_s',         trialSegmentTime_s(:), ...
        'PrescribedPower_units', []);
    trialCertificate = bmtpEngine.checkFinalMotion(request, trialMotion, ...
        roundoffReserve_units, obstacleTarget_units);
    failedPairs = ~reshape([trialCertificate.Planes.Verified], ...
        size(trialCertificate.Planes)) & trialCertificate.RegionActiveBySegment;
    collisionFree = ~any(failedPairs, 'all');

    trialIsFullyCertified = collisionFree && trialCertificate.WorkspacePassed && ...
        trialCertificate.DynamicsPassed && trialCertificate.ContinuityPassed;
    if trialIsFullyCertified
        % The exact planes of this feasible iterate constrain the next solve.
        % They become the returned planes only if the iterate is retained, so
        % the returned planes and mask always describe the returned motion.
        [trialPlanes, trialPairs, complete] = ...
            bmtpEngine.createTimeScopedPlanes(trialControl_units, ...
            trialSegmentTime_s, request, obstacleTarget_units, roundoffReserve_units);
        if ~complete
            lastAttemptMessage = "The feasible timed motion did not produce a complete exact plane set.";
            break
        end
        previousFeasibleDuration_s = Inf;
        if ~isempty(selectedControl_units)
            previousFeasibleDuration_s = sum(selectedSegmentTime_s);
        end
        if duration_s < previousFeasibleDuration_s
            selectedControl_units      = trialControl_units;
            selectedSegmentTime_s      = trialSegmentTime_s;
            selectedPlanes             = trialPlanes;
            selectedPairs              = trialPairs;
            selectedPairCount          = nnz(trialPairs);
            selectedCollisionPairCount = nnz(failedPairs);
            selectedSolverMessage      = "A complete time-scoped feasible iterate was retained.";
            selectedStepOutput         = output;
        end
        arrivalImprovementReachedTolerance = ...
            previousFeasibleDuration_s - duration_s <= request.Options.ArrivalTimeTolerance_s;
        if arrivalImprovementReachedTolerance
            if duration_s < previousFeasibleDuration_s
                selectedSolverMessage = "The feasible arrival improvement reached tolerance.";
                lastAttemptMessage    = selectedSolverMessage;
            else
                lastAttemptMessage = ...
                    "A later feasible trial did not improve the retained duration within tolerance.";
            end
            break
        end
        planes              = trialPlanes;
        previousDuration_s  = duration_s;
        previousFailedPairs = false(size(failedPairs));
        continue
    end

    [planes, ~, complete] = ...
        bmtpEngine.createTimeScopedPlanes(warmStart.ControlPoint_units, ...
        trialSegmentTime_s, request, obstacleTarget_units, roundoffReserve_units);
    if ~complete
        lastAttemptMessage = "The timed visibility guide could not initialize every exact clock pair.";
        break
    end
    unchangedClock = abs(duration_s - previousDuration_s) <= ...
        request.Options.ArrivalTimeTolerance_s;
    if unchangedClock && isequal(failedPairs, previousFailedPairs)
        lastAttemptMessage = "The exact timed pair set stopped changing before feasibility.";
        break
    end
    previousDuration_s  = duration_s;
    previousFailedPairs = failedPairs;
end

%% Section 3: Return Only A Clock-Consistent Feasible Iterate

% A returned motion carries the pair count of its own exact planes.
if ~isempty(selectedControl_units)
    diagnostics.ApplicablePairCount     = selectedPairCount;
    diagnostics.FinalCollisionPairCount = selectedCollisionPairCount;
    diagnostics.Converged               = selectedStepOutput.OptimizationConverged;
    solverMessage                       = selectedSolverMessage;
    diagnosticStepOutput                = selectedStepOutput;
else
    diagnostics.ApplicablePairCount     = 0;
    diagnostics.FinalCollisionPairCount = 0;
    solverMessage                       = lastAttemptMessage;
    diagnosticStepOutput                = lastStepOutput;
end
if ~isempty(fieldnames(diagnosticStepOutput)) && ...
        diagnosticStepOutput.ConstraintGenerationApplied
    diagnostics.LoadedPlanePairCount = diagnosticStepOutput.LoadedPlanePairCount;
    diagnostics.ConstraintGenerationRoundCount = ...
        diagnosticStepOutput.ConstraintGenerationRoundCount;
    diagnostics.ConstraintGenerationComplete = ...
        diagnosticStepOutput.ConstraintGenerationComplete;
    diagnostics.MaximumPlaneConstraintResidual = ...
        diagnosticStepOutput.MaximumPlaneConstraintResidual;
    diagnostics.ConstraintGenerationReturnedSolveIndex = ...
        diagnosticStepOutput.ReturnedSolveIndex;
    diagnostics.ConstraintGenerationLastAttemptExitFlag = ...
        diagnosticStepOutput.LastAttemptExitFlag;
    diagnostics.ConstraintGenerationTerminatedAfterRetainedIterate = ...
        diagnosticStepOutput.TerminatedAfterRetainedIterate;
end
diagnostics.TaggedPairCount    = nnz(selectedPairs);
diagnostics.SolverMessage      = solverMessage;
diagnostics.LastAttemptMessage = lastAttemptMessage;
result = struct('Success', ~isempty(selectedControl_units), ...
    'SolverMessage',      solverMessage, ...
    'ControlPoint_units', selectedControl_units, ...
    'SegmentTime_s',      selectedSegmentTime_s, ...
    'Planes',             selectedPlanes, ...
    'TaggedPairs',        selectedPairs);
end
