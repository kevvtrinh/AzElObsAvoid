function [result, diagnostics] = solveTimedAlternatingTrajectory( ...
    solverRequest, warmStart, diagnostics, separationTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.optimization.solveTimedAlternatingTrajectory( ...
%       solverRequest, warmStart, diagnostics, separationTarget_units, ...
%       roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Seek an earlier arrival around moving obstacles. After durations change,
%     rebuild separating lines for the actual times when each curve segment
%     overlaps each obstacle interval. A line keeps the two on opposite sides.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked solve request with regions, coverage, limits, and options.
%   - warmStart (scalar struct)
%       Starting curve controls, segment durations, and duration ratios.
%   - diagnostics (scalar struct)
%       Solver diagnostics accumulated so far in this solve.
%   - separationTarget_units (finite scalar)
%       Required distance from each obstacle to its separating line.
%   - roundoffReserve_units (finite scalar)
%       Numerical separation reserve.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Earliest retained candidate with matching durations and separating
%       lines. Expected infeasibility returns Success = false with empty
%       controls. The caller still runs public validation. Invalid input throws.
%   - diagnostics (scalar struct)
%       Solver counts, remaining unverified pairs, and termination data.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Build Initial Constraints Around The Timed Starting Curve

% Keep duration ratios fixed while the solver changes total travel time.
% For ratios [1 2], the second segment always lasts twice as long as the first.
segmentCount      = warmStart.SegmentCount;
segmentTimeRatios = warmStart.SegmentRatio(:);
emptyPlane        = bmtpEngine.separation.createEmptyPlane();

selectedControl_units      = zeros(0, solverRequest.Degree + 1, 2);
selectedSegmentTime_s      = NaN;
selectedPlanes             = repmat(emptyPlane, 0, 0);
selectedPairs              = [];
selectedPairCount          = 0;
selectedCollisionPairCount = 0;
selectedSolverMessage      = "";
selectedSolverOutput       = struct();
lastSolverOutput           = struct();
previousUnverifiedPairs    = false(segmentCount, numel(solverRequest.Regions_units));
lastAttemptMessage         = "The time-scoped alternating iteration limit was reached.";
failureStage               = "optimization";
failureKind                = "iterationLimit";
diagnostics.ConicSolver    = bmtpEngine.optimization.accumulateConicDiagnostics();

% Build obstacle constraints before the first solve, using the starting
% curve and its segment times. Otherwise the solver could choose a straight
% path through an obstacle before any separating lines were available.
[separatingPlanes, ~, allRequiredLinesVerified] = ...
    bmtpEngine.separation.createTimeScopedPlanes(warmStart.ControlPoint_units, ...
    warmStart.SegmentTime_s, solverRequest, separationTarget_units, roundoffReserve_units);
if ~allRequiredLinesVerified
    diagnostics.ApplicablePairCount     = 0;
    diagnostics.FinalCollisionPairCount = 0;
    diagnostics.TaggedPairCount         = 0;
    diagnostics.SolverMessage           = "The timed visibility guide could not initialize its exact corridor.";
    result = struct( ...
        'Success',                  false, ...
        'SolverMessage',            diagnostics.SolverMessage, ...
        'FailureStage',             "proposal", ...
        'FailureKind',              "timedCorridorInitializationUnavailable", ...
        'AlternativeGuideEligible', false, ...
        'ControlPoint_units',       selectedControl_units, ...
        'SegmentTime_s',            selectedSegmentTime_s, ...
        'Planes',                   selectedPlanes, ...
        'TaggedPairs',              selectedPairs);
    return
end
previousDuration_s         = warmStart.Duration_s;
savedTrajectoryConstraints = struct();

%% Section 2: Solve And Recheck Obstacles At The New Segment Times

% A shorter motion meets moving obstacles at different times. Check every
% returned curve with its own durations before retaining it or using its
% separating lines for the next solve.
for iterationIndex = 1:solverRequest.MaximumAlternatingIterations
    diagnostics.IterationCount = iterationIndex;
    trajectoryStep             = struct( ...
        'SegmentCount',            segmentCount, ...
        'Planes',                  separatingPlanes, ...
        'RoundoffReserve_units',   roundoffReserve_units, ...
        'MaximumMotionDuration_s', solverRequest.MotionHorizon_s, ...
        'GoalTimeMode',            solverRequest.Options.GoalTimeMode, ...
        'MinimumMotionDuration_s', solverRequest.MinimumMotionDuration_s, ...
        'SegmentRatio',            segmentTimeRatios, ...
        'ConstraintBase',          savedTrajectoryConstraints);
    [trialControl_units, trialSegmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
        bmtpEngine.optimization.solveTimedTrajectoryStep(solverRequest, trajectoryStep);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + solverOutput.SolveCount;
    diagnostics.ConicSolver         = bmtpEngine.optimization.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, solverOutput);
    lastSolverOutput = solverOutput;
    if ~bmtpEngine.optimization.hasUsableConicIterate(trialControl_units, exitFlag)
        lastAttemptMessage = "Trajectory SOCP failed: " + string(solverOutput.message);
        diagnostics.LastTrajectoryExitFlag = exitFlag;
        failureStage = "numericalSolver";
        failureKind  = "optimizerIterateUnavailable";
        if exitFlag == 0
            failureStage = "optimization";
            failureKind  = "trajectorySolverIterationLimit";
        elseif exitFlag == -2
            failureStage = "proposal";
            failureKind  = "trajectorySubproblemInfeasible";
        end
        break
    end

    trialDuration_s = sum(trialSegmentTime_s);
    trialMotion     = struct( ...
        'ProvenControlPoint_units', trialControl_units, ...
        'ControlPoint_units',       trialControl_units, ...
        'SegmentTime_s',            trialSegmentTime_s(:), ...
        'FinalTime_s',              solverRequest.InitialState.time_s + trialDuration_s, ...
        'GivenPower_units',         []);
    trialMotionCheck = bmtpEngine.validation.checkFinalMotion(solverRequest, trialMotion, ...
        roundoffReserve_units, separationTarget_units);
    unverifiedPairs = ~reshape([trialMotionCheck.Planes.Verified], ...
        size(trialMotionCheck.Planes)) & trialMotionCheck.RegionActiveBySegment;
    allObstaclePairsPassed = ~any(unverifiedPairs, 'all');

    % A solver status alone is insufficient: workspace bounds, motion-rate
    % limits, segment joins, and every obstacle pair must all pass.
    trialMotionChecksPassed = allObstaclePairsPassed && trialMotionCheck.WorkspacePassed && ...
        trialMotionCheck.DynamicsPassed && trialMotionCheck.ContinuityPassed;
    if trialMotionChecksPassed
        % Build lines for the next solve using this checked curve and its
        % times. Save controls, times, lines, and measurements together only
        % when the candidate improves arrival, so the returned fields agree.
        [trialPlanes, trialPairs, allRequiredLinesVerified] = ...
            bmtpEngine.separation.createTimeScopedPlanes(trialControl_units, ...
            trialSegmentTime_s, solverRequest, separationTarget_units, roundoffReserve_units);
        if ~allRequiredLinesVerified
            lastAttemptMessage = "The feasible timed motion did not produce a complete exact plane set.";
            failureStage       = "certification";
            failureKind        = "feasibleTimedPlaneSetUnavailable";
            break
        end
        retainedDuration_s = Inf;
        if ~isempty(selectedControl_units)
            retainedDuration_s = sum(selectedSegmentTime_s);
        end
        if trialDuration_s < retainedDuration_s
            selectedControl_units      = trialControl_units;
            selectedSegmentTime_s      = trialSegmentTime_s;
            selectedPlanes             = trialPlanes;
            selectedPairs              = trialPairs;
            selectedPairCount          = nnz(trialPairs);
            selectedCollisionPairCount = nnz(unverifiedPairs);
            selectedSolverMessage      = "A complete time-scoped feasible iterate was retained.";
            selectedSolverOutput       = solverOutput;
        end
        arrivalImprovementReachedTolerance = ...
            retainedDuration_s - trialDuration_s <= solverRequest.Options.ArrivalTimeTolerance_s;
        if arrivalImprovementReachedTolerance
            if trialDuration_s < retainedDuration_s
                selectedSolverMessage = "The feasible arrival improvement reached tolerance.";
                lastAttemptMessage    = selectedSolverMessage;
            else
                lastAttemptMessage = ...
                    "A later feasible trial did not improve the retained duration within tolerance.";
            end
            break
        end
        separatingPlanes        = trialPlanes;
        previousDuration_s      = trialDuration_s;
        previousUnverifiedPairs = false(size(unverifiedPairs));
        continue
    end

    % This trial did not pass. Keep the starting curve's shape and rebuild
    % its lines at the new durations before another trajectory solve.
    [separatingPlanes, ~, allRequiredLinesVerified] = ...
        bmtpEngine.separation.createTimeScopedPlanes(warmStart.ControlPoint_units, ...
        trialSegmentTime_s, solverRequest, separationTarget_units, roundoffReserve_units);
    if ~allRequiredLinesVerified
        lastAttemptMessage = "The timed visibility guide could not initialize every exact clock pair.";
        failureStage       = "proposal";
        failureKind        = "timedClockPairInitializationUnavailable";
        break
    end
    % Stop when neither travel time nor the unresolved obstacle pairs change.
    durationChangeReachedTolerance = abs(trialDuration_s - previousDuration_s) <= ...
        solverRequest.Options.ArrivalTimeTolerance_s;
    if durationChangeReachedTolerance && isequal(unverifiedPairs, previousUnverifiedPairs)
        lastAttemptMessage = "The exact timed pair set stopped changing before feasibility.";
        failureStage       = "optimization";
        failureKind        = "timedPairSetStalled";
        break
    end
    previousDuration_s      = trialDuration_s;
    previousUnverifiedPairs = unverifiedPairs;
end

%% Section 3: Return The Retained Motion With Its Own Measurements

% A later trial may fail or be slower. Preserve the earlier retained motion
% and its matching lines and solver output; use the last failure details
% only when no candidate was retained.
if ~isempty(selectedControl_units)
    diagnostics.ApplicablePairCount     = selectedPairCount;
    diagnostics.FinalCollisionPairCount = selectedCollisionPairCount;
    diagnostics.Converged               = selectedSolverOutput.OptimizationConverged;

    solverMessage        = selectedSolverMessage;
    reportedSolverOutput = selectedSolverOutput;
    failureStage         = "";
    failureKind          = "";
else
    diagnostics.ApplicablePairCount     = 0;
    diagnostics.FinalCollisionPairCount = 0;

    solverMessage        = lastAttemptMessage;
    reportedSolverOutput = lastSolverOutput;
end
if ~isempty(fieldnames(reportedSolverOutput)) && ...
        reportedSolverOutput.ConstraintGenerationApplied
    diagnostics.LoadedPlanePairCount           = reportedSolverOutput.LoadedPlanePairCount;
    diagnostics.ConstraintGenerationRoundCount = ...
        reportedSolverOutput.ConstraintGenerationRoundCount;
    diagnostics.ConstraintGenerationComplete = ...
        reportedSolverOutput.ConstraintGenerationComplete;
    diagnostics.MaximumPlaneConstraintResidual = ...
        reportedSolverOutput.MaximumPlaneConstraintResidual;
    diagnostics.ConstraintGenerationReturnedSolveIndex = ...
        reportedSolverOutput.ReturnedSolveIndex;
    diagnostics.ConstraintGenerationLastAttemptExitFlag = ...
        reportedSolverOutput.LastAttemptExitFlag;
    diagnostics.ConstraintGenerationTerminatedAfterRetainedIterate = ...
        reportedSolverOutput.TerminatedAfterRetainedIterate;
end
diagnostics.TaggedPairCount = nnz(selectedPairs);
diagnostics.SolverMessage   = solverMessage;
result = struct( ...
    'Success',                  ~isempty(selectedControl_units), ...
    'SolverMessage',            solverMessage, ...
    'FailureStage',             failureStage, ...
    'FailureKind',              failureKind, ...
    'AlternativeGuideEligible', false, ...
    'ControlPoint_units',       selectedControl_units, ...
    'SegmentTime_s',            selectedSegmentTime_s, ...
    'Planes',                   selectedPlanes, ...
    'TaggedPairs',              selectedPairs);
end
