function [result, diagnostics] = refineTimedTravel( ...
    solverRequest, retainedSolveResult, diagnostics, separationTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.pipeline.refineTimedTravel( ...
%       solverRequest, retainedSolveResult, diagnostics, separationTarget_units, roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Try to shorten the control polygon after an earliest-arrival motion
%     has been retained. Keep its segment times fixed and require the complete
%     motion checks to pass before replacing that result.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked BMTP inputs for a solve that could choose an earlier arrival.
%   - retainedSolveResult (scalar struct)
%       Selected motion from the alternating solver, with its separating lines.
%   - diagnostics (scalar struct)
%       Diagnostics accumulated so far by the caller.
%   - separationTarget_units (finite numeric scalar)
%       Required distance from each obstacle to its separating line.
%   - roundoffReserve_units (finite numeric scalar)
%       Numerical reserve applied on the trajectory side.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Selected controls and segment durations. Keep the original result
%       when refinement does not shorten its control polygon or pass checks.
%   - diagnostics (scalar struct)
%       Updated active-pair count and travel-refinement measurements.
%**************************************************************************
% UNITS
%   - Position and travel are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Read The Retained Candidate

% Start with the original result so an unsuccessful refinement preserves it.
result = retainedSolveResult;

originalControl_units = result.ControlPoint_units;
originalSegmentTime_s = result.SegmentTime_s(:);
segmentCount          = size(originalControl_units, 1);
assert(numel(originalSegmentTime_s) == segmentCount, ...
    'bmtpEngine:InvalidTimedSegmentClock', ...
    'Timed refinement requires one physical duration per motion span.');
% The objective uses distances between adjacent controls. That sum bounds
% the Bezier curve length; it is not the exact distance along the curve.
originalDuration_s   = sum(originalSegmentTime_s);
originalLength_units = controlPolygonLength(originalControl_units);

selectedControl_units    = originalControl_units;
selectedSegmentTime_s    = originalSegmentTime_s;
selectedPlanes           = retainedSolveResult.Planes;
selectedPairs            = retainedSolveResult.TaggedPairs;
selectedSolverMessage    = retainedSolveResult.SolverMessage;
selectedLength_units     = originalLength_units;
travelRefinementAccepted = false;
separatingPlanes         = retainedSolveResult.Planes;

%% Section 2: Shorten The Path At The Retained Segment Times

assert(solverRequest.UsesVariableClock && ...
    solverRequest.Options.GoalTimeMode == "earliestArrival", ...
    'bmtpEngine:InvalidTimedTravelRefinement', ...
    'Timed travel refinement requires an earliest-arrival variable clock.');
% Fix total duration and all segment-duration ratios to preserve the
% retained timing. Fixed-arrival requests use the other alternating solver
% and do not enter this refinement.
fixedMotionDuration_s = originalDuration_s;
diagnostics.TravelRefinementAttempted           = true;
diagnostics.TravelRefinementInitialLength_units = originalLength_units;
diagnostics.TravelRefinementFinalLength_units   = originalLength_units;
diagnostics.TravelRefinementInitialDuration_s   = originalDuration_s;
diagnostics.TravelRefinementFinalDuration_s     = originalDuration_s;
diagnostics.TravelRefinementAccepted            = false;
segmentTimeRatios          = originalSegmentTime_s / mean(originalSegmentTime_s);
savedTrajectoryConstraints = struct();
% Only a changed, verified line set allows another solve. Stop at the first
% fully checked candidate or after at most eight attempts.
for refinementIndex = 1:8
    trajectoryStep = struct( ...
        'Formulation',             "scaledClock", ...
        'SegmentCount',            segmentCount, ...
        'Planes',                  separatingPlanes, ...
        'RoundoffReserve_units',   roundoffReserve_units, ...
        'MaximumMotionDuration_s', fixedMotionDuration_s, ...
        'FixedClock',              true, ...
        'MinimumMotionDuration_s', 0, ...
        'SegmentRatio',            segmentTimeRatios, ...
        'IntrinsicVariationEnabled', false, ...
        'ConstraintBase',          savedTrajectoryConstraints);
    [refinedControl_units, refinedSegmentTime_s, refinementExitFlag, solverOutput, savedTrajectoryConstraints] = ...
        bmtpEngine.optimization.solveTrajectoryStep(solverRequest, trajectoryStep);
    diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, solverOutput);
    diagnostics.TravelRefinementExitFlag             = refinementExitFlag;
    diagnostics.TravelRefinementOptimizationConverged = solverOutput.OptimizationConverged;
    if ~bmtpEngine.optimization.hasUsableConicIterate(refinedControl_units, refinementExitFlag)
        break
    end
    % Do not inspect a refinement whose full line-constraint set is unresolved.
    if ~solverOutput.ConstraintGenerationComplete
        break
    end
    physicalSegmentTime_s = refinedSegmentTime_s(:);
    refinedMotion = struct( ...
        'ControlPoint_units',       refinedControl_units, ...
        'SegmentTime_s',            physicalSegmentTime_s, ...
        'FinalTime_s',              solverRequest.InitialState.time_s + sum(physicalSegmentTime_s), ...
        'GivenPower_units',         []);
    refinedMotionCheck = bmtpEngine.validation.checkFinalMotion(solverRequest, ...
        refinedMotion, roundoffReserve_units, separationTarget_units);
    unverifiedPairs = ~reshape([refinedMotionCheck.Planes.Verified], ...
        size(refinedMotionCheck.Planes)) & refinedMotionCheck.RegionActiveBySegment;
    if any(unverifiedPairs, "all")
        % Rebuild lines around the original retained curve and times. A new
        % attempt is useful only if every required line is verified and the
        % rebuilt set differs from the one that produced this rejected curve.
        [rebuiltSeparatingPlanes, ~, allRequiredLinesVerified] = ...
            bmtpEngine.separation.createTimeScopedPlanes(originalControl_units, ...
            originalSegmentTime_s, solverRequest, ...
            separationTarget_units, roundoffReserve_units);
        planeSetChanged = ~isequaln(rebuiltSeparatingPlanes, separatingPlanes);
        if ~allRequiredLinesVerified || ~planeSetChanged
            break
        end
        separatingPlanes = rebuiltSeparatingPlanes;
        continue
    end
    if ~refinedMotionCheck.Passed
        break
    end
    refinedLength_units = controlPolygonLength(refinedControl_units);
    % Keep the original motion unless all checks pass and the measured
    % control-polygon length is strictly shorter.
    refinementIsBetter = refinedLength_units < selectedLength_units;
    if refinementIsBetter
        selectedControl_units    = refinedControl_units;
        selectedSegmentTime_s    = refinedSegmentTime_s;
        selectedPlanes           = refinedMotionCheck.Planes;
        selectedPairs            = refinedMotionCheck.RegionActiveBySegment;
        selectedSolverMessage    = "A travel-shortened time-scoped feasible iterate was retained.";
        selectedLength_units     = refinedLength_units;
        travelRefinementAccepted = true;
    end
    break
end

%% Section 3: Return The Selected Motion And Its Matching Lines

if travelRefinementAccepted
    result.ControlPoint_units = selectedControl_units;
    result.SegmentTime_s      = selectedSegmentTime_s;
    result.Planes             = selectedPlanes;
    result.TaggedPairs        = selectedPairs;
    result.SolverMessage      = selectedSolverMessage;
    diagnostics.ApplicablePairCount     = nnz(result.TaggedPairs);
    diagnostics.TaggedPairCount         = nnz(result.TaggedPairs);
    diagnostics.FinalCollisionPairCount = 0;
    diagnostics.SolverMessage           = result.SolverMessage;
end
diagnostics.TravelRefinementFinalLength_units = selectedLength_units;
diagnostics.TravelRefinementFinalDuration_s   = sum(selectedSegmentTime_s);
diagnostics.TravelRefinementAccepted          = travelRefinementAccepted;
end

%% Section 4: Local Functions

function length_units = controlPolygonLength(controlPoint_units)
    % Sum distances between adjacent Bezier controls over every segment.
    controlEdgeVectors_units = diff(controlPoint_units, 1, 2);
    length_units             = sum(vecnorm(controlEdgeVectors_units, 2, 3), "all");
end
