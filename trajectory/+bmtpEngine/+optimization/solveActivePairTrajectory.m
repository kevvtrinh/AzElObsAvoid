function [result, diagnostics] = solveActivePairTrajectory( ...
    solverRequest, warmStart, diagnostics, separationTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.optimization.solveActivePairTrajectory( ...
%       solverRequest, warmStart, diagnostics, separationTarget_units, roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Improve a motion whose segments share one duration. Add obstacle
%     constraints as the candidate encounters curve-segment/obstacle pairs.
%     A separating line keeps a curve segment and an obstacle on opposite sides.
%   - Return a candidate for the remaining motion checks and public independent
%     validation; solver success alone does not establish planner success.
%**************************************************************************
% INPUTS
%   - solverRequest (scalar struct)
%       Checked BMTP inputs and prepared obstacle polygons.
%   - warmStart (scalar struct)
%       Starting curve controls, segment count, and applicable obstacle pairs.
%   - diagnostics (scalar struct)
%       Diagnostics accumulated by the caller.
%   - separationTarget_units (nonnegative numeric scalar)
%       Required distance from each obstacle to its separating line.
%   - roundoffReserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Fastest retained candidate and any accepted shorter-path refinement.
%       Success = false when no candidate passes the selection checks below.
%       The caller must still validate the final motion.
%   - diagnostics (scalar struct)
%       Updated conic, plane, collision-pair, and refinement measurements.
%**************************************************************************
% UNITS
%   - Position and clearance are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Initialize Candidates And Obstacle Constraints

% Each pair identifies one curve segment and one obstacle region. Remember
% encountered pairs so later iterations continue to account for them. The
% starting curve supplies the first reference for finding separating lines.

segmentCount          = warmStart.SegmentCount;
regionCount           = numel(solverRequest.Regions_units);
regionActiveBySegment = warmStart.RegionActiveBySegment;
separationReferenceControl_units = warmStart.ControlPoint_units;

diagnostics.ConicSolver                = bmtpEngine.optimization.accumulateConicDiagnostics();
diagnostics.TrajectorySocpCount        = 0;
diagnostics.PlaneSocpCount             = 0;
diagnostics.TransientPlaneRemovalCount = 0;

bestControl_units  = zeros(0, solverRequest.Degree + 1, 2);
bestSegmentTime_s  = NaN;
bestDuration_s     = Inf;
bestPreparedMotion = struct('Success', false);
bestMotionCheck    = struct('Passed', false);

encounteredPairs     = false(segmentCount, regionCount);
emptyPlane           = bmtpEngine.separation.createEmptyPlane();
separatingPlanes     = repmat(emptyPlane, segmentCount, regionCount);
bestSeparatingPlanes = separatingPlanes;
bestPlanePairs       = encounteredPairs;

bestSolverMessage        = "";
lastAttemptMessage       = "The active-pair BMTP iteration limit was reached.";
failureStage             = "optimization";
failureKind              = "iterationLimit";
alternativeGuideEligible = true;

%% Section 2: Improve The Trajectory And Update Separating Lines

% Solve with the current lines, check the new curve, then update the lines
% for the next solve. Retain the fastest candidate found before stopping.

maximumIterationCount      = solverRequest.MaximumAlternatingIterations;
savedTrajectoryConstraints = struct();
for iterationIndex = 1:maximumIterationCount
    diagnostics.IterationCount = iterationIndex;
    trajectoryPlanes           = separatingPlanes;
    % Before a candidate is retained, reduce a large set of line constraints
    % by removing those already enforced by the other lines and workspace.
    if isempty(bestControl_units) && nnz([separatingPlanes.Active]) > segmentCount * solverRequest.Degree
        trajectoryPlanes = bmtpEngine.separation.removeRedundantPlanes( ...
            separatingPlanes, solverRequest.Limits, roundoffReserve_units, true);
        diagnostics.TransientPlaneRemovalCount = diagnostics.TransientPlaneRemovalCount + ...
            nnz([separatingPlanes.Active]) - nnz([trajectoryPlanes.Active]);
    end

    % Equal duration ratios let the solver choose one shared segment time.
    trajectoryStep = struct( ...
        'SegmentCount',              segmentCount, ...
        'Planes',                    trajectoryPlanes, ...
        'RoundoffReserve_units',     roundoffReserve_units, ...
        'MaximumMotionDuration_s',   solverRequest.MotionHorizon_s, ...
        'SegmentRatio',              ones(segmentCount, 1), ...
        'FixedClock',                false, ...
        'IntrinsicVariationEnabled', true, ...
        'ConstraintBase',            savedTrajectoryConstraints);
    [trialControl_units, trialSegmentTime_s, exitFlag, solverOutput, savedTrajectoryConstraints] = ...
        bmtpEngine.optimization.solveTrajectoryStep(solverRequest, trajectoryStep);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + solverOutput.SolveCount;
    diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, solverOutput);
    % Record why this attempt stopped so the caller can decide whether a
    % different route is allowed. A numerical failure must not be hidden.
    if ~bmtpEngine.optimization.hasUsableConicIterate(trialControl_units, exitFlag)
        lastAttemptMessage = "Trajectory SOCP failed: " + string(solverOutput.message);
        if exitFlag == 0
            failureStage             = "optimization";
            failureKind              = "trajectorySolverIterationLimit";
            alternativeGuideEligible = true;
        elseif exitFlag == -2
            failureStage             = "proposal";
            failureKind              = "trajectorySubproblemInfeasible";
            alternativeGuideEligible = true;
        else
            failureStage             = "numericalSolver";
            failureKind              = "optimizerIterateUnavailable";
            alternativeGuideEligible = false;
        end
        break
    end

    % Sample checks find obvious overlaps quickly. If none are found, check
    % the complete motion and all obstacle pairs between sample points.
    collisionPairs = bmtpEngine.separation.findSampledObstacleOverlaps(trialControl_units, ...
        solverRequest.Regions_units, solverRequest.RegionMinimum_units, solverRequest.RegionMaximum_units, ...
        regionActiveBySegment);
    trialDuration_s     = sum(trialSegmentTime_s);
    trialMotionCheck    = struct('Passed', false);
    trialPreparedMotion = struct('Success', false);
    if ~any(collisionPairs, 'all')
        trialPreparedMotion = bmtpEngine.pipeline.prepareFinalMotion( ...
            solverRequest, trialControl_units, trialSegmentTime_s);
        [trialPreparedMotion, trialMotionCheck] = bmtpEngine.pipeline.refineMotionSeparation( ...
            solverRequest, trialPreparedMotion, roundoffReserve_units, separationTarget_units);
        nonCollisionChecksPassed = trialPreparedMotion.Success && trialMotionCheck.WorkspacePassed && ...
            trialMotionCheck.DynamicsPassed && trialMotionCheck.ContinuityPassed;
        % Once workspace, motion limits, and joins pass, treat every obstacle
        % pair that could not be proved clear as another pair to separate.
        if nonCollisionChecksPassed
            unverifiedPairs = ~reshape([trialMotionCheck.Planes.Verified], size(trialMotionCheck.Planes)) & ...
                trialMotionCheck.RegionActiveBySegment;
            % Every piece retains its original optimizer segment even after
            % several selective splits. Add each unresolved pair to that row.
            for pieceIndex = reshape(find(any(unverifiedPairs, 2)), 1, [])
                segmentIndex = trialPreparedMotion.SourceSegmentIndex(pieceIndex);
                collisionPairs(segmentIndex, :) = collisionPairs(segmentIndex, :) | unverifiedPairs(pieceIndex, :);
            end
            collisionPairs = collisionPairs & regionActiveBySegment;
        end
    end
    % Keep every newly encountered pair. A sampled-clear candidate can still
    % need later validation when its other motion checks did not pass.
    newConstraintPairs = collisionPairs & ~encounteredPairs;
    encounteredPairs   = encounteredPairs | newConstraintPairs;
    trialWasRetained   = false;
    if ~any(collisionPairs, 'all')
        % An unproved candidate can be retained for later checking; save the
        % prepared motion and its check results only when every check passed.
        arrivalImprovement_s             = bestDuration_s - trialDuration_s;
        separationReferenceControl_units = trialControl_units;
        if trialDuration_s < bestDuration_s
            bestControl_units    = trialControl_units;
            bestSegmentTime_s    = trialSegmentTime_s;
            bestDuration_s       = trialDuration_s;
            bestPreparedMotion   = struct('Success', false);
            bestMotionCheck      = struct('Passed', false);
            bestSeparatingPlanes = trajectoryPlanes;
            bestPlanePairs       = reshape([bestSeparatingPlanes.Active], size(bestSeparatingPlanes));
            bestSolverMessage    = "A complete active-pair feasible iterate was retained.";
            trialWasRetained     = true;
            if trialMotionCheck.Passed
                bestPreparedMotion = trialPreparedMotion;
                bestMotionCheck    = trialMotionCheck;
            end
        end
        improvementReachedTolerance = isfinite(arrivalImprovement_s) && ...
            arrivalImprovement_s <= solverRequest.Options.ArrivalTimeTolerance_s;
        if improvementReachedTolerance
            diagnostics.Converged = true;
            if trialWasRetained
                bestSolverMessage = "The feasible arrival improvement reached tolerance.";
            end
            break
        end
        % Rebuild the encountered lines around the newly clear curve.
        separatingPlanes(:) = emptyPlane;
        pairsToUpdate       = encounteredPairs;
    elseif any(newConstraintPairs, 'all')
        % Add constraints only for pairs that this attempt discovered.
        pairsToUpdate = newConstraintPairs;
    else
        % An already constrained pair overlapped again. Stop this attempt.
        lastAttemptMessage       = "A tagged pair crossed its retained separating plane.";
        failureStage             = "proposal";
        failureKind              = "retainedSeparatingPlaneViolated";
        alternativeGuideEligible = true;
        break
    end

    % Find lines using the most recent sampled-clear reference curve, then
    % apply those lines to the next trajectory solve.
    lineUpdateFailed = false;
    for pairIndex = reshape(find(pairsToUpdate), 1, [])
        [segmentIndex, regionIndex] = ind2sub(size(pairsToUpdate), pairIndex);
        [plane, planeExitFlag, planeOutput] = bmtpEngine.separation.solveMaximumMarginLine( ...
            squeeze(separationReferenceControl_units(segmentIndex, :, :)), solverRequest.Regions_units{regionIndex}, ...
            separationTarget_units, roundoffReserve_units, solverRequest.TrajectoryOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ...
            ~(isfield(planeOutput, 'IsAnalytic') && planeOutput.IsAnalytic);
        diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
            diagnostics.ConicSolver, planeOutput);
        planeUpdateFailed = (planeExitFlag <= 0 && planeExitFlag ~= -7) || ~plane.Active;
        if planeUpdateFailed
            lastAttemptMessage       = "A separating-plane update failed.";
            failureStage             = "proposal";
            failureKind              = "separatingPlaneUpdateUnavailable";
            alternativeGuideEligible = true;
            lineUpdateFailed         = true;
            break
        end
        separatingPlanes(segmentIndex, regionIndex) = plane;
    end
    if lineUpdateFailed
        break
    end
    if trialWasRetained
        bestSeparatingPlanes = separatingPlanes;
        bestPlanePairs       = reshape([bestSeparatingPlanes.Active], size(bestSeparatingPlanes));
    end
end

%% Section 3: Shorten The Path Without Delaying Arrival

% Fix the best duration and seek a shorter control polygon. Its length is
% the sum of distances between adjacent control points, not the curve's exact
% length. The arrival-time solve above only minimizes time, so an axis that
% does not limit the arrival can wander wherever the conic solver leaves it;
% that curve may pass an obstacle it was never constrained by. Shortening
% then steers back toward that obstacle, so each pair the shorter polygon
% meets gets a separating line built around the retained curve and the
% solve repeats with that line. The attempt bound matches the timed
% refinement. Accept only if preparation succeeds, the full motion checks
% pass, the polygon length does not increase, and the prepared arrival is
% not later than the retained one beyond a small allowance. The solver clock
% stays the retained one, but preparation may stretch it slightly to meet
% its control-point bounds: 2.7 parts in ten million was measured on an
% 8.5 s motion (2.3 microseconds). Allow one part per million so that
% stretch passes and anything larger is refused.

diagnostics.TravelRefinementAttempted = ~isempty(bestControl_units);
diagnostics.TravelRefinementAccepted  = false;
refinementAttemptLimit = 8;
refinementPlanes       = bestSeparatingPlanes;
retainedFinalTime_s    = solverRequest.InitialState.time_s + bestDuration_s;
if bestMotionCheck.Passed
    retainedFinalTime_s = bestPreparedMotion.FinalTime_s;
end
arrivalAllowance_s = 1e-6 * (retainedFinalTime_s - solverRequest.InitialState.time_s);
for refinementIndex = 1:refinementAttemptLimit * ~isempty(bestControl_units)
    travelRefinementStep = struct( ...
        'SegmentCount',              segmentCount, ...
        'Planes',                    refinementPlanes, ...
        'RoundoffReserve_units',     roundoffReserve_units, ...
        'MaximumMotionDuration_s',   bestDuration_s, ...
        'SegmentRatio',              ones(segmentCount, 1), ...
        'FixedClock',                true, ...
        'IntrinsicVariationEnabled', true, ...
        'ConstraintBase',            struct());
    [refinedControl_units, refinedSegmentTime_s, refinementExitFlag, refinementOutput] = ...
        bmtpEngine.optimization.solveTrajectoryStep(solverRequest, travelRefinementStep);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + refinementOutput.SolveCount;
    diagnostics.ConicSolver         = bmtpEngine.optimization.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, refinementOutput);
    if ~bmtpEngine.optimization.hasUsableConicIterate(refinedControl_units, refinementExitFlag)
        break
    end

    % Sample checks find obvious overlaps quickly. If none are found, run
    % the complete checks and treat every unproved pair as an overlap, the
    % same way the arrival-time loop above does.
    refinementOverlaps = bmtpEngine.separation.findSampledObstacleOverlaps(refinedControl_units, ...
        solverRequest.Regions_units, solverRequest.RegionMinimum_units, solverRequest.RegionMaximum_units, ...
        regionActiveBySegment);
    if ~any(refinementOverlaps, 'all')
        refinedPreparedMotion = bmtpEngine.pipeline.prepareFinalMotion( ...
            solverRequest, refinedControl_units, refinedSegmentTime_s);
        [refinedPreparedMotion, refinedMotionCheck] = bmtpEngine.pipeline.refineMotionSeparation( ...
            solverRequest, refinedPreparedMotion, roundoffReserve_units, separationTarget_units);
        if refinedMotionCheck.Passed
            retainedControlPolygonLength_units = sum(vecnorm(diff(bestControl_units, 1, 2), 2, 3), 'all');
            refinedControlPolygonLength_units  = sum(vecnorm(diff(refinedControl_units, 1, 2), 2, 3), 'all');
            arrivalIsNotLater = refinedPreparedMotion.Success && ...
                refinedPreparedMotion.FinalTime_s <= retainedFinalTime_s + arrivalAllowance_s;
            if arrivalIsNotLater && refinedControlPolygonLength_units <= retainedControlPolygonLength_units
                bestControl_units    = refinedControl_units;
                bestSegmentTime_s    = refinedSegmentTime_s;
                bestPreparedMotion   = refinedPreparedMotion;
                bestMotionCheck      = refinedMotionCheck;
                bestSeparatingPlanes = refinementPlanes;
                bestPlanePairs       = reshape([bestSeparatingPlanes.Active], size(bestSeparatingPlanes));
                diagnostics.TravelRefinementAccepted = true;
            end
            break
        end
        % Only unresolved obstacle pairs can be fixed by adding a line. A
        % workspace, motion-limit, or join failure ends the refinement.
        nonCollisionChecksPassed = refinedPreparedMotion.Success && refinedMotionCheck.WorkspacePassed && ...
            refinedMotionCheck.DynamicsPassed && refinedMotionCheck.ContinuityPassed;
        if ~nonCollisionChecksPassed
            break
        end
        unverifiedPairs = ~reshape([refinedMotionCheck.Planes.Verified], size(refinedMotionCheck.Planes)) & ...
            refinedMotionCheck.RegionActiveBySegment;
        for pieceIndex = reshape(find(any(unverifiedPairs, 2)), 1, [])
            segmentIndex = refinedPreparedMotion.SourceSegmentIndex(pieceIndex);
            refinementOverlaps(segmentIndex, :) = refinementOverlaps(segmentIndex, :) | unverifiedPairs(pieceIndex, :);
        end
        refinementOverlaps = refinementOverlaps & regionActiveBySegment;
    end

    % Stop when none of the overlapping pairs is new, because a pair that
    % already has a line and still overlaps cannot be fixed by another line
    % (an old overlap beside a new pair does not stop the loop). Also stop
    % when no solve would follow the new lines. Otherwise build a line for
    % each new pair around the retained curve, which passed the sampled
    % checks against that obstacle, and require the line to be verified.
    newRefinementPairs = refinementOverlaps & ~reshape([refinementPlanes.Active], size(refinementPlanes));
    if ~any(newRefinementPairs, 'all') || refinementIndex == refinementAttemptLimit
        break
    end
    lineUpdateFailed = false;
    for pairIndex = reshape(find(newRefinementPairs), 1, [])
        [segmentIndex, regionIndex] = ind2sub(size(newRefinementPairs), pairIndex);
        [plane, planeExitFlag, planeOutput] = bmtpEngine.separation.solveMaximumMarginLine( ...
            squeeze(bestControl_units(segmentIndex, :, :)), solverRequest.Regions_units{regionIndex}, ...
            separationTarget_units, roundoffReserve_units, solverRequest.TrajectoryOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ...
            ~(isfield(planeOutput, 'IsAnalytic') && planeOutput.IsAnalytic);
        diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
            diagnostics.ConicSolver, planeOutput);
        if (planeExitFlag <= 0 && planeExitFlag ~= -7) || ~plane.Active || ~plane.Verified
            lineUpdateFailed = true;
            break
        end
        refinementPlanes(segmentIndex, regionIndex) = plane;
    end
    if lineUpdateFailed
        break
    end
end

%% Section 4: Return The Retained Candidate And Matching Checks

% When a full check passed, use its prepared controls, durations, and lines
% together: preparation may have split the original segments. Otherwise
% return the retained candidate for the caller's remaining checks.

selectedControl_units      = bestControl_units;
selectedSegmentTime_s      = bestSegmentTime_s;
selectedPlanes             = bestSeparatingPlanes;
selectedPairs              = bestPlanePairs;
selectedCollisionPairCount = 0;
solverMessage              = lastAttemptMessage;
if ~isempty(bestControl_units)
    solverMessage            = bestSolverMessage;
    failureStage             = "";
    failureKind              = "";
    alternativeGuideEligible = false;
    if bestMotionCheck.Passed
        selectedControl_units      = bestPreparedMotion.ControlPoint_units;
        selectedSegmentTime_s      = bestPreparedMotion.SegmentTime_s;
        selectedPlanes             = bestMotionCheck.Planes;
        selectedPairs              = bestMotionCheck.RegionActiveBySegment;
        selectedCollisionPairCount = bestMotionCheck.AllPairCount - ...
            bestMotionCheck.VerifiedPairCount;
    end
end
diagnostics.ApplicablePairCount     = nnz(selectedPairs);
diagnostics.FinalCollisionPairCount = selectedCollisionPairCount;
diagnostics.TaggedPairCount         = nnz(selectedPairs);
diagnostics.SolverMessage           = solverMessage;
result = struct( ...
    'Success',                  ~isempty(selectedControl_units), ...
    'SolverMessage',            solverMessage, ...
    'FailureStage',             failureStage, ...
    'FailureKind',              failureKind, ...
    'AlternativeGuideEligible', alternativeGuideEligible, ...
    'ControlPoint_units',       selectedControl_units, ...
    'SegmentTime_s',            selectedSegmentTime_s, ...
    'Planes',                   selectedPlanes, ...
    'TaggedPairs',              selectedPairs, ...
    'PreparedMotion',           bestPreparedMotion, ...
    'Proof',                    bestMotionCheck);
end
