function [result, diagnostics] = solveActivePairTrajectory(request, warmStart, diagnostics, target_units, reserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.optimization.solveActivePairTrajectory( ...
%       request, warmStart, diagnostics, target_units, reserve_units)
%**************************************************************************
% PURPOSE
%   - Alternate common-clock BMTP trajectory solves with separating planes
%     only for curve-region pairs encountered by the current iterate.
%   - This function generates proposals; public independent validation
%     remains authoritative.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked BMTP request and exact prepared regions.
%   - warmStart (scalar struct)
%       Initial controls, segment count, and applicable region pairs.
%   - diagnostics (scalar struct)
%       Diagnostics accumulated by the caller.
%   - target_units (nonnegative numeric scalar)
%       Required obstacle-side separation target.
%   - reserve_units (nonnegative numeric scalar)
%       Numerical reserve applied on the trajectory side.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Best sampled-clear controls and any accepted proven travel
%       refinement. Expected infeasibility returns Success = false.
%   - diagnostics (scalar struct)
%       Updated conic, plane, collision-pair, and refinement measurements.
%**************************************************************************
% UNITS
%   - Position and clearance are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Initialize The Active Pair Set

segmentCount                           = warmStart.SegmentCount;
regionCount                            = numel(request.Regions_units);
regionActiveBySegment                  = warmStart.RegionActiveBySegment;
feasibleControl_units                  = warmStart.ControlPoint_units;
diagnostics.ConicSolver                = bmtpEngine.optimization.accumulateConicDiagnostics();
diagnostics.TrajectorySocpCount        = 0;
diagnostics.PlaneSocpCount             = 0;
diagnostics.TransientPlaneRemovalCount = 0;
bestControl_units                      = zeros(0, request.Degree + 1, 2);
bestTimes_s                            = NaN;
bestDuration_s                         = Inf;
bestPreparedMotion                     = struct('Success', false);
bestProof                        = struct('Passed', false);
taggedPairs                            = false(segmentCount, regionCount);
emptyPlane                             = bmtpEngine.separation.createEmptyPlane();
planes                                 = repmat(emptyPlane, segmentCount, regionCount);
bestPlanes                             = planes;
bestTaggedPairs                        = taggedPairs;
bestSolverMessage                      = "";
lastAttemptMessage                     = "The active-pair BMTP iteration limit was reached.";
failureStage                           = "optimization";
failureKind                            = "iterationLimit";
alternativeGuideEligible               = true;

%% Section 2: Alternate Trajectory And Plane Updates

maximumIterationCount = request.MaximumAlternatingIterations;
constraintBase        = struct();
for iterationIndex = 1:maximumIterationCount
    diagnostics.IterationCount = iterationIndex;
    trajectoryPlanes           = planes;
    if isempty(bestControl_units) && nnz([planes.Active]) > segmentCount * request.Degree
        trajectoryPlanes = bmtpEngine.separation.removeRedundantPlanes( ...
            planes, request.Limits, reserve_units, true);
        diagnostics.TransientPlaneRemovalCount = diagnostics.TransientPlaneRemovalCount + ...
            nnz([planes.Active]) - nnz([trajectoryPlanes.Active]);
    end

    [trialControl_units, trialTimes_s, exitFlag, output, constraintBase] = ...
        bmtpEngine.optimization.solveTrajectoryStep( ...
        segmentCount, request.Degree, request.InitialState, request.GoalState, request.Limits, ...
        trajectoryPlanes, reserve_units, request.MotionHorizon_s, request.TrajectoryOptions, ...
        ones(segmentCount, 1), false, true, constraintBase);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + output.SolveCount;
    diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    if ~bmtpEngine.optimization.hasUsableConicIterate(trialControl_units, exitFlag)
        lastAttemptMessage = "Trajectory SOCP failed: " + string(output.message);
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

    collisionPairs = bmtpEngine.separation.findSampledObstacleOverlaps(trialControl_units, ...
        request.Regions_units, request.RegionMinimum_units, request.RegionMaximum_units, ...
        regionActiveBySegment);
    duration_s = sum(trialTimes_s);
    % Sampling only guides optimization. An iterate becomes the feasible
    % best iterate so far when every applicable pair holds a proven separating
    % line; a pair the proof cannot verify is a collision to separate.
    trialProof    = struct('Passed', false);
    trialPreparedMotion = struct('Success', false);
    if ~any(collisionPairs, 'all')
        [trialProof, trialPreparedMotion] = proveTravelCandidate( ...
            request, trialControl_units, trialTimes_s, reserve_units, target_units);
        proofIsDecisive = trialPreparedMotion.Success && trialProof.WorkspacePassed && ...
            trialProof.DynamicsPassed && trialProof.ContinuityPassed;
        if proofIsDecisive
            collisionPairs = unprovenPairsBySegment(trialProof, ...
                trialPreparedMotion.SegmentTime_s, trialTimes_s, regionActiveBySegment);
        end
    end
    newPairs    = collisionPairs & ~taggedPairs;
    taggedPairs = taggedPairs | newPairs;
    trialWasRetained = false;
    if ~any(collisionPairs, 'all')
        retainedImprovement_s = bestDuration_s - duration_s;
        feasibleControl_units = trialControl_units;
        if duration_s < bestDuration_s
            bestControl_units          = trialControl_units;
            bestTimes_s                = trialTimes_s;
            bestDuration_s             = duration_s;
            bestPreparedMotion         = struct('Success', false);
            bestProof            = struct('Passed', false);
            bestPlanes                 = trajectoryPlanes;
            bestTaggedPairs            = reshape([bestPlanes.Active], size(bestPlanes));
            bestSolverMessage          = "A complete active-pair feasible iterate was retained.";
            trialWasRetained           = true;
            if trialProof.Passed
                bestPreparedMotion = trialPreparedMotion;
                bestProof    = trialProof;
            end
        end
        improvementReachedTolerance = isfinite(retainedImprovement_s) && ...
            retainedImprovement_s <= request.Options.ArrivalTimeTolerance_s;
        if improvementReachedTolerance
            diagnostics.Converged = true;
            if trialWasRetained
                bestSolverMessage = "The feasible arrival improvement reached tolerance.";
            end
            break
        end
        planes(:)   = emptyPlane;
        activePairs = taggedPairs;
    elseif any(newPairs, 'all')
        activePairs = newPairs;
    else
        lastAttemptMessage = "A tagged pair crossed its retained separating plane.";
        failureStage             = "proposal";
        failureKind              = "retainedSeparatingPlaneViolated";
        alternativeGuideEligible = true;
        break
    end

    updateFailed = false;
    for pairIndex = reshape(find(activePairs), 1, [])
        [segmentIndex, regionIndex] = ind2sub(size(activePairs), pairIndex);
        [plane, planeExitFlag, planeOutput] = bmtpEngine.separation.solveMaximumMarginLine( ...
            squeeze(feasibleControl_units(segmentIndex, :, :)), request.Regions_units{regionIndex}, ...
            target_units, reserve_units, request.TrajectoryOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ...
            ~(isfield(planeOutput, 'IsAnalytic') && planeOutput.IsAnalytic);
        diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
            diagnostics.ConicSolver, planeOutput);
        planeUpdateFailed = (planeExitFlag <= 0 && planeExitFlag ~= -7) || ~plane.Active;
        if planeUpdateFailed
            lastAttemptMessage = "A separating-plane update failed.";
            failureStage             = "proposal";
            failureKind              = "separatingPlaneUpdateUnavailable";
            alternativeGuideEligible = true;
            updateFailed  = true;
            break
        end
        planes(segmentIndex, regionIndex) = plane;
    end
    if updateFailed
        break
    end
    if trialWasRetained
        bestPlanes      = planes;
        bestTaggedPairs = reshape([bestPlanes.Active], size(bestPlanes));
    end
end

%% Section 3: Shorten Travel At The Retained Arrival

diagnostics.TravelRefinementAttempted = ~isempty(bestControl_units);
diagnostics.TravelRefinementAccepted  = false;
if ~isempty(bestControl_units)
    [shortControl_units, shortTimes_s, shortFlag, shortOutput] = bmtpEngine.optimization.solveTrajectoryStep( ...
        segmentCount, request.Degree, request.InitialState, request.GoalState, request.Limits, ...
        bestPlanes, reserve_units, bestDuration_s, request.TrajectoryOptions, ...
        ones(segmentCount, 1), true);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + shortOutput.SolveCount;
    diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, shortOutput);
    if bmtpEngine.optimization.hasUsableConicIterate(shortControl_units, shortFlag)
        overlaps = bmtpEngine.separation.findSampledObstacleOverlaps(shortControl_units, ...
            request.Regions_units, request.RegionMinimum_units, request.RegionMaximum_units, ...
            regionActiveBySegment);
        if ~any(overlaps, 'all')
            originalLength_units = sum(vecnorm(diff(bestControl_units, 1, 2), 2, 3), 'all');
            shortLength_units    = sum(vecnorm(diff(shortControl_units, 1, 2), 2, 3), 'all');
            [shortProof, shortPreparedMotion] = proveTravelCandidate( ...
                request, shortControl_units, shortTimes_s, reserve_units, target_units);
            if shortLength_units <= originalLength_units && shortProof.Passed
                bestControl_units                  = shortControl_units;
                bestTimes_s                        = shortTimes_s;
                bestPreparedMotion                 = shortPreparedMotion;
                bestProof                    = shortProof;
                diagnostics.TravelRefinementAccepted = true;
            end
        end
    end
end

%% Section 4: Return The Best Sampled-Clear Proposal

selectedControl_units      = bestControl_units;
selectedTimes_s            = bestTimes_s;
selectedPlanes             = bestPlanes;
selectedPairs              = bestTaggedPairs;
selectedCollisionPairCount = 0;
solverMessage              = lastAttemptMessage;
if ~isempty(bestControl_units)
    solverMessage = bestSolverMessage;
    failureStage             = "";
    failureKind              = "";
    alternativeGuideEligible = false;
    if bestProof.Passed
        selectedControl_units      = bestPreparedMotion.ControlPoint_units;
        selectedTimes_s            = bestPreparedMotion.SegmentTime_s;
        selectedPlanes             = bestProof.Planes;
        selectedPairs              = bestProof.RegionActiveBySegment;
        selectedCollisionPairCount = bestProof.AllPairCount - ...
            bestProof.VerifiedPairCount;
    end
end
diagnostics.ApplicablePairCount     = nnz(selectedPairs);
diagnostics.FinalCollisionPairCount = selectedCollisionPairCount;
diagnostics.TaggedPairCount         = nnz(selectedPairs);
diagnostics.SolverMessage           = solverMessage;
result = struct( ...
    'Success',            ~isempty(selectedControl_units), ...
    'SolverMessage',      solverMessage, ...
    'FailureStage',       failureStage, ...
    'FailureKind',        failureKind, ...
    'AlternativeGuideEligible', alternativeGuideEligible, ...
    'ControlPoint_units', selectedControl_units, ...
    'SegmentTime_s',      selectedTimes_s, ...
    'Planes',             selectedPlanes, ...
    'TaggedPairs',        selectedPairs, ...
    'PreparedMotion',     bestPreparedMotion, ...
    'Proof',        bestProof);
end

%% Section 5: Local Functions

function collisionPairs = unprovenPairsBySegment(proof, spanTime_s, segmentTime_s, regionActiveBySegment)
    % Map every unverified proven-span pair back to its optimizer segment.
    % Spans partition the segments in order; a uniform dilation of the span
    % clock preserves each span's fraction of the total motion time.
    failedPairs         = ~reshape([proof.Planes.Verified], size(proof.Planes)) & ...
        proof.RegionActiveBySegment;
    collisionPairs      = false(size(regionActiveBySegment));
    segmentEndFraction  = cumsum(segmentTime_s(:)) / sum(segmentTime_s);
    spanEndFraction     = cumsum(spanTime_s(:)) / sum(spanTime_s);
    spanMidFraction     = spanEndFraction - 0.5 * spanTime_s(:) / sum(spanTime_s);
    for spanIndex = reshape(find(any(failedPairs, 2)), 1, [])
        segmentIndex = find(segmentEndFraction >= spanMidFraction(spanIndex), 1, 'first');
        collisionPairs(segmentIndex, :) = collisionPairs(segmentIndex, :) | failedPairs(spanIndex, :);
    end
    collisionPairs = collisionPairs & regionActiveBySegment;
end

function [proof, preparedMotion] = proveTravelCandidate(request, controls_units, ...
        times_s, reserve_units, target_units)
    % Prepare and independently prove a shortened fixed-clock candidate.
    preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, controls_units, times_s);
    [proof, proofCache] = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, reserve_units, target_units);
    for refinementIndex = 1:10
        if proof.Passed || ~proof.WorkspacePassed || ...
                ~proof.DynamicsPassed || ~proof.ContinuityPassed
            break
        end
        failedPairs = ~reshape([proof.Planes.Verified], size(proof.Planes)) & ...
            proof.RegionActiveBySegment;
        splitMask = any(failedPairs, 2);
        if ~any(splitMask)
            break
        end
        preparedMotion = bmtpEngine.pipeline.prepareFinalMotion(request, preparedMotion.ControlPoint_units, ...
            preparedMotion.SegmentTime_s, preparedMotion.PrescribedPower_units, splitMask);
        [proof, proofCache] = bmtpEngine.validation.checkFinalMotion( ...
            request, preparedMotion, reserve_units, target_units, proofCache);
    end
end
