function [result, diagnostics] = solveActivePairTrajectory(request, warmStart, diagnostics, target_units, reserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.solveActivePairTrajectory( ...
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
%       Best sampled-clear controls and any accepted certified travel
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
diagnostics.ConicSolver                = bmtpEngine.accumulateConicDiagnostics();
diagnostics.TrajectorySocpCount        = 0;
diagnostics.PlaneSocpCount             = 0;
diagnostics.TransientPlaneRemovalCount = 0;
bestControl_units                      = zeros(0, request.Degree + 1, 2);
bestTimes_s                            = NaN;
bestDuration_s                         = Inf;
bestPreparedMotion                     = struct('Success', false);
bestCertificate                        = struct('Passed', false);
taggedPairs                            = false(segmentCount, regionCount);
emptyPlane                             = bmtpEngine.createEmptyPlane();
planes                                 = repmat(emptyPlane, segmentCount, regionCount);
solverMessage                          = "The active-pair BMTP iteration limit was reached.";

%% Section 2: Alternate Trajectory And Plane Updates

maximumIterationCount = 16;
for iterationIndex = 1:maximumIterationCount
    diagnostics.IterationCount = iterationIndex;
    trajectoryPlanes           = planes;
    if isempty(bestControl_units) && nnz([planes.Active]) > segmentCount * request.Degree
        trajectoryPlanes = bmtpEngine.removeRedundantPlanes( ...
            planes, request.Limits, reserve_units, true);
        diagnostics.TransientPlaneRemovalCount = diagnostics.TransientPlaneRemovalCount + ...
            nnz([planes.Active]) - nnz([trajectoryPlanes.Active]);
    end

    [trialControl_units, trialTimes_s, exitFlag, output] = bmtpEngine.solveTrajectoryStep( ...
        segmentCount, request.Degree, request.InitialState, request.GoalState, request.Limits, ...
        trajectoryPlanes, reserve_units, request.MotionHorizon_s, request.TrajectoryOptions, ...
        ones(segmentCount, 1), false);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + output.SolveCount;
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    if ~bmtpEngine.hasUsableConicIterate(trialControl_units, exitFlag)
        solverMessage = "Trajectory SOCP failed: " + string(output.message);
        break
    end

    collisionPairs = bmtpEngine.findSampledObstacleOverlaps(trialControl_units, ...
        request.Regions_units, request.RegionMinimum_units, request.RegionMaximum_units, ...
        regionActiveBySegment);
    duration_s                           = sum(trialTimes_s);
    diagnostics.FinalCollisionPairCount = nnz(collisionPairs);
    newPairs                             = collisionPairs & ~taggedPairs;
    taggedPairs                          = taggedPairs | newPairs;
    if ~any(collisionPairs, 'all')
        retainedImprovement_s = bestDuration_s - duration_s;
        feasibleControl_units = trialControl_units;
        if duration_s < bestDuration_s
            bestControl_units       = trialControl_units;
            bestTimes_s             = trialTimes_s;
            bestDuration_s          = duration_s;
            diagnostics.BestDuration_s = duration_s;
        end
        improvementReachedTolerance = isfinite(retainedImprovement_s) && ...
            retainedImprovement_s <= request.Options.ArrivalTimeTolerance_s;
        if improvementReachedTolerance
            diagnostics.Converged = true;
            solverMessage         = "The feasible arrival improvement reached tolerance.";
            break
        end
        planes(:)   = emptyPlane;
        activePairs = taggedPairs;
    elseif any(newPairs, 'all')
        activePairs = newPairs;
    else
        solverMessage = "A tagged pair crossed its retained separating plane.";
        break
    end

    updateFailed = false;
    for pairIndex = reshape(find(activePairs), 1, [])
        [segmentIndex, regionIndex] = ind2sub(size(activePairs), pairIndex);
        [plane, planeExitFlag, planeOutput] = bmtpEngine.solveMaximumMarginLine( ...
            squeeze(feasibleControl_units(segmentIndex, :, :)), request.Regions_units{regionIndex}, ...
            target_units, reserve_units, request.TrajectoryOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ...
            ~(isfield(planeOutput, 'IsAnalytic') && planeOutput.IsAnalytic);
        diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics( ...
            diagnostics.ConicSolver, planeOutput);
        planeUpdateFailed = (planeExitFlag <= 0 && planeExitFlag ~= -7) || ~plane.Active;
        if planeUpdateFailed
            solverMessage = "A separating-plane update failed.";
            updateFailed  = true;
            break
        end
        planes(segmentIndex, regionIndex) = plane;
    end
    if updateFailed
        break
    end
end

%% Section 3: Shorten Travel At The Retained Arrival

diagnostics.TravelRefinementAttempted = ~isempty(bestControl_units);
diagnostics.TravelRefinementAccepted  = false;
if ~isempty(bestControl_units)
    [shortControl_units, shortTimes_s, shortFlag, shortOutput] = bmtpEngine.solveTrajectoryStep( ...
        segmentCount, request.Degree, request.InitialState, request.GoalState, request.Limits, ...
        planes, reserve_units, bestDuration_s, request.TrajectoryOptions, ...
        ones(segmentCount, 1), true);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + shortOutput.SolveCount;
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver, shortOutput);
    if bmtpEngine.hasUsableConicIterate(shortControl_units, shortFlag)
        overlaps = bmtpEngine.findSampledObstacleOverlaps(shortControl_units, ...
            request.Regions_units, request.RegionMinimum_units, request.RegionMaximum_units, ...
            regionActiveBySegment);
        if ~any(overlaps, 'all')
            originalLength_units = sum(vecnorm(diff(bestControl_units, 1, 2), 2, 3), 'all');
            shortLength_units    = sum(vecnorm(diff(shortControl_units, 1, 2), 2, 3), 'all');
            [shortCertificate, shortPreparedMotion] = certifyTravelCandidate( ...
                request, shortControl_units, shortTimes_s, reserve_units, target_units);
            if shortLength_units <= originalLength_units && shortCertificate.Passed
                bestControl_units                  = shortControl_units;
                bestTimes_s                        = shortTimes_s;
                bestPreparedMotion                 = shortPreparedMotion;
                bestCertificate                    = shortCertificate;
                diagnostics.TravelRefinementAccepted = true;
            end
        end
    end
end

%% Section 4: Return The Best Sampled-Clear Proposal

diagnostics.TaggedPairCount = nnz(taggedPairs);
diagnostics.SolverMessage   = solverMessage;
result = struct( ...
    'Success',            ~isempty(bestControl_units), ...
    'SolverMessage',      solverMessage, ...
    'ControlPoint_units', bestControl_units, ...
    'SegmentTime_s',      bestTimes_s, ...
    'Planes',             planes, ...
    'TaggedPairs',        taggedPairs, ...
    'PreparedMotion',     bestPreparedMotion, ...
    'Certificate',        bestCertificate);
end

%% Section 5: Local Functions

function [certificate, preparedMotion] = certifyTravelCandidate(request, controls_units, ...
        times_s, reserve_units, target_units)
    % Prepare and independently certify a shortened fixed-clock candidate.
    preparedMotion = bmtpEngine.prepareFinalMotion(request, controls_units, times_s);
    [certificate, certificateCache] = bmtpEngine.checkFinalMotion( ...
        request, preparedMotion, reserve_units, target_units);
    for refinementIndex = 1:10
        if certificate.Passed || ~certificate.WorkspacePassed || ...
                ~certificate.DynamicsPassed || ~certificate.ContinuityPassed
            break
        end
        failedPairs = ~reshape([certificate.Planes.Verified], size(certificate.Planes)) & ...
            certificate.RegionActiveBySegment;
        splitMask = any(failedPairs, 2);
        if ~any(splitMask)
            break
        end
        preparedMotion = bmtpEngine.prepareFinalMotion(request, preparedMotion.ControlPoint_units, ...
            preparedMotion.SegmentTime_s, preparedMotion.PrescribedPower_units, splitMask);
        [certificate, certificateCache] = bmtpEngine.checkFinalMotion( ...
            request, preparedMotion, reserve_units, target_units, certificateCache);
    end
end
