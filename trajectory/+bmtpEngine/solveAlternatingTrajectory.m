function [result, diagnostics] = solveAlternatingTrajectory(request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.solveAlternatingTrajectory( ...
%       request, warmStart, diagnostics, obstacleTarget_units, ...
%       roundoffReserve_units)
%
% PURPOSE
%   - Alternate trajectory and separating-line solves until a sampled-clear
%     motion is retained or the bounded iteration fails.
%
% INPUTS
%   - request, warmStart, diagnostics (scalar structs)
%       Checked engine request, feasible starting curve, and diagnostics.
%   - obstacleTarget_units, roundoffReserve_units (finite scalars)
%       Required obstacle-side target and numerical reserve in coordinate units.
%
% OUTPUTS
%   - result (scalar struct)
%       Best sampled-clear controls, timing, planes, tags, and failure reason.
%   - diagnostics (scalar struct)
%       Updated iteration, solver, overlap, and separating-line evidence.
%
% UNITS
%   - Position and clearance are coordinate units; time is seconds.
%

%% Section 1: Initialize The Alternating State

segmentCount = warmStart.SegmentCount;
diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics();
degree                = request.Degree;
regions_units           = request.Regions_units;
regionActiveBySegment = warmStart.RegionActiveBySegment;
feasibleControl_units   = warmStart.ControlPoint_units;
feasibleSegmentTime_s = warmStart.SegmentTime_s;
diagnostics.WarmStartDuration_s = segmentCount * feasibleSegmentTime_s;
bestControl_units = zeros(0, degree + 1, 2);
[bestSegmentTime_s, bestDuration_s] = deal(NaN, Inf);
diagnostics.RetainedBestTrialDuration_s = bestDuration_s;
taggedPairs           = false(segmentCount, numel(regions_units));
planes                = repmat(createEmptyPlane(), segmentCount, numel(regions_units));
optimizationHorizon_s = request.MotionHorizon_s;
solverMessage         = "The biconvex iteration limit was reached.";

%% Section 2: Alternate Trajectory And Separating-Line Solves

for iterationIndex = 1:35
    diagnostics.IterationCount = iterationIndex;
    usedRequestHorizon = optimizationHorizon_s == request.MotionHorizon_s;
    [trialControl_units, trialTime_s, exitFlag, output] = bmtpEngine.solveTrajectoryStep(segmentCount, degree, request.InitialState.position_units, request.GoalState.position_units, request.Limits, planes, roundoffReserve_units, optimizationHorizon_s, "earliestArrival", request.TrajectoryOptions);
    diagnostics.TrajectorySocpCount     = diagnostics.TrajectorySocpCount + 1;
    diagnostics.ConicSolver             = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    diagnostics.FinalTrajectoryExitFlag = exitFlag;
    if exitFlag <= 0 || isempty(trialControl_units)
        % Allow the longer starting duration while finding separating planes.
        % The final motion must still meet the requested horizon.
        canExpandHorizon = exitFlag == -2 && isempty(bestControl_units) && optimizationHorizon_s < diagnostics.WarmStartDuration_s;
        if canExpandHorizon
            optimizationHorizon_s = min(2 * optimizationHorizon_s, diagnostics.WarmStartDuration_s);
            continue;
        end
        % If fixed planes make the deadline infeasible, continue improving the
        % collision-free curve at its current duration within the iteration budget.
        if exitFlag == -2 && ~isempty(bestControl_units) && optimizationHorizon_s < bestDuration_s
            diagnostics.RetainedHorizonRetryCount = diagnostics.RetainedHorizonRetryCount + 1;
            optimizationHorizon_s = bestDuration_s;
            continue;
        end
        solverMessage = "Trajectory SOCP failed: " + string(output.message);
        break;
    end
    collisionPairs = bmtpEngine.findSampledObstacleOverlaps(trialControl_units, regions_units, request.RegionMinimum_units, request.RegionMaximum_units, regionActiveBySegment, 1201);
    diagnostics.TrialDuration_s(iterationIndex) = segmentCount * trialTime_s;
    diagnostics.CollisionPairCountHistory(iterationIndex) = nnz(collisionPairs);
    diagnostics.TrialWasCollisionFree(iterationIndex) = ~any(collisionPairs, "all");
    diagnostics.FinalCollisionPairCount = nnz(collisionPairs);
    previousTaggedPairs = taggedPairs;
    newPairs            = collisionPairs & ~taggedPairs;
    taggedPairs         = taggedPairs | newPairs;
    if ~any(collisionPairs, "all")
        optimizationHorizon_s     = request.MotionHorizon_s;
        previousDuration_s        = segmentCount * feasibleSegmentTime_s;
        feasibleControl_units       = trialControl_units;
        feasibleSegmentTime_s     = trialTime_s;
        duration_s                = segmentCount * trialTime_s;
        retainedBestImprovement_s = bestDuration_s - duration_s;
        % Promote this feasible trajectory only when its duration improves the incumbent.
        if duration_s < bestDuration_s
            bestControl_units   = trialControl_units;
            bestSegmentTime_s = trialTime_s;
            bestDuration_s    = duration_s;
            diagnostics.BestDuration_s              = duration_s;
            diagnostics.RetainedBestTrialDuration_s = duration_s;
        end
        improvement_s = previousDuration_s - duration_s;
        if improvement_s >= 0 && improvement_s <= request.Options.ArrivalTimeTolerance_s
            diagnostics.Converged = true;
            solverMessage = "The feasible arrival improvement reached tolerance.";
            break;
        end
        taggedPairSetUnchanged = isequal(taggedPairs, previousTaggedPairs);
        reusePlanes            = retainedBestImprovement_s <= request.Options.ArrivalTimeTolerance_s && taggedPairSetUnchanged;
        if reusePlanes
            diagnostics.PlaneReuseApplied = true;
            diagnostics.PlaneReuseCount   = diagnostics.PlaneReuseCount + 1;
            if usedRequestHorizon
                diagnostics.Converged = true;
                solverMessage = "The next trajectory SOCP would be unchanged.";
                break;
            end
            continue;
        end
        planes(:) = createEmptyPlane();
        activePairs = taggedPairs;
    % Restart alternating optimization when new obstacle-time pairs are discovered; otherwise the active set has stabilized.
    elseif any(newPairs, "all")
        activePairs = newPairs;
    else
        solverMessage = "A tagged pair crossed its retained separating plane.";
        break;
    end

    % Add separating lines where samples overlap. Final certification follows later.
    updateFailed      = false;
    activePairIndices = reshape(find(activePairs), 1, []);
    % Process each active needed to find alternating trajectory.
    for activeIndex = 1:numel(activePairIndices)
        pairIndex = activePairIndices(activeIndex);
        [segmentIndex, regionIndex]         = ind2sub(size(activePairs), pairIndex);
        [plane, planeExitFlag, planeOutput] = bmtpEngine.solveSeparatingLine(squeeze(feasibleControl_units(segmentIndex, :, :)), regions_units{regionIndex}, obstacleTarget_units, roundoffReserve_units, request.PlaneOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + 1;
        diagnostics.ConicSolver    = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, planeOutput);
        if planeExitFlag <= 0 || ~plane.Active
            [diagnostics.FailedPlaneSegmentIndex, ...
                diagnostics.FailedPlaneRegionIndex, diagnostics.FailedPlane] = deal(segmentIndex, regionIndex, plane);
            solverMessage = "A maximum-margin plane solve failed.";
            updateFailed  = true;
            break;
        end
        if ~plane.Verified
            diagnostics.UnverifiedPlaneInitializationCount = diagnostics.UnverifiedPlaneInitializationCount + 1;
        end
        planes(segmentIndex, regionIndex) = plane;
    end
    % Terminate with the recorded failure if an alternating update cannot produce a valid control or separating plane.
    if updateFailed
        break;
    end
end

%% Section 3: Return The Best Sampled-Clear Attempt

[diagnostics.TaggedPairCount, diagnostics.SolverMessage] = deal(nnz(taggedPairs), solverMessage);
result = struct("Success", ~isempty(bestControl_units), ...
    "SolverMessage", solverMessage, ...
    "ControlPoint_units", bestControl_units, ...
    "SegmentTime_s", bestSegmentTime_s, ...
    "Planes", planes, ...
    "TaggedPairs", taggedPairs);
end

%% Section 4: Local Functions

function plane = createEmptyPlane()
    % Initialize an inactive separating-plane record.
    plane = struct();
    plane.Active        = false;
    plane.Verified      = false;
    plane.ExitFlag      = NaN;
    plane.Normal        = zeros(2, 2);
    plane.Offset_units    = zeros(1, 2);
    plane.SignedGap_units = NaN;
end
