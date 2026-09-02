function [result, diagnostics] = solveAlternatingTrajectory( ...
        request, warmStart, diagnostics, obstacleTarget_deg, ...
        roundoffReserve_deg, operations)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.solveAlternatingTrajectory( ...
%       request, warmStart, diagnostics, obstacleTarget_deg, ...
%       roundoffReserve_deg, operations)
%**************************************************************************
% PURPOSE
%   - Alternate trajectory and separating-line solves until a sampled-clear
%     motion is retained or the bounded iteration fails.
%**************************************************************************
% INPUTS
%   - request, warmStart, diagnostics (scalar structs)
%       Checked engine request, feasible starting curve, and diagnostics.
%   - obstacleTarget_deg, roundoffReserve_deg (finite scalars)
%       Required obstacle-side target and numerical reserve in degrees.
%   - operations (scalar struct of function handles)
%       Numerical kernels owned by the BMTP solve implementation.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Best sampled-clear controls, timing, planes, tags, and failure reason.
%   - diagnostics (scalar struct)
%       Updated iteration, solver, overlap, and separating-line evidence.
%**************************************************************************
% UNITS
%   - Position and clearance are degrees; time is seconds.
%**************************************************************************

%% Section 1: Initialize The Alternating State

segmentCount = warmStart.SegmentCount;
degree = request.Degree;
regions_deg = request.Regions_deg;
regionActiveBySegment = warmStart.RegionActiveBySegment;
feasibleControl_deg = warmStart.ControlPoint_deg;
feasibleSegmentTime_s = warmStart.SegmentTime_s;
diagnostics.WarmStartDuration_s = segmentCount * feasibleSegmentTime_s;
bestControl_deg = zeros(0, degree + 1, 2);
[bestSegmentTime_s, bestDuration_s] = deal(NaN, Inf);
diagnostics.RetainedBestTrialDuration_s = bestDuration_s;
taggedPairs = false(segmentCount, numel(regions_deg));
planes = repmat(operations.createEmptyPlane(), ...
    segmentCount, numel(regions_deg));
optimizationHorizon_s = request.MotionHorizon_s;
solverMessage = "The biconvex iteration limit was reached.";

%% Section 2: Alternate Trajectory And Separating-Line Solves

for iterationIndex = 1:35
    diagnostics.IterationCount = iterationIndex;
    usedRequestHorizon = ...
        optimizationHorizon_s == request.MotionHorizon_s;
    [trialControl_deg, trialTime_s, exitFlag, output] = ...
        operations.solveTrajectoryStep( ...
        segmentCount, degree, request.InitialState.position_deg, ...
        request.GoalState.position_deg, request.Limits, planes, ...
        roundoffReserve_deg, optimizationHorizon_s, ...
        "earliestArrival", 0, feasibleSegmentTime_s, ...
        request.TrajectoryOptions);
    diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + 1;
    diagnostics.FinalTrajectoryExitFlag = exitFlag;
    if exitFlag <= 0 || isempty(trialControl_deg)
        % The requested horizon constrains the returned motion, but the first
        % useful iterate may need the longer warm-start duration to establish
        % separating lines before the horizon is imposed again.
        canExpandHorizon = exitFlag == -2 && isempty(bestControl_deg) && ...
            optimizationHorizon_s < diagnostics.WarmStartDuration_s;
        if canExpandHorizon
            optimizationHorizon_s = min(2 * optimizationHorizon_s, ...
                diagnostics.WarmStartDuration_s);
            continue;
        end
        solverMessage = "Trajectory SOCP failed: " + string(output.message);
        break;
    end
    collisionPairs = operations.findSampledObstacleOverlaps( ...
        trialControl_deg, regions_deg, request.RegionMinimum_deg, ...
        request.RegionMaximum_deg, regionActiveBySegment, 1201);
    diagnostics.TrialDuration_s(iterationIndex) = ...
        segmentCount * trialTime_s;
    diagnostics.CollisionPairCountHistory(iterationIndex) = ...
        nnz(collisionPairs);
    diagnostics.TrialWasCollisionFree(iterationIndex) = ...
        ~any(collisionPairs, "all");
    diagnostics.FinalCollisionPairCount = nnz(collisionPairs);
    previousTaggedPairs = taggedPairs;
    newPairs = collisionPairs & ~taggedPairs;
    taggedPairs = taggedPairs | newPairs;
    if ~any(collisionPairs, "all")
        optimizationHorizon_s = request.MotionHorizon_s;
        previousDuration_s = segmentCount * feasibleSegmentTime_s;
        feasibleControl_deg = trialControl_deg;
        feasibleSegmentTime_s = trialTime_s;
        duration_s = segmentCount * trialTime_s;
        retainedBestImprovement_s = bestDuration_s - duration_s;
        if duration_s < bestDuration_s
            bestControl_deg = trialControl_deg;
            bestSegmentTime_s = trialTime_s;
            bestDuration_s = duration_s;
            diagnostics.BestDuration_s = duration_s;
            diagnostics.RetainedBestTrialDuration_s = duration_s;
        end
        improvement_s = previousDuration_s - duration_s;
        if improvement_s >= 0 && ...
                improvement_s <= request.Options.ArrivalTimeTolerance_s
            diagnostics.Converged = true;
            solverMessage = ...
                "The feasible arrival improvement reached tolerance.";
            break;
        end
        taggedPairSetUnchanged = isequal(taggedPairs, previousTaggedPairs);
        reusePlanes = retainedBestImprovement_s <= ...
            request.Options.ArrivalTimeTolerance_s && taggedPairSetUnchanged;
        if reusePlanes
            diagnostics.PlaneReuseApplied = true;
            diagnostics.PlaneReuseCount = diagnostics.PlaneReuseCount + 1;
            if usedRequestHorizon
                diagnostics.Converged = true;
                solverMessage = ...
                    "The next trajectory SOCP would be unchanged.";
                break;
            end
            continue;
        end
        planes(:) = operations.createEmptyPlane();
        activePairs = taggedPairs;
    elseif any(newPairs, "all")
        activePairs = newPairs;
    else
        solverMessage = ...
            "A tagged pair crossed its retained separating plane.";
        break;
    end

    % A sampled overlap only identifies where a separator is needed. Solve
    % and retain a line for each newly active curve-region pair; final direct
    % certification remains a later, independent stage.
    updateFailed = false;
    activePairIndices = reshape(find(activePairs), 1, []);
    for activeIndex = 1:numel(activePairIndices)
        pairIndex = activePairIndices(activeIndex);
        [segmentIndex, regionIndex] = ind2sub(size(activePairs), pairIndex);
        [plane, planeExitFlag] = operations.updateSeparatingLine( ...
            squeeze(feasibleControl_deg(segmentIndex, :, :)), ...
            regions_deg{regionIndex}, obstacleTarget_deg, ...
            roundoffReserve_deg, request.PlaneOptions);
        diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + 1;
        if planeExitFlag <= 0 || ~plane.Active
            [diagnostics.FailedPlaneSegmentIndex, ...
                diagnostics.FailedPlaneRegionIndex, diagnostics.FailedPlane] = ...
                deal(segmentIndex, regionIndex, plane);
            solverMessage = "A maximum-margin plane solve failed.";
            updateFailed = true;
            break;
        end
        if ~plane.Verified
            diagnostics.UnverifiedPlaneInitializationCount = ...
                diagnostics.UnverifiedPlaneInitializationCount + 1;
        end
        planes(segmentIndex, regionIndex) = plane;
    end
    if updateFailed
        break;
    end
end

%% Section 3: Return The Best Sampled-Clear Attempt

[diagnostics.TaggedPairCount, diagnostics.SolverMessage] = ...
    deal(nnz(taggedPairs), solverMessage);
result = struct( ...
    "Success", ~isempty(bestControl_deg), ...
    "SolverMessage", solverMessage, ...
    "ControlPoint_deg", bestControl_deg, ...
    "SegmentTime_s", bestSegmentTime_s, ...
    "Planes", planes, ...
    "TaggedPairs", taggedPairs);
end
