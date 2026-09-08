function [result, diagnostics] = solveAlternatingTrajectory(request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.solveAlternatingTrajectory( ...
%       request, warmStart, diagnostics, obstacleTarget_units, ...
%       roundoffReserve_units)
%
% PURPOSE
%   - Alternate one trajectory SOCP with exact all-pair separating-line
%     SOCPs until a completely verified static motion is found.
%
% INPUTS
%   - request, warmStart, diagnostics: checked BMTP state and diagnostics.
%   - obstacleTarget_units, roundoffReserve_units: required separation.
%
% OUTPUTS
%   - result: best all-pair-verified controls and per-segment durations.
%   - diagnostics: solver counts, residual pair counts, and termination data.
%
% UNITS
%   - Position and clearance are coordinate units; time is seconds.

%% Section 1: Initialize Every Segment-Obstacle Pair

segmentCount = warmStart.SegmentCount;
regionCount = numel(request.Regions_units);
request.RegionActiveBySegment = warmStart.RegionActiveBySegment;
fixedControl_units = [];
if isfield(warmStart,'FixedControl_units'), fixedControl_units = warmStart.FixedControl_units; end
diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics();
diagnostics.WarmStartDuration_s = sum(warmStart.SegmentTime_s);
planes = repmat(createEmptyPlane(), segmentCount, regionCount);
[planes, allPlanesActive, verifiedPairs, diagnostics] = solveAllPlanes(warmStart.ControlPoint_units, warmStart.SegmentTime_s, planes, request, diagnostics, obstacleTarget_units, roundoffReserve_units);
diagnostics.UnverifiedPlaneInitializationCount = nnz(~verifiedPairs);
selectedControl_units = zeros(0, request.Degree + 1, 2);
selectedSegmentTime_s = NaN;
solverMessage = "The all-pair alternating iteration limit was reached.";

%% Section 2: Alternate The Complete Static Formulation

if allPlanesActive
    for iterationIndex = 1:35
        diagnostics.IterationCount = iterationIndex;
        [trialControl_units, trialTime_s, exitFlag, output] = bmtpEngine.solveTrajectoryStep(segmentCount, request.Degree, request.InitialState, request.GoalState, request.Limits, planes, roundoffReserve_units, request.MotionHorizon_s, request.TrajectoryOptions, warmStart.SegmentRatio, request.Options.GoalTimeMode=="fixedArrival",fixedControl_units);
        diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + output.SolveCount;
        diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
        diagnostics.FinalTrajectoryExitFlag = exitFlag;
        if isfield(output,'MaximumClearanceSlack_units')
            diagnostics.MaximumClearanceSlack_units = output.MaximumClearanceSlack_units;
        end
        if (exitFlag <= 0 && exitFlag ~= -7) || isempty(trialControl_units)
            solverMessage = "Trajectory SOCP failed: " + string(output.message);
            break;
        end

        [updatedPlanes, allPlanesActive, verifiedPairs, diagnostics] = solveAllPlanes(trialControl_units, trialTime_s, planes, request, diagnostics, obstacleTarget_units, roundoffReserve_units);
        planes = updatedPlanes;
        unverifiedPairCount = nnz(~verifiedPairs);
        duration_s = sum(trialTime_s);
        diagnostics.TrialDuration_s(iterationIndex) = duration_s;
        diagnostics.CollisionPairCountHistory(iterationIndex) = unverifiedPairCount;
        diagnostics.TrialWasCollisionFree(iterationIndex) = unverifiedPairCount == 0;
        diagnostics.FinalCollisionPairCount = unverifiedPairCount;
        [unverifiedSegment,unverifiedRegion] = find(~verifiedPairs);
        diagnostics.UnverifiedPairs = [unverifiedSegment,unverifiedRegion];
        diagnostics.UnverifiedGaps_units = reshape([planes(~verifiedPairs).SignedGap_units],[],1);
        if ~allPlanesActive
            solverMessage = "A complete separating-line update failed.";
            break;
        end
        if unverifiedPairCount > 0
            continue;
        end

        selectedControl_units = trialControl_units;
        selectedSegmentTime_s = trialTime_s;
        diagnostics.BestDuration_s = duration_s;
        diagnostics.Converged = output.OptimizationConverged;
        solverMessage = "A complete all-pair-verified iterate was found.";
        break;
    end
else
    solverMessage = "The visibility seed did not produce a complete separating-line initialization.";
end

%% Section 3: Return The Best Fully Verified Iterate

diagnostics.SolverMessage = solverMessage;
result = struct();
result.Success = ~isempty(selectedControl_units);
result.SolverMessage = solverMessage;
result.ControlPoint_units = selectedControl_units;
result.SegmentTime_s = selectedSegmentTime_s;
result.Planes = planes;
end

%% Section 4: Local Functions

function [planes, allActive, verifiedPairs, diagnostics] = solveAllPlanes(controlPoint_units, segmentTime_s, planes, request, diagnostics, target_units, reserve_units)
    % Update every static curve-region pair without sampled discovery or pruning.
    verifiedPairs = ~request.RegionActiveBySegment;
    allActive = true;
    breaks_s = request.InitialState.time_s+[0;cumsum(segmentTime_s)];
    for segmentIndex = 1:size(planes, 1)
        for regionIndex = 1:size(planes, 2)
            if ~request.RegionActiveBySegment(segmentIndex,regionIndex), continue; end
            interval_s = [];
            if request.Options.GoalTimeMode=="fixedArrival"
                interval_s = breaks_s(segmentIndex:segmentIndex+1).';
            end
            vertices_units = bmtpEngine.regionOnInterval(request.Regions_units{regionIndex},request.Coverage,regionIndex,interval_s);
            [plane, exitFlag, output] = bmtpEngine.solveSeparatingLine(squeeze(controlPoint_units(segmentIndex, :, :)), vertices_units, target_units, reserve_units);
            diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ~(isfield(output,'IsAnalytic') && output.IsAnalytic);
            diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
            planes(segmentIndex, regionIndex) = plane;
            verifiedPairs(segmentIndex, regionIndex) = plane.Verified;
            if exitFlag <= 0 || ~plane.Active
                diagnostics.FailedPlaneSegmentIndex = segmentIndex;
                diagnostics.FailedPlaneRegionIndex = regionIndex;
                diagnostics.FailedPlane = plane;
                allActive = false;
                return;
            end
        end
    end
end

function plane = createEmptyPlane()
    % Initialize one inactive separating-plane record.
    plane = struct();
    plane.Active = false;
    plane.Verified = false;
    plane.ExitFlag = NaN;
    plane.Normal = zeros(2, 2);
    plane.Offset_units = zeros(1, 2);
    plane.SignedGap_units = NaN;
end
