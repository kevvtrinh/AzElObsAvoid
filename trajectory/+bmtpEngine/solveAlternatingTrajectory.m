function [result, diagnostics] = solveAlternatingTrajectory(request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX: [result, diagnostics] = bmtpEngine.solveAlternatingTrajectory( request, warmStart,
%   diagnostics, obstacleTarget_units, roundoffReserve_units)
% PURPOSE: Alternate one trajectory SOCP with exact all-pair separating-line SOCPs until a
%   completely verified motion is found. Timed cells constrain only their exact overlap with each
%   fixed-duration motion span.
% INPUTS: request, warmStart, diagnostics: checked BMTP state and diagnostics.
%   obstacleTarget_units, roundoffReserve_units: required separation.
% OUTPUTS: result: best all-pair-verified controls and per-segment durations.
%   diagnostics: solver counts, residual pair counts, and termination data.
% UNITS: Position and clearance are coordinate units; time is seconds.

%% Section 1: Initialize Every Segment-Obstacle Pair
segmentCount = warmStart.SegmentCount;
regionCount = numel(request.Regions_units);
request.RegionActiveBySegment = warmStart.RegionActiveBySegment;
fixedControl_units = [];
if isfield(warmStart,'FixedControl_units'), fixedControl_units = warmStart.FixedControl_units; end
diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics();
diagnostics.WarmStartDuration_s = sum(warmStart.SegmentTime_s);
diagnostics.ExistingPlanePairVerificationCount = 0;
diagnostics.FullPlaneUpdateSkippedCount = 0;
emptyPlane = struct('Active',false,'Verified',false,'ExitFlag',NaN, ...
    'Normal',zeros(2,2),'Offset_units',zeros(1,2),'SignedGap_units',NaN,'TimeFraction',[0,1]);
planes = repmat(emptyPlane, segmentCount, regionCount);
[planes, allPlanesActive, verifiedPairs, diagnostics] = updatePlanes(warmStart.ControlPoint_units, warmStart.SegmentTime_s, planes, request, diagnostics, obstacleTarget_units, roundoffReserve_units, false);
diagnostics.UnverifiedPlaneInitializationCount = nnz(~verifiedPairs);
selectedControl_units = zeros(0, request.Degree + 1, 2);
selectedSegmentTime_s = NaN;
solverMessage = "The all-pair alternating iteration limit was reached.";
meshRefinementCount = 0;
diagnostics.MeshRefinementCount = 0;
diagnostics.MeshRefinementSpanIndex = cell(3,1);

%% Section 2: Alternate The Complete Formulation
if allPlanesActive
    for iterationIndex = 1:35
        diagnostics.IterationCount = iterationIndex;
        [trialControl_units, trialTime_s, exitFlag, output] = bmtpEngine.solveTrajectoryStep(segmentCount, request.Degree, request.InitialState, request.GoalState, request.Limits, planes, roundoffReserve_units, request.MotionHorizon_s, request.TrajectoryOptions, warmStart.SegmentRatio, request.Options.GoalTimeMode=="fixedArrival",fixedControl_units);
        diagnostics.TrajectorySocpCount = diagnostics.TrajectorySocpCount + output.SolveCount;
        diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
        diagnostics.FinalTrajectoryExitFlag = exitFlag;
        diagnostics.PlaneReduction=[output.OriginalPlaneCount,output.RetainedPlaneCount];
        if isfield(output,'MaximumClearanceSlack_units')
            diagnostics.MaximumClearanceSlack_units = output.MaximumClearanceSlack_units;
        end
        if (exitFlag <= 0 && exitFlag ~= -7) || isempty(trialControl_units)
            solverMessage = "Trajectory SOCP failed: " + string(output.message);
            break;
        end

        [updatedPlanes,~,verifiedPairs,diagnostics,verifiedPairCount] = updatePlanes(trialControl_units, ...
            trialTime_s,planes,request,diagnostics,obstacleTarget_units,roundoffReserve_units,true);
        diagnostics.ExistingPlanePairVerificationCount = ...
            diagnostics.ExistingPlanePairVerificationCount+verifiedPairCount;
        allPlanesActive = all(verifiedPairs,'all');
        if ~allPlanesActive
            [updatedPlanes, allPlanesActive, verifiedPairs, diagnostics] = ...
                updatePlanes(trialControl_units,trialTime_s,planes,request, ...
                diagnostics,obstacleTarget_units,roundoffReserve_units,false);
        else
            diagnostics.FullPlaneUpdateSkippedCount = diagnostics.FullPlaneUpdateSkippedCount+1;
        end
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
            if meshRefinementCount < 3 && request.Options.GoalTimeMode=="fixedArrival" && ...
                    isempty(fixedControl_units)
                splitMask = any(~verifiedPairs,2);
                [refinedControl_units,refinedTime_s] = bisectSelectedSpans( ...
                    trialControl_units,trialTime_s,splitMask);
                meshRefinementCount = meshRefinementCount+1;
                diagnostics.MeshRefinementCount = meshRefinementCount;
                diagnostics.MeshRefinementSpanIndex{meshRefinementCount} = find(splitMask).';
                segmentCount = numel(refinedTime_s);
                diagnostics.OptimizerSpanCount = segmentCount;
                warmStart.SegmentRatio = refinedTime_s/mean(refinedTime_s);
                request.RegionActiveBySegment = true(segmentCount,regionCount);
                if isfield(request.Coverage,'ActiveTimeInterval_s')
                    breaks_s = request.InitialState.time_s+[0;cumsum(refinedTime_s)];
                    intervals_s = request.Coverage.ActiveTimeInterval_s;
                    request.RegionActiveBySegment = breaks_s(1:end-1)<intervals_s(:,2).' & ...
                        breaks_s(2:end)>intervals_s(:,1).';
                end
                diagnostics.ApplicablePairCount = nnz(request.RegionActiveBySegment);
                planes = repmat(emptyPlane,segmentCount,regionCount);
                [planes,allPlanesActive,~,diagnostics] = updatePlanes(refinedControl_units, ...
                    refinedTime_s,planes,request,diagnostics,obstacleTarget_units,roundoffReserve_units,false);
                if ~allPlanesActive
                    solverMessage = "A refined separating-line initialization failed.";
                    break
                end
            end
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
function [refinedControl_units,refinedTime_s] = bisectSelectedSpans(controlPoint_units,segmentTime_s,splitMask)
    refinedControl_units = zeros(numel(segmentTime_s)+nnz(splitMask),size(controlPoint_units,2),2);
    refinedTime_s = zeros(numel(segmentTime_s)+nnz(splitMask),1);
    targetIndex = 0;
    for spanIndex = 1:numel(segmentTime_s)
        if splitMask(spanIndex)
            for halfIndex = 1:2
                targetIndex = targetIndex+1;
                refinedControl_units(targetIndex,:,:) = bmtpEngine.restrictBezier( ...
                    squeeze(controlPoint_units(spanIndex,:,:)),[(halfIndex-1)/2,halfIndex/2]);
                refinedTime_s(targetIndex) = segmentTime_s(spanIndex)/2;
            end
        else
            targetIndex = targetIndex+1;
            refinedControl_units(targetIndex,:,:) = controlPoint_units(spanIndex,:,:);
            refinedTime_s(targetIndex) = segmentTime_s(spanIndex);
        end
    end
end

function [planes, allActive, verifiedPairs, diagnostics, verifiedPairCount] = updatePlanes(controlPoint_units, segmentTime_s, planes, request, diagnostics, target_units, reserve_units, verifyOnly)
    % Update every active curve-region pair without sampled discovery or pruning.
    verifiedPairs = ~request.RegionActiveBySegment;
    allActive = true;
    verifiedPairCount = 0;
    breaks_s = request.InitialState.time_s+[0;cumsum(segmentTime_s)];
    for segmentIndex = 1:size(planes, 1)
        for regionIndex = 1:size(planes, 2)
            if ~request.RegionActiveBySegment(segmentIndex,regionIndex), continue; end
            interval_s = [];
            controls_units=squeeze(controlPoint_units(segmentIndex,:,:));
            timeFraction=[0,1];
            if request.Options.GoalTimeMode=="fixedArrival"
                interval_s = breaks_s(segmentIndex:segmentIndex+1).';
                if isfield(request.Coverage,'ActiveTimeInterval_s')
                    active_s=request.Coverage.ActiveTimeInterval_s(regionIndex,:);
                    interval_s=[max(interval_s(1),active_s(1)),min(interval_s(2),active_s(2))];
                    if interval_s(2)<=interval_s(1)
                        % Adjacent closed cells can meet a span at one instant.
                        % That zero-measure contact creates no trajectory
                        % constraint and must not become a degenerate scope.
                        plane=planes(segmentIndex,regionIndex);
                        plane.Active=false;
                        plane.Verified=false;
                        plane.TimeFraction=[0,1];
                        planes(segmentIndex,regionIndex)=plane;
                        verifiedPairs(segmentIndex,regionIndex)=true;
                        continue
                    end
                    timeFraction=max(0,min(1,(interval_s-breaks_s(segmentIndex))/segmentTime_s(segmentIndex)));
                    controls_units=bmtpEngine.restrictBezier(controls_units,timeFraction);
                end
            end
            vertices_units = bmtpEngine.regionOnInterval(request.Regions_units{regionIndex},request.Coverage,regionIndex,interval_s);
            if verifyOnly
                plane = planes(segmentIndex,regionIndex);
                normal = plane.Normal(1,:);
                plane.Offset_units = target_units-[min(vertices_units(:,:,1)*normal.'), ...
                    min(vertices_units(:,:,end)*normal.')];
                plane.TimeFraction = timeFraction;
                plane = bmtpEngine.verifySeparatingLine(plane,controls_units,vertices_units,reserve_units,target_units);
                verifiedPairCount = verifiedPairCount+1;
            else
                [plane, exitFlag, output] = bmtpEngine.solveSeparatingLine( ...
                    controls_units,vertices_units,target_units,reserve_units);
                plane.TimeFraction = timeFraction;
                diagnostics.PlaneSocpCount = diagnostics.PlaneSocpCount + ~(isfield(output,'IsAnalytic') && output.IsAnalytic);
                diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
            end
            planes(segmentIndex, regionIndex) = plane;
            verifiedPairs(segmentIndex, regionIndex) = plane.Verified;
            if verifyOnly
                if ~plane.Verified, allActive = false; return; end
            elseif exitFlag <= 0 || ~plane.Active
                diagnostics.FailedPlaneSegmentIndex = segmentIndex;
                diagnostics.FailedPlaneRegionIndex = regionIndex;
                diagnostics.FailedPlane = plane;
                allActive = false;
                return;
            end
        end
    end
end
