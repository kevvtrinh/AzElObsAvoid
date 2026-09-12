function [result, diagnostics] = solveTimedAlternatingTrajectory( ...
        request, warmStart, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX: [result,diagnostics] = bmtpEngine.solveTimedAlternatingTrajectory(
%   request,warmStart,diagnostics,obstacleTarget,reserve)
% PURPOSE: Minimize arrival while rebuilding every moving-cell constraint on
%   the current physical clock and the exact timed visibility-guide homotopy.
% INPUTS: Checked solve request, timed warm start, diagnostics, and separation
%   targets in coordinate units.
% OUTPUTS: Earliest exactly time-scoped BMTP candidate and solver diagnostics.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Initialize The Variable-Clock Formulation
segmentCount=warmStart.SegmentCount;
segmentRatio=warmStart.SegmentRatio(:);
emptyPlane=struct('Active',false,'Verified',false,'ExitFlag',NaN, ...
    'Normal',zeros(2,2),'Offset_units',zeros(1,2), ...
    'SignedGap_units',NaN,'TimeFraction',[0,1]);
planes=repmat(emptyPlane,segmentCount,numel(request.Regions_units));
selectedControl_units=zeros(0,request.Degree+1,2);
selectedSegmentTime_s=NaN;
selectedPlanes=planes;
selectedPairs=false(size(planes));
previousFailedPairs=false(size(planes));
previousDuration_s=Inf;
solverMessage="The time-scoped alternating iteration limit was reached.";
trajectoryOptions=optimoptions("coneprog","Display","none", ...
    "MaxIterations",300);
diagnostics.ConicSolver=bmtpEngine.accumulateConicDiagnostics();
diagnostics.WarmStartDuration_s=warmStart.Duration_s;
diagnostics.RetainedHorizonRetryCount=0;
diagnostics.PlaneReuseApplied=false;
diagnostics.PlaneReuseCount=0;

%% Section 2: Solve And Rebuild Constraints On Every Returned Clock
for iterationIndex=1:35
    diagnostics.IterationCount=iterationIndex;
    trajectoryGoalTimeMode=request.Options.GoalTimeMode;
    [trialControl_units,trialSegmentTime_s,exitFlag,output]= ...
        bmtpEngine.solveTimedTrajectoryStep(segmentCount,request.Degree, ...
        request.InitialState.position_units,request.GoalState.position_units, ...
        request.Limits,planes,roundoffReserve_units,request.MotionHorizon_s, ...
        trajectoryGoalTimeMode,trajectoryOptions,request.MinimumMotionDuration_s, ...
        segmentRatio);
    diagnostics.TrajectorySocpCount=diagnostics.TrajectorySocpCount+1;
    diagnostics.ConicSolver=bmtpEngine.accumulateConicDiagnostics( ...
        diagnostics.ConicSolver,output);
    diagnostics.FinalTrajectoryExitFlag=exitFlag;
    if exitFlag<=0 || isempty(trialControl_units)
        solverMessage="Trajectory SOCP failed: "+string(output.message);
        break
    end
    duration_s=sum(trialSegmentTime_s);
    trialMotion=struct('CertifiedControlPoint_units',trialControl_units, ...
        'ControlPoint_units',trialControl_units, ...
        'SegmentTime_s',trialSegmentTime_s(:), ...
        'PrescribedPower_units',[]);
    trialCertificate=bmtpEngine.checkFinalMotion(request,warmStart,trialMotion, ...
        roundoffReserve_units,obstacleTarget_units);
    failedPairs=~reshape([trialCertificate.Planes.Verified], ...
        size(trialCertificate.Planes)) & trialCertificate.RegionActiveBySegment;
    collisionFree=~any(failedPairs,'all');
    diagnostics.TrialDuration_s(iterationIndex)=duration_s;
    diagnostics.CollisionPairCountHistory(iterationIndex)=nnz(failedPairs);
    diagnostics.TrialWasCollisionFree(iterationIndex)=collisionFree;
    diagnostics.FinalCollisionPairCount=nnz(failedPairs);
    diagnostics.UnverifiedPairs=indicesOf(failedPairs);
    diagnostics.UnverifiedGaps_units=reshape( ...
        [trialCertificate.Planes(failedPairs).SignedGap_units],[],1);
    if collisionFree && trialCertificate.DynamicsPassed && ...
            trialCertificate.ContinuityPassed
        [selectedPlanes,selectedPairs,complete,planeStatistics]= ...
            bmtpEngine.createTimeScopedPlanes(trialControl_units, ...
            trialSegmentTime_s,request,obstacleTarget_units,roundoffReserve_units);
        diagnostics.ApplicablePairCount=planeStatistics.ActivePairCount;
        diagnostics.UnverifiedPlaneInitializationCount= ...
            planeStatistics.ActivePairCount-planeStatistics.VerifiedPairCount;
        if ~complete
            solverMessage="The feasible timed motion did not produce a complete exact plane set.";
            break
        end
        selectedControl_units=trialControl_units;
        selectedSegmentTime_s=trialSegmentTime_s;
        diagnostics.BestDuration_s=duration_s;
        diagnostics.RetainedBestTrialDuration_s=duration_s;
        diagnostics.Converged=output.OptimizationConverged;
        solverMessage="A complete time-scoped feasible iterate was found.";
        break
    end
    [planes,activePairs,complete,planeStatistics]= ...
        bmtpEngine.createTimeScopedPlanes(warmStart.ControlPoint_units, ...
        trialSegmentTime_s,request,obstacleTarget_units,roundoffReserve_units);
    diagnostics.ApplicablePairCount=planeStatistics.ActivePairCount;
    diagnostics.UnverifiedPlaneInitializationCount= ...
        planeStatistics.ActivePairCount-planeStatistics.VerifiedPairCount;
    diagnostics.FailedPlaneSegmentIndex=planeStatistics.FailedSegmentIndex;
    diagnostics.FailedPlaneRegionIndex=planeStatistics.FailedRegionIndex;
    if ~complete
        solverMessage="The timed visibility guide could not initialize every exact clock pair.";
        break
    end
    unchangedClock=abs(duration_s-previousDuration_s)<= ...
        request.Options.ArrivalTimeTolerance_s;
    if unchangedClock && isequal(failedPairs,previousFailedPairs)
        solverMessage="The exact timed pair set stopped changing before feasibility.";
        break
    end
    previousDuration_s=duration_s;
    previousFailedPairs=failedPairs;
    selectedPairs=activePairs;
end

%% Section 3: Return Only A Clock-Consistent Feasible Iterate
diagnostics.TaggedPairCount=nnz(selectedPairs);
diagnostics.SolverMessage=solverMessage;
result=struct('Success',~isempty(selectedControl_units), ...
    'SolverMessage',solverMessage, ...
    'ControlPoint_units',selectedControl_units, ...
    'SegmentTime_s',selectedSegmentTime_s, ...
    'Planes',selectedPlanes,'TaggedPairs',selectedPairs);
end

function indices=indicesOf(mask)
    [segmentIndex,regionIndex]=find(mask);
    indices=[segmentIndex,regionIndex];
end
