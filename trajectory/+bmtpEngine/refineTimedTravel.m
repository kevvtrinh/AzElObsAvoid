function [result, diagnostics] = refineTimedTravel(request, warmStart, alternatingResult, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX: [result, diagnostics] = bmtpEngine.refineTimedTravel( request, warmStart,
%   alternatingResult, diagnostics, obstacleTarget_units, roundoffReserve_units)
% PURPOSE: Reduce the convex travel surrogate after the alternating solve has established a feasible
%   obstacle homotopy.
% INPUTS: request, warmStart, alternatingResult, diagnostics (scalar structs) Checked request,
%   prepared curve, retained attempt, and diagnostics. obstacleTarget_units, roundoffReserve_units
%   (finite scalars) Required obstacle-side target and numerical reserve in coordinate units.
% OUTPUTS: result (scalar struct) Selected controls and segment time. diagnostics (scalar struct)
%   Updated active-pair count after optional refinement.
% UNITS: Position and travel are coordinate units; time is seconds.

%% Section 1: Preserve The Feasible Alternating Result
result = struct("ControlPoint_units", alternatingResult.ControlPoint_units, ...
    "SegmentTime_s", alternatingResult.SegmentTime_s);

%% Section 2: Refine Travel At The Selected Arrival Clock
segmentCount             = warmStart.SegmentCount;
baseControl_units          = result.ControlPoint_units;
baseSegmentTime_s        = result.SegmentTime_s;
baseDuration_s=physicalDuration(baseSegmentTime_s,segmentCount);
baseLength_units           = controlPolygonLength(baseControl_units);
selectedControl_units      = baseControl_units;
selectedSegmentTime_s    = baseSegmentTime_s;
selectedPlanes           = alternatingResult.Planes;
selectedLength_units       = baseLength_units;
travelRefinementAccepted = false;
travelPlanes             = alternatingResult.Planes;
taggedPairs              = alternatingResult.TaggedPairs;
refinementHorizon_s      = request.MotionHorizon_s;
if request.Options.GoalTimeMode == "earliestArrival"
    % Preserve the proven earliest clock exactly, then minimize travel at that
    % clock. This realizes the documented path-length tie-break without paying
    % extra arrival time when the former balanced policy cannot justify it.
    refinementHorizon_s = baseDuration_s;
end
diagnostics.TravelRefinementAttempted         = true;
diagnostics.TravelRefinementInitialLength_units = baseLength_units;
diagnostics.TravelRefinementFinalLength_units   = baseLength_units;
diagnostics.TravelRefinementInitialDuration_s = baseDuration_s;
diagnostics.TravelRefinementFinalDuration_s   = baseDuration_s;
diagnostics.TravelRefinementAccepted          = false;
trajectoryOptions = optimoptions("coneprog", "Display", "none", "MaxIterations", 300);
for refinementIndex = 1:8
    if isfield(warmStart,'SegmentRatio')
        [refinedControl_units, refinedSegmentTime_s, travelExitFlag, output] = ...
            bmtpEngine.solveTimedTrajectoryStep(segmentCount,request.Degree, ...
            request.InitialState.position_units,request.GoalState.position_units, ...
            request.Limits,travelPlanes,roundoffReserve_units, ...
            refinementHorizon_s,"fixedArrival",trajectoryOptions,0, ...
            warmStart.SegmentRatio);
    else
        [refinedControl_units, refinedSegmentTime_s, travelExitFlag, output] = ...
            bmtpEngine.solveTimedTrajectoryStep(segmentCount,request.Degree, ...
            request.InitialState.position_units,request.GoalState.position_units, ...
            request.Limits,travelPlanes,roundoffReserve_units, ...
            refinementHorizon_s,"fixedArrival",trajectoryOptions);
    end
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    diagnostics.TravelRefinementExitFlag = travelExitFlag;
    diagnostics.TravelRefinementOptimizationConverged = output.OptimizationConverged;
    if (travelExitFlag <= 0 && travelExitFlag ~= -7) || isempty(refinedControl_units)
        break;
    end
    physicalSegmentTime_s=expandSegmentTime(refinedSegmentTime_s,segmentCount);
    refinedMotion=struct('CertifiedControlPoint_units',refinedControl_units, ...
        'ControlPoint_units',refinedControl_units, ...
        'SegmentTime_s',physicalSegmentTime_s,'PrescribedPower_units',[]);
    refinedCertificate=bmtpEngine.checkFinalMotion(request,warmStart, ...
        refinedMotion,roundoffReserve_units,obstacleTarget_units);
    refinedCollisionPairs=~reshape([refinedCertificate.Planes.Verified], ...
        size(refinedCertificate.Planes)) & refinedCertificate.RegionActiveBySegment;
    if any(refinedCollisionPairs, "all")
        [travelPlanes,activeTravelPairs,complete,planeStatistics]= ...
            bmtpEngine.createTimeScopedPlanes(baseControl_units, ...
            expandSegmentTime(baseSegmentTime_s,segmentCount),request, ...
            obstacleTarget_units,roundoffReserve_units);
        diagnostics.TaggedPairCount=planeStatistics.ActivePairCount;
        if ~complete
            break;
        end
        taggedPairs=taggedPairs|activeTravelPairs;
        continue;
    end
    refinedLength_units  = controlPolygonLength(refinedControl_units);
    refinementIsBetter = refinedLength_units < selectedLength_units;
    % Replace the current travel profile only when refinement improves the declared objective and remains feasible.
    if refinementIsBetter
        selectedControl_units      = refinedControl_units;
        selectedSegmentTime_s    = refinedSegmentTime_s;
        selectedPlanes           = travelPlanes;
        selectedLength_units       = refinedLength_units;
        travelRefinementAccepted = true;
    end
    break;
end

%% Section 3: Return The Best Travel Attempt
if travelRefinementAccepted
    result.ControlPoint_units = selectedControl_units;
    result.SegmentTime_s    = selectedSegmentTime_s;
    taggedPairs = taggedPairs | reshape([selectedPlanes.Active], size(selectedPlanes));
end
diagnostics.TaggedPairCount = nnz(taggedPairs);
diagnostics.TravelRefinementFinalLength_units = selectedLength_units;
diagnostics.TravelRefinementFinalDuration_s = ...
    physicalDuration(selectedSegmentTime_s,segmentCount);
diagnostics.TravelRefinementAccepted        = travelRefinementAccepted;
end

function duration_s=physicalDuration(segmentTime_s,segmentCount)
    if isscalar(segmentTime_s)
        duration_s=segmentCount*segmentTime_s;
    else
        duration_s=sum(segmentTime_s);
    end
end

function segmentTime_s=expandSegmentTime(segmentTime_s,segmentCount)
    if isscalar(segmentTime_s)
        segmentTime_s=repmat(segmentTime_s,segmentCount,1);
    else
        segmentTime_s=segmentTime_s(:);
    end
end

%% Section 4: Local Functions
function length_units = controlPolygonLength(controlPoint_units)
    % Sum Bezier control-edge lengths as a convex travel estimate.
    edge_units   = diff(controlPoint_units, 1, 2);
    length_units = sum(vecnorm(edge_units, 2, 3), "all");
end
