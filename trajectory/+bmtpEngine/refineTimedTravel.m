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
baseControl_units          = result.ControlPoint_units;
baseSegmentTime_s          = result.SegmentTime_s(:);
segmentCount             = size(baseControl_units,1);
assert(numel(baseSegmentTime_s)==segmentCount, ...
    'bmtpEngine:InvalidTimedSegmentClock', ...
    'Timed refinement requires one physical duration per motion span.');
baseDuration_s=sum(baseSegmentTime_s);
baseLength_units           = controlPolygonLength(baseControl_units);
selectedControl_units      = baseControl_units;
selectedSegmentTime_s    = baseSegmentTime_s;
selectedPlanes           = alternatingResult.Planes;
selectedLength_units       = baseLength_units;
travelRefinementAccepted = false;
travelPlanes             = alternatingResult.Planes;
taggedPairs              = alternatingResult.TaggedPairs;
assert(request.UsesVariableClock && ...
    request.Options.GoalTimeMode=="earliestArrival", ...
    'bmtpEngine:InvalidTimedTravelRefinement', ...
    'Timed travel refinement requires an earliest-arrival variable clock.');
% Preserve the selected earliest clock exactly while minimizing its travel
% surrogate. Fixed-arrival requests use the ordinary alternating solver and
% never enter this refinement path.
refinementHorizon_s=baseDuration_s;
diagnostics.TravelRefinementAttempted         = true;
diagnostics.TravelRefinementInitialLength_units = baseLength_units;
diagnostics.TravelRefinementFinalLength_units   = baseLength_units;
diagnostics.TravelRefinementInitialDuration_s = baseDuration_s;
diagnostics.TravelRefinementFinalDuration_s   = baseDuration_s;
diagnostics.TravelRefinementAccepted          = false;
trajectoryOptions = optimoptions("coneprog", "Display", "none", "MaxIterations", 300);
segmentRatio=baseSegmentTime_s/mean(baseSegmentTime_s);
for refinementIndex = 1:8
    [refinedControl_units, refinedSegmentTime_s, travelExitFlag, output] = ...
        bmtpEngine.solveTimedTrajectoryStep(segmentCount,request.Degree, ...
        request.InitialState.position_units,request.GoalState.position_units, ...
        request.Limits,travelPlanes, ...
        roundoffReserve_units,refinementHorizon_s,"fixedArrival", ...
        trajectoryOptions,0,segmentRatio);
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    diagnostics.TravelRefinementExitFlag = travelExitFlag;
    diagnostics.TravelRefinementOptimizationConverged = output.OptimizationConverged;
    if ~bmtpEngine.hasUsableConicIterate(refinedControl_units,travelExitFlag)
        break;
    end
    if ~output.ConstraintGenerationComplete
        break;
    end
    physicalSegmentTime_s=refinedSegmentTime_s(:);
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
            baseSegmentTime_s,request, ...
            obstacleTarget_units,roundoffReserve_units);
        diagnostics.TaggedPairCount=planeStatistics.ActivePairCount;
        if ~complete
            break;
        end
        taggedPairs=taggedPairs|activeTravelPairs;
        continue;
    end
    if ~refinedCertificate.Passed
        break;
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
diagnostics.TravelRefinementFinalDuration_s=sum(selectedSegmentTime_s);
diagnostics.TravelRefinementAccepted        = travelRefinementAccepted;
end

%% Section 4: Local Functions
function length_units = controlPolygonLength(controlPoint_units)
    % Sum Bezier control-edge lengths as a convex travel estimate.
    edge_units   = diff(controlPoint_units, 1, 2);
    length_units = sum(vecnorm(edge_units, 2, 3), "all");
end
