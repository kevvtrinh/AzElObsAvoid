function [result, diagnostics] = refineTimedTravel(request, alternatingResult, diagnostics, ...
        obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.pipeline.refineTimedTravel(request, alternatingResult, ...
%       diagnostics, obstacleTarget_units, roundoffReserve_units)
%**************************************************************************
% PURPOSE
%   - Reduce the convex travel surrogate after the alternating solve has
%     established a feasible obstacle homotopy.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Checked BMTP request with an earliest-arrival variable clock.
%   - alternatingResult (scalar struct)
%       Retained alternating attempt, its planes, and its tagged pairs.
%   - diagnostics (scalar struct)
%       Diagnostics accumulated so far by the caller.
%   - obstacleTarget_units (finite numeric scalar)
%       Required obstacle-side separation target.
%   - roundoffReserve_units (finite numeric scalar)
%       Numerical reserve applied on the trajectory side.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Selected controls and segment time. A refinement that does not
%       improve or prove leaves the alternating result unchanged.
%   - diagnostics (scalar struct)
%       Updated active-pair count and travel-refinement measurements.
%**************************************************************************
% UNITS
%   - Position and travel are coordinate units; time is seconds.
%**************************************************************************

%% Section 1: Preserve The Feasible Alternating Result

result = alternatingResult;

%% Section 2: Refine Travel At The Selected Arrival Clock

baseControl_units = result.ControlPoint_units;
baseSegmentTime_s = result.SegmentTime_s(:);
segmentCount      = size(baseControl_units, 1);
assert(numel(baseSegmentTime_s) == segmentCount, ...
    'bmtpEngine:InvalidTimedSegmentClock', ...
    'Timed refinement requires one physical duration per motion span.');
baseDuration_s   = sum(baseSegmentTime_s);
baseLength_units = controlPolygonLength(baseControl_units);

selectedControl_units    = baseControl_units;
selectedSegmentTime_s    = baseSegmentTime_s;
selectedPlanes           = alternatingResult.Planes;
selectedPairs            = alternatingResult.TaggedPairs;
selectedSolverMessage    = alternatingResult.SolverMessage;
selectedLength_units     = baseLength_units;
travelRefinementAccepted = false;
travelPlanes             = alternatingResult.Planes;

assert(request.UsesVariableClock && ...
    request.Options.GoalTimeMode == "earliestArrival", ...
    'bmtpEngine:InvalidTimedTravelRefinement', ...
    'Timed travel refinement requires an earliest-arrival variable clock.');
% Preserve the selected earliest clock exactly while minimizing its travel
% surrogate. Fixed-arrival requests use the ordinary alternating solver and
% never enter this refinement path.
refinementHorizon_s = baseDuration_s;
diagnostics.TravelRefinementAttempted           = true;
diagnostics.TravelRefinementInitialLength_units = baseLength_units;
diagnostics.TravelRefinementFinalLength_units   = baseLength_units;
diagnostics.TravelRefinementInitialDuration_s   = baseDuration_s;
diagnostics.TravelRefinementFinalDuration_s     = baseDuration_s;
diagnostics.TravelRefinementAccepted            = false;
segmentRatio   = baseSegmentTime_s / mean(baseSegmentTime_s);
constraintBase = struct();
for refinementIndex = 1:8
    [refinedControl_units, refinedSegmentTime_s, travelExitFlag, output, constraintBase] = ...
        bmtpEngine.optimization.solveTimedTrajectoryStep(segmentCount, request.Degree, ...
        request.InitialState.position_units, request.GoalState.position_units, ...
        request.Limits, travelPlanes, ...
        roundoffReserve_units, refinementHorizon_s, "fixedArrival", ...
        request.TimedTrajectoryOptions, 0, segmentRatio, constraintBase);
    diagnostics.ConicSolver = bmtpEngine.optimization.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    diagnostics.TravelRefinementExitFlag             = travelExitFlag;
    diagnostics.TravelRefinementOptimizationConverged = output.OptimizationConverged;
    if ~bmtpEngine.optimization.hasUsableConicIterate(refinedControl_units, travelExitFlag)
        break
    end
    if ~output.ConstraintGenerationComplete
        break
    end
    physicalSegmentTime_s = refinedSegmentTime_s(:);
    refinedMotion = struct('ProvenControlPoint_units', refinedControl_units, ...
        'ControlPoint_units', refinedControl_units, ...
        'SegmentTime_s', physicalSegmentTime_s, ...
        'FinalTime_s', request.InitialState.time_s + sum(physicalSegmentTime_s), ...
        'GivenPower_units', []);
    refinedProof = bmtpEngine.validation.checkFinalMotion(request, ...
        refinedMotion, roundoffReserve_units, obstacleTarget_units);
    refinedCollisionPairs = ~reshape([refinedProof.Planes.Verified], ...
        size(refinedProof.Planes)) & refinedProof.RegionActiveBySegment;
    if any(refinedCollisionPairs, "all")
        [rebuiltPlanes, ~, complete] = ...
            bmtpEngine.separation.createTimeScopedPlanes(baseControl_units, ...
            baseSegmentTime_s, request, ...
            obstacleTarget_units, roundoffReserve_units);
        planeSetChanged = ~isequaln(rebuiltPlanes, travelPlanes);
        if ~complete || ~planeSetChanged
            break
        end
        travelPlanes = rebuiltPlanes;
        continue
    end
    if ~refinedProof.Passed
        break
    end
    refinedLength_units = controlPolygonLength(refinedControl_units);
    % Replace the current travel profile only when refinement improves the
    % declared objective and remains feasible.
    refinementIsBetter = refinedLength_units < selectedLength_units;
    if refinementIsBetter
        selectedControl_units    = refinedControl_units;
        selectedSegmentTime_s    = refinedSegmentTime_s;
        selectedPlanes           = refinedProof.Planes;
        selectedPairs            = refinedProof.RegionActiveBySegment;
        selectedSolverMessage    = "A travel-shortened time-scoped feasible iterate was retained.";
        selectedLength_units     = refinedLength_units;
        travelRefinementAccepted = true;
    end
    break
end

%% Section 3: Return The Best Travel Attempt

if travelRefinementAccepted
    result.ControlPoint_units = selectedControl_units;
    result.SegmentTime_s      = selectedSegmentTime_s;
    result.Planes             = selectedPlanes;
    result.TaggedPairs        = selectedPairs;
    result.SolverMessage      = selectedSolverMessage;
    diagnostics.ApplicablePairCount     = nnz(result.TaggedPairs);
    diagnostics.TaggedPairCount         = nnz(result.TaggedPairs);
    diagnostics.FinalCollisionPairCount = 0;
    diagnostics.SolverMessage           = result.SolverMessage;
end
diagnostics.TravelRefinementFinalLength_units = selectedLength_units;
diagnostics.TravelRefinementFinalDuration_s   = sum(selectedSegmentTime_s);
diagnostics.TravelRefinementAccepted          = travelRefinementAccepted;
end

%% Section 4: Local Functions

function length_units = controlPolygonLength(controlPoint_units)
    % Sum Bezier control-edge lengths as a convex travel estimate.
    edge_units   = diff(controlPoint_units, 1, 2);
    length_units = sum(vecnorm(edge_units, 2, 3), "all");
end
