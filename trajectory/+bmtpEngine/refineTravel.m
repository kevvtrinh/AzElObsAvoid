function [result, diagnostics] = refineTravel(request, warmStart, alternatingResult, diagnostics, obstacleTarget_deg, roundoffReserve_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.refineTravel( ...
%       request, warmStart, alternatingResult, diagnostics, ...
%       obstacleTarget_deg, roundoffReserve_deg)
%
% PURPOSE
%   - Reduce the convex travel surrogate after the alternating solve has
%     established a feasible obstacle homotopy.
%
% INPUTS
%   - request, warmStart, alternatingResult, diagnostics (scalar structs)
%       Checked request, prepared curve, retained attempt, and diagnostics.
%   - obstacleTarget_deg, roundoffReserve_deg (finite scalars)
%       Required obstacle-side target and numerical reserve in degrees.
%
% OUTPUTS
%   - result (scalar struct)
%       Selected controls and segment time.
%   - diagnostics (scalar struct)
%       Updated active-pair count after optional refinement.
%
% UNITS
%   - Position and travel are degrees; time is seconds.
%

%% Section 1: Preserve The Feasible Alternating Result

result = struct("ControlPoint_deg", alternatingResult.ControlPoint_deg, ...
    "SegmentTime_s", alternatingResult.SegmentTime_s);

%% Section 2: Refine Travel At The Selected Arrival Clock

segmentCount             = warmStart.SegmentCount;
baseControl_deg          = result.ControlPoint_deg;
baseSegmentTime_s        = result.SegmentTime_s;
baseLength_deg           = controlPolygonLength(baseControl_deg);
selectedControl_deg      = baseControl_deg;
selectedSegmentTime_s    = baseSegmentTime_s;
selectedPlanes           = alternatingResult.Planes;
selectedLength_deg       = baseLength_deg;
travelRefinementAccepted = false;
travelPlanes             = alternatingResult.Planes;
taggedPairs              = alternatingResult.TaggedPairs;
refinementHorizon_s      = request.MotionHorizon_s;
if request.Options.GoalTimeMode == "earliestArrival"
    % Preserve the proven earliest clock exactly, then minimize travel at that
    % clock. This realizes the documented path-length tie-break without paying
    % extra arrival time when the former balanced policy cannot justify it.
    refinementHorizon_s = segmentCount * baseSegmentTime_s;
end
diagnostics.TravelRefinementAttempted         = true;
diagnostics.TravelRefinementInitialLength_deg = baseLength_deg;
diagnostics.TravelRefinementFinalLength_deg   = baseLength_deg;
diagnostics.TravelRefinementInitialDuration_s = segmentCount * baseSegmentTime_s;
diagnostics.TravelRefinementFinalDuration_s   = segmentCount * baseSegmentTime_s;
diagnostics.TravelRefinementAccepted          = false;
% Repeat the refinement alternatives needed to refine the current solution.
for refinementIndex = 1:8
    [refinedControl_deg, refinedSegmentTime_s, travelExitFlag, output] = bmtpEngine.solveTrajectoryStep(segmentCount, request.Degree, request.InitialState.position_deg, request.GoalState.position_deg, request.Limits, travelPlanes, roundoffReserve_deg, refinementHorizon_s, "fixedArrival", request.TrajectoryOptions);
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    if travelExitFlag <= 0 || isempty(refinedControl_deg)
        break;
    end
    refinedCollisionPairs = bmtpEngine.findSampledObstacleOverlaps(refinedControl_deg, request.Regions_deg, request.RegionMinimum_deg, request.RegionMaximum_deg, warmStart.RegionActiveBySegment, 1201);
    if any(refinedCollisionPairs, "all")
        activeTravelPairs = reshape([travelPlanes.Active], size(travelPlanes));
        newPairs          = refinedCollisionPairs & ~activeTravelPairs;
        newPairIndices    = reshape(find(newPairs), 1, []);
        if isempty(newPairIndices)
            break;
        end
        planeUpdateFailed = false;
        % Process each new pair needed to find travel.
        for newPairIndex = newPairIndices
            [segmentIndex, regionIndex]               = ind2sub(size(newPairs), newPairIndex);
            [travelPlane, planeExitFlag, planeOutput] = bmtpEngine.solveSeparatingLine(squeeze(baseControl_deg(segmentIndex, :, :)), request.Regions_deg{regionIndex}, obstacleTarget_deg, roundoffReserve_deg, request.PlaneOptions);
            diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, planeOutput);
            if planeExitFlag <= 0 || ~travelPlane.Active
                planeUpdateFailed = true;
                break;
            end
            travelPlanes(segmentIndex, regionIndex) = travelPlane;
        end
        % Terminate with the recorded failure if an alternating update cannot produce a valid control or separating plane.
        if planeUpdateFailed
            break;
        end
        continue;
    end
    refinedLength_deg  = controlPolygonLength(refinedControl_deg);
    refinementIsBetter = refinedLength_deg < selectedLength_deg;
    % Replace the current travel profile only when refinement improves the declared objective and remains feasible.
    if refinementIsBetter
        selectedControl_deg      = refinedControl_deg;
        selectedSegmentTime_s    = refinedSegmentTime_s;
        selectedPlanes           = travelPlanes;
        selectedLength_deg       = refinedLength_deg;
        travelRefinementAccepted = true;
    end
    break;
end

%% Section 3: Return The Best Travel Attempt

if travelRefinementAccepted
    result.ControlPoint_deg = selectedControl_deg;
    result.SegmentTime_s    = selectedSegmentTime_s;
    taggedPairs = taggedPairs | reshape([selectedPlanes.Active], size(selectedPlanes));
end
diagnostics.TaggedPairCount = nnz(taggedPairs);
diagnostics.TravelRefinementFinalLength_deg = selectedLength_deg;
diagnostics.TravelRefinementFinalDuration_s = segmentCount * selectedSegmentTime_s;
diagnostics.TravelRefinementAccepted        = travelRefinementAccepted;
end

%% Section 4: Local Functions

function length_deg = controlPolygonLength(controlPoint_deg)
    % Sum Bezier control-edge lengths as a convex travel estimate.
    edge_deg   = diff(controlPoint_deg, 1, 2);
    length_deg = sum(vecnorm(edge_deg, 2, 3), "all");
end
