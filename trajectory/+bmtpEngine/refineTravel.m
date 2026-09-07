function [result, diagnostics] = refineTravel(request, warmStart, alternatingResult, diagnostics, obstacleTarget_units, roundoffReserve_units)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.refineTravel( ...
%       request, warmStart, alternatingResult, diagnostics, ...
%       obstacleTarget_units, roundoffReserve_units)
%
% PURPOSE
%   - Reduce the convex travel surrogate after the alternating solve has
%     established a feasible obstacle homotopy.
%
% INPUTS
%   - request, warmStart, alternatingResult, diagnostics (scalar structs)
%       Checked request, prepared curve, retained attempt, and diagnostics.
%   - obstacleTarget_units, roundoffReserve_units (finite scalars)
%       Required obstacle-side target and numerical reserve in coordinate units.
%
% OUTPUTS
%   - result (scalar struct)
%       Selected controls and segment time.
%   - diagnostics (scalar struct)
%       Updated active-pair count after optional refinement.
%
% UNITS
%   - Position and travel are coordinate units; time is seconds.
%

%% Section 1: Preserve The Feasible Alternating Result

result = struct("ControlPoint_units", alternatingResult.ControlPoint_units, ...
    "SegmentTime_s", alternatingResult.SegmentTime_s);

%% Section 2: Refine Travel At The Selected Arrival Clock

segmentCount             = warmStart.SegmentCount;
baseControl_units          = result.ControlPoint_units;
baseSegmentTime_s        = result.SegmentTime_s;
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
    refinementHorizon_s = segmentCount * baseSegmentTime_s;
end
diagnostics.TravelRefinementAttempted         = true;
diagnostics.TravelRefinementInitialLength_units = baseLength_units;
diagnostics.TravelRefinementFinalLength_units   = baseLength_units;
diagnostics.TravelRefinementInitialDuration_s = segmentCount * baseSegmentTime_s;
diagnostics.TravelRefinementFinalDuration_s   = segmentCount * baseSegmentTime_s;
diagnostics.TravelRefinementAccepted          = false;
% Repeat the refinement alternatives needed to refine the current solution.
for refinementIndex = 1:8
    [refinedControl_units, refinedSegmentTime_s, travelExitFlag, output] = bmtpEngine.solveTrajectoryStep(segmentCount, request.Degree, request.InitialState.position_units, request.GoalState.position_units, request.Limits, travelPlanes, roundoffReserve_units, refinementHorizon_s, "fixedArrival", request.TrajectoryOptions);
    diagnostics.ConicSolver = bmtpEngine.accumulateConicDiagnostics(diagnostics.ConicSolver, output);
    if travelExitFlag <= 0 || isempty(refinedControl_units)
        break;
    end
    refinedCollisionPairs = bmtpEngine.findSampledObstacleOverlaps(refinedControl_units, request.Regions_units, request.RegionMinimum_units, request.RegionMaximum_units, warmStart.RegionActiveBySegment, 1201);
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
            [travelPlane, planeExitFlag, planeOutput] = bmtpEngine.solveSeparatingLine(squeeze(baseControl_units(segmentIndex, :, :)), request.Regions_units{regionIndex}, obstacleTarget_units, roundoffReserve_units, request.PlaneOptions);
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
diagnostics.TravelRefinementFinalDuration_s = segmentCount * selectedSegmentTime_s;
diagnostics.TravelRefinementAccepted        = travelRefinementAccepted;
end

%% Section 4: Local Functions

function length_units = controlPolygonLength(controlPoint_units)
    % Sum Bezier control-edge lengths as a convex travel estimate.
    edge_units   = diff(controlPoint_units, 1, 2);
    length_units = sum(vecnorm(edge_units, 2, 3), "all");
end
