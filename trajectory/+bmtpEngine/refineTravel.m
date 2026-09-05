function [result, diagnostics] = refineTravel( ...
        request, warmStart, alternatingResult, diagnostics, ...
        obstacleTarget_deg, roundoffReserve_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnostics] = bmtpEngine.refineTravel( ...
%       request, warmStart, alternatingResult, diagnostics, ...
%       obstacleTarget_deg, roundoffReserve_deg)
%**************************************************************************
% PURPOSE
%   - Reduce the convex travel surrogate after the alternating solve has
%     established a feasible obstacle homotopy.
%**************************************************************************
% INPUTS
%   - request, warmStart, alternatingResult, diagnostics (scalar structs)
%       Checked request, prepared curve, retained attempt, and diagnostics.
%   - obstacleTarget_deg, roundoffReserve_deg (finite scalars)
%       Required obstacle-side target and numerical reserve in degrees.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Selected controls, segment time, planes, and active-pair tags.
%   - diagnostics (scalar struct)
%       Updated active-pair count after optional refinement.
%**************************************************************************
% UNITS
%   - Position and travel are degrees; time is seconds.
%**************************************************************************

%% Section 1: Preserve The Feasible Alternating Result

result = struct( ...
    "ControlPoint_deg", alternatingResult.ControlPoint_deg, ...
    "SegmentTime_s", alternatingResult.SegmentTime_s, ...
    "Planes", alternatingResult.Planes, ...
    "TaggedPairs", alternatingResult.TaggedPairs);
if request.Options.GoalTimeMode == "earliestArrival"
    return;
end

%% Section 2: Try Travel-Weighted Solves

segmentCount = warmStart.SegmentCount;
baseControl_deg = result.ControlPoint_deg;
baseSegmentTime_s = result.SegmentTime_s;
baseLength_deg = controlPolygonLength(baseControl_deg);
selectedControl_deg = baseControl_deg;
selectedSegmentTime_s = baseSegmentTime_s;
selectedPlanes = result.Planes;
selectedLength_deg = baseLength_deg;
selectedCost_deg = baseLength_deg + request.TravelSavingsRate_deg_s * ...
    segmentCount * baseSegmentTime_s;
trialSavingsRates_deg_s = request.TravelSavingsRate_deg_s;
travelRefinementAccepted = false;
for portfolioIndex = 1:numel(trialSavingsRates_deg_s)
    travelPlanes = result.Planes;
    trialRate_deg_s = trialSavingsRates_deg_s(portfolioIndex);
    for refinementIndex = 1:8
        [refinedControl_deg, refinedSegmentTime_s, travelExitFlag, output] = ...
            bmtpEngine.solveTrajectoryStep( ...
            segmentCount, request.Degree, ...
            request.InitialState.position_deg, request.GoalState.position_deg, ...
            request.Limits, travelPlanes, roundoffReserve_deg, ...
            request.MotionHorizon_s, request.Options.GoalTimeMode, ...
            trialRate_deg_s, baseSegmentTime_s, request.TrajectoryOptions);
        diagnostics.ConicSolver = fastcone.accumulate(diagnostics.ConicSolver, output);
        if travelExitFlag <= 0 || isempty(refinedControl_deg)
            break;
        end
        refinedCollisionPairs = bmtpEngine.findSampledObstacleOverlaps( ...
            refinedControl_deg, request.Regions_deg, ...
            request.RegionMinimum_deg, request.RegionMaximum_deg, ...
            warmStart.RegionActiveBySegment, 1201);
        if any(refinedCollisionPairs, "all")
            activeTravelPairs = reshape( ...
                [travelPlanes.Active], size(travelPlanes));
            newPairs = refinedCollisionPairs & ~activeTravelPairs;
            newPairIndices = reshape(find(newPairs), 1, []);
            if isempty(newPairIndices)
                break;
            end
            planeUpdateFailed = false;
            for newPairIndex = newPairIndices
                [segmentIndex, regionIndex] = ind2sub( ...
                    size(newPairs), newPairIndex);
                [travelPlane, planeExitFlag, planeOutput] = ...
                    bmtpEngine.solveSeparatingLine( ...
                    squeeze(baseControl_deg(segmentIndex, :, :)), ...
                    request.Regions_deg{regionIndex}, obstacleTarget_deg, ...
                    roundoffReserve_deg, request.PlaneOptions);
                diagnostics.ConicSolver = fastcone.accumulate( ...
                    diagnostics.ConicSolver, planeOutput);
                if planeExitFlag <= 0 || ~travelPlane.Active
                    planeUpdateFailed = true;
                    break;
                end
                travelPlanes(segmentIndex, regionIndex) = travelPlane;
            end
            if planeUpdateFailed
                break;
            end
            continue;
        end
        refinedLength_deg = controlPolygonLength(refinedControl_deg);
        refinedCost_deg = refinedLength_deg + ...
            request.TravelSavingsRate_deg_s * ...
            segmentCount * refinedSegmentTime_s;
        if request.Options.GoalTimeMode == "fixedArrival"
            refinementIsBetter = refinedLength_deg < selectedLength_deg;
        else
            refinementIsBetter = refinedCost_deg < selectedCost_deg;
        end
        if refinementIsBetter
            selectedControl_deg = refinedControl_deg;
            selectedSegmentTime_s = refinedSegmentTime_s;
            selectedPlanes = travelPlanes;
            selectedLength_deg = refinedLength_deg;
            selectedCost_deg = refinedCost_deg;
            travelRefinementAccepted = true;
        end
        break;
    end
end

%% Section 3: Return The Best Travel Attempt

if travelRefinementAccepted
    result.ControlPoint_deg = selectedControl_deg;
    result.SegmentTime_s = selectedSegmentTime_s;
    result.Planes = selectedPlanes;
    result.TaggedPairs = result.TaggedPairs | reshape( ...
        [selectedPlanes.Active], size(selectedPlanes));
end
diagnostics.TaggedPairCount = nnz(result.TaggedPairs);
end

%% Section 4: Local Functions

function length_deg = controlPolygonLength(controlPoint_deg)
% Return the convex Bezier travel surrogate used by the secondary SOCP.
edge_deg = diff(controlPoint_deg, 1, 2);
length_deg = sum(vecnorm(edge_deg, 2, 3), "all");
end
