function [candidate, diagnostics] = solveStaticBmtpTrajectory(seed, geometry, initialState, goalState, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [candidate, diagnostics] = solveStaticBmtpTrajectory( ...
%       seed, geometry, initialState, goalState, limits, options)
% PURPOSE
%   Solve against prepared regions; retry exact regions if grouping fails.
% INPUTS
%   seed contains N-by-2 positions and tau from zero to one.
%   geometry comes from prepareStaticSolverGeometry; states, limits, and options
%   are normalized. Matching fixed-target metadata is removed at this boundary.
% OUTPUTS
%   Candidate motion or failure, with engine and geometry-retry diagnostics.
% UNITS
%   Degrees, seconds, and derivatives in deg/s, deg/s^2, and deg/s^3.

%% Section 1: Read The Prepared Exclusion Regions
goalState        = resolveFixedEndpointForSolver(goalState, options);
exactRegions_deg = geometry.ExactRegions_deg;
regions_deg      = geometry.Regions_deg;
grouping         = geometry.Grouping;
coverage         = geometry.Coverage;

%% Section 2: Generate The Motion In The Independent Engine

[candidate, diagnostics] = bmtpEngine.solve(seed, regions_deg, coverage, initialState, goalState, limits, options);
fallback = struct("Attempted", false, ...
    "PrimaryTerminationReason", candidate.TerminationReason, ...
    "Outcome", "notApplicable", ...
    "ExactRegionCount", numel(exactRegions_deg), ...
    "PrimarySolverDiagnostics", struct());
% Try grouped static regions first to reduce solver size; the ungrouped retry remains available if grouping fails.
if grouping.Applied
    fallback.Outcome = "groupedAttemptAccepted";
end
if grouping.Applied && ~candidate.Success
    % Convex hulls may close real gaps. Retry the exact regions if grouping fails.
    fallback.Attempted                = true;
    fallback.Outcome                  = "exactRegionAttemptFailed";
    fallback.PrimarySolverDiagnostics = diagnostics;
    exactCoverage = coverage;
    exactCoverage.SolverRegionCount = numel(exactRegions_deg);
    exactGrouping = grouping;
    exactGrouping.Applied                 = false;
    exactGrouping.SolverRegionCount       = numel(exactRegions_deg);
    exactGrouping.RelationToExactGeometry = "equal";
    exactGrouping.GroupMemberIndices      = num2cell((1:numel(exactRegions_deg)).');
    exactCoverage.ConservativeGrouping = exactGrouping;
    [candidate, diagnostics] = bmtpEngine.solve(seed, exactRegions_deg, exactCoverage, initialState, goalState, limits, options);
    % Promote the successful candidate; otherwise continue the configured fallback or search path.
    if candidate.Success
        fallback.Outcome = "exactRegionAttemptAccepted";
    end
end
diagnostics.ExactRegionFallback = fallback;
candidate.SolverDiagnostics = diagnostics;
end

function solverGoalState = resolveFixedEndpointForSolver(goalState, options)
    % Remove target history only when the fixed endpoint matches the target.
    solverGoalState  = goalState;
    hasTargetHistory = isfield(goalState, "targetTime_s") && ~isempty(goalState.targetTime_s);
    if ~hasTargetHistory || string(options.GoalTimeMode) ~= "fixedArrival"
        return;
    end
    targetPosition_deg  = obstacleAvoidance.input.goalPositionAtTime(goalState, goalState.time_s);
    coordinateScale_deg = bmtpEngine.createCoordinateTolerances(targetPosition_deg, goalState.position_deg);
    if max(abs(targetPosition_deg - goalState.position_deg)) > 256 * eps(coordinateScale_deg)
        return;
    end
    metadataFields  = intersect(fieldnames(solverGoalState), {'targetTime_s', 'targetPosition_deg', 'InterpolationMethod'});
    solverGoalState = rmfield(solverGoalState, metadataFields);
end
