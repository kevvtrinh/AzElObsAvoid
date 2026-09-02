function result = createPlannerResult( ...
        result, seeds, candidates, summaries, selection, ...
        firstValidatedMotionTime_s, planningTimer, stageTiming)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planner.createPlannerResult( ...
%       result, seeds, candidates, summaries, selection, ...
%       firstValidatedMotionTime_s, planningTimer, stageTiming)
%**************************************************************************
% PURPOSE
%   - Assemble the stable planner result from completed candidate selection.
%   - Preserve failure evidence and copy motion only from the selected pass.
%**************************************************************************
% INPUTS
%   - result (scalar stable empty planner result)
%       Request, options, obstacles, and search diagnostics accumulated so far.
%   - seeds (route-seed struct array)
%       Attempted seeds in deterministic order.
%   - candidates (cell vector of scalar motion structs)
%       Attempted motions corresponding one-for-one with seeds.
%   - summaries (candidate-summary struct array)
%       Stable solve and full-check evidence corresponding to seeds.
%   - selection (scalar struct)
%       Output from selectValidatedCandidate.
%   - firstValidatedMotionTime_s (scalar numeric)
%       First-passing wall-clock observation or NaN.
%   - planningTimer (MATLAB tic handle)
%       Overall planning clock.
%   - stageTiming (scalar struct)
%       Accumulated stage durations before result assembly.
%**************************************************************************
% OUTPUTS
%   - result (scalar planner result)
%       Stable success-or-failure result with selected motion only on success.
%**************************************************************************
% UNITS
%   - Position is degrees; physical and measured times are seconds.
%**************************************************************************

%% Section 1: Attach Candidate And Selection Evidence

% These fields are meaningful on both success and failure. Attach them before
% branching so expected no-path outcomes retain the same stable result shape.

result.Seeds = seeds;
result.SeedSummaries = summaries;
result.SearchDiagnostics.SeedSummaries = summaries;
result.SearchDiagnostics.AttemptedSeedCount = numel(seeds);
result.SearchDiagnostics.FirstValidatedMotionTime_s = ...
    firstValidatedMotionTime_s;
result.FirstValidatedMotionTime_s = firstValidatedMotionTime_s;
result.SearchDiagnostics.ValidatedCandidateCount = ...
    selection.ValidatedCandidateCount;
result.SearchDiagnostics.BestPartialSeedIndex = ...
    selection.BestPartialSeedIndex;
result.Message = selection.Message;
result.TerminationReason = selection.TerminationReason;

%% Section 2: Copy Only A Fully Checked Motion

% selectValidatedCandidate exposes only indices whose summaries carry the
% authoritative passing check. Failure therefore returns diagnostic evidence
% without a trajectory, while success copies exactly one selected candidate.

if selection.Success
    selectedIndex = selection.SelectedCandidateIndex;
    selectedCandidate = candidates{selectedIndex};
    result.Success = true;
    result.SelectedSeedIndex = selectedIndex;
    result.SelectedSeed_deg = seeds(selectedIndex).position_deg;
    result = copyMotion(result, selectedCandidate);
end
result.SearchDiagnostics.TerminationReason = result.TerminationReason;
result = obstacleAvoidance.planner.stageTiming( ...
    result, planningTimer, stageTiming);
end

%% Section 3: Local Functions

function result = copyMotion(result, candidate)
% Copy the stable public motion payload and authoritative arrival fields.
for fieldName = ["time_s", "position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3", "Polynomial", ...
        "SeedCorridorBoundary_deg", "SeedCorridor", ...
        "PlaneCertificate", "Validation"]
    result.(fieldName) = candidate.(fieldName);
end
result.ArrivalTime_s = candidate.FinalTime_s;
result.TrajectoryDuration_s = candidate.MotionDuration_s;
end
