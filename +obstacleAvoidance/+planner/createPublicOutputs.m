function [result, diagnosis] = createPublicOutputs(record, includeDiagnosis)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnosis] = createPublicOutputs(record, includeDiagnosis)
% PURPOSE
%   Separate the usable motion from optional search and solver evidence.
% INPUTS
%   record is the completed internal planner record.
%   includeDiagnosis selects whether to assemble the second output.
% OUTPUTS
%   result contains motion, plotting inputs, and independent-validation data.
%   diagnosis contains shallow search records and flat solver-detail tables.
% UNITS
%   Positions are degrees, times are seconds, and derivatives retain their units.

%% Section 1: Keep The Motion And Its Validation Inputs
names = ["Success", "Message", "TerminationReason", "Inputs", "Options", ...
    "time_s", "position_deg", "velocity_deg_s", "acceleration_deg_s2", ...
    "jerk_deg_s3", "Polynomial", "PlaneCertificate", "SeedCorridor", ...
    "SeedCorridorBoundary_deg", "Validation", "ArrivalTime_s", ...
    "TrajectoryDuration_s", "ElapsedPlanningTime_s"];
result = struct();
for name = names
    result.(name) = record.(name);
end
result.Route_deg = record.SelectedSeed_deg;
result.BestPartialRoute_deg = zeros(0, 2);
search = record.SearchDiagnostics;
if search.BestPartialSeedIndex > 0 && ~record.Success
    result.BestPartialRoute_deg = record.Seeds(search.BestPartialSeedIndex).position_deg;
elseif isfield(search.Grid, "BestPartialRoute_deg") && ~record.Success
    result.BestPartialRoute_deg = search.Grid.BestPartialRoute_deg;
end

%% Section 2: Assemble Optional Diagnosis Without Duplicate Records
diagnosis = struct();
if ~includeDiagnosis, return; end
attempts = rmfield(record.SeedSummaries, "SolverDiagnostics");
solverDetails = table(zeros(0,1), strings(0,1), cell(0,1), ...
    'VariableNames', {'Attempt', 'Field', 'Value'});
for index = 1:numel(record.SeedSummaries)
    details = obstacleAvoidance.planner.flattenDiagnosis(record.SeedSummaries(index).SolverDiagnostics);
    solverDetails = [solverDetails; table(repmat(index,height(details),1), ...
        details.Field, details.Value, 'VariableNames', {'Attempt','Field','Value'})]; %#ok<AGROW>
end
searchRecord = search.Grid;
coverage = struct();
if isfield(searchRecord, "Coverage")
    coverage = searchRecord.Coverage;
    searchRecord = rmfield(searchRecord, "Coverage");
end
diagnosis = struct( ...
    "SelectedAttemptIndex", record.SelectedSeedIndex, ...
    "BestPartialAttemptIndex", search.BestPartialSeedIndex, ...
    "AttemptedCount", search.AttemptedSeedCount, ...
    "ValidatedCount", search.ValidatedCandidateCount, ...
    "FirstValidatedMotionTime_s", record.FirstValidatedMotionTime_s, ...
    "Timing", search.StageTiming, ...
    "Search", searchRecord, "SearchCoverage", coverage, ...
    "Routes", record.Seeds, "Attempts", attempts, ...
    "SolverDetails", solverDetails, ...
    "DirectMotion", obstacleAvoidance.planner.flattenDiagnosis(search.DirectAttempt), ...
    "PathRefinement", obstacleAvoidance.planner.flattenDiagnosis(search.FixedClockExcursion), ...
    "Selection", search.SelectionPolicy);
end
