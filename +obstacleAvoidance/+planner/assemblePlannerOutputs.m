function [result, diagnosis] = assemblePlannerOutputs(record, includeDiagnosis)
%% Section 0: Header & Readme
% SYNTAX
%   [result, diagnosis] = assemblePlannerOutputs(record, includeDiagnosis)
% PURPOSE
%   Separate the usable motion from optional search and solver evidence.
% INPUTS
%   record is the completed internal planner record.
%   includeDiagnosis selects whether to assemble the second output.
% OUTPUTS
%   result contains motion, plotting inputs, and independent-validation data.
%   diagnosis contains shallow search records and flat solver-detail tables.
% UNITS
%   Positions are coordinate units, times are seconds, and derivatives retain their units.

%% Section 1: Keep The Motion And Its Validation Inputs
names = ["Success", "Message", "TerminationReason", "Inputs", "Options", ...
    "time_s", "position_units", "velocity_units_s", "acceleration_units_s2", ...
    "jerk_units_s3", "Polynomial", "PlaneCertificate", "SeedCorridor", ...
    "SeedCorridorBoundary_units", "Validation", "ArrivalTime_s", ...
    "TrajectoryDuration_s", "ElapsedPlanningTime_s"];
result = struct();
% Apply the required validation or transfer to each name.
for name = names
    result.(name) = record.(name);
end
result.Route_units            = record.SelectedSeed_units;
result.BestPartialRoute_units = zeros(0, 2);
search = record.SearchDiagnostics;
% Expose the best partial seed only when no complete candidate succeeded.
if search.BestPartialSeedIndex > 0 && ~record.Success
    result.BestPartialRoute_units = record.Seeds(search.BestPartialSeedIndex).position_units;
% Use the graph search's partial route only when no seed produced a more concrete failed motion.
elseif isfield(search.GraphSearch, "BestPartialRoute_units") && ~record.Success
    result.BestPartialRoute_units = search.GraphSearch.BestPartialRoute_units;
end

%% Section 2: Assemble Optional Diagnosis Without Duplicate Records
diagnosis = struct();
% Skip optional diagnostic assembly when the caller requested only the planning result.
if ~includeDiagnosis, return; end
attempts           = rmfield(record.SeedSummaries, "SolverDiagnostics");
solverDetails      = flattenAttempts({record.SeedSummaries.SolverDiagnostics});
searchRecord       = search.GraphSearch;
visibilityAttempts = flattenAttempts({});
if isfield(searchRecord, "VisibilityAttempts")
    visibilityAttempts = flattenAttempts(num2cell(searchRecord.VisibilityAttempts));
    searchRecord       = rmfield(searchRecord, "VisibilityAttempts");
end
coverage = struct();
if isfield(searchRecord, "Coverage")
    coverage     = searchRecord.Coverage;
    searchRecord = rmfield(searchRecord, "Coverage");
end
diagnosis = struct("SelectedAttemptIndex", record.SelectedSeedIndex, ...
    "BestPartialAttemptIndex", search.BestPartialSeedIndex, ...
    "AttemptedCount", search.AttemptedSeedCount, ...
    "ValidatedCount", search.ValidatedCandidateCount, ...
    "FirstValidatedMotionTime_s", record.FirstValidatedMotionTime_s, ...
    "Timing", search.StageTiming, ...
    "Search", searchRecord, "SearchCoverage", coverage, ...
    "Routes", record.Seeds, "Attempts", attempts, ...
    "SolverDetails", solverDetails, "VisibilityAttempts", visibilityAttempts, ...
    "DirectMotion", obstacleAvoidance.planner.flattenDiagnosis(search.DirectAttempt), ...
    "PathRefinement", obstacleAvoidance.planner.flattenDiagnosis(search.FixedClockExcursion), ...
    "Selection", search.SelectionPolicy);
end

function combined = flattenAttempts(records)
    % Preserve per-attempt evidence in one shallow table.
    combined = table(zeros(0,1), strings(0,1), cell(0,1), ...
        'VariableNames', {'Attempt', 'Field', 'Value'});
    % Process each item needed to complete flatten attempts.
    for index = 1:numel(records)
        details  = obstacleAvoidance.planner.flattenDiagnosis(records{index});
        combined = [combined; table(repmat(index,height(details),1), ...
            details.Field, details.Value, 'VariableNames', {'Attempt','Field','Value'})]; %#ok<AGROW>
    end
end
