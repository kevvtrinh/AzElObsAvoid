function [result, diagnosis] = assemblePlannerOutputs(record, includeDiagnosis)
%% Section 0: Header & Readme
% SYNTAX: [result, diagnosis] = assemblePlannerOutputs(record, includeDiagnosis)
% PURPOSE: Separate usable motion from optional complete search/solver evidence.
% INPUTS: Completed internal record and a logical selector for the second output.
% OUTPUTS: Motion with validation inputs; optional structured evidence without duplication.
% UNITS: Coordinate units, seconds, and physical derivatives.

%% Section 1: Keep The Motion And Its Validation Inputs
names = ["Success", "Message", "TerminationReason", "Inputs", "Options", ...
    "time_s", "position_units", "velocity_units_s", "acceleration_units_s2", ...
    "jerk_units_s3", "Polynomial", "PlaneCertificate", "SeedCorridor", ...
    "SeedCorridorBoundary_units", "Validation", "ArrivalTime_s", ...
    "TrajectoryDuration_s", "ElapsedPlanningTime_s"];
result = struct();
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
solverDetails      = {record.SeedSummaries.SolverDiagnostics};
searchRecord       = search.GraphSearch;
visibilityAttempts = struct([]);
if isfield(searchRecord, "VisibilityAttempts")
    visibilityAttempts = searchRecord.VisibilityAttempts;
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
    "SolverDetails", {solverDetails}, "VisibilityAttempts", visibilityAttempts, ...
    "DirectMotion", search.DirectAttempt, ...
    "PathRefinement", search.FixedClockExcursion, ...
    "Selection", search.SelectionPolicy);
end
