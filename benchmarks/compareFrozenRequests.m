function comparison = compareFrozenRequests(baselinePath, candidatePath, outputPath)
%% Section 0: Header & Readme
% SYNTAX: comparison = compareFrozenRequests(baselinePath,candidatePath,outputPath)
% PURPOSE: Compare complete motion and route decisions separately from timings.
% INPUTS: Captures from benchmarkFrozenRequests and an output MAT path.
% OUTPUTS: Per-case equality, adaptive lengths, individual and median timings.
% UNITS: Coordinate units and seconds.

%% Section 1: Require Identical Case Inventories And Repetition Counts
before = load(baselinePath,'report'); before = before.report;
after = load(candidatePath,'report'); after = after.report;
assert(isequal(before.CaseNames,after.CaseNames));
assert(isequal(size(before.Results),size(after.Results)));
physicalFields = {'Success','TerminationReason','Route_units','time_s', ...
    'position_units','velocity_units_s','acceleration_units_s2','jerk_units_s3', ...
    'Polynomial','PlaneCertificate','SeedCorridor','SeedCorridorBoundary_units', ...
    'ArrivalTime_s','TrajectoryDuration_s'};
comparison = struct();
comparison.CaseNames = before.CaseNames;
comparison.PhysicalEqual = true(size(before.Results));
comparison.SearchEqual = true(size(before.Results));
comparison.IndependentValid = true(size(before.Results));

%% Section 2: Compare Physical Outputs And Fresh Acceptance Evidence
for k = 1:numel(before.Results)
    first = before.Results{k}; second = after.Results{k};
    for fieldIndex = 1:numel(physicalFields)
        name = physicalFields{fieldIndex};
        if ~isequaln(stripPlannerRefactorRuntime(first.(name)),stripPlannerRefactorRuntime(second.(name)))
            comparison.PhysicalEqual(k) = false;
            fprintf('PHYSICAL_DIFFERENCE record=%d field=%s\n',k,name);
        end
    end
    comparison.SearchEqual(k) = isequaln(stripPlannerRefactorRuntime(before.Diagnoses{k}.Search), ...
        stripPlannerRefactorRuntime(after.Diagnoses{k}.Search));
    comparison.IndependentValid(k) = (~first.Success || before.Validation{k}.Passed) && ...
        (~second.Success || after.Validation{k}.Passed);
end
comparison.LengthEqual = isequaln(before.Length_units,after.Length_units);
comparison.Passed = all(comparison.PhysicalEqual,'all') && all(comparison.SearchEqual,'all') && ...
    all(comparison.IndependentValid,'all') && comparison.LengthEqual;
comparison.BaselineTime_s = before.ElapsedTime_s;
comparison.CandidateTime_s = after.ElapsedTime_s;
comparison.Summary = table(before.CaseNames,median(before.ElapsedTime_s,2),median(after.ElapsedTime_s,2), ...
    'VariableNames',{'Case','BaselineMedian_s','CandidateMedian_s'});
comparison.Summary.Ratio = comparison.Summary.CandidateMedian_s ./ comparison.Summary.BaselineMedian_s;
disp(comparison.Summary);
fprintf('FROZEN_COMPARISON passed=%d physical_equal=%d search_equal=%d independently_valid=%d length_equal=%d\n', ...
    comparison.Passed,all(comparison.PhysicalEqual,'all'),all(comparison.SearchEqual,'all'), ...
    all(comparison.IndependentValid,'all'),comparison.LengthEqual);
save(outputPath,'comparison');
end
