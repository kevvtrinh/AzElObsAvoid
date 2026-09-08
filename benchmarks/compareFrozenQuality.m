function comparison = compareFrozenQuality(baselinePath, candidatePath, outputPath)
%% Section 0: Header & Readme
% SYNTAX: comparison = compareFrozenQuality(baselinePath,candidatePath,outputPath)
% PURPOSE: Gate solver replacements on supported outcomes, arrival, and true length.
% INPUTS: Frozen request benchmark captures and an output MAT path.
% OUTPUTS: Per-record quality gates, individual timings, medians, and differences.
% UNITS: Seconds and coordinate units; length is independent adaptive integration.

%% Section 1: Require The Same Requests And Verified Successful Outcomes
before=load(baselinePath,'report'); before=before.report;
after=load(candidatePath,'report'); after=after.report;
assert(isequal(before.CaseNames,after.CaseNames));
assert(isequal(size(before.Results),size(after.Results)));
comparison=struct();
comparison.CaseNames=before.CaseNames;
comparison.PassedByRecord=true(size(before.Results));
comparison.ArrivalDifference_s=NaN(size(before.Results));
comparison.LengthDifference_units=after.Length_units-before.Length_units;
for k=1:numel(before.Results)
    original=before.Results{k}; candidate=after.Results{k};
    originalInputs=original.Inputs; candidateInputs=candidate.Inputs;
    if isfield(originalInputs.obstacles,'InternalPreparation')
        originalInputs.obstacles=rmfield(originalInputs.obstacles,'InternalPreparation');
    end
    if isfield(candidateInputs.obstacles,'InternalPreparation')
        candidateInputs.obstacles=rmfield(candidateInputs.obstacles,'InternalPreparation');
    end
    assert(isequaln(originalInputs,candidateInputs),'The physical request changed.');
    assert(isequaln(original.Options,candidate.Options),'Planner options changed.');
    passed=original.Success==candidate.Success;
    if original.Success && candidate.Success
        arrivalDifference_s=candidate.ArrivalTime_s-original.ArrivalTime_s;
        comparison.ArrivalDifference_s(k)=arrivalDifference_s;
        if original.Options.GoalTimeMode=="earliestArrival"
            arrivalPassed=arrivalDifference_s<=original.Options.ArrivalTimeTolerance_s;
        else
            arrivalPassed=abs(arrivalDifference_s)<=original.Options.ConstraintTolerance;
        end
        % Both lengths were independently integrated with 1e-10 quadrature
        % tolerances. Allow only a small numerical comparison reserve.
        lengthTolerance_units=1e-8*max(1,before.Length_units(k));
        passed=passed && after.Validation{k}.Passed && arrivalPassed && ...
            comparison.LengthDifference_units(k)<=lengthTolerance_units;
    end
    comparison.PassedByRecord(k)=passed;
end

%% Section 2: Report Individual Measurements And Median Runtime
comparison.BeforeTime_s=before.ElapsedTime_s;
comparison.AfterTime_s=after.ElapsedTime_s;
comparison.Passed=all(comparison.PassedByRecord,'all');
summary=table(before.CaseNames,median(before.ElapsedTime_s,2),median(after.ElapsedTime_s,2), ...
    max(comparison.ArrivalDifference_s,[],2),max(comparison.LengthDifference_units,[],2), ...
    'VariableNames',{'Case','BeforeMedian_s','AfterMedian_s','ArrivalDifference_s','LengthDifference_units'});
disp(summary);
fprintf('FROZEN_QUALITY passed=%d records=%d\n',comparison.Passed,numel(comparison.PassedByRecord));
save(outputPath,'comparison','summary');
assert(comparison.Passed,'A supported outcome, independent check, arrival, or true-length gate failed.');
end
