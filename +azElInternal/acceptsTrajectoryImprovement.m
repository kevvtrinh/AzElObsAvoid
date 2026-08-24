function [accepted, comparison] = acceptsTrajectoryImprovement(baseline, trial)
%% Section 0: Header & Readme
% SYNTAX
%   [accepted, comparison] = azElInternal.acceptsTrajectoryImprovement( ...
%       baseline, trial)
%**************************************************************************
% PURPOSE
%   - Compare method-neutral validated arrival-and-jerk quality records.
%   - Admit recovery from an invalid baseline, or a machine-scale no-worse
%     valid trial with at least one material strict improvement.
%**************************************************************************

%% Section 1: Resolve Comparable Quality Records

requiredFields = {'Validation'};
if ~isstruct(baseline) || ~isscalar(baseline) || ...
        ~isstruct(trial) || ~isscalar(trial) || ...
        ~all(isfield(baseline, requiredFields)) || ...
        ~all(isfield(trial, requiredFields))
    error("azElInternal:acceptsTrajectoryImprovement:InvalidRecord", ...
        "Both records need Validation, ArrivalTime_s, and integrated jerk.");
end
[baselineValid, baselineArrival_s, baselineJerk] = quality(baseline);
[trialValid, trialArrival_s, trialJerk] = quality(trial);
arrivalTolerance_s = 8 * eps(max([1, abs(baselineArrival_s), abs(trialArrival_s)]));
jerkTolerance = 8 * eps(max([1, abs(baselineJerk), abs(trialJerk)]));

%% Section 2: Apply The Strict Monotone Acceptance Rule

arrivalNoWorse = trialValid && trialArrival_s <= ...
    baselineArrival_s + arrivalTolerance_s;
jerkNoWorse = trialValid && trialJerk <= baselineJerk + jerkTolerance;
arrivalStrictlyBetter = trialValid && trialArrival_s < ...
    baselineArrival_s - arrivalTolerance_s;
jerkStrictlyBetter = trialValid && trialJerk < baselineJerk - jerkTolerance;
if baselineValid
    accepted = arrivalNoWorse && jerkNoWorse && ...
        (arrivalStrictlyBetter || jerkStrictlyBetter);
else
    accepted = trialValid;
end
comparison = struct( ...
    "BaselineValid", baselineValid, "TrialValid", trialValid, ...
    "ArrivalTolerance_s", arrivalTolerance_s, ...
    "JerkTolerance_deg2_s5", jerkTolerance, ...
    "ArrivalNoWorse", arrivalNoWorse, "JerkNoWorse", jerkNoWorse, ...
    "ArrivalStrictlyBetter", arrivalStrictlyBetter, ...
    "JerkStrictlyBetter", jerkStrictlyBetter);
end

function passed = validationPassed(validation)
passed = isstruct(validation) && isscalar(validation) && ...
    isfield(validation, "Passed") && isequal(validation.Passed, true);
end

function [valid, arrival_s, jerk] = quality(record)
valid = validationPassed(record.Validation);
if isfield(record, "Success")
    valid = valid && isequal(record.Success, true);
end
if isfield(record, "ArrivalTime_s")
    arrival_s = double(record.ArrivalTime_s);
elseif isfield(record, "FinalTime_s")
    arrival_s = double(record.FinalTime_s);
else
    error("azElInternal:acceptsTrajectoryImprovement:MissingArrival", ...
        "Each record needs ArrivalTime_s or FinalTime_s.");
end
if isfield(record, "IntegratedSquaredJerk_deg2_s5")
    jerk = double(record.IntegratedSquaredJerk_deg2_s5);
elseif isfield(record, "Polynomial")
    jerk = azElInternal.integratedSquaredPolynomialJerk(record.Polynomial);
else
    error("azElInternal:acceptsTrajectoryImprovement:MissingJerk", ...
        "Each record needs integrated jerk or a shared Polynomial.");
end
if valid && (~isscalar(arrival_s) || ~isfinite(arrival_s) || ...
        ~isscalar(jerk) || ~isfinite(jerk))
    error("azElInternal:acceptsTrajectoryImprovement:InvalidQuality", ...
        "Validated records need finite scalar arrival and jerk quality.");
end
end
