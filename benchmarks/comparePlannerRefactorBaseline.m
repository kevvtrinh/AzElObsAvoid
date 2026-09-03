function comparison = comparePlannerRefactorBaseline( ...
        baselinePath, candidatePath)
%% Section 0: Header & Readme
% SYNTAX
%   comparison = comparePlannerRefactorBaseline( ...
%       baselinePath, candidatePath)
%**************************************************************************
% PURPOSE
%   - Compare planner decisions and physical outputs from two example runs.
%   - Report runtime separately from the behavior-preservation decision.
%**************************************************************************
% INPUTS
%   - baselinePath (scalar text)
%       MAT-file created before restructuring.
%   - candidatePath (scalar text)
%       MAT-file created after a restructuring phase.
%**************************************************************************
% OUTPUTS
%   - comparison (scalar struct)
%       Per-example equality, runtime ratios, and overall pass state.
%**************************************************************************
% UNITS
%   - Runtime values are seconds. Physical units remain in the saved fields.
%**************************************************************************

%% Section 1: Load And Check The Captures

% Matching inventories ensure a missing or reordered example cannot appear as
% a passing comparison. The physical records deliberately exclude runtime.

baselineCapture = loadCapture(baselinePath);
candidateCapture = loadCapture(candidatePath);
if ~isequal(baselineCapture.ExampleNames, candidateCapture.ExampleNames)
    error("comparePlannerRefactorBaseline:ExampleInventoryMismatch", ...
        "Baseline and candidate example names must match exactly.");
end

%% Section 2: Compare Physical Results And Runtime

% Code movement must leave every retained planning decision and numeric output
% exactly equal. Runtime is shown for investigation but cannot make physics pass.

exampleNames = baselineCapture.ExampleNames;
exampleCount = numel(exampleNames);
examplePassed = false(exampleCount, 1);
runtimeRatio = NaN(exampleCount, 1);
for exampleIndex = 1:exampleCount
    examplePassed(exampleIndex) = isequaln( ...
        stripPlannerRefactorRuntime( ...
        baselineCapture.PhysicalRecords{exampleIndex}), ...
        stripPlannerRefactorRuntime( ...
        candidateCapture.PhysicalRecords{exampleIndex}));
    baselineRuntime_s = baselineCapture.ElapsedTime_s(exampleIndex);
    candidateRuntime_s = candidateCapture.ElapsedTime_s(exampleIndex);
    if baselineRuntime_s > 0
        runtimeRatio(exampleIndex) = candidateRuntime_s / baselineRuntime_s;
    end
    fprintf("REFACTOR_COMPARE name=%s physical_equal=%d runtime_ratio=%.6f\n", ...
        exampleNames(exampleIndex), examplePassed(exampleIndex), ...
        runtimeRatio(exampleIndex));
end
focusedPassed = isequaln( ...
    stripPlannerRefactorRuntime(baselineCapture.FocusedRecords), ...
    stripPlannerRefactorRuntime(candidateCapture.FocusedRecords));
fprintf("REFACTOR_COMPARE focused_equal=%d\n", focusedPassed);

%% Section 3: Return One Comparison Record

% A phase passes only when every physical record is unchanged. The caller can
% inspect the saved complete results when one record differs.

comparison = struct( ...
    "Passed", all(examplePassed) && focusedPassed, ...
    "ExampleNames", exampleNames, ...
    "ExamplePassed", examplePassed, ...
    "FocusedPassed", focusedPassed, ...
    "RuntimeRatio", runtimeRatio, ...
    "BaselineCommit", baselineCapture.StartingCommit, ...
    "CandidateCommit", candidateCapture.StartingCommit);
fprintf("REFACTOR_COMPARE_SUMMARY passed=%d equal=%d total=%d\n", ...
    comparison.Passed, nnz(examplePassed), exampleCount);
end

%% Section 4: Local Functions

function capture = loadCapture(capturePath)
% Load and validate the small set of fields needed for comparison.
capturePath = string(capturePath);
if ~isscalar(capturePath) || ismissing(capturePath) || ...
        strlength(capturePath) == 0 || ~isfile(capturePath)
    error("comparePlannerRefactorBaseline:MissingCapture", ...
        "Capture path must name an existing MAT-file.");
end
loaded = load(capturePath, "baseline");
if ~isfield(loaded, "baseline") || ~isstruct(loaded.baseline) || ...
        ~isscalar(loaded.baseline)
    error("comparePlannerRefactorBaseline:InvalidCapture", ...
        "Capture must contain one scalar baseline structure.");
end
capture = loaded.baseline;
requiredFields = ["StartingCommit", "ExampleNames", ...
    "PhysicalRecords", "FocusedRecords", "ElapsedTime_s"];
if ~all(isfield(capture, cellstr(requiredFields)))
    error("comparePlannerRefactorBaseline:InvalidCapture", ...
        "Capture is missing required comparison fields.");
end
end
