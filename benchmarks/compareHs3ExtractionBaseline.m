function comparison = compareHs3ExtractionBaseline( ...
        baselinePath, candidateReports)
%% Section 0: Header & Readme
% SYNTAX
%   comparison = compareHs3ExtractionBaseline()
%   comparison = compareHs3ExtractionBaseline(baselinePath)
%   comparison = compareHs3ExtractionBaseline( ...
%       baselinePath, candidateReports)
%**************************************************************************
% PURPOSE
%   - Compare HS3 extraction behavior and serial runtime against the frozen
%     commit-4827e47 scaling baseline.
%**************************************************************************
% INPUTS
%   - baselinePath (string scalar, optional)
%       MAT file containing baselineReports; defaults beside this function.
%   - candidateReports (struct array or "self", optional)
%       Three benchmarkStandaloneHs3Scaling reports. Omission runs three
%       current-worktree reports; "self" verifies the frozen harness.
%**************************************************************************
% OUTPUTS
%   - comparison (scalar struct)
%       Pass state, first mismatch, all mismatches, per-case runtime gates,
%       baseline and candidate medians, and the compared reports.
%**************************************************************************
% UNITS
%   - Positions and path lengths use degrees; time uses seconds; derivatives
%     use degrees per second powers; integrated jerk uses deg^2/s^5.
%**************************************************************************

%% Section 1: Load The Frozen Baseline And Candidate

% Load the saved reference before running current code. Check version and case
% labels first. A mismatch here means the two data sets are not comparable.

if nargin < 1 || isempty(baselinePath)
    baselinePath = fullfile(fileparts(mfilename("fullpath")), ...
        "hs3_extraction_baseline.mat");
end
baselinePath = string(baselinePath);
if ~isscalar(baselinePath) || strlength(baselinePath) == 0
    error("compareHs3ExtractionBaseline:InvalidBaselinePath", ...
        "baselinePath must be a nonempty string scalar.");
end
loaded = load(baselinePath, "baselineReports", "baselineCommit");
if ~isfield(loaded, "baselineReports") || isempty(loaded.baselineReports)
    error("compareHs3ExtractionBaseline:MissingReports", ...
        "The baseline file must contain nonempty baselineReports.");
end
baselineReports = loaded.baselineReports(:);
if nargin < 2 || isempty(candidateReports)
    candidateReports = runCandidateReports();
elseif (isstring(candidateReports) || ischar(candidateReports)) && ...
        isscalar(string(candidateReports)) && string(candidateReports) == "self"
    candidateReports = baselineReports;
end
if ~isstruct(candidateReports) || isempty(candidateReports)
    error("compareHs3ExtractionBaseline:InvalidCandidateReports", ...
        "candidateReports must be a nonempty struct array or the text 'self'.");
end
candidateReports = candidateReports(:);
referenceReport = baselineReports(1);
caseCount = numel(referenceReport.Cases);
hasCaseCountMismatch = false;
for reportIndex = 1:numel(candidateReports)
    hasCaseCountMismatch = hasCaseCountMismatch || ...
        numel(candidateReports(reportIndex).Cases) ~= caseCount;
end
if hasCaseCountMismatch
    error("compareHs3ExtractionBaseline:CaseCountMismatch", ...
        "Every candidate report must contain %d cases.", caseCount);
end

%% Section 2: Compare Discrete And Numerical Behavior

% Compare exact text and logical values without tolerance. Compare floating-
% point values with named tolerances. A mismatch path identifies the result
% field or polynomial value that changed.

allMismatches = strings(0, 1);
runCasePassed = true(numel(candidateReports), caseCount);
for runIndex = 1:numel(candidateReports)
    for caseIndex = 1:caseCount
        referenceCase = referenceReport.Cases(caseIndex);
        candidateCase = candidateReports(runIndex).Cases(caseIndex);
        caseLabel = sprintf("run %d, %s scale %g", runIndex, ...
            referenceCase.ScenarioFamily, referenceCase.ScenarioScale);
        caseMismatches = compareCase( ...
            referenceCase, candidateCase, caseLabel);
        runCasePassed(runIndex, caseIndex) = isempty(caseMismatches);
        % The trace is bounded by the declared case and field gate counts.
        allMismatches = [allMismatches; caseMismatches]; %#ok<AGROW>
    end
end
kernelMismatches = compareExtractedKernel(referenceReport);
allMismatches = [allMismatches; kernelMismatches];

%% Section 3: Compare Serial Runtime Medians

% Use serial repeats and medians to reduce timing noise. Review behavior
% mismatches before runtime. Faster code is not acceptable if results changed.

baselineRuntime_s = runtimeMatrix(baselineReports, caseCount);
candidateRuntime_s = runtimeMatrix(candidateReports, caseCount);
baselineMedian_s = median(baselineRuntime_s, 1);
candidateMedian_s = median(candidateRuntime_s, 1);
caseAllowance_s = max(0.10 * baselineMedian_s, 0.25);
caseRuntimePassed = candidateMedian_s <= ...
    baselineMedian_s + caseAllowance_s;
baselineTotalMedian_s = median(sum(baselineRuntime_s, 2));
candidateTotalMedian_s = median(sum(candidateRuntime_s, 2));
totalRuntimePassed = candidateTotalMedian_s <= ...
    1.05 * baselineTotalMedian_s;
for caseIndex = find(~caseRuntimePassed)
    referenceCase = referenceReport.Cases(caseIndex);
    % Runtime failures are bounded by the fixed benchmark case count.
    allMismatches(end + 1, 1) = sprintf( ...
        "runtime %s scale %g: baseline median %.9g s, candidate %.9g s, " + ...
        "allowance %.9g s", referenceCase.ScenarioFamily, ...
        referenceCase.ScenarioScale, baselineMedian_s(caseIndex), ...
        candidateMedian_s(caseIndex), caseAllowance_s(caseIndex)); %#ok<AGROW>
end
if ~totalRuntimePassed
    allMismatches(end + 1, 1) = sprintf( ...
        "total runtime: baseline median %.9g s, candidate %.9g s, " + ...
        "maximum %.9g s", baselineTotalMedian_s, ...
        candidateTotalMedian_s, 1.05 * baselineTotalMedian_s);
end

%% Section 4: Assemble The Comparison Record

% Keep baseline and candidate summaries beside all mismatch details. This lets
% a developer separate expected timing variation from a functional change.

if isempty(allMismatches)
    firstMismatch = "";
else
    firstMismatch = allMismatches(1);
end
comparison = struct( ...
    "Passed", isempty(allMismatches), ...
    "BaselineCommit", string(loaded.baselineCommit), ...
    "BaselinePath", baselinePath, ...
    "FirstMismatch", firstMismatch, ...
    "Mismatches", allMismatches, ...
    "RunCasePassed", runCasePassed, ...
    "CaseRuntimePassed", caseRuntimePassed, ...
    "TotalRuntimePassed", totalRuntimePassed, ...
    "BaselineMedianRuntime_s", baselineMedian_s, ...
    "CandidateMedianRuntime_s", candidateMedian_s, ...
    "BaselineTotalMedianRuntime_s", baselineTotalMedian_s, ...
    "CandidateTotalMedianRuntime_s", candidateTotalMedian_s, ...
    "CandidateReports", candidateReports);
fprintf("HS3 extraction parity: passed=%s, mismatches=%d.\n", ...
    logicalText(comparison.Passed), numel(allMismatches));
if ~comparison.Passed
    fprintf("First mismatch: %s\n", comparison.FirstMismatch);
end
end

%% Section 5: Local Functions

function reports = runCandidateReports()
% Run the frozen scaling matrix three times with deterministic work.
controls = struct( ...
    "PrintProgress", true, ...
    "PlannerOverrides", struct());
firstReport = benchmarkStandaloneHs3Scaling([1 5 10 20], controls);
reports = repmat(firstReport, 3, 1);
for runIndex = 2:3
    fprintf("HS3 extraction candidate run %d of 3.\n", runIndex);
    reports(runIndex) = benchmarkStandaloneHs3Scaling( ...
        [1 5 10 20], controls);
end
end

function mismatches = compareExtractedKernel(referenceReport)
% Reconstruct frozen production polynomials through the neutral kernel.
mismatches = strings(0, 1);
for caseIndex = 1:numel(referenceReport.Cases)
    result = referenceReport.Cases(caseIndex).PlannerResult;
    if ~result.Success
        continue;
    end
    polynomial = result.Polynomial;
    segmentCount = polynomial.SegmentCount;
    dimensionCount = size(polynomial.jerkPower_deg_s3, 2);
    controlJerk = controlsFromPolynomial(polynomial.jerkPower_deg_s3);
    initialState = struct( ...
        "time", result.Inputs.initialState.time_s, ...
        "position", result.Inputs.initialState.position_deg, ...
        "velocity", result.Inputs.initialState.velocity_deg_s, ...
        "acceleration", result.Inputs.initialState.acceleration_deg_s2);
    extracted = hs3Internal.polynomial.createTrajectoryPolynomial( ...
        controlJerk, initialState, polynomial.FinalTime_s, segmentCount);
    caseLabel = sprintf("kernel %s scale %g", ...
        referenceReport.Cases(caseIndex).ScenarioFamily, ...
        referenceReport.Cases(caseIndex).ScenarioScale);
    fieldPairs = [ ...
        "SegmentStartTime", "SegmentStartTime_s"; ...
        "SegmentDuration", "SegmentDuration_s"; ...
        "FinalTime", "FinalTime_s"; ...
        "positionPower", "positionPower_deg"; ...
        "velocityPower", "velocityPower_deg_s"; ...
        "accelerationPower", "accelerationPower_deg_s2"; ...
        "jerkPower", "jerkPower_deg_s3"];
    for fieldIndex = 1:size(fieldPairs, 1)
        extractedName = fieldPairs(fieldIndex, 1);
        referenceName = fieldPairs(fieldIndex, 2);
        mismatches = appendNumericMismatch(mismatches, ...
            polynomial.(referenceName), extracted.(extractedName), ...
            1e-12, 1e-10, caseLabel + "." + extractedName);
    end
    [~, position, velocity, acceleration, jerk] = ...
        hs3Internal.polynomial.evaluateTrajectoryPolynomial(extracted, result.time_s);
    histories = {position, velocity, acceleration, jerk};
    historyNames = [ ...
        "position_deg", "velocity_deg_s", ...
        "acceleration_deg_s2", "jerk_deg_s3"];
    for historyIndex = 1:numel(historyNames)
        mismatches = appendNumericMismatch(mismatches, ...
            result.(historyNames(historyIndex)), histories{historyIndex}, ...
            1e-9, 0, caseLabel + "." + historyNames(historyIndex));
    end
    isFreeTime = result.Options.GoalTimeMode == "earliestArrival";
    if isFreeTime
        objectiveDecision = [controlJerk(:); polynomial.FinalTime_s];
    else
        objectiveDecision = controlJerk(:);
    end
    extractedObjective = hs3Internal.polynomial.evaluateIntegratedSquaredJerk( ...
        objectiveDecision, isFreeTime, polynomial.FinalTime_s, ...
        segmentCount, initialState.time, dimensionCount);
    [selectedSummary, ~] = selectedSummaries(result, result);
    mismatches = appendNumericMismatch(mismatches, ...
        selectedSummary.IntegratedSquaredJerk_deg2_s5, ...
        extractedObjective, 1e-8, 0, caseLabel + ".IntegratedSquaredJerk");
end
end

function controlJerk = controlsFromPolynomial(jerkPower)
% Recover shared start, midpoint, and end ordinates from power records.
segmentCount = size(jerkPower, 1);
dimensionCount = size(jerkPower, 2);
controlJerk = zeros(2 * segmentCount + 1, dimensionCount);
for segmentIndex = 1:segmentCount
    constant = jerkPower(segmentIndex, :, 1);
    linear = jerkPower(segmentIndex, :, 2);
    quadratic = jerkPower(segmentIndex, :, 3);
    controlJerk(2 * segmentIndex - 1, :) = constant;
    controlJerk(2 * segmentIndex, :) = ...
        constant + 0.5 * linear + 0.25 * quadratic;
    controlJerk(2 * segmentIndex + 1, :) = ...
        constant + linear + quadratic;
end
end

function mismatches = compareCase(referenceCase, candidateCase, caseLabel)
% Apply every discrete and numerical extraction parity gate to one case.
mismatches = strings(0, 1);
exactPaths = [ ...
    "Success", "IndependentValidationAttempted", ...
    "IndependentValidationPassed", "TerminationReason", ...
    "RouteVertexCount", ...
    "PlannerResult.SelectedSeedIndex", ...
    "PlannerResult.Options.GoalTimeMode", ...
    "PlannerResult.Polynomial.SegmentCount", ...
    "SolverDiagnostics.StageOneExitFlag", ...
    "SolverDiagnostics.StageTwoExitFlag"];
for fieldPath = exactPaths
    mismatches = appendExactMismatch(mismatches, referenceCase, ...
        candidateCase, fieldPath, caseLabel);
end

mismatches = appendNumericMismatch(mismatches, referenceCase.Route_deg, ...
    candidateCase.Route_deg, 1e-10, 1e-10, caseLabel + ".Route_deg");
referenceResult = referenceCase.PlannerResult;
candidateResult = candidateCase.PlannerResult;
mismatches = appendNumericMismatch(mismatches, ...
    referenceResult.SelectedSeed_deg, candidateResult.SelectedSeed_deg, ...
    1e-10, 1e-10, caseLabel + ".PlannerResult.SelectedSeed_deg");
mismatches = appendNumericMismatch(mismatches, ...
    referenceResult.GoalHorizon_s, candidateResult.GoalHorizon_s, ...
    0, 0, caseLabel + ".PlannerResult.GoalHorizon_s");
durationAbsoluteTolerance_s = 0;
durationRelativeTolerance = 0;
if referenceResult.Options.GoalTimeMode == "earliestArrival"
    durationAbsoluteTolerance_s = 1e-8;
    durationRelativeTolerance = 1e-8;
end
for fieldName = ["ArrivalTime_s", "TrajectoryDuration_s"]
    mismatches = appendNumericMismatch(mismatches, ...
        referenceResult.(fieldName), candidateResult.(fieldName), ...
        durationAbsoluteTolerance_s, durationRelativeTolerance, ...
        caseLabel + ".PlannerResult." + fieldName);
end
mismatches = appendNumericMismatch(mismatches, ...
    selectedRouteLength(referenceResult), ...
    selectedRouteLength(candidateResult), 1e-10, 1e-10, ...
    caseLabel + ".SelectedRouteLength_deg");
mismatches = appendNumericMismatch(mismatches, ...
    trajectoryLength(referenceResult), trajectoryLength(candidateResult), ...
    1e-8, 1e-8, caseLabel + ".TrajectoryPathLength_deg");
historyFields = [ ...
    "position_deg", "velocity_deg_s", "acceleration_deg_s2", "jerk_deg_s3"];
for fieldName = historyFields
    mismatches = appendNumericMismatch(mismatches, ...
        referenceResult.(fieldName), candidateResult.(fieldName), ...
        1e-9, 0, caseLabel + ".PlannerResult." + fieldName);
end
mismatches = appendNumericMismatch(mismatches, referenceResult.time_s, ...
    candidateResult.time_s, 1e-10, 1e-10, ...
    caseLabel + ".PlannerResult.time_s");
mismatches = appendNumericTreeMismatches(mismatches, ...
    referenceResult.Polynomial, candidateResult.Polynomial, ...
    caseLabel + ".PlannerResult.Polynomial", 1e-12, 1e-10);
[referenceSummary, candidateSummary] = selectedSummaries( ...
    referenceResult, candidateResult);
if ~isempty(referenceSummary) && ~isempty(candidateSummary)
    if ~isequal(referenceSummary.SeedSource, candidateSummary.SeedSource)
        mismatches(end + 1, 1) = caseLabel + ...
            ".SeedSummary.SeedSource differs";
    end
    for fieldName = [ ...
            "RelinearizationCount", "MeshRefinementPassCount"]
        mismatches = appendNumericMismatch(mismatches, ...
            referenceSummary.(fieldName), candidateSummary.(fieldName), ...
            0, 0, caseLabel + ".SeedSummary." + fieldName);
    end
    mismatches = appendNumericMismatch(mismatches, ...
        referenceSummary.IntegratedSquaredJerk_deg2_s5, ...
        candidateSummary.IntegratedSquaredJerk_deg2_s5, ...
        0, 1e-8, caseLabel + ".IntegratedSquaredJerk_deg2_s5");
    mismatches = appendNumericMismatch(mismatches, ...
        referenceSummary.MinimumClearance_deg, ...
        candidateSummary.MinimumClearance_deg, 1e-8, 0, ...
        caseLabel + ".MinimumClearance_deg");
    maximumAllowedViolation = ...
        referenceSummary.MaximumConstraintViolation + 1e-10;
    if candidateSummary.MaximumConstraintViolation > maximumAllowedViolation
        mismatches(end + 1, 1) = sprintf( ...
            "%s.MaximumConstraintViolation increased from %.16g to %.16g", ...
            caseLabel, referenceSummary.MaximumConstraintViolation, ...
            candidateSummary.MaximumConstraintViolation);
    end
end
mismatches = appendExactMismatch(mismatches, referenceCase, candidateCase, ...
    "SolverDiagnostics.Identifier", caseLabel);
mismatches = appendExactMismatch(mismatches, referenceCase, candidateCase, ...
    "SolverDiagnostics.ConstraintRepresentation", caseLabel);
for outputName = ["StageOneOutput", "StageTwoOutput"]
    referenceOutput = referenceCase.SolverDiagnostics.(outputName);
    candidateOutput = candidateCase.SolverDiagnostics.(outputName);
    if isfield(referenceOutput, "iterations") || ...
            isfield(candidateOutput, "iterations")
        mismatches = appendNumericMismatch(mismatches, ...
            fieldOrNaN(referenceOutput, "iterations"), ...
            fieldOrNaN(candidateOutput, "iterations"), 0, 0, ...
            caseLabel + ".SolverDiagnostics." + outputName + ".iterations");
    end
    if isfield(referenceOutput, "algorithm") || ...
            isfield(candidateOutput, "algorithm")
        referenceAlgorithm = fieldOrText(referenceOutput, "algorithm");
        candidateAlgorithm = fieldOrText(candidateOutput, "algorithm");
        if ~isequal(referenceAlgorithm, candidateAlgorithm)
            % Solver fields are bounded by the two recorded solver stages.
            mismatches(end + 1, 1) = caseLabel + ...
                ".SolverDiagnostics." + outputName + ...
                ".algorithm differs"; %#ok<AGROW>
        end
    end
end
end

function mismatches = appendExactMismatch( ...
        mismatches, reference, candidate, fieldPath, caseLabel)
% Append an exact discrete-field mismatch without hiding missing fields.
[referenceValue, referenceFound] = nestedField(reference, fieldPath);
[candidateValue, candidateFound] = nestedField(candidate, fieldPath);
if ~referenceFound || ~candidateFound
    mismatches(end + 1, 1) = sprintf( ...
        "%s.%s missing (reference=%d, candidate=%d)", ...
        caseLabel, fieldPath, referenceFound, candidateFound);
elseif ~isequaln(referenceValue, candidateValue)
    mismatches(end + 1, 1) = sprintf( ...
        "%s.%s differs", caseLabel, fieldPath);
end
end

function mismatches = appendNumericMismatch( ...
        mismatches, reference, candidate, absoluteTolerance, ...
        relativeTolerance, fieldLabel)
% Append a shape, finiteness, or numerical-tolerance mismatch.
if ~isnumeric(reference) || ~isnumeric(candidate) || ...
        ~isequal(size(reference), size(candidate))
    mismatches(end + 1, 1) = fieldLabel + " has a type or shape mismatch";
    return;
end
if isempty(reference)
    return;
end
sameNonfinite = isequal(isnan(reference), isnan(candidate)) && ...
    isequal(isinf(reference), isinf(candidate)) && ...
    isequal(sign(reference(isinf(reference))), sign(candidate(isinf(candidate))));
if ~sameNonfinite
    mismatches(end + 1, 1) = fieldLabel + " has a nonfinite-value mismatch";
    return;
end
finiteMask = isfinite(reference) & isfinite(candidate);
if ~any(finiteMask, "all")
    return;
end
difference = abs(reference(finiteMask) - candidate(finiteMask));
referenceScale = max(abs(reference(finiteMask)), [], "all");
allowed = max(absoluteTolerance, relativeTolerance * referenceScale);
maximumDifference = max(difference, [], "all");
if maximumDifference > allowed
    mismatches(end + 1, 1) = sprintf( ...
        "%s differs by %.16g; allowed %.16g", ...
        fieldLabel, maximumDifference, allowed);
end
end

function mismatches = appendNumericTreeMismatches( ...
        mismatches, reference, candidate, fieldLabel, ...
        absoluteTolerance, relativeTolerance)
% Recursively compare every numeric polynomial field and discrete leaf.
if isstruct(reference) && isstruct(candidate) && ...
        isscalar(reference) && isscalar(candidate)
    referenceNames = string(fieldnames(reference));
    candidateNames = string(fieldnames(candidate));
    if ~isequal(referenceNames, candidateNames)
        mismatches(end + 1, 1) = fieldLabel + " field order differs";
        return;
    end
    for fieldName = referenceNames.'
        mismatches = appendNumericTreeMismatches(mismatches, ...
            reference.(fieldName), candidate.(fieldName), ...
            fieldLabel + "." + fieldName, absoluteTolerance, ...
            relativeTolerance);
    end
elseif isnumeric(reference) || isnumeric(candidate)
    mismatches = appendNumericMismatch(mismatches, reference, candidate, ...
        absoluteTolerance, relativeTolerance, fieldLabel);
elseif ~isequaln(reference, candidate)
    mismatches(end + 1, 1) = fieldLabel + " differs";
end
end

function [value, found] = nestedField(record, fieldPath)
% Read one dot-separated scalar-structure field path safely.
value = record;
found = true;
for fieldName = split(string(fieldPath), ".").'
    if ~isstruct(value) || ~isscalar(value) || ~isfield(value, fieldName)
        value = [];
        found = false;
        return;
    end
    value = value.(fieldName);
end
end

function length_deg = selectedRouteLength(result)
% Read the selected seed's stored geometric length.
length_deg = NaN;
if result.SelectedSeedIndex >= 1 && ...
        result.SelectedSeedIndex <= numel(result.Seeds)
    length_deg = result.Seeds(result.SelectedSeedIndex).Length_deg;
end
end

function length_deg = trajectoryLength(result)
% Compute the sampled trajectory path length without planner shortcuts.
length_deg = NaN;
if result.Success
    length_deg = sum(vecnorm(diff(result.position_deg, 1, 1), 2, 2));
end
end

function [referenceSummary, candidateSummary] = ...
        selectedSummaries(referenceResult, candidateResult)
% Return matching selected-seed summaries when both results have them.
referenceSummary = [];
candidateSummary = [];
referenceIndex = referenceResult.SelectedSeedIndex;
candidateIndex = candidateResult.SelectedSeedIndex;
if referenceIndex >= 1 && candidateIndex >= 1 && ...
        referenceIndex <= numel(referenceResult.SeedSummaries) && ...
        candidateIndex <= numel(candidateResult.SeedSummaries)
    referenceSummary = referenceResult.SeedSummaries(referenceIndex);
    candidateSummary = candidateResult.SeedSummaries(candidateIndex);
end
end

function value = fieldOrNaN(record, fieldName)
% Read an optional scalar solver-output field for exact comparison.
value = NaN;
if isstruct(record) && isscalar(record) && isfield(record, fieldName)
    value = record.(fieldName);
end
end

function value = fieldOrText(record, fieldName)
% Read an optional solver-output text field for exact comparison.
value = "";
if isstruct(record) && isscalar(record) && isfield(record, fieldName)
    value = string(record.(fieldName));
end
end

function values_s = runtimeMatrix(reports, caseCount)
% Collect total serial case runtimes without changing report ordering.
values_s = zeros(numel(reports), caseCount);
for runIndex = 1:numel(reports)
    values_s(runIndex, :) = [reports(runIndex).Cases.TotalWallTime_s];
end
end

function text = logicalText(value)
% Render scalar logical comparison status as true or false.
if logical(value)
    text = "true";
else
    text = "false";
end
end
