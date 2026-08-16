function comparison = compareAzElBenchmarkReports( ...
        baselineReport, currentReport, comparisonMode, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = compareAzElBenchmarkReports()
%   comparison = compareAzElBenchmarkReports( ...
%       baselineReport, currentReport)
%   comparison = compareAzElBenchmarkReports( ...
%       baselineReport, currentReport, comparisonMode)
%   comparison = compareAzElBenchmarkReports( ...
%       baselineReport, currentReport, comparisonMode, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Compare reproducible Az/El benchmark reports without rerunning or
%     changing planner decisions.
%   - Apply explicit exact-parity or lower-is-better dominance criteria.
%**************************************************************************
% INPUTS
%   - baselineReport, currentReport (scalar structs)
%       Reports returned by benchmarkAzElExamples with a Runs table. Rows
%       are paired by Example and JerkConstrained, independent of order.
%   - comparisonMode (scalar text, optional; default "parity")
%       "parity" requires metric equality within tolerance. "dominance"
%       requires each reported physical metric to be no greater than its
%       paired baseline value within tolerance.
%   - optionOverrides (scalar struct, optional; default struct())
%       MetricAbsoluteTolerance (1e-10), MetricRelativeTolerance (0),
%       AssessRuntime (false), MaximumRuntimeRatio (1.25), and
%       RuntimeAbsoluteTolerance_s (0).
%**************************************************************************
% OUTPUTS
%   - comparison (scalar struct)
%       Stable row-level and aggregate comparison report. Input reports
%       are read only; invalid report schemas throw.
%**************************************************************************
% UNITS
%   - Path lengths use degrees, durations and runtimes use seconds, and
%     ratios and logical outcomes are dimensionless.
%**************************************************************************

%% Section 1: Validate Reports & Resolve Controls

defaults = comparatorDefaults();
if nargin == 0
    comparison = defaults;
    return;
end
if nargin < 3 || isempty(comparisonMode)
    comparisonMode = "parity";
end
if nargin < 4 || isempty(optionOverrides)
    optionOverrides = struct();
end

options = resolveComparisonOptions(defaults, optionOverrides);
comparisonMode = lower(string(comparisonMode));
if ~isscalar(comparisonMode) || ...
        ~any(comparisonMode == ["parity" "dominance"])
    error("compareAzElBenchmarkReports:InvalidMode", ...
        "comparisonMode must be parity or dominance.");
end
if ~isstruct(baselineReport) || ~isscalar(baselineReport) || ...
        ~isstruct(currentReport) || ~isscalar(currentReport) || ...
        ~isfield(baselineReport, "Runs") || ...
        ~isfield(currentReport, "Runs")
    error("compareAzElBenchmarkReports:InvalidReport", ...
        "Each report must be a scalar struct containing a Runs table.");
end

baselineRuns = normalizeRunsTable( ...
    baselineReport.Runs, "baselineReport.Runs", options.AssessRuntime);
currentRuns = normalizeRunsTable( ...
    currentReport.Runs, "currentReport.Runs", options.AssessRuntime);
[baselineKeys, baselineExamples, baselineJerk] = rowKeys(baselineRuns);
[currentKeys, currentExamples, currentJerk] = rowKeys(currentRuns);
allKeys = unique([baselineKeys; currentKeys]);

%% Section 2: Compare Paired Benchmark Rows

rowCount = numel(allKeys);
examples = strings(rowCount, 1);
jerkConstrained = false(rowCount, 1);
hasBaseline = false(rowCount, 1);
hasCurrent = false(rowCount, 1);
runCompletedSame = false(rowCount, 1);
plannerSuccessSame = false(rowCount, 1);
validationPassedSame = false(rowCount, 1);
collisionFreeSame = false(rowCount, 1);
kinematicsPassedSame = false(rowCount, 1);
certificatePassedSame = false(rowCount, 1);
finiteJerkCertifiedSame = false(rowCount, 1);
terminationReasonSame = false(rowCount, 1);
outcomePassed = false(rowCount, 1);

baselinePolyline_deg = nan(rowCount, 1);
currentPolyline_deg = nan(rowCount, 1);
polylineDelta_deg = nan(rowCount, 1);
polylineTolerance_deg = nan(rowCount, 1);
polylinePassed = false(rowCount, 1);
baselineSmooth_deg = nan(rowCount, 1);
currentSmooth_deg = nan(rowCount, 1);
smoothDelta_deg = nan(rowCount, 1);
smoothTolerance_deg = nan(rowCount, 1);
smoothPassed = false(rowCount, 1);
baselineDuration_s = nan(rowCount, 1);
currentDuration_s = nan(rowCount, 1);
durationDelta_s = nan(rowCount, 1);
durationTolerance_s = nan(rowCount, 1);
durationPassed = false(rowCount, 1);
metricPassed = false(rowCount, 1);

baselineRuntime_s = nan(rowCount, 1);
currentRuntime_s = nan(rowCount, 1);
runtimeRatio = nan(rowCount, 1);
runtimePassed = true(rowCount, 1);

for rowIndex = 1:rowCount
    key = allKeys(rowIndex);
    baselineIndex = find(baselineKeys == key, 1, "first");
    currentIndex = find(currentKeys == key, 1, "first");
    hasBaseline(rowIndex) = ~isempty(baselineIndex);
    hasCurrent(rowIndex) = ~isempty(currentIndex);

    if hasBaseline(rowIndex)
        examples(rowIndex) = baselineExamples(baselineIndex);
        jerkConstrained(rowIndex) = baselineJerk(baselineIndex);
    else
        examples(rowIndex) = currentExamples(currentIndex);
        jerkConstrained(rowIndex) = currentJerk(currentIndex);
    end
    if ~(hasBaseline(rowIndex) && hasCurrent(rowIndex))
        continue;
    end

    runCompletedSame(rowIndex) = baselineRuns.RunCompleted(baselineIndex) == ...
        currentRuns.RunCompleted(currentIndex);
    plannerSuccessSame(rowIndex) = ...
        baselineRuns.PlannerSuccess(baselineIndex) == ...
        currentRuns.PlannerSuccess(currentIndex);
    validationPassedSame(rowIndex) = ...
        baselineRuns.ValidationPassed(baselineIndex) == ...
        currentRuns.ValidationPassed(currentIndex);
    collisionFreeSame(rowIndex) = ...
        baselineRuns.CollisionFree(baselineIndex) == ...
        currentRuns.CollisionFree(currentIndex);
    kinematicsPassedSame(rowIndex) = ...
        baselineRuns.KinematicsPassed(baselineIndex) == ...
        currentRuns.KinematicsPassed(currentIndex);
    certificatePassedSame(rowIndex) = ...
        baselineRuns.CertificatePassed(baselineIndex) == ...
        currentRuns.CertificatePassed(currentIndex);
    finiteJerkCertifiedSame(rowIndex) = ...
        baselineRuns.FiniteJerkCertified(baselineIndex) == ...
        currentRuns.FiniteJerkCertified(currentIndex);
    terminationReasonSame(rowIndex) = ...
        baselineRuns.TerminationReason(baselineIndex) == ...
        currentRuns.TerminationReason(currentIndex);
    outcomePassed(rowIndex) = all([ ...
        runCompletedSame(rowIndex), plannerSuccessSame(rowIndex), ...
        validationPassedSame(rowIndex), collisionFreeSame(rowIndex), ...
        kinematicsPassedSame(rowIndex), certificatePassedSame(rowIndex), ...
        finiteJerkCertifiedSame(rowIndex), ...
        terminationReasonSame(rowIndex)]);

    baselinePolyline_deg(rowIndex) = ...
        baselineRuns.SelectedPolylineLength_deg(baselineIndex);
    currentPolyline_deg(rowIndex) = ...
        currentRuns.SelectedPolylineLength_deg(currentIndex);
    [polylinePassed(rowIndex), polylineDelta_deg(rowIndex), ...
        polylineTolerance_deg(rowIndex)] = compareMetric( ...
        baselinePolyline_deg(rowIndex), currentPolyline_deg(rowIndex), ...
        comparisonMode, options);

    baselineSmooth_deg(rowIndex) = ...
        baselineRuns.SmoothedPathLength_deg(baselineIndex);
    currentSmooth_deg(rowIndex) = ...
        currentRuns.SmoothedPathLength_deg(currentIndex);
    [smoothPassed(rowIndex), smoothDelta_deg(rowIndex), ...
        smoothTolerance_deg(rowIndex)] = compareMetric( ...
        baselineSmooth_deg(rowIndex), currentSmooth_deg(rowIndex), ...
        comparisonMode, options);

    baselineDuration_s(rowIndex) = ...
        baselineRuns.MinimumMotionDuration_s(baselineIndex);
    currentDuration_s(rowIndex) = ...
        currentRuns.MinimumMotionDuration_s(currentIndex);
    [durationPassed(rowIndex), durationDelta_s(rowIndex), ...
        durationTolerance_s(rowIndex)] = compareMetric( ...
        baselineDuration_s(rowIndex), currentDuration_s(rowIndex), ...
        comparisonMode, options);
    metricPassed(rowIndex) = polylinePassed(rowIndex) && ...
        smoothPassed(rowIndex) && durationPassed(rowIndex);

    if options.AssessRuntime
        baselineRuntime_s(rowIndex) = baselineRuns.WallTime_s(baselineIndex);
        currentRuntime_s(rowIndex) = currentRuns.WallTime_s(currentIndex);
        [runtimePassed(rowIndex), runtimeRatio(rowIndex)] = ...
            compareRuntime(baselineRuntime_s(rowIndex), ...
            currentRuntime_s(rowIndex), options);
    end
end

%% Section 3: Assemble Stable Comparison Output

keySetMatched = all(hasBaseline & hasCurrent);
outcomesPassed = keySetMatched && all(outcomePassed);
metricsPassed = keySetMatched && all(metricPassed);
runtimesPassed = ~options.AssessRuntime || ...
    (keySetMatched && all(runtimePassed));
passed = keySetMatched && outcomesPassed && metricsPassed && ...
    runtimesPassed;

issues = strings(0, 1);
if ~keySetMatched
    issues(end + 1, 1) = "rowKeys";
end
if ~outcomesPassed
    issues(end + 1, 1) = "outcomes";
end
if ~metricsPassed
    issues(end + 1, 1) = "physicalMetrics";
end
if ~runtimesPassed
    issues(end + 1, 1) = "runtime";
end
message = comparisonMode + " benchmark comparison passed for " + ...
    rowCount + " row keys.";
if ~passed
    message = comparisonMode + " benchmark comparison failed: " + ...
        strjoin(issues, ", ") + ".";
end

rows = table(examples, jerkConstrained, hasBaseline, hasCurrent, ...
    runCompletedSame, plannerSuccessSame, validationPassedSame, ...
    collisionFreeSame, kinematicsPassedSame, certificatePassedSame, ...
    finiteJerkCertifiedSame, terminationReasonSame, outcomePassed, ...
    baselinePolyline_deg, currentPolyline_deg, polylineDelta_deg, ...
    polylineTolerance_deg, polylinePassed, baselineSmooth_deg, ...
    currentSmooth_deg, smoothDelta_deg, smoothTolerance_deg, smoothPassed, ...
    baselineDuration_s, currentDuration_s, durationDelta_s, ...
    durationTolerance_s, durationPassed, metricPassed, baselineRuntime_s, ...
    currentRuntime_s, runtimeRatio, runtimePassed, ...
    'VariableNames', { ...
    'Example', 'JerkConstrained', 'HasBaseline', 'HasCurrent', ...
    'RunCompletedSame', 'PlannerSuccessSame', 'ValidationPassedSame', ...
    'CollisionFreeSame', 'KinematicsPassedSame', 'CertificatePassedSame', ...
    'FiniteJerkCertifiedSame', 'TerminationReasonSame', 'OutcomePassed', ...
    'BaselinePolyline_deg', 'CurrentPolyline_deg', 'PolylineDelta_deg', ...
    'PolylineTolerance_deg', 'PolylinePassed', 'BaselineSmooth_deg', ...
    'CurrentSmooth_deg', 'SmoothDelta_deg', 'SmoothTolerance_deg', ...
    'SmoothPassed', 'BaselineDuration_s', 'CurrentDuration_s', ...
    'DurationDelta_s', 'DurationTolerance_s', 'DurationPassed', ...
    'MetricPassed', 'BaselineRuntime_s', 'CurrentRuntime_s', ...
    'RuntimeRatio', 'RuntimePassed'});

comparison = struct( ...
    "Passed", passed, ...
    "Message", message, ...
    "Mode", comparisonMode, ...
    "Options", options, ...
    "BaselineRowCount", height(baselineRuns), ...
    "CurrentRowCount", height(currentRuns), ...
    "ComparedRowCount", rowCount, ...
    "KeySetMatched", keySetMatched, ...
    "OutcomesPassed", outcomesPassed, ...
    "MetricsPassed", metricsPassed, ...
    "RuntimeAssessed", options.AssessRuntime, ...
    "RuntimesPassed", runtimesPassed, ...
    "OutcomeMismatchCount", nnz(~outcomePassed), ...
    "MetricMismatchCount", nnz(~metricPassed), ...
    "RuntimeMismatchCount", nnz(~runtimePassed), ...
    "BaselineKeys", baselineKeys, ...
    "CurrentKeys", currentKeys, ...
    "Rows", rows);
end

%% Section 4: Local Functions

function options = comparatorDefaults()
%% Section 0: Header & Readme
% SYNTAX
%   options = comparatorDefaults()
%**************************************************************************
% PURPOSE
%   - Return the argument-independent benchmark-comparison defaults.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Fully resolved metric and optional runtime comparison controls.
%**************************************************************************
% UNITS
%   - Tolerances use reported metric units; ratios are dimensionless.
%**************************************************************************
options = struct( ...
    "MetricAbsoluteTolerance", 1e-10, ...
    "MetricRelativeTolerance", 0, ...
    "AssessRuntime", false, ...
    "MaximumRuntimeRatio", 1.25, ...
    "RuntimeAbsoluteTolerance_s", 0);
end

function options = resolveComparisonOptions(defaults, overrides)
%% Section 0: Header & Readme
% SYNTAX
%   options = resolveComparisonOptions(defaults, overrides)
%**************************************************************************
% PURPOSE
%   - Merge partial comparison controls and validate their semantics.
%**************************************************************************
% INPUTS
%   - defaults, overrides (scalar structs)
%       Complete defaults and optional caller-supplied controls.
%**************************************************************************
% OUTPUTS
%   - options (scalar struct)
%       Fully resolved, normalized comparison controls.
%**************************************************************************
% UNITS
%   - Tolerances use metric units; runtime ratio is dimensionless.
%**************************************************************************
if ~isstruct(overrides) || ~isscalar(overrides)
    error("compareAzElBenchmarkReports:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
[options, unknownNames] = azElInternal.resolveOptions(defaults, overrides);
if ~isempty(unknownNames)
    warning("compareAzElBenchmarkReports:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
validateattributes(options.MetricAbsoluteTolerance, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
validateattributes(options.MetricRelativeTolerance, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
options.AssessRuntime = azElInternal.normalizeLogicalScalar( ...
    options.AssessRuntime, "AssessRuntime", ...
    "compareAzElBenchmarkReports:InvalidAssessRuntime");
validateattributes(options.MaximumRuntimeRatio, {'numeric'}, ...
    {'real', 'scalar', 'nonnegative'});
if isnan(options.MaximumRuntimeRatio)
    error("compareAzElBenchmarkReports:InvalidRuntimeRatio", ...
        "MaximumRuntimeRatio cannot be NaN.");
end
validateattributes(options.RuntimeAbsoluteTolerance_s, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
end

function runs = normalizeRunsTable(runs, label, assessRuntime)
%% Section 0: Header & Readme
% SYNTAX
%   runs = normalizeRunsTable(runs, label, assessRuntime)
%**************************************************************************
% PURPOSE
%   - Validate and locally normalize the stable benchmark row schema.
%**************************************************************************
% INPUTS
%   - runs (table), label (scalar text)
%       Benchmark rows and the caller-facing name used in diagnostics.
%   - assessRuntime (logical scalar)
%       Whether WallTime_s is required and normalized.
%**************************************************************************
% OUTPUTS
%   - runs (table)
%       Local normalized copy; caller-owned report data is not changed.
%**************************************************************************
% UNITS
%   - Metric columns use degrees or seconds; outcomes are dimensionless.
%**************************************************************************
if ~istable(runs) || isempty(runs)
    error("compareAzElBenchmarkReports:InvalidRuns", ...
        "%s must be a nonempty table.", label);
end
requiredNames = ["Example" "JerkConstrained" "RunCompleted" ...
    "PlannerSuccess" "ValidationPassed" "CollisionFree" ...
    "KinematicsPassed" "CertificatePassed" "FiniteJerkCertified" ...
    "TerminationReason" "SelectedPolylineLength_deg" ...
    "SmoothedPathLength_deg" "MinimumMotionDuration_s"];
if assessRuntime
    requiredNames(end + 1) = "WallTime_s";
end
missingNames = requiredNames(~ismember( ...
    requiredNames, string(runs.Properties.VariableNames)));
if ~isempty(missingNames)
    error("compareAzElBenchmarkReports:MissingRunFields", ...
        "%s is missing: %s.", label, strjoin(missingNames, ", "));
end

runs.Example = string(runs.Example);
runs.TerminationReason = string(runs.TerminationReason);
if any(strlength(runs.Example) == 0) || ...
        any(strlength(runs.TerminationReason) == 0)
    error("compareAzElBenchmarkReports:EmptyRunText", ...
        "%s Example and TerminationReason values must be nonempty.", label);
end
logicalNames = ["JerkConstrained" "RunCompleted" "PlannerSuccess" ...
    "ValidationPassed" "CollisionFree" "KinematicsPassed" ...
    "CertificatePassed" "FiniteJerkCertified"];
for nameIndex = 1:numel(logicalNames)
    name = logicalNames(nameIndex);
    runs.(name) = normalizeLogicalColumn(runs.(name), label, name);
end
metricNames = ["SelectedPolylineLength_deg" "SmoothedPathLength_deg" ...
    "MinimumMotionDuration_s"];
for nameIndex = 1:numel(metricNames)
    name = metricNames(nameIndex);
    runs.(name) = normalizeMetricColumn(runs.(name), label, name);
end
if assessRuntime
    runs.WallTime_s = normalizeMetricColumn( ...
        runs.WallTime_s, label, "WallTime_s");
end
end

function values = normalizeLogicalColumn(values, label, name)
%% Section 0: Header & Readme
% SYNTAX
%   values = normalizeLogicalColumn(values, label, name)
%**************************************************************************
% PURPOSE
%   - Convert one benchmark outcome column to a logical column safely.
%**************************************************************************
% INPUTS
%   - values (logical or numeric vector), label, name (scalar text)
%       Source values and their caller-facing report and field names.
%**************************************************************************
% OUTPUTS
%   - values (logical column vector)
%       Normalized binary outcome values.
%**************************************************************************
% UNITS
%   - Outcomes are dimensionless.
%**************************************************************************
isBinaryNumeric = isnumeric(values) && isreal(values) && ...
    all(isfinite(values), "all") && all(ismember(values, [0 1]), "all");
if ~(islogical(values) || isBinaryNumeric) || ~isvector(values)
    error("compareAzElBenchmarkReports:InvalidOutcomeColumn", ...
        "%s.%s must be a logical or binary numeric vector.", label, name);
end
values = logical(values(:));
end

function values = normalizeMetricColumn(values, label, name)
%% Section 0: Header & Readme
% SYNTAX
%   values = normalizeMetricColumn(values, label, name)
%**************************************************************************
% PURPOSE
%   - Normalize one nonnegative benchmark metric while preserving NaN for
%     an unavailable value.
%**************************************************************************
% INPUTS
%   - values (numeric vector), label, name (scalar text)
%       Source metric values and caller-facing diagnostic names.
%**************************************************************************
% OUTPUTS
%   - values (double column vector)
%       Finite nonnegative values or NaN for unavailable metrics.
%**************************************************************************
% UNITS
%   - Units are defined by the metric field name.
%**************************************************************************
if ~isnumeric(values) || ~isreal(values) || ~isvector(values)
    error("compareAzElBenchmarkReports:InvalidMetricColumn", ...
        "%s.%s must be a real numeric vector.", label, name);
end
values = double(values(:));
if any(isinf(values)) || any(values < 0, "all")
    error("compareAzElBenchmarkReports:InvalidMetricValue", ...
        "%s.%s must contain nonnegative finite values or NaN.", ...
        label, name);
end
end

function [keys, examples, jerkConstrained] = rowKeys(runs)
%% Section 0: Header & Readme
% SYNTAX
%   [keys, examples, jerkConstrained] = rowKeys(runs)
%**************************************************************************
% PURPOSE
%   - Build deterministic, unique benchmark pairing keys from public rows.
%**************************************************************************
% INPUTS
%   - runs (normalized table)
%       Table exposing Example and JerkConstrained columns.
%**************************************************************************
% OUTPUTS
%   - keys, examples (string columns), jerkConstrained (logical column)
%       Pairing keys and their public components.
%**************************************************************************
% UNITS
%   - Keys and names are text; jerk status is dimensionless.
%**************************************************************************
examples = string(runs.Example(:));
jerkConstrained = logical(runs.JerkConstrained(:));
separator = string(char(31));
if any(contains(examples, separator))
    error("compareAzElBenchmarkReports:InvalidExampleKey", ...
        "Example values cannot contain the benchmark key separator.");
end
keys = examples + separator + string(double(jerkConstrained));
if numel(unique(keys)) ~= numel(keys)
    error("compareAzElBenchmarkReports:DuplicateRunKey", ...
        "Each Example and JerkConstrained pair must occur exactly once.");
end
end

function [passed, delta, tolerance] = compareMetric( ...
        baselineValue, currentValue, comparisonMode, options)
%% Section 0: Header & Readme
% SYNTAX
%   [passed, delta, tolerance] = compareMetric( ...
%       baselineValue, currentValue, comparisonMode, options)
%**************************************************************************
% PURPOSE
%   - Apply the requested equality or lower-is-better metric rule.
%**************************************************************************
% INPUTS
%   - baselineValue, currentValue (scalar doubles)
%       Paired physical metrics, or paired NaNs when unavailable.
%   - comparisonMode ("parity" or "dominance")
%   - options (scalar struct)
%       Resolved absolute and relative metric tolerances.
%**************************************************************************
% OUTPUTS
%   - passed (logical), delta, tolerance (double scalars)
%       Criterion result, current-minus-baseline value, and applied bound.
%**************************************************************************
% UNITS
%   - Delta and tolerance share the paired metric's units.
%**************************************************************************
delta = NaN;
tolerance = NaN;
if isnan(baselineValue) && isnan(currentValue)
    passed = true;
    return;
end
if ~(isfinite(baselineValue) && isfinite(currentValue))
    passed = false;
    return;
end
delta = currentValue - baselineValue;
tolerance = options.MetricAbsoluteTolerance + ...
    options.MetricRelativeTolerance * max( ...
    1, max(abs([baselineValue currentValue])));
if comparisonMode == "parity"
    passed = abs(delta) <= tolerance;
else
    passed = currentValue <= baselineValue + tolerance;
end
end

function [passed, ratio] = compareRuntime( ...
        baselineRuntime_s, currentRuntime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [passed, ratio] = compareRuntime( ...
%       baselineRuntime_s, currentRuntime_s, options)
%**************************************************************************
% PURPOSE
%   - Apply the caller-selected runtime ratio and absolute tolerance rule.
%**************************************************************************
% INPUTS
%   - baselineRuntime_s, currentRuntime_s (scalar doubles)
%       Paired runtimes, or paired NaNs when unavailable.
%   - options (scalar struct)
%       Resolved maximum ratio and absolute runtime tolerance.
%**************************************************************************
% OUTPUTS
%   - passed (logical scalar), ratio (double scalar)
%       Runtime acceptance state and current-to-baseline ratio.
%**************************************************************************
% UNITS
%   - Runtimes use seconds; ratio is dimensionless.
%**************************************************************************
ratio = NaN;
if isnan(baselineRuntime_s) && isnan(currentRuntime_s)
    passed = true;
    return;
end
if ~(isfinite(baselineRuntime_s) && isfinite(currentRuntime_s))
    passed = false;
    return;
end
if baselineRuntime_s > 0
    ratio = currentRuntime_s / baselineRuntime_s;
    passed = currentRuntime_s <= ...
        baselineRuntime_s * options.MaximumRuntimeRatio + ...
        options.RuntimeAbsoluteTolerance_s;
else
    passed = currentRuntime_s <= options.RuntimeAbsoluteTolerance_s;
end
end
