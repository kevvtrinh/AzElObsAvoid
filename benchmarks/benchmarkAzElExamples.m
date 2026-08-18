function report = benchmarkAzElExamples(optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   report = benchmarkAzElExamples()
%   report = benchmarkAzElExamples(optionOverrides)
%**************************************************************************
% PURPOSE
%   - Run every maintained example with a reproducible headless setup.
%   - Record behavior, motion, validation, and runtime metrics for
%     before-and-after comparisons.
%   - Store MAT and CSV artifacts when an output directory is requested.
%**************************************************************************
% INPUTS
%   - optionOverrides (scalar struct, optional; default struct())
%       .ExampleNames is a string vector; empty discovers example*.m.
%       .OutputDirectory is scalar text; empty disables artifact writes.
%       .ArtifactPrefix is nonempty scalar text (default "azElExamples").
%       .SourceRevision is scalar text identifying the tested worktree.
%       .Verbose is a logical or binary numeric scalar (default true).
%**************************************************************************
% OUTPUTS
%   - report (scalar struct)
%       Contains resolved options, environment metadata, one row per
%       finite-jerk example and aggregate completion/pass flags.
%**************************************************************************
% UNITS
%   - Angles and path lengths are degrees; time is seconds. Jerk modes and
%     validation states are dimensionless.
%**************************************************************************

%% Section 1: Resolve Benchmark Controls

if nargin < 1 || isempty(optionOverrides)
    optionOverrides = struct();
end
options = resolveBenchmarkOptions(optionOverrides);
benchmarkPath = mfilename("fullpath");
repositoryRoot = fileparts(fileparts(benchmarkPath));
originalPath = path;
pathCleanup = onCleanup(@() path(originalPath));
addpath(repositoryRoot, fullfile(repositoryRoot, "examples"));

if isempty(options.ExampleNames)
    options.ExampleNames = discoverExampleNames(repositoryRoot);
end

%% Section 2: Execute Every Example And Motion Mode

runCount = numel(options.ExampleNames);
runs = repmat(emptyRunRecord(), runCount, 1);
for exampleIndex = 1:numel(options.ExampleNames)
    exampleName = options.ExampleNames(exampleIndex);
    runs(exampleIndex) = executeExample( ...
        exampleName, options.Verbose, exampleIndex, runCount);
end
runTable = struct2table(runs);

%% Section 3: Assemble And Store The Reproducible Report

report = struct( ...
    "BenchmarkName", "Maintained Az/El example suite", ...
    "Timestamp", datetime("now", "TimeZone", "local"), ...
    "SourceRevision", options.SourceRevision, ...
    "MATLABRelease", string(version("-release")), ...
    "MATLABVersion", string(version), ...
    "Platform", string(computer), ...
    "Options", options, ...
    "Runs", runTable, ...
    "AllRunsCompleted", all(runTable.RunCompleted), ...
    "AllValidationsPassed", all(runTable.ValidationPassed), ...
    "Passed", all(runTable.RunCompleted & runTable.ValidationPassed));

if strlength(options.OutputDirectory) > 0
    if ~isfolder(options.OutputDirectory)
        mkdir(options.OutputDirectory);
    end
    matPath = fullfile(options.OutputDirectory, ...
        options.ArtifactPrefix + ".mat");
    csvPath = fullfile(options.OutputDirectory, ...
        options.ArtifactPrefix + "_runs.csv");
    save(matPath, "report", "-v7.3");
    writetable(runTable, csvPath);
end
clear pathCleanup;
end

%% Section 4: Local Functions

function options = resolveBenchmarkOptions(optionOverrides)
% PURPOSE
%   - Merge benchmark overrides once and validate the public controls.
defaults = struct( ...
    "ExampleNames", strings(0, 1), ...
    "OutputDirectory", "", ...
    "ArtifactPrefix", "azElExamples", ...
    "SourceRevision", "", ...
    "Verbose", true);
if ~isstruct(optionOverrides) || ~isscalar(optionOverrides)
    error("benchmarkAzElExamples:InvalidOptions", ...
        "optionOverrides must be a scalar struct.");
end
[options, unknownNames] = azElInternal.resolveOptions( ...
    defaults, optionOverrides);
if ~isempty(unknownNames)
    warning("benchmarkAzElExamples:UnknownOptions", ...
        "Ignoring unknown option fields: %s. No behavior changed.", ...
        strjoin(unknownNames, ", "));
end
options.ExampleNames = reshape(string(options.ExampleNames), [], 1);
options.OutputDirectory = string(options.OutputDirectory);
options.ArtifactPrefix = string(options.ArtifactPrefix);
options.SourceRevision = string(options.SourceRevision);
if ~isscalar(options.OutputDirectory) || ...
        ~isscalar(options.ArtifactPrefix) || ...
        strlength(options.ArtifactPrefix) == 0 || ...
        ~isscalar(options.SourceRevision)
    error("benchmarkAzElExamples:InvalidTextOption", ...
        "OutputDirectory, ArtifactPrefix, and SourceRevision must be " + ...
        "scalar text; ArtifactPrefix cannot be empty.");
end
options.Verbose = azElInternal.normalizeLogicalScalar( ...
    options.Verbose, "Verbose", ...
    "benchmarkAzElExamples:InvalidVerbose");
end

function exampleNames = discoverExampleNames(repositoryRoot)
% PURPOSE
%   - Discover maintained example entry points without a duplicate catalog.
files = dir(fullfile(repositoryRoot, "examples", "example*.m"));
exampleNames = sort(erase(string({files.name}).', ".m"));
if isempty(exampleNames)
    error("benchmarkAzElExamples:NoExamples", ...
        "No maintained example*.m files were found under examples.");
end
end

function record = executeExample(exampleName, verbose, runIndex, runCount)
% PURPOSE
%   - Execute one headless example and extract stable comparison metrics.
record = emptyRunRecord();
record.Example = exampleName;
record.JerkConstrained = true;
exampleOptions = struct( ...
    "FigureVisible", "off", ...
    "PlotOutputs", false, ...
    "ShowAnimation", false, ...
    "ShowKinematicPlot", false, ...
    "ShowVisibilityGraphs", false, ...
    "ShowSweptSurfaces", false, ...
    "Verbose", false, ...
    "UseParallel", "off");

runTimer = tic;
try
    result = feval(exampleName, exampleOptions);
    record.WallTime_s = toc(runTimer);
    record.RunCompleted = true;
    record.PlannerSuccess = logical(result.Success);
    record.TerminationReason = string(result.TerminationReason);
    record.Message = string(result.Message);

    validation = result.Validation;
    if isfield(result, "ExampleValidation")
        validation = result.ExampleValidation;
    end
    record.ValidationPassed = logical(validation.Passed);
    if isfield(validation, "CollisionFree")
        record.CollisionFree = logical(validation.CollisionFree);
    end
    kinematicChecks = ["VelocityWithinLimits" ...
        "AccelerationWithinLimits" "JerkWithinLimits"];
    kinematicsPassed = true;
    for checkIndex = 1:numel(kinematicChecks)
        checkName = kinematicChecks(checkIndex);
        if isfield(validation, checkName)
            kinematicsPassed = kinematicsPassed && ...
                logical(validation.(checkName));
        end
    end
    record.KinematicsPassed = kinematicsPassed;

    if result.Success
        % Loop equivalent: add norm(route(i,:) - route(i-1,:)) for every
        % segment. The vector form keeps the geometric definition visible.
        record.SelectedPolylineLength_deg = sum(vecnorm( ...
            diff(result.selectedRoute_deg, 1, 1), 2, 2));
        record.SmoothedPathLength_deg = ...
            azElInternal.polylineLength(result.position_deg);
        record.MinimumMotionDuration_s = ...
            result.time_s(end) - result.time_s(1);
        record.FiniteJerkCertified = logical( ...
            validation.JerkWithinLimits && ...
            validation.ContinuousPolynomialExtremaVerified);
        record.FiniteJerkNumericallyVerified = ...
            all(isfinite(result.jerk_deg_s3), "all");
        record.CertificatePassed = record.FiniteJerkCertified && ...
            logical(validation.DynamicsDefectWithinTolerance);
        record.SampleCount = numel(result.time_s);
    end
catch exception
    record.WallTime_s = toc(runTimer);
    record.TerminationReason = "executionError";
    record.Message = string(getReport(exception, "extended", ...
        "hyperlinks", "off"));
end

if verbose
    fprintf("[%d/%d] %s | jerk=%d | success=%d | valid=%d | %.3f s\n", ...
        runIndex, runCount, exampleName, true, ...
        record.PlannerSuccess, record.ValidationPassed, record.WallTime_s);
end
end

function record = emptyRunRecord()
% PURPOSE
%   - Centralize the stable benchmark row schema and unavailable values.
record = struct( ...
    "Example", "", ...
    "JerkConstrained", false, ...
    "RunCompleted", false, ...
    "PlannerSuccess", false, ...
    "ValidationPassed", false, ...
    "CollisionFree", false, ...
    "KinematicsPassed", false, ...
    "CertificatePassed", false, ...
    "FiniteJerkCertified", false, ...
    "FiniteJerkNumericallyVerified", false, ...
    "TerminationReason", "notRun", ...
    "Message", "", ...
    "SelectedPolylineLength_deg", NaN, ...
    "SmoothedPathLength_deg", NaN, ...
    "MinimumMotionDuration_s", NaN, ...
    "SampleCount", 0, ...
    "WallTime_s", NaN);
end
