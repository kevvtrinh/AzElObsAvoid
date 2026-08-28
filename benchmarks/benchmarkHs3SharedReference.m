function measurements = benchmarkHs3SharedReference(repeatCount)
%% Section 0: Header & Readme
% SYNTAX
%   measurements = benchmarkHs3SharedReference()
%   measurements = benchmarkHs3SharedReference(repeatCount)
%**************************************************************************
% PURPOSE
%   - Measure HS3 on the 21 state-to-state cases retrieved from the shared
%     reference script and compare arrival, wall time, and spatial length.
%**************************************************************************
% INPUTS
%   - repeatCount (positive integer scalar, optional; default 1)
%       Repetitions run serially in one MATLAB process. The first repetition
%       records cold solver startup while later repetitions measure warm use.
%**************************************************************************
% OUTPUTS
%   - measurements (table with one row per case and repetition)
%       Includes success, validation, arrival and wall margins, dense-sampled
%       Euclidean path length, and per-axis total-variation excess.
%**************************************************************************
% UNITS
%   - Time is seconds. Position and path length use caller coordinate units.
%**************************************************************************

%% Section 1: Validate Inputs & Locate Reference Artifacts

if nargin < 1 || isempty(repeatCount)
    repeatCount = 1;
end
validateattributes(repeatCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});

benchmarkFolder = fileparts(mfilename("fullpath"));
repositoryRoot = fileparts(benchmarkFolder);
referenceFolder = fullfile(benchmarkFolder, "reference");
referenceScript = fullfile(referenceFolder, "testSlewTrajectoriesHS3.m");
referenceTable = readtable( ...
    fullfile(referenceFolder, "hs3_slew_reference.csv"), ...
    "TextType", "string", "VariableNamingRule", "preserve");

trajectoryFolder = fullfile(repositoryRoot, "trajectory");
pathWasPresent = contains(path, trajectoryFolder);
if ~pathWasPresent
    addpath(trajectoryFolder);
end
pathCleanup = onCleanup(@() restoreTrajectoryPath( ...
    trajectoryFolder, pathWasPresent));

%% Section 2: Run The Shared Cases Serially

caseCount = height(referenceTable);
rowCount = repeatCount * caseCount;
rows = repmat(createEmptyMeasurement(), rowCount, 1);
rowIndex = 0;

for repeatIndex = 1:repeatCount
    % The reference remains a runnable script. These named values are its
    % documented caller-owned controls, so assignments are consumed by run.
    manuallyRun = false; %#ok<NASGU>
    caseToRun = "stateCases"; %#ok<NASGU>
    plotLastCase = false; %#ok<NASGU>
    compareAnalyticalSolver = false; %#ok<NASGU>
    hs3Options = struct( ...
        "SegmentCount", 10, ...
        "SampleTime", 0.05, ...
        "MaximumSolveTime", 30, ...
        "MaximumIterations", 300, ...
        "Verbose", false); %#ok<NASGU>

    % Initialize script outputs so Code Analyzer can verify the post-run
    % requirement. The script overwrites all three values before they are read.
    hs3Results = cell(0, 1);
    selectedCases = cell(0, 1);
    summary = strings(0, 8);

    run(referenceScript);
    assert(numel(hs3Results) == caseCount, ...
        "benchmarkHs3SharedReference:CaseCountMismatch", ...
        "Reference table has %d cases but the script ran %d.", ...
        caseCount, numel(hs3Results));

    for caseIndex = 1:caseCount
        rowIndex = rowIndex + 1;
        trajectory = hs3Results{caseIndex};
        testCase = selectedCases{caseIndex};
        referenceCaseName = referenceTable.Case(caseIndex);
        assert(testCase.Name == referenceCaseName, ...
            "benchmarkHs3SharedReference:CaseOrderMismatch", ...
            "Script case %d is '%s' while the table names '%s'.", ...
            caseIndex, testCase.Name, referenceCaseName);

        rows(rowIndex) = createMeasurement( ...
            repeatIndex, testCase, trajectory, summary(caseIndex, :), ...
            referenceTable(caseIndex, :));
    end
end

%% Section 3: Assemble Measurements

measurements = struct2table(rows);
end

%% Section 4: Local Functions

function measurement = createMeasurement( ...
        repeatIndex, testCase, trajectory, summaryRow, referenceRow)
% Create one benchmark row and measure spatial quality on a dense time grid.
measurement = createEmptyMeasurement();
measurement.Repeat = repeatIndex;
measurement.Case = testCase.Name;
measurement.ReferenceSuccess = parseBoolean(referenceRow.AnalyticalSuccess);
measurement.Success = trajectory.Success;
measurement.ValidationPassed = trajectory.Validation.Passed;
measurement.TerminationReason = string(trajectory.TerminationReason);
measurement.ArrivalTime_s = trajectory.Duration;
measurement.WallTime_s = str2double(summaryRow(7));
measurement.ReferenceArrival_s = referenceRow.AnalyticalArrival_s;
measurement.ReferenceWall_s = referenceRow.AnalyticalWall_s;
measurement.ArrivalMargin_s = ...
    measurement.ArrivalTime_s - measurement.ReferenceArrival_s;
measurement.WallMargin_s = ...
    measurement.WallTime_s - measurement.ReferenceWall_s;
measurement.MaximumConstraintViolation = trajectory.MaximumConstraintViolation;

if ~trajectory.Success
    return;
end

sampleCount = max(1001, ceil(trajectory.Duration / 0.002) + 1);
denseTime_s = linspace( ...
    trajectory.time(1), trajectory.time(end), sampleCount).';
[~, densePosition] = ...
    hs3Engine.polynomial.evaluateTrajectoryPolynomial( ...
    trajectory.Polynomial, denseTime_s);
positionDelta = diff(densePosition, 1, 1);
axisVariation = sum(abs(positionDelta), 1);
endpointDisplacement = abs( ...
    testCase.TerminalPosition - testCase.InitialPosition);

measurement.PathLength = sum(vecnorm(positionDelta, 2, 2));
measurement.DirectDistance = norm( ...
    testCase.TerminalPosition - testCase.InitialPosition);
measurement.PathExcess = ...
    measurement.PathLength - measurement.DirectDistance;
measurement.Axis1Variation = axisVariation(1);
measurement.Axis1Excess = axisVariation(1) - endpointDisplacement(1);
if numel(axisVariation) >= 2
    measurement.Axis2Variation = axisVariation(2);
    measurement.Axis2Excess = axisVariation(2) - endpointDisplacement(2);
end
end

function measurement = createEmptyMeasurement()
% Define stable fields for successful and unsuccessful reference cases.
measurement = struct( ...
    "Repeat", NaN, ...
    "Case", "", ...
    "ReferenceSuccess", false, ...
    "Success", false, ...
    "ValidationPassed", false, ...
    "TerminationReason", "", ...
    "ArrivalTime_s", NaN, ...
    "WallTime_s", NaN, ...
    "ReferenceArrival_s", NaN, ...
    "ReferenceWall_s", NaN, ...
    "ArrivalMargin_s", NaN, ...
    "WallMargin_s", NaN, ...
    "MaximumConstraintViolation", Inf, ...
    "PathLength", NaN, ...
    "DirectDistance", NaN, ...
    "PathExcess", NaN, ...
    "Axis1Variation", NaN, ...
    "Axis1Excess", NaN, ...
    "Axis2Variation", NaN, ...
    "Axis2Excess", NaN);
end

function value = parseBoolean(textValue)
% Parse the lowercase true/false spelling preserved in the reference CSV.
value = strcmpi(string(textValue), "true");
end

function restoreTrajectoryPath(trajectoryFolder, pathWasPresent)
% Remove only the temporary benchmark path entry that this function added.
if ~pathWasPresent
    rmpath(trajectoryFolder);
end
end
