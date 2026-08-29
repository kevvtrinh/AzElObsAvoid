function measurements = benchmarkHs3SharedReference(repeatCount)
%% Section 0: Header & Readme
% SYNTAX
%   measurements = benchmarkHs3SharedReference()
%   measurements = benchmarkHs3SharedReference(repeatCount)
%**************************************************************************
% PURPOSE
%   - Measure HS3 on the shared 21-case state-to-state corpus and compare
%     arrival, wall time, and spatial length.
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
cases = createHs3ReferenceCases();
assert(numel(cases) == caseCount, ...
    "benchmarkHs3SharedReference:CaseCountMismatch", ...
    "Reference table has %d cases but the fixture defines %d.", ...
    caseCount, numel(cases));
rowCount = repeatCount * caseCount;
rows = repmat(createEmptyMeasurement(), rowCount, 1);
rowIndex = 0;

for repeatIndex = 1:repeatCount
    hs3Options = struct( ...
        "SegmentCount", 10, ...
        "SampleTime", 0.05, ...
        "MaximumSolveTime", 30, ...
        "MaximumIterations", 300, ...
        "Verbose", false);

    for caseIndex = 1:caseCount
        rowIndex = rowIndex + 1;
        testCase = cases{caseIndex};
        referenceCaseName = referenceTable.Case(caseIndex);
        assert(testCase.Name == referenceCaseName, ...
            "benchmarkHs3SharedReference:CaseOrderMismatch", ...
            "Script case %d is '%s' while the table names '%s'.", ...
            caseIndex, testCase.Name, referenceCaseName);

        solveTimer = tic;
        trajectory = runHs3Case(testCase, hs3Options);
        wallTime_s = toc(solveTimer);
        rows(rowIndex) = createMeasurement( ...
            repeatIndex, testCase, trajectory, wallTime_s, ...
            referenceTable(caseIndex, :));
    end
end

%% Section 3: Assemble Measurements

measurements = struct2table(rows);
end

%% Section 4: Local Functions

function measurement = createMeasurement( ...
        repeatIndex, testCase, trajectory, wallTime_s, referenceRow)
% Create one benchmark row and measure spatial quality on a dense time grid.
measurement = createEmptyMeasurement();
measurement.Repeat = repeatIndex;
measurement.Case = testCase.Name;
measurement.ReferenceSuccess = parseBoolean(referenceRow.AnalyticalSuccess);
measurement.Success = trajectory.Success;
measurement.ValidationPassed = trajectory.Validation.Passed;
measurement.TerminationReason = string(trajectory.TerminationReason);
measurement.ArrivalTime_s = trajectory.Duration;
measurement.WallTime_s = wallTime_s;
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

function trajectory = runHs3Case(testCase, options)
% Translate one fixture record into the direct HS3 engine contract.
dimensionCount = numel(testCase.InitialPosition);
initialState = struct( ...
    "time", 0, ...
    "position", testCase.InitialPosition, ...
    "velocity", testCase.InitialVelocity, ...
    "acceleration", testCase.InitialAcceleration);
limits = struct( ...
    "maximumVelocity", testCase.MaximumVelocity, ...
    "maximumAcceleration", testCase.MaximumAcceleration, ...
    "maximumJerk", testCase.MaximumJerk, ...
    "positionLower", reshape( ...
        testCase.PositionBounds(:, 1), 1, dimensionCount), ...
    "positionUpper", reshape( ...
        testCase.PositionBounds(:, 2), 1, dimensionCount));
if isempty(testCase.FixedDuration)
    timeMode = "earliestArrival";
    finalTime = [];
    maximumTime = estimateTimeHorizon(testCase);
else
    timeMode = "fixed";
    finalTime = testCase.FixedDuration;
    maximumTime = testCase.FixedDuration;
end
terminalState = struct( ...
    "position", testCase.TerminalPosition, ...
    "velocity", testCase.TerminalVelocity, ...
    "acceleration", testCase.TerminalAcceleration, ...
    "maximumTime", maximumTime);
options.TimeMode = timeMode;
options.FinalTime = finalTime;
trajectory = hs3Engine.solve(initialState, terminalState, limits, options);
end

function maximumTime = estimateTimeHorizon(testCase)
% Preserve the reference corpus's input-derived finite search horizon.
distanceTime = max(abs(testCase.TerminalPosition - ...
    testCase.InitialPosition) ./ testCase.MaximumVelocity);
velocityTime = max(abs(testCase.TerminalVelocity - ...
    testCase.InitialVelocity) ./ testCase.MaximumAcceleration);
accelerationTime = max(abs(testCase.TerminalAcceleration - ...
    testCase.InitialAcceleration) ./ testCase.MaximumJerk);
maximumTime = max( ...
    1, 8 * max([distanceTime, velocityTime, accelerationTime]) + 10);
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
