function measurements = benchmarkTrajectoryEngines(engineNames, repeatCount)
%% Section 0: Header & Readme
% SYNTAX
%   measurements = benchmarkTrajectoryEngines()
%   measurements = benchmarkTrajectoryEngines(engineNames)
%   measurements = benchmarkTrajectoryEngines(engineNames, repeatCount)
%**************************************************************************
% PURPOSE
%   - Compare forced Ruckig-derived and pure HS3 engines on the identical 21
%     state-to-state reference requests.
%**************************************************************************
% INPUTS
%   - engineNames (text vector, optional; default ["ruckig", "hs3"])
%       Ordered independent engines to execute directly.
%   - repeatCount (positive integer scalar, optional; default 1)
%       Serial repetitions for every engine and case.
%**************************************************************************
% OUTPUTS
%   - measurements (table)
%       Engine, success, validation, duration, path length, violation, and
%       wall time for every requested engine/case/repetition.
%**************************************************************************
% UNITS
%   - Time is seconds; coordinates and path length use reference case units.
%**************************************************************************

%% Section 1: Validate Controls And Load Reference Cases

if nargin < 1 || isempty(engineNames)
    engineNames = ["ruckig", "hs3"];
end
if nargin < 2 || isempty(repeatCount)
    repeatCount = 1;
end
engineNames = lower(string(engineNames(:).'));
if isempty(engineNames) || any(~ismember(engineNames, ["ruckig", "hs3"]))
    error("benchmarkTrajectoryEngines:InvalidEngines", ...
        "engineNames must contain only 'ruckig' and 'hs3'.");
end
validateattributes(repeatCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
benchmarkFolder = fileparts(mfilename("fullpath"));
repositoryRoot = fileparts(benchmarkFolder);
addpath(fullfile(repositoryRoot, "trajectory"));
cases = createHs3ReferenceCases();

%% Section 2: Run Every Forced Engine Serially

caseCount = numel(cases);
rowCount = repeatCount * numel(engineNames) * caseCount;
rows = repmat(createEmptyMeasurement(), rowCount, 1);
rowIndex = 0;
for repeatIndex = 1:repeatCount
    for engineName = engineNames
        for caseIndex = 1:caseCount
            testCase = cases{caseIndex};
            [initialState, terminalState, limits, options] = ...
                createRequest(testCase, engineName);
            solveTimer = tic;
            if engineName == "ruckig"
                result = ruckigEngine.solve( ...
                    initialState, terminalState, limits, options);
            else
                result = hs3Engine.solve( ...
                    initialState, terminalState, limits, options);
            end
            wallTime_s = toc(solveTimer);
            rowIndex = rowIndex + 1;
            rows(rowIndex) = createMeasurement( ...
                repeatIndex, engineName, testCase.Name, result, wallTime_s);
            fprintf( ...
                "[%s %d/%d] success=%s validation=%s duration=%.9g wall=%.6g reason=%s\n", ...
                engineName, caseIndex, caseCount, string(result.Success), ...
                string(result.Validation.Passed), result.Duration, ...
                wallTime_s, result.TerminationReason);
        end
    end
end

%% Section 3: Assemble Measurements

measurements = struct2table(rows);
end

%% Section 4: Local Functions

function [initialState, terminalState, limits, options] = ...
        createRequest(testCase, engineName)
% Translate one retrieved case into a direct request for either engine.
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
    maximumTime = estimateTimeHorizon(testCase);
    timeMode = "earliestArrival";
    finalTime = [];
else
    maximumTime = testCase.FixedDuration;
    timeMode = "fixed";
    finalTime = testCase.FixedDuration;
end
terminalState = struct( ...
    "position", testCase.TerminalPosition, ...
    "velocity", testCase.TerminalVelocity, ...
    "acceleration", testCase.TerminalAcceleration, ...
    "maximumTime", maximumTime);
options = struct( ...
    "TimeMode", timeMode, ...
    "FinalTime", finalTime, ...
    "SampleTime", 0.05, ...
    "Verbose", false);
if engineName == "hs3"
    hs3Options = createHs3Options();
    for fieldName = string(fieldnames(hs3Options)).'
        options.(fieldName) = hs3Options.(fieldName);
    end
end
end

function maximumTime = estimateTimeHorizon(testCase)
% Preserve the retrieved script's input-derived generous finite horizon.
distanceTime = max(abs(testCase.TerminalPosition - ...
    testCase.InitialPosition) ./ testCase.MaximumVelocity);
velocityTime = max(abs(testCase.TerminalVelocity - ...
    testCase.InitialVelocity) ./ testCase.MaximumAcceleration);
accelerationTime = max(abs(testCase.TerminalAcceleration - ...
    testCase.InitialAcceleration) ./ testCase.MaximumJerk);
maximumTime = max( ...
    1, 8 * max([distanceTime, velocityTime, accelerationTime]) + 10);
end

function options = createHs3Options()
% Reproduce the numerical controls used by the retrieved reference script.
options = struct( ...
    "SegmentCount", 10, ...
    "SampleTime", 0.05, ...
    "MaximumSolveTime", 30, ...
    "MaximumIterations", 300, ...
    "Verbose", false);
end

function measurement = createMeasurement( ...
        repeatIndex, engineName, caseName, result, wallTime_s)
% Create one stable engine/case measurement with dense spatial length.
measurement = createEmptyMeasurement();
measurement.Repeat = repeatIndex;
measurement.Engine = engineName;
measurement.Case = caseName;
measurement.Success = result.Success;
measurement.ValidationPassed = result.Validation.Passed;
measurement.TerminationReason = result.TerminationReason;
measurement.Duration_s = result.Duration;
measurement.WallTime_s = wallTime_s;
measurement.MaximumConstraintViolation = ...
    result.MaximumConstraintViolation;
measurement.IntegratedSquaredJerk = result.IntegratedSquaredJerk;
if ~result.Success
    return;
end
denseTime_s = linspace( ...
    result.time(1), result.time(end), ...
    max(1001, ceil(result.Duration / 0.002) + 1)).';
if engineName == "ruckig"
    [~, densePosition] = ruckigEngine.internal.evaluatePolynomial( ...
        result.Polynomial, denseTime_s);
else
    [~, densePosition] = ...
        hs3Engine.polynomial.evaluateTrajectoryPolynomial( ...
        result.Polynomial, denseTime_s);
end
measurement.PathLength = ...
    sum(vecnorm(diff(densePosition, 1, 1), 2, 2));
end

function measurement = createEmptyMeasurement()
% Define stable evidence fields for successes and expected failures.
measurement = struct( ...
    "Repeat", NaN, ...
    "Engine", "", ...
    "Case", "", ...
    "Success", false, ...
    "ValidationPassed", false, ...
    "TerminationReason", "", ...
    "Duration_s", NaN, ...
    "WallTime_s", NaN, ...
    "PathLength", NaN, ...
    "IntegratedSquaredJerk", Inf, ...
    "MaximumConstraintViolation", Inf);
end
