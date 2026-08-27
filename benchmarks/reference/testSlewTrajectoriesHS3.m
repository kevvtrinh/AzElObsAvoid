%% HS3 Slew Engine Examples And Regression Cases
% Add the repository root or its hs3 folder to the MATLAB path before running.
% This script replaces calcSlewTrajToStateJerkConst calls with solveTrajHS3.
%
% HS3 always enforces velocity, acceleration, and jerk constraints. There is
% therefore no acceleration-only constraintType branch in this version.

if ~exist("manuallyRun", "var")
    manuallyRun = false;
end
if ~exist("caseToRun", "var")
    caseToRun = "all";
end
if ~exist("plotLastCase", "var")
    plotLastCase = true;
end
if ~exist("compareAnalyticalSolver", "var")
    compareAnalyticalSolver = true;
end

% Numerical controls shared by every direct HS3 call. Increase SegmentCount
% when a difficult case needs more freedom. MaximumSolveTime is per case.
if ~exist("hs3Options", "var")
    hs3Options = struct( ...
        "SegmentCount", 10, ...
        "SampleTime", 0.05, ...
        "MaximumSolveTime", 30, ...
        "MaximumIterations", 300, ...
        "Verbose", false);
end

% When this file is placed in the repository root or examples folder, find
% the standalone engine automatically. An already configured path is retained.
if exist("solveTrajHS3", "file") ~= 2
    scriptFolder = fileparts(mfilename("fullpath"));
    hs3Candidates = [ ...
        string(fullfile(scriptFolder, "hs3")), ...
        string(fullfile(fileparts(scriptFolder), "hs3"))];
    for hs3Folder = hs3Candidates
        if isfile(fullfile(hs3Folder, "solveTrajHS3.m"))
            addpath(hs3Folder);
            break;
        end
    end
end
assert(exist("solveTrajHS3", "file") == 2, ...
    "solveTrajHS3 is not on the MATLAB path. Add <repo>/hs3 first.");
if compareAnalyticalSolver
    assert(exist("calcSlewTrajToStateJerkConst", "file") == 2, ...
        "calcSlewTrajToStateJerkConst is not on the MATLAB path.");
end

%% Define State-To-State Cases

cases = { ...
    makeCase("1D optimized time", 0.1, 2, 0.1, 0.8, 0.2, 0.99, ...
        1, 1, 1, [-100 100], []), ...
    makeCase("2D optimized time", [0.1 0.1], [2 4], [0.1 0.1], ...
        [0.8 0.8], [0.2 0.2], [0.99 0.99], [1 1], [1 1], [1 1], ...
        [-100 100; -100 100], []), ...
    makeCase("1D fixed time", 0.1, 2, 0.1, 0.8, 0.1, 0.99, ...
        1, 1, 1, [-100 100], 4), ...
    makeCase("2D fixed time", [0.1 0.1], [2 4], [0.1 0.1], ...
        [0.8 0.8], [0.2 0.2], [0.99 0.99], [1 1], [1 1], [1 1], ...
        [-100 100; -100 100], 1), ...
    makeCase("case 1 both directions", [5 5], [0 0], [0 0], [0 0], ...
        [0 0], [0 0], [3 3], [1 1], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 8 both directions", [15 15], [0 0], [0 -0.5], [0 0], ...
        [0 -0.5], [0 0], [10 10], [5 5], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 2 positive initial state", [15 15], [0 0], ...
        [0.5 0.5], [0 0], [0.5 0.5], [0 0], [3.6 3.6], [4 4], [5 5], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 3 negative terminal state", [0 0], [15 15], [0 0], ...
        [-0.5 -0.5], [0 0], [-0.5 -0.5], [3.6 3.6], [4 4], [4 4], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("cases 4 and 8", [0 0], [15 15], [0 0], [0.5 0.5], ...
        [0 0], [0.5 0.5], [3.6 3.6], [5 5], [4 4], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("cases 5 and 1", [0 15], [15 0], [0 0.5], [0.5 0], ...
        [0 0.5], [0.5 0], [5 5], [4 4], [5 5], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 6", [-0.1 -0.1], [2 2], [-0.1 -0.1], [0.8 0.8], ...
        [-0.2 -0.2], [0.8 0.8], [1 1], [1 1], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("cases 4 and 7", [-0.1 -0.1], [2 2], [-0.1 -0.1], [0.4 0.4], ...
        [-0.1 -0.1], [1 1], [1 1], [1 1], [0.8 0.8], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 8", [0 0], [2 2], [-0.1 -0.1], [0.8 0.8], ...
        [-0.2 -0.2], [0.8 0.8], [1 1], [1 1], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 13", [-10 -10], [613.5 613.5], [-2 -2], [55 55], ...
        [-1 -1], [5 5], [60 60], [5 5], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 14", [-10 -10], [513.5 613.5], [-2 -2], [45 45], ...
        [-1 -1], [5 5], [60 60], [5 5], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 15", [-10 -10], [21 21], [-2 -2], [21 21], ...
        [-1 -1], [5 5], [60 60], [5 5], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 16", [-10 -10], [396.5 396.5], [-2 -2], [34.5 34.5], ...
        [-1 -1], [2 2], [100 100], [5 5], [1 1], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 2 constrained velocity", [-10 -10], [50 50], [-2 -2], [2 2], ...
        [-1 -1], [5 5], [2 2], [5 5], [10 10], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("mixed case 7 and fixed case 8", [0 -0.1], [15 2], [0 -0.1], ...
        [0.5 0.4], [0 -0.2], [0.5 0.1], [5 1], [4 1], [5 0.8], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("mixed case 2 and fixed case 8", [0 -0.1], [15 15], [0 -0.1], ...
        [0.5 0], [0 -0.2], [0.5 0], [5 3.5], [4 1], [5 1000], ...
        [-1000 1000; -1000 1000], []), ...
    makeCase("case 16 reordered", [-10 -10], [200 21], [-2 -2], [-21 21], ...
        [-1 -1], [-5 5], [60 60], [5 5], [1 1], ...
        [-1000 1000; -1000 1000], [])};

%% Run State-To-State Cases

selectedCases = selectCases(cases, caseToRun);
hs3Results = cell(size(selectedCases));
analyticalResults = cell(size(selectedCases));
summary = strings(numel(selectedCases), 8);

for caseIndex = 1:numel(selectedCases)
    testCase = selectedCases{caseIndex};
    if manuallyRun
        clc;
        close all force;
    end

    fprintf("\n[%d/%d] %s\n", caseIndex, numel(selectedCases), testCase.Name);
    analyticalWallTime_s = NaN;
    analytical = createFailedAnalyticalResult("Analytical comparison disabled.");
    if compareAnalyticalSolver
        analyticalTimer = tic;
        analytical = runAnalyticalCase(testCase, hs3Options.SampleTime);
        analyticalWallTime_s = toc(analyticalTimer);
    end
    analyticalResults{caseIndex} = analytical;

    hs3Timer = tic;
    hs3 = runHs3Case(testCase, hs3Options);
    hs3WallTime_s = toc(hs3Timer);
    hs3Results{caseIndex} = hs3;

    summary(caseIndex, :) = [testCase.Name, ...
        string(analytical.Success), string(analytical.Duration), string(analyticalWallTime_s), ...
        string(hs3.Success), string(hs3.Duration), string(hs3WallTime_s), ...
        string(hs3.Duration - analytical.Duration)];
    fprintf("  Analytical | Success: %d | Slew: %.6g s | Wall: %.3f s\n", ...
        analytical.Success, analytical.Duration, analyticalWallTime_s);
    fprintf("  HS3    | Success: %d | Slew: %.6g s | Wall: %.3f s\n", ...
        hs3.Success, hs3.Duration, hs3WallTime_s);
    if ~analytical.Success
        fprintf("  Analytical failure: %s\n", analytical.Message);
    end
    if ~hs3.Success
        fprintf("  HS3 %s: %s\n", hs3.TerminationReason, hs3.Message);
    end

    if manuallyRun
        plotSolverComparison(analytical, hs3, testCase.Name);
        drawnow;
        input("Press Enter to run the next case...", "s");
    end
end

summaryTable = array2table(summary, "VariableNames", ...
    ["Case", "AnalyticalSuccess", "AnalyticalSlew_s", "AnalyticalWall_s", ...
    "HS3Success", "HS3Slew_s", "HS3Wall_s", "HS3MinusAnalyticalSlew_s"]);
disp(summaryTable);

if ~manuallyRun && plotLastCase && ~isempty(hs3Results)
    plotSolverComparison(analyticalResults{end}, hs3Results{end}, ...
        selectedCases{end}.Name);
end

%% Moving-Target Intercept
% The target state columns are [time, x, y, vx, vy, ax, ay]. The helper uses
% fixed-time HS3 calls and matches target position, velocity, and acceleration.
% It tests supplied target times chronologically, then bisects the first
% observed failed-to-passed interval. This preserves the old state-matching
% intent without adding obstacle-planner dependencies to the HS3 engine.

if caseToRun == "all" || caseToRun == "movingTarget"
    initialTargetState = struct( ...
        "time", 0, "position", [0 0], ...
        "velocity", [0 0], "acceleration", [0 0]);
    targetTime = (0:0.1:120).';
    targetX = 3 + 0.1 * targetTime;
    targetY = 3 * cosd(10 * targetTime);

    % Analytic derivatives avoid the artificial first-sample spikes produced
    % by repeatedly prepending zero to diff(...).
    targetXVelocity = 0.1 * ones(size(targetTime));
    targetYVelocity = -(pi / 6) * sind(10 * targetTime);
    targetXAcceleration = zeros(size(targetTime));
    targetYAcceleration = -(pi^2 / 108) * cosd(10 * targetTime);
    targetHistory = [targetTime, targetX, targetY, ...
        targetXVelocity, targetYVelocity, ...
        targetXAcceleration, targetYAcceleration];

    targetLimits = struct( ...
        "maximumVelocity", [1 1], ...
        "maximumAcceleration", [0.1 0.1], ...
        "maximumJerk", [0.1 0.1], ...
        "positionLower", [-100 -100], ...
        "positionUpper", [100 100]);
    targetOptions = hs3Options;
    targetOptions.SampleTime = 0.05;

    analyticalTargetWallTime_s = NaN;
    analyticalTarget = createFailedAnalyticalResult("Analytical comparison disabled.");
    if compareAnalyticalSolver
        analyticalTargetTimer = tic;
        analyticalTarget = runAnalyticalMovingTarget( ...
            initialTargetState, targetHistory, targetLimits, targetOptions.SampleTime);
        analyticalTargetWallTime_s = toc(analyticalTargetTimer);
    end

    hs3TargetTimer = tic;
    [targetTrajectory, targetSearch] = solveHs3MovingTarget( ...
        initialTargetState, targetHistory, targetLimits, targetOptions, 1e-3);
    fprintf("\nMoving-target intercept\n");
    fprintf("  Analytical | Success: %d | Slew: %.6g s | Wall: %.3f s\n", ...
        analyticalTarget.Success, analyticalTarget.Duration, analyticalTargetWallTime_s);
    fprintf("  HS3    | Success: %d | Slew: %.6g s | Wall: %.3f s | Trials: %d\n", ...
        targetTrajectory.Success, targetTrajectory.Duration, ...
        toc(hs3TargetTimer), targetSearch.TrialCount);
    if (analyticalTarget.Success || targetTrajectory.Success) && ...
            (manuallyRun || plotLastCase)
        plotMovingTargetComparison(analyticalTarget, targetTrajectory, targetHistory);
    elseif ~targetTrajectory.Success
        fprintf("  %s: %s\n", ...
            targetTrajectory.TerminationReason, targetTrajectory.Message);
    end
end

%% Local Functions

function testCase = makeCase(name, p0, pf, v0, vf, a0, af, ...
        vMax, aMax, jMax, positionBounds, fixedDuration)
% Normalize one shared benchmark case into a row-oriented record.
testCase = struct( ...
    "Name", string(name), ...
    "InitialPosition", p0(:).', ...
    "TerminalPosition", pf(:).', ...
    "InitialVelocity", v0(:).', ...
    "TerminalVelocity", vf(:).', ...
    "InitialAcceleration", a0(:).', ...
    "TerminalAcceleration", af(:).', ...
    "MaximumVelocity", vMax(:).', ...
    "MaximumAcceleration", aMax(:).', ...
    "MaximumJerk", jMax(:).', ...
    "PositionBounds", positionBounds, ...
    "FixedDuration", fixedDuration);
end

function selectedCases = selectCases(cases, caseToRun)
% Select all state cases, the core subset, the moving case, or one named case.
caseToRun = string(caseToRun);
if caseToRun == "all"
    selectedCases = cases;
elseif caseToRun == "stateCases"
    selectedCases = cases;
elseif caseToRun == "core"
    selectedCases = cases(1:4);
elseif any(caseToRun == ["movingTarget", "defineOnly"])
    selectedCases = {};
else
    names = cellfun(@(testCase) testCase.Name, cases);
    selectedIndex = find(strcmpi(names, caseToRun), 1);
    assert(~isempty(selectedIndex), "Unknown caseToRun value: %s", caseToRun);
    selectedCases = cases(selectedIndex);
end
end

function result = runAnalyticalCase(testCase, sampleTime)
% Run the legacy state-to-state solver when it is available for comparison.
warningState = warning;
warningCleanup = onCleanup(@() warning(warningState));
warning("off", "MATLAB:singularMatrix");
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:illConditionedMatrix");
try
    arguments = {testCase.InitialPosition, testCase.TerminalPosition, ...
        testCase.InitialVelocity, testCase.TerminalVelocity, ...
        testCase.InitialAcceleration, testCase.TerminalAcceleration, ...
        testCase.MaximumVelocity, testCase.MaximumAcceleration, ...
        testCase.MaximumJerk, testCase.PositionBounds};
    if isempty(testCase.FixedDuration)
        arguments = [arguments, {false}];
    else
        arguments = [arguments, {testCase.FixedDuration, false}];
    end
    if isscalar(testCase.InitialPosition)
        [xPolynomial, slewTime] = calcSlewTrajToStateJerkConst(arguments{:});
        polynomials = {xPolynomial};
    else
        [xPolynomial, yPolynomial, slewTime] = ...
            calcSlewTrajToStateJerkConst(arguments{:});
        polynomials = {xPolynomial, yPolynomial};
    end
    result = sampleAnalyticalPolynomials(polynomials, slewTime, sampleTime);
catch exception
    result = createFailedAnalyticalResult(exception.message);
end
end

function result = runAnalyticalMovingTarget(initialState, targetHistory, limits, sampleTime)
% Run the legacy moving-target solver when it is available for comparison.
warningState = warning;
warningCleanup = onCleanup(@() warning(warningState));
warning("off", "MATLAB:singularMatrix");
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:illConditionedMatrix");
try
    initialVector = [initialState.time, initialState.position, ...
        initialState.velocity, initialState.acceleration];
    bounds = [limits.positionLower(:), limits.positionUpper(:)];
    [xPolynomial, yPolynomial, slewTime] = calcSlewTrajToTargetJerkConst( ...
        initialVector, targetHistory, limits.maximumVelocity, ...
        limits.maximumAcceleration, limits.maximumJerk, bounds, false);
    result = sampleAnalyticalPolynomials( ...
        {xPolynomial, yPolynomial}, slewTime, sampleTime);
catch exception
    result = createFailedAnalyticalResult(exception.message);
end
end

function result = sampleAnalyticalPolynomials(polynomials, slewTime, sampleTime)
% Sample legacy piecewise polynomials into the common trajectory fields.
time = unique([(0:sampleTime:slewTime).'; slewTime]);
dimensionCount = numel(polynomials);
position = zeros(numel(time), dimensionCount);
velocity = position;
acceleration = position;
jerk = position;
for dimensionIndex = 1:dimensionCount
    positionPolynomial = polynomials{dimensionIndex};
    velocityPolynomial = differentiatePiecewisePolynomial(positionPolynomial);
    accelerationPolynomial = differentiatePiecewisePolynomial(velocityPolynomial);
    jerkPolynomial = differentiatePiecewisePolynomial(accelerationPolynomial);
    position(:, dimensionIndex) = ppval(positionPolynomial, time);
    velocity(:, dimensionIndex) = ppval(velocityPolynomial, time);
    acceleration(:, dimensionIndex) = ppval(accelerationPolynomial, time);
    jerk(:, dimensionIndex) = ppval(jerkPolynomial, time);
end
result = struct("Success", true, "Duration", slewTime, ...
    "FinalTime", slewTime, "time", time, "position", position, ...
    "velocity", velocity, "acceleration", acceleration, ...
    "jerk", jerk, "Message", "");
end

function derivative = differentiatePiecewisePolynomial(polynomial)
% Differentiate a scalar or vector piecewise polynomial exactly.
[breaks, coefficients, pieceCount, order, dimension] = unmkpp(polynomial);
if order <= 1
    derivative = mkpp(breaks, zeros(pieceCount * dimension, 1), dimension);
    return;
end
coefficients = reshape(coefficients, pieceCount * dimension, order);
derivativeCoefficients = coefficients(:, 1:end-1) .* (order - 1:-1:1);
derivative = mkpp(breaks, derivativeCoefficients, dimension);
end

function result = createFailedAnalyticalResult(message)
% Create the stable empty legacy-comparison record used on failure or disable.
result = struct("Success", false, "Duration", NaN, "FinalTime", NaN, ...
    "time", [], "position", [], "velocity", [], ...
    "acceleration", [], "jerk", [], "Message", string(message));
end

function trajectory = runHs3Case(testCase, commonOptions)
% Translate one reference case into the public dimension-neutral HS3 contract.
warningState = warning;
warningCleanup = onCleanup(@() warning(warningState));
warning("off", "MATLAB:singularMatrix");
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:illConditionedMatrix");
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
    "positionLower", reshape(testCase.PositionBounds(:, 1), 1, dimensionCount), ...
    "positionUpper", reshape(testCase.PositionBounds(:, 2), 1, dimensionCount));

if isempty(testCase.FixedDuration)
    maximumTime = estimateTimeHorizon(testCase);
    options = commonOptions;
    options.TimeMode = "earliestArrival";
    options.FinalTime = [];
else
    maximumTime = testCase.FixedDuration;
    options = commonOptions;
    options.TimeMode = "fixed";
    options.FinalTime = testCase.FixedDuration;
end
terminalState = struct( ...
    "position", testCase.TerminalPosition, ...
    "velocity", testCase.TerminalVelocity, ...
    "acceleration", testCase.TerminalAcceleration, ...
    "maximumTime", maximumTime);
trajectory = solveTrajHS3(initialState, terminalState, limits, options);
end

function maximumTime = estimateTimeHorizon(testCase)
% A generous finite horizon is required by the current earliest-arrival API.
distanceTime = max(abs(testCase.TerminalPosition - ...
    testCase.InitialPosition) ./ testCase.MaximumVelocity);
velocityTime = max(abs(testCase.TerminalVelocity - ...
    testCase.InitialVelocity) ./ testCase.MaximumAcceleration);
accelerationTime = max(abs(testCase.TerminalAcceleration - ...
    testCase.InitialAcceleration) ./ testCase.MaximumJerk);
maximumTime = max(1, 8 * max([distanceTime, velocityTime, accelerationTime]) + 10);
end

function [trajectory, search] = solveHs3MovingTarget( ...
        initialState, targetHistory, limits, commonOptions, timeTolerance)
% Search target samples chronologically, then bisect the first feasible bracket.
validateattributes(targetHistory, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 7});
assert(all(diff(targetHistory(:, 1)) > 0), ...
    "Target times must be strictly increasing.");

candidateTimes = targetHistory(targetHistory(:, 1) > initialState.time, 1);
trialCount = 0;
trajectory = createUnattemptedResult();
lastFailedTime = initialState.time;
firstPassedTime = NaN;

for candidateTime = candidateTimes.'
    trial = solveTargetAtTime(initialState, targetHistory, ...
        limits, commonOptions, candidateTime);
    trialCount = trialCount + 1;
    if trial.Success
        trajectory = trial;
        firstPassedTime = candidateTime;
        break;
    end
    trajectory = trial;
    lastFailedTime = candidateTime;
end

refinementCount = 0;
while isfinite(firstPassedTime) && ...
        firstPassedTime - lastFailedTime > timeTolerance && refinementCount < 30
    candidateTime = 0.5 * (lastFailedTime + firstPassedTime);
    trial = solveTargetAtTime(initialState, targetHistory, ...
        limits, commonOptions, candidateTime);
    trialCount = trialCount + 1;
    refinementCount = refinementCount + 1;
    if trial.Success
        trajectory = trial;
        firstPassedTime = candidateTime;
    else
        lastFailedTime = candidateTime;
    end
end

search = struct( ...
    "TrialCount", trialCount, ...
    "RefinementCount", refinementCount, ...
    "LastFailedTime", lastFailedTime, ...
    "FirstPassedTime", firstPassedTime, ...
    "TimeTolerance", timeTolerance);
end

function trajectory = solveTargetAtTime( ...
        initialState, targetHistory, limits, commonOptions, interceptTime)
% Solve one fixed-time intercept against the interpolated target state.
warningState = warning;
warningCleanup = onCleanup(@() warning(warningState));
warning("off", "MATLAB:singularMatrix");
warning("off", "MATLAB:nearlySingularMatrix");
warning("off", "MATLAB:illConditionedMatrix");
targetState = interp1(targetHistory(:, 1), targetHistory(:, 2:7), ...
    interceptTime, "pchip");
terminalState = struct( ...
    "position", targetState(1:2), ...
    "velocity", targetState(3:4), ...
    "acceleration", targetState(5:6), ...
    "maximumTime", interceptTime);
options = commonOptions;
options.TimeMode = "fixed";
options.FinalTime = interceptTime;
trajectory = solveTrajHS3(initialState, terminalState, limits, options);
end

function trajectory = createUnattemptedResult()
% Create the moving-target result returned before any candidate time is tried.
trajectory = struct( ...
    "Success", false, "Duration", NaN, ...
    "TerminationReason", "noCandidateTime", ...
    "Message", "No target time after the initial time was supplied.");
end

function plotSolverComparison(analytical, hs3, plotTitle)
% Compare available legacy and HS3 state-to-state trajectories.
if ~analytical.Success && ~hs3.Success
    warning("plotSolverComparison:NoMotion", ...
        "Neither solver produced a trajectory for '%s'.", plotTitle);
    return;
end
if hs3.Success
    dimensionCount = size(hs3.position, 2);
else
    dimensionCount = size(analytical.position, 2);
end
figure("Name", char(plotTitle), "Color", "w");
if dimensionCount == 1
    tiledlayout(4, 1, "TileSpacing", "compact");
    plotComparisonHistory(analytical, hs3, "position", "Position");
    plotComparisonHistory(analytical, hs3, "velocity", "Velocity");
    plotComparisonHistory(analytical, hs3, "acceleration", "Acceleration");
    plotComparisonHistory(analytical, hs3, "jerk", "Jerk");
else
    tiledlayout(3, 2, "TileSpacing", "compact");
    nexttile;
    hold on;
    plotPathIfSuccessful(analytical, "--", [0.85 0.33 0.10]);
    plotPathIfSuccessful(hs3, "-", [0 0.45 0.74]);
    axis equal;
    grid on;
    xlabel("Coordinate 1");
    ylabel("Coordinate 2");
    title("Path");
    legendSuccessful(analytical, hs3);
    nexttile;
    plotBothHistories(analytical, hs3, "position");
    grid on;
    xlabel("Time");
    ylabel("Position");
    title("Position");
    nexttile([1 2]);
    plotBothHistories(analytical, hs3, "velocity");
    grid on;
    xlabel("Time");
    ylabel("Velocity");
    title("Velocity");
    nexttile([1 2]);
    plotBothHistories(analytical, hs3, "acceleration");
    grid on;
    xlabel("Time");
    ylabel("Acceleration");
    title("Acceleration");
end
sgtitle(sprintf("%s | Analytical %.6g s | HS3 %.6g s", ...
    plotTitle, analytical.Duration, hs3.Duration));
end

function plotComparisonHistory(analytical, hs3, fieldName, quantity)
% Plot one labeled scalar history in the current comparison layout.
nexttile;
plotBothHistories(analytical, hs3, fieldName);
grid on;
xlabel("Time");
ylabel(quantity);
title(quantity);
legendSuccessful(analytical, hs3);
end

function plotBothHistories(analytical, hs3, fieldName)
% Overlay available legacy and HS3 histories for one state quantity.
hold on;
if analytical.Success
    plot(analytical.time, analytical.(fieldName), "--", ...
        "Color", [0.85 0.33 0.10], "LineWidth", 1.2);
end
if hs3.Success
    plot(hs3.time, hs3.(fieldName), "-", ...
        "Color", [0 0.45 0.74], "LineWidth", 1.2);
end
end

function plotPathIfSuccessful(result, lineStyle, color)
% Add a two-dimensional path only when the solver returned success.
if result.Success
    plot(result.position(:, 1), result.position(:, 2), ...
        lineStyle, "Color", color, "LineWidth", 1.5);
end
end

function legendSuccessful(analytical, hs3)
% Create a legend containing only solvers that returned a trajectory.
labels = strings(0, 1);
if analytical.Success
    labels(end + 1) = "Analytical";
end
if hs3.Success
    labels(end + 1) = "HS3";
end
if ~isempty(labels)
    legend(labels, "Location", "best");
end
end

function plotMovingTargetComparison(analytical, hs3, targetHistory)
% Compare moving-target intercept geometry and state histories.
figure("Name", "Moving-target solver comparison", "Color", "w");
tiledlayout(2, 2, "TileSpacing", "compact");
nexttile;
plot(targetHistory(:, 2), targetHistory(:, 3), ":", "LineWidth", 1.3);
hold on;
plotPathIfSuccessful(analytical, "--", [0.85 0.33 0.10]);
plotPathIfSuccessful(hs3, "-", [0 0.45 0.74]);
axis equal;
grid on;
xlabel("x");
ylabel("y");
labels = "Target";
if analytical.Success
    labels(end + 1) = "Analytical";
end
if hs3.Success
    labels(end + 1) = "HS3";
end
legend(labels, "Location", "best");
title("Intercept geometry");
nexttile;
plotBothHistories(analytical, hs3, "position");
grid on;
title("Position");
xlabel("Time");
nexttile;
plotBothHistories(analytical, hs3, "velocity");
grid on;
title("Velocity");
xlabel("Time");
nexttile;
plotBothHistories(analytical, hs3, "acceleration");
grid on;
title("Acceleration");
xlabel("Time");
sgtitle(sprintf("Moving target | Analytical %.6g s | HS3 %.6g s", ...
    analytical.Duration, hs3.Duration));
end
