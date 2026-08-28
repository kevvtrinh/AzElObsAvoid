function measurements = probeHs3SharedReferenceTimes(segmentCount)
%% Section 0: Header & Readme
% SYNTAX
%   measurements = probeHs3SharedReferenceTimes()
%   measurements = probeHs3SharedReferenceTimes(segmentCount)
%**************************************************************************
% PURPOSE
%   - Test whether each published analytical arrival is feasible in the
%     current fixed-time HS3 transcription before changing free-time search.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar, optional; default 10)
%       HS3 equal-duration segments used for every fixed-time probe.
%**************************************************************************
% OUTPUTS
%   - measurements (21-row table)
%       Reports reference success, fixed-time HS3 success and validation,
%       constraint violation, wall time, and termination reason.
%**************************************************************************
% UNITS
%   - Time is seconds. Coordinates use the shared scenario's abstract units.
%**************************************************************************

%% Section 1: Load The Reference Cases Without Solving Them

if nargin < 1 || isempty(segmentCount)
    segmentCount = 10;
end
validateattributes(segmentCount, {'numeric'}, ...
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

manuallyRun = false; %#ok<NASGU>
caseToRun = "defineOnly"; %#ok<NASGU>
plotLastCase = false; %#ok<NASGU>
compareAnalyticalSolver = false; %#ok<NASGU>
hs3Options = createSharedOptions(segmentCount); %#ok<NASGU>
cases = cell(0, 1);
run(referenceScript);

%% Section 2: Solve Every Published Arrival As Fixed Time

caseCount = height(referenceTable);
assert(numel(cases) == caseCount, ...
    "probeHs3SharedReferenceTimes:CaseCountMismatch", ...
    "Reference table has %d cases but the script defines %d.", ...
    caseCount, numel(cases));

caseName = strings(caseCount, 1);
referenceSuccess = false(caseCount, 1);
referenceArrival_s = NaN(caseCount, 1);
success = false(caseCount, 1);
validationPassed = false(caseCount, 1);
maximumConstraintViolation = Inf(caseCount, 1);
denseConstraintViolation = Inf(caseCount, 1);
wallTime_s = NaN(caseCount, 1);
terminationReason = strings(caseCount, 1);

for caseIndex = 1:caseCount
    testCase = cases{caseIndex};
    caseName(caseIndex) = testCase.Name;
    referenceSuccess(caseIndex) = ...
        strcmpi(referenceTable.AnalyticalSuccess(caseIndex), "true");
    referenceArrival_s(caseIndex) = ...
        referenceTable.AnalyticalArrival_s(caseIndex);
    if ~isfinite(referenceArrival_s(caseIndex))
        terminationReason(caseIndex) = "noReferenceArrival";
        continue;
    end

    solveTimer = tic;
    trajectory = solveCaseAtTime( ...
        testCase, referenceArrival_s(caseIndex), ...
        createSharedOptions(segmentCount));
    wallTime_s(caseIndex) = toc(solveTimer);
    success(caseIndex) = trajectory.Success;
    validationPassed(caseIndex) = trajectory.Validation.Passed;
    maximumConstraintViolation(caseIndex) = ...
        trajectory.MaximumConstraintViolation;
    denseConstraintViolation(caseIndex) = ...
        measureDenseViolation(trajectory, testCase);
    terminationReason(caseIndex) = trajectory.TerminationReason;
end

%% Section 3: Assemble Measurements

measurements = table( ...
    caseName, referenceSuccess, referenceArrival_s, success, ...
    validationPassed, maximumConstraintViolation, ...
    denseConstraintViolation, wallTime_s, ...
    terminationReason);
end

%% Section 4: Local Functions

function options = createSharedOptions(segmentCount)
% Reproduce the numerical options embedded in the retrieved script.
options = struct( ...
    "SegmentCount", segmentCount, ...
    "SampleTime", 0.05, ...
    "MaximumSolveTime", 30, ...
    "MaximumIterations", 300, ...
    "Verbose", false, ...
    "TimeMode", "fixed", ...
    "FinalTime", []);
end

function trajectory = solveCaseAtTime(testCase, finalTime_s, options)
% Translate one retrieved case into a fixed-time public HS3 request.
dimensionCount = numel(testCase.InitialPosition);
initialState = struct( ...
    "time", 0, ...
    "position", testCase.InitialPosition, ...
    "velocity", testCase.InitialVelocity, ...
    "acceleration", testCase.InitialAcceleration);
terminalState = struct( ...
    "position", testCase.TerminalPosition, ...
    "velocity", testCase.TerminalVelocity, ...
    "acceleration", testCase.TerminalAcceleration, ...
    "maximumTime", finalTime_s);
limits = struct( ...
    "maximumVelocity", testCase.MaximumVelocity, ...
    "maximumAcceleration", testCase.MaximumAcceleration, ...
    "maximumJerk", testCase.MaximumJerk, ...
    "positionLower", reshape( ...
    testCase.PositionBounds(:, 1), 1, dimensionCount), ...
    "positionUpper", reshape( ...
    testCase.PositionBounds(:, 2), 1, dimensionCount));
options.FinalTime = finalTime_s;
trajectory = hs3Engine.solve(initialState, terminalState, limits, options);
end

function maximumViolation = measureDenseViolation(trajectory, testCase)
% Check physical histories densely to distinguish hull conservatism from motion.
if isempty(trajectory.time) || isempty(fieldnames(trajectory.Polynomial))
    maximumViolation = Inf;
    return;
end
denseTime_s = linspace( ...
    trajectory.time(1), trajectory.time(end), 20001).';
[~, position, velocity, acceleration, jerk] = ...
    hs3Engine.polynomial.evaluateTrajectoryPolynomial( ...
    trajectory.Polynomial, denseTime_s);
positionLower = reshape(testCase.PositionBounds(:, 1), 1, []);
positionUpper = reshape(testCase.PositionBounds(:, 2), 1, []);
violation = [ ...
    position - positionUpper; ...
    positionLower - position; ...
    abs(velocity) - testCase.MaximumVelocity; ...
    abs(acceleration) - testCase.MaximumAcceleration; ...
    abs(jerk) - testCase.MaximumJerk];
endpointError = [ ...
    position(end, :) - testCase.TerminalPosition, ...
    velocity(end, :) - testCase.TerminalVelocity, ...
    acceleration(end, :) - testCase.TerminalAcceleration];
maximumViolation = max([0; violation(:); abs(endpointError(:))]);
end

function restoreTrajectoryPath(trajectoryFolder, pathWasPresent)
% Remove only the temporary engine path entry added by this diagnostic.
if ~pathWasPresent
    rmpath(trajectoryFolder);
end
end
