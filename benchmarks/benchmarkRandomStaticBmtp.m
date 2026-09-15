function runs = benchmarkRandomStaticBmtp(caseIndices, repetitions, outputFolder)
%% Section 0: Header & Readme
% SYNTAX
%   runs = benchmarkRandomStaticBmtp()
%   runs = benchmarkRandomStaticBmtp(caseIndices)
%   runs = benchmarkRandomStaticBmtp(caseIndices, repetitions)
%   runs = benchmarkRandomStaticBmtp(caseIndices, repetitions, outputFolder)
%**************************************************************************
% PURPOSE
%   - Measure deterministic high-density static BMTP cases that exercise
%     transient implied-plane removal and dense exact visibility construction.
%**************************************************************************
% INPUTS
%   - caseIndices (positive integer vector, optional; default 1:4)
%       Selects frozen benchmark cases.
%   - repetitions (positive integer scalar, optional; default 3)
%       Number of measured runs for every selected case.
%   - outputFolder (string scalar, optional; default "")
%       Folder for compact MAT and CSV evidence; empty text writes nothing.
%**************************************************************************
% OUTPUTS
%   - runs (table)
%       One row per run with validation, motion, graph, and solver data.
%       Ordinary planning failures remain Success = false; invalid input throws.
%**************************************************************************
% UNITS
%   - Coordinates and path length are units; time is seconds.
%**************************************************************************

%% Section 1: Resolve Benchmark Controls And Frozen Inputs

if nargin < 1 || isempty(caseIndices)
    caseIndices = 1:4;
end
if nargin < 2 || isempty(repetitions)
    repetitions = 3;
end
if nargin < 3
    outputFolder = "";
end
validateattributes(caseIndices, {'numeric'}, ...
    {'vector', 'integer', 'positive', '<=', 4, 'nonempty'});
validateattributes(repetitions, {'numeric'}, {'scalar', 'integer', 'positive'});
outputFolder = string(outputFolder);
assert(isscalar(outputFolder), 'benchmarkRandomStaticBmtp:InvalidOutputFolder', ...
    'outputFolder must be a string scalar.');
repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'));
allCases = createFrozenCases();
cases    = allCases(caseIndices);
if strlength(outputFolder) > 0 && ~isfolder(outputFolder)
    mkdir(outputFolder);
end

cacheControl = createCacheControl();
planner(cacheControl.Obstacles, cacheControl.InitialState, cacheControl.GoalState, ...
    cacheControl.Limits, cacheControl.Options);
rows     = cell(numel(cases) * repetitions, 1);
rowIndex = 0;

%% Section 2: Run Every Case And Preserve Failures

for repetition = 1:repetitions
    for localCaseIndex = 1:numel(cases)
        caseInput = cases{localCaseIndex};
        % Force the measured request to rebuild its graph even when a caller
        % benchmarks one case repeatedly and the planner's one-entry cache matches.
        planner(cacheControl.Obstacles, cacheControl.InitialState, cacheControl.GoalState, ...
            cacheControl.Limits, cacheControl.Options);
        timer  = tic;
        result = planner(caseInput.Obstacles, caseInput.InitialState, caseInput.GoalState, ...
            caseInput.Limits, caseInput.Options);
        wallTime_s   = toc(timer);
        validation   = obstacleAvoidance.validateTrajectory(result);
        arrival_s    = NaN;
        length_units = NaN;
        if result.Success
            arrival_s    = result.ArrivalTime_s;
            length_units = result.MotionLength_units;
        end
        diagnostics = result.SolverDiagnostics;
        conicDiagnostics = diagnosticValue(diagnostics, 'ConicSolver', ...
            struct('TotalTime_s', 0, 'CallCount', 0));
        rowIndex = rowIndex + 1;
        rows{rowIndex} = table(caseInput.Index, repetition, caseInput.Seed, caseInput.ToothCount, ...
            wallTime_s, result.Success, validation.Passed, string(result.TerminationReason), ...
            arrival_s, length_units, diagnosticValue(diagnostics, 'ElapsedTime_s', 0), ...
            conicDiagnostics.TotalTime_s, conicDiagnostics.CallCount, ...
            diagnosticValue(diagnostics, 'TaggedPairCount', 0), ...
            diagnosticValue(diagnostics, 'TrajectorySocpCount', 0), ...
            diagnosticValue(diagnostics, 'PlaneSocpCount', 0), ...
            diagnosticValue(diagnostics, 'OptimizerSpanCount', 0), ...
            diagnosticValue(diagnostics, 'IterationCount', 0), ...
            diagnosticValue(diagnostics, 'TransientPlaneRemovalCount', 0), ...
            diagnosticValue(result.PlaneCertificate, 'ExactRegionCount', 0), ...
            'VariableNames', {'CaseIndex', 'Repetition', 'Seed', 'ToothCount', ...
            'WallTime_s', 'Success', 'ValidationPassed', 'TerminationReason', ...
            'Arrival_s', 'Length_units', 'BmtpTime_s', 'ConicTime_s', 'ConicCallCount', ...
            'TaggedPairCount', 'TrajectorySocpCount', 'PlaneSocpCount', ...
            'OptimizerSpanCount', 'IterationCount', 'TransientPlaneRemovalCount', ...
            'ExactRegionCount'});
        fprintf(['Case %d rep=%d: success=%d valid=%d wall=%.4f bmtp=%.4f ' ...
            'conic=%.4f regions=%d tags=%d removed=%d\n'], caseInput.Index, repetition, ...
            result.Success, validation.Passed, wallTime_s, rows{rowIndex}.BmtpTime_s, ...
            conicDiagnostics.TotalTime_s, rows{rowIndex}.ExactRegionCount, ...
            rows{rowIndex}.TaggedPairCount, rows{rowIndex}.TransientPlaneRemovalCount);
    end
end
runs = vertcat(rows{:});
if strlength(outputFolder) > 0
    save(fullfile(outputFolder, 'random_static_bmtp.mat'), 'runs');
    writetable(runs, fullfile(outputFolder, 'random_static_bmtp.csv'));
end
end

%% Section 3: Local Functions

function cases = createFrozenCases()
    % Generate the four deterministic dense-band inputs from one saved RNG state.
    previousRandomState = rng;
    restoreRandomState  = onCleanup(@() rng(previousRandomState));
    rng(20260914, 'twister');
    cases = cell(4, 1);
    for caseIndex = 1:numel(cases)
        toothCount       = randi([250, 350]);
        halfWidth_units  = 5 + rand;
        halfHeight_units = 3 + rand;
        notchDepth_units = 1 + 0.8 * rand;

        x_units      = linspace(-halfWidth_units, halfWidth_units, 2 * toothCount + 1).';
        phase        = mod((0:numel(x_units) - 1).', 2);
        top_units    = halfHeight_units - notchDepth_units * phase .* (0.8 + 0.2 * rand(size(x_units)));
        bottom_units = -halfHeight_units + notchDepth_units * phase .* (0.8 + 0.2 * rand(size(x_units)));
        vertices_units = [x_units, top_units; flipud(x_units), flipud(bottom_units)];

        angle_rad      = -0.12 + 0.24 * rand;
        rotationMatrix = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
        vertices_units = vertices_units * rotationMatrix.';
        horizon_s      = 240;
        obstacle       = obstacleAvoidance.obstacles.createObstacle( ...
            "high-density band " + caseIndex, [0; horizon_s], ...
            vertices_units(:, 1), vertices_units(:, 2), 0.02 + 0.04 * rand);
        initialState = state([-12, 0], 0);
        goalState    = state([12, 0], horizon_s);
        limits = struct( ...
            'xInterval_units',         [-16, 16], ...
            'yInterval_units',         [-12, 12], ...
            'maxVelocity_units_s',      [2, 2], ...
            'maxAcceleration_units_s2', 0.7 + 0.5 * rand(1, 2), ...
            'maxJerk_units_s3',         2.5 + rand(1, 2));
        options = struct( ...
            'GoalTimeMode',          'earliestArrival', ...
            'SampleTime_s',          0.1, ...
            'ArrivalTimeTolerance_s', 1e-8);
        cases{caseIndex} = struct( ...
            'Index',        caseIndex, ...
            'Seed',         20260914, ...
            'ToothCount',   toothCount, ...
            'Obstacles',    obstacle, ...
            'InitialState', initialState, ...
            'GoalState',    goalState, ...
            'Limits',       limits, ...
            'Options',      options);
    end
end

function request = createCacheControl()
    % Create a distinct request that evicts the planner's one-entry graph cache.
    request = struct( ...
        'Obstacles',    [], ...
        'InitialState', state([-1, 0], 0), ...
        'GoalState',    state([1, 0], 20), ...
        'Limits',       struct( ...
            'xInterval_units',         [-2, 2], ...
            'yInterval_units',         [-2, 2], ...
            'maxVelocity_units_s',      [1, 1], ...
            'maxAcceleration_units_s2', [1, 1], ...
            'maxJerk_units_s3',         [2, 2]), ...
        'Options',      struct('GoalTimeMode', 'fixedArrival'));
end

function value = state(position_units, time_s)
    % Assemble one rest state with the planner's established public fields.
    value = struct( ...
        'time_s',                time_s, ...
        'position_units',        position_units, ...
        'velocity_units_s',      [0, 0], ...
        'acceleration_units_s2', [0, 0]);
end

function value = diagnosticValue(container, fieldName, defaultValue)
    % Read an optional diagnostic while preserving the caller's default.
    value = defaultValue;
    if isstruct(container) && isfield(container, fieldName)
        value = container.(fieldName);
    end
end
