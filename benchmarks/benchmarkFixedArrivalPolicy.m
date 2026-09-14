function [runs, results] = benchmarkFixedArrivalPolicy(repetitions)
%% Section 0: Header & Readme
% SYNTAX
%   [runs, results] = benchmarkFixedArrivalPolicy(repetitions)
%
% PURPOSE
%   - Measure the unified fixed-arrival seed policy on three deterministic
%     requests that previously favored different caller-selected modes.
%
% INPUTS
%   - repetitions (positive integer; default 3): measured runs after warmup.
%
% OUTPUTS
%   - runs: per-run timing, selected guide, and motion-quality table.
%   - results: complete corresponding public planner results.
%
% UNITS
%   - Coordinates are degrees and time is seconds.

%% Section 1: Load Deterministic Source Requests

if nargin < 1
    repetitions = 3;
end
validateattributes(repetitions, {'numeric'}, ...
    {'scalar', 'integer', 'positive', 'finite'});
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root, 'trajectory'), fullfile(root, 'examples'));
spatialScenario = createRandomAzimuthScenario(1, true);
cases(1) = struct('Name', "Random 1 with static obstacle", ...
    'obstacles', spatialScenario.Obstacles, ...
    'initial', spatialScenario.InitialState, ...
    'goal', spatialScenario.GoalState, ...
    'limits', spatialScenario.Limits, ...
    'options', spatialScenario.Options);
request = jsondecode(fileread(fullfile(root, 'tests', 'fixtures', ...
    'savedMovingDetour.json')));
sources = cell(numel(request.obstacles), 1);
for sourceIndex = 1:numel(sources)
    source = request.obstacles(sourceIndex);
    frames = source.keyframes;
    sources{sourceIndex} = obstacleAvoidance.obstacles.createObstacle( ...
        source.name, [frames.time_s].', ...
        arrayfun(@(frame) frame.vertices_units(:, 1), frames, ...
            'UniformOutput', false), ...
        arrayfun(@(frame) frame.vertices_units(:, 2), frames, ...
            'UniformOutput', false), source.safetyMargin_units);
end
request.options.GoalTimeMode = 'fixedArrival';
cases(2) = struct('Name', "Saved moving detour", ...
    'obstacles', obstacleAvoidance.obstacles.combineObstacles(sources), ...
    'initial', request.initialState, 'goal', request.goalState, ...
    'limits', request.limits, 'options', request.options);
timedScenario = createRandomAzimuthScenario(26, true);
cases(3) = struct('Name', "Random 26 with static obstacle", ...
    'obstacles', timedScenario.Obstacles, ...
    'initial', timedScenario.InitialState, ...
    'goal', timedScenario.GoalState, ...
    'limits', timedScenario.Limits, ...
    'options', timedScenario.Options);

%% Section 2: Warm And Measure The Unified Policy

measuredRunCount = numel(cases) * repetitions;
rows = cell(measuredRunCount, 1);
results = cell(measuredRunCount, 1);
rowIndex = 0;
for caseIndex = 1:numel(cases)
    input = cases(caseIndex);
    for repetition = 0:repetitions
        timer = tic;
        result = planner(input.obstacles, input.initial, input.goal, ...
            input.limits, input.options);
        elapsed_s = toc(timer);
        passed = result.Success && ...
            obstacleAvoidance.validateTrajectory(result).Passed;
        arrival_s = NaN;
        length_units = NaN;
        searchTime_s = NaN;
        if result.Success
            arrival_s = result.ArrivalTime_s;
            length_units = result.MotionLength_units;
        end
        if isfield(result.VisibilityGraph, 'TimedSearch')
            searchTime_s = result.VisibilityGraph.TimedSearch.ElapsedTime_s;
        end
        selectedGuide = "notSelected";
        if isfield(result, 'SeedSource')
            selectedGuide = string(result.SeedSource);
        end
        fprintf(['%s repetition %d: %.6f s; valid %d; arrival %.9f; ' ...
            'length %.12f; guide %s; %s\n'], input.Name, repetition, ...
            elapsed_s, passed, arrival_s, length_units, selectedGuide, ...
            result.TerminationReason);
        if repetition == 0
            continue
        end
        rowIndex = rowIndex + 1;
        rows{rowIndex} = struct('Case', input.Name, ...
            'Repetition', repetition, 'Runtime_s', elapsed_s, ...
            'Success', result.Success, 'IndependentValidation', passed, ...
            'Arrival_s', arrival_s, 'Length_units', length_units, ...
            'SelectedGuide', selectedGuide, 'SearchTime_s', searchTime_s, ...
            'TerminationReason', result.TerminationReason);
        results{rowIndex} = result;
    end
end
runs = struct2table(vertcat(rows{:}));
end
