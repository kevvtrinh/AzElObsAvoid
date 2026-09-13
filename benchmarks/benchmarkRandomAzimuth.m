function [runs, results] = benchmarkRandomAzimuth(caseIndices, outputFolder)
%% Section 0: Header & Readme
% SYNTAX
%   [runs, results] = benchmarkRandomAzimuth(caseIndices, outputFolder)
%
% PURPOSE
%   - Measure the unified fixed-arrival planner on paired moving-rectangle
%     slews with and without a static obstacle.
%   - Preserve failures and independently validate every successful motion.
%
% INPUTS
%   - caseIndices (positive integer vector; default 1:80)
%   - outputFolder (optional checkpoint folder; default writes nothing)
%
% OUTPUTS
%   - runs: per-case timing, selected guide, and motion-quality table.
%   - results: complete public planner results.
%     Planner wall time excludes generation, extra validation, and file I/O.
%
% UNITS
%   - Coordinates and path lengths are degrees; time is seconds.

%% Section 1: Validate Controls And Warm The Planner

if nargin < 1
    caseIndices = 1:80;
end
if nargin < 2
    outputFolder = "";
end
validateattributes(caseIndices, {'numeric'}, ...
    {'vector', 'integer', 'positive', 'finite', 'nonempty'});
outputFolder = string(outputFolder);
assert(isscalar(outputFolder));
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root, fullfile(root, 'trajectory'), fullfile(root, 'examples'));
if strlength(outputFolder) > 0 && ~isfolder(outputFolder)
    mkdir(outputFolder);
end
warm = createRandomAzimuthScenario(1, false);
planner([], warm.InitialState, warm.GoalState, warm.Limits, warm.Options);
count = numel(caseIndices) * 2;
results = cell(count, 1);
rows = cell(count, 1);
rowIndex = 0;

%% Section 2: Measure Both Obstacle Variants

for caseIndex = reshape(caseIndices, 1, [])
    for includeStaticObstacle = [false, true]
        scenario = createRandomAzimuthScenario(caseIndex, includeStaticObstacle);
        rowIndex = rowIndex + 1;
        timer = tic;
        result = planner(scenario.Obstacles, scenario.InitialState, ...
            scenario.GoalState, scenario.Limits, scenario.Options);
        wallTime_s = toc(timer);
        validationPassed = false;
        if result.Success
            validation = obstacleAvoidance.validateTrajectory(result);
            validationPassed = validation.Passed;
        end
        length_deg = NaN;
        arrival_s = NaN;
        if result.Success && validationPassed
            length_deg = result.MotionLength_units;
            arrival_s = result.ArrivalTime_s;
        end
        lowerBound_deg = scenario.EndpointDistance_deg;
        if includeStaticObstacle
            staticScene = obstacleAvoidance.obstacles.snapshot( ...
                result.PreparedObstacles(2), scenario.InitialState.time_s);
            staticGraph = obstacleAvoidance.search.createVisibilityGraph( ...
                staticScene, scenario.InitialState.position_units, ...
                scenario.GoalState.position_units, result.Limits, result.Options);
            lowerBound_deg = staticGraph.RouteLength_units;
        end
        searchTime_s = NaN;
        solveTime_s = NaN;
        conicCount = NaN;
        if isfield(result.VisibilityGraph, 'TimedSearch')
            searchTime_s = result.VisibilityGraph.TimedSearch.ElapsedTime_s;
        end
        if isfield(result.SolverDiagnostics, 'ElapsedTime_s')
            solveTime_s = result.SolverDiagnostics.ElapsedTime_s;
        end
        if isfield(result.SolverDiagnostics, 'ConicSolver')
            conicCount = result.SolverDiagnostics.ConicSolver.CallCount;
        end
        selectedGuide = "notSelected";
        if isfield(result, 'SeedSource')
            selectedGuide = string(result.SeedSource);
        end
        rows{rowIndex} = table(caseIndex, includeStaticObstacle, ...
            selectedGuide, scenario.RandomSeed, scenario.AzimuthSpan_deg, ...
            wallTime_s, result.Success, validationPassed, ...
            string(result.TerminationReason), arrival_s, length_deg, ...
            scenario.EndpointDistance_deg, ...
            length_deg / scenario.EndpointDistance_deg, lowerBound_deg, ...
            length_deg - lowerBound_deg, searchTime_s, solveTime_s, conicCount, ...
            'VariableNames', {'CaseIndex', 'StaticObstacle', 'SelectedGuide', ...
            'Seed', 'AzimuthSpan_deg', 'WallTime_s', 'Success', ...
            'ValidationPassed', 'TerminationReason', 'Arrival_s', 'Length_deg', ...
            'EndpointDistance_deg', 'LengthRatio', 'StaticLowerBound_deg', ...
            'LengthAboveLowerBound_deg', 'SearchTime_s', 'SolveTime_s', ...
            'ConicCount'});
        results{rowIndex} = result;
        runs = vertcat(rows{1:rowIndex});
        fprintf(['Case %d static=%d guide=%s: %s, %.3f s, ' ...
            'length %.6f deg, ratio %.6f\n'], caseIndex, ...
            includeStaticObstacle, selectedGuide, result.TerminationReason, ...
            wallTime_s, length_deg, ...
            length_deg / scenario.EndpointDistance_deg);
        if strlength(outputFolder) > 0
            writetable(runs, fullfile(outputFolder, 'runs.csv'));
            save(fullfile(outputFolder, sprintf('result_%03d.mat', rowIndex)), ...
                'result', 'scenario', '-v7.3');
        end
    end
end
if strlength(outputFolder) > 0
    save(fullfile(outputFolder, 'results.mat'), 'runs', 'results', ...
        'caseIndices', '-v7.3');
end
end
