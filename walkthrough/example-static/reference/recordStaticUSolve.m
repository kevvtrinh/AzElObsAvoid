function trace = recordStaticUSolve(obstacles, request, projectFolder)
%% Section 0: Header & Readme
% SYNTAX
%   trace = recordStaticUSolve(obstacles, request, projectFolder)
%**************************************************************************
% PURPOSE
%   Record the actual Static U solver stages for teaching and compare them
%   with the public planner. Diagnostic copies live only in ignored output.
%**************************************************************************
% INPUTS
%   obstacles, request: checked example inputs from the lesson setup.
%   projectFolder: repository folder containing planner.m and trajectory.
%**************************************************************************
% OUTPUTS
%   trace: solver inputs, iterates, line proposals and the public result.
%   A mismatched diagnostic solve throws; no alternate motion is substituted.
%**************************************************************************
% UNITS
%   Coordinates use units; time is seconds and rates use units/s^order.
%**************************************************************************

%% Section 1: Reuse A Recording Only For Identical Inputs And Sources
persistent savedTrace savedInputs savedSources
engineFolder = fullfile(projectFolder, 'trajectory', '+bmtpEngine');
sourceFiles = dir(fullfile(engineFolder, '**', '*.m'));
sourceFiles = [sourceFiles; dir(fullfile(projectFolder, '+obstacleAvoidance', '**', '*.m')); ...
    dir(fullfile(projectFolder, 'planner.m'))];
sourceContents = cell(numel(sourceFiles), 1);
for sourceIndex = 1:numel(sourceFiles)
    sourceContents{sourceIndex} = fileread(fullfile(sourceFiles(sourceIndex).folder, sourceFiles(sourceIndex).name));
end
builderPath = fullfile(projectFolder, 'walkthrough', 'example-static', 'reference', 'buildStaticUTraceRuntime.py');
sourceContents{end + 1} = fileread(builderPath);
recordInputs = struct('Obstacles', obstacles, 'Request', request);
if ~isempty(savedTrace) && isequaln(savedInputs, recordInputs) && isequaln(savedSources, sourceContents)
    trace = savedTrace;
    return
end

%% Section 2: Build The Upstream Geometry Already Taught
preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacles, ...
    [request.initialState.time_s, request.goalState.time_s], true);
snapshot = obstacleAvoidance.obstacles.snapshot(preparedObstacles, request.initialState.time_s, true);
vertexVisibility = obstacleAvoidance.search.createVertexVisibility(snapshot, request.limits, request.options);
visibilityGraph = obstacleAvoidance.search.createVisibilityGraph(vertexVisibility, ...
    request.initialState.position_units, request.goalState.position_units);
route_units = visibilityGraph.Route_units;
edgeLength_units = vecnorm(diff(route_units), 2, 2);
startingPath = struct('position_units', route_units, 'tau', [0; cumsum(edgeLength_units)] / sum(edgeLength_units));
regions_units = vertcat(snapshot.Regions_units);
planningEnvironment = struct('preparedObstacles', preparedObstacles, 'snapshot', snapshot, ...
    'vertexVisibility', vertexVisibility, 'regions_units', {regions_units}, ...
    'coverage', struct('ExactRegionCount', numel(regions_units)));

%% Section 3: Run The Diagnostic Copy And The Public Planner
runtimeFolder = fullfile(projectFolder, 'output', 'static-u', '.steps09-16-runtime');
pythonExecutable = fullfile(getenv('USERPROFILE'), '.cache', 'codex-runtimes', ...
    'codex-primary-runtime', 'dependencies', 'python', 'python.exe');
if ~isfile(pythonExecutable)
    pythonExecutable = 'python';
end
[buildStatus, buildMessage] = system(sprintf('"%s" "%s" "%s"', pythonExecutable, builderPath, projectFolder));
if buildStatus ~= 0
    error('staticULesson:RuntimeBuildFailed', 'Cannot build teaching recorder: %s', buildMessage);
end
addpath(runtimeFolder, '-begin');
% The diagnostic copy shares this recording only; physical inputs use arguments.
global staticULessonTrace %#ok<GVMIS>
staticULessonTrace = struct('Steps', {cell(0, 1)}, 'Lines', {cell(0, 1)}, ...
    'Iterations', {cell(0, 1)}, 'Preparations', {cell(0, 1)});
[candidate, diagnostics] = staticULessonEngine.solve(startingPath, planningEnvironment, request, struct());
publicResult = planner(obstacles, request.initialState, request.goalState, request.limits, request.options);
if ~candidate.Success || ~publicResult.Success
    error('staticULesson:RecordingFailed', 'Static U did not return a complete successful motion.');
end
% Instrumentation writes records only. Compare the exact returned polynomial
% and its clock to detect any changed numerical outcome before using a trace.
if ~isequaln(candidate.Polynomial, publicResult.Diagnostics.Polynomial)
    error('staticULesson:TraceMismatch', 'Recorded polynomial differs from the public planner result.');
end
trace = staticULessonTrace;
trace.EngineCandidate = candidate;
trace.Diagnostics = diagnostics;
trace.PublicResult = publicResult;
trace.Environment = planningEnvironment;
savedTrace = trace;
savedInputs = recordInputs;
savedSources = sourceContents;
staticULessonTrace = [];
end
