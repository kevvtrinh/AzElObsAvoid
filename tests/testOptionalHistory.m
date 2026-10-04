function tests = testOptionalHistory
% Validate the optional owner-provided az/el history independently.
% Run with runtests('tests/testOptionalHistory.m').
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testStandInRingChangeWindowPlans(testCase)
    root        = fileparts(fileparts(mfilename('fullpath')));
    historyPath = fullfile(root, 'tmp', 'azel', 'azel_history.mat');
    assumeTrue(testCase, isfile(historyPath), ...
        'The optional tmp/azel/azel_history.mat stand-in is unavailable.');

    loaded       = load(historyPath, 'obstacles');
    initialState = struct('time_s', 300, 'position_units', [10, 0]);
    goalState    = struct('time_s', 530, 'position_units', [110, 10]);
    limits = struct( ...
        'xInterval_units',          [-180, 180], ...
        'yInterval_units',          [-90, 90], ...
        'maxVelocity_units_s',      [2, 2], ...
        'maxAcceleration_units_s2', [0.5, 0.5], ...
        'maxJerk_units_s3',         [1, 1]);
    options = struct('GoalTimeMode', 'fixedArrival');

    wallTimer  = tic;
    result     = planner(loaded.obstacles, initialState, goalState, limits, options);
    wallTime_s = toc(wallTimer);
    validation = obstacleAvoidance.validateTrajectory(result);
    fprintf(['[ring-count regression] t=[300,530] wall %.2f s, success %d, ', ...
        '%s, valid %d; %s\n'], wallTime_s, result.Success, ...
        result.TerminationReason, validation.Passed, result.Message);

    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
end
