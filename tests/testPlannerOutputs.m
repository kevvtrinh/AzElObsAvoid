function tests = testPlannerOutputs
% Check compact results, optional evidence, and plots without rerunning planning.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    initial = struct();
    initial.time_s       = 0;
    initial.position_deg = [0 0];
    goal = struct();
    goal.time_s       = 10;
    goal.position_deg = [1 0];
    limits = struct();
    limits.maxVelocity_deg_s      = [2 2];
    limits.maxAcceleration_deg_s2 = [1 1];
    limits.maxJerk_deg_s3         = [2 2];
    [result, diagnosis] = obstacleAvoidance.planTrajectory([], initial, goal, limits);
    single   = obstacleAvoidance.planTrajectory([], initial, goal, limits);
    obstacle = obstacleAvoidance.obstacles.createObstacle('blocked start', [0;10], [-1;1;1;-1], [-1;-1;1;1], 0);
    [failure, failureDiagnosis] = obstacleAvoidance.planTrajectory(obstacle, initial, goal, limits);
    testCase.TestData = struct('Result', result, 'Diagnosis', diagnosis, ...
        'Single', single, 'Failure', failure, 'FailureDiagnosis', failureDiagnosis);
end

function testOptionalOutputPreservesMotion(testCase)
    r      = testCase.TestData.Result;
    single = testCase.TestData.Single;
    verifyTrue(testCase, r.Success, r.Message);
    verifyEqual(testCase, r.time_s, single.time_s);
    verifyEqual(testCase, r.position_deg, single.position_deg);
    verifyEqual(testCase, r.Polynomial, single.Polynomial);
    validation = obstacleAvoidance.validateTrajectory(r);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyFalse(testCase, any(isfield(r, {'Seeds','SeedSummaries', 'SearchDiagnostics','SelectedSeedIndex','GoalHorizon_s'})));
end

function testStableSuccessAndFailureSchemas(testCase)
    verifyEqual(testCase, fieldnames(testCase.TestData.Result), fieldnames(testCase.TestData.Failure));
    verifyEqual(testCase, fieldnames(testCase.TestData.Diagnosis), fieldnames(testCase.TestData.FailureDiagnosis));
    verifyFalse(testCase, testCase.TestData.Failure.Success);
    verifyEqual(testCase, testCase.TestData.Failure.TerminationReason, "endpointBlocked");
    [options, diagnosis] = obstacleAvoidance.planTrajectory();
    verifyEqual(testCase, options, obstacleAvoidance.planTrajectory());
    verifyEmpty(testCase, fieldnames(diagnosis));
end

function testSolverEvidenceHasNoNestedStructures(testCase)
    d = testCase.TestData.Diagnosis;
    verifyEqual(testCase, numel(d.Routes), numel(d.Attempts));
    verifyFalse(testCase, isfield(d.Attempts, 'SolverDiagnostics'));
    for details = {d.SolverDetails, d.DirectMotion, d.PathRefinement}
        values = details{1}.Value;
        for index = 1:numel(values)
            verifyFalse(testCase, isstruct(values{index}));
            verifyFalse(testCase, iscell(values{index}));
        end
    end
    verifyNotEmpty(testCase, d.SolverDetails);
end

function testPlotWithAndWithoutDiagnosis(testCase)
    cleanup = onCleanup(@() close('all', 'force')); %#ok<NASGU>
    options = struct();
    options.FigureVisible  = 'off';
    options.ShowKinematics = false;
    options.ShowAnimation  = false;
    handles = obstacleAvoidance.plotting.plotTrajectory(testCase.TestData.Result, options);
    verifyTrue(testCase, isgraphics(handles.WorkspaceAxes));
    handles = obstacleAvoidance.plotting.plotTrajectory(testCase.TestData.Failure, options, testCase.TestData.FailureDiagnosis);
    verifyTrue(testCase, isgraphics(handles.WorkspaceAxes));
end
