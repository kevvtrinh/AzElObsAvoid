function tests = testPlannerOutputs
% Check compact results, optional evidence, and plots without rerunning planning.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    initial = struct();
    initial.time_s       = 0;
    initial.position_units = [0 0];
    goal = struct();
    goal.time_s       = 10;
    goal.position_units = [1 0];
    limits = struct();
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3         = [2 2];
    [result, diagnosis] = planner([], initial, goal, limits);
    single   = planner([], initial, goal, limits);
    obstacle = obstacleAvoidance.obstacles.createObstacle('blocked start', [0;10], [-1;1;1;-1], [-1;-1;1;1], 0);
    [failure, failureDiagnosis] = planner(obstacle, initial, goal, limits);
    testCase.TestData = struct('Result', result, 'Diagnosis', diagnosis, ...
        'Single', single, 'Failure', failure, 'FailureDiagnosis', failureDiagnosis);
end

function testOptionalOutputPreservesMotion(testCase)
    r      = testCase.TestData.Result;
    single = testCase.TestData.Single;
    verifyTrue(testCase, r.Success, r.Message);
    verifyEqual(testCase, r.time_s, single.time_s);
    verifyEqual(testCase, r.position_units, single.position_units);
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
    [options, diagnosis] = planner();
    verifyEqual(testCase, options, planner());
    verifyEmpty(testCase, fieldnames(diagnosis));
end

function testSolverEvidencePreservesStructuredRecords(testCase)
    d = testCase.TestData.Diagnosis;
    verifyEqual(testCase, numel(d.Routes), numel(d.Attempts));
    verifyFalse(testCase, isfield(d.Attempts, 'SolverDiagnostics'));
    verifyClass(testCase, d.SolverDetails, 'cell');
    verifyEqual(testCase, numel(d.SolverDetails), numel(d.Attempts));
    verifyClass(testCase, d.DirectMotion, 'struct');
    verifyClass(testCase, d.PathRefinement, 'struct');
    verifyNotEmpty(testCase, d.SolverDetails);
    % A nested certificate and failure record must survive assembly exactly.
    fixture = d.SolverDetails{d.SelectedAttemptIndex};
    verifyClass(testCase, fixture, 'struct');
    details = testSupport.flattenDiagnosis(fixture);
    verifyNotEmpty(testCase, details);
    verifyEqual(testCase, testSupport.solverDetails(d, d.SelectedAttemptIndex), details);
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

function testAssemblyPreservesEverySolverAndSearchRecord(testCase)
    r = testCase.TestData.Result;
    [record, summary] = obstacleAvoidance.planner.createPlanningRecord(r.Inputs.obstacles, r.Inputs.initialState, r.Inputs.goalState, r.Inputs.limits, r.Options, obstacleAvoidance.validateTrajectory());
    evidence = struct('Failed',true,'Certificate',struct('Normals',[1 0;0 1], ...
        'Intervals',[0 1;1 2],'Reasons',{{"unresolved", "clear"}}));
    summary.SolverDiagnostics = evidence;
    record.SeedSummaries = [summary summary];
    record.SearchDiagnostics.SelectionPolicy = struct();
    record.SearchDiagnostics.DirectAttempt = evidence;
    record.SearchDiagnostics.FixedClockExcursion = evidence;
    record.SearchDiagnostics.GraphSearch.VisibilityAttempts = [evidence evidence];
    [~, diagnosis] = obstacleAvoidance.planner.assemblePlannerOutputs(record, true);
    verifyEqual(testCase, diagnosis.SolverDetails, {evidence evidence});
    verifyEqual(testCase, diagnosis.DirectMotion, evidence);
    verifyEqual(testCase, diagnosis.PathRefinement, evidence);
    verifyEqual(testCase, diagnosis.VisibilityAttempts, [evidence evidence]);
end
