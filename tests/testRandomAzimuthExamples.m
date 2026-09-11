function tests = testRandomAzimuthExamples
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testRandomAzimuthExamples.m')
% PURPOSE: Check reproducible wide-azimuth inputs and paired obstacle semantics.
% INPUTS: MATLAB function-based test framework.
% OUTPUTS: Source-geometry, coverage, determinism, and input-validation checks.
% UNITS: Degrees and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'examples'));
    testCase.TestData.Root = root;
end

function testEightyDistinctPairedSlews(testCase)
    starts_deg = zeros(80,2); goals_deg = starts_deg;
    for caseIndex = 1:80
        movingOnly = createRandomAzimuthScenario(caseIndex,false);
        withStatic = createRandomAzimuthScenario(caseIndex,true);
        starts_deg(caseIndex,:) = movingOnly.InitialState.position_units;
        goals_deg(caseIndex,:) = movingOnly.GoalState.position_units;
        verifyGreaterThanOrEqual(testCase,abs(goals_deg(caseIndex,1)-starts_deg(caseIndex,1)),80);
        verifyEqual(testCase,movingOnly.InitialState,withStatic.InitialState);
        verifyEqual(testCase,movingOnly.GoalState,withStatic.GoalState);
        verifyEqual(testCase,movingOnly.Obstacles,withStatic.Obstacles(1));
        verifyNumElements(testCase,withStatic.Obstacles,2);
        obstacle = movingOnly.Obstacles;
        first_deg = [obstacle.x_units{1},obstacle.y_units{1}];
        final_deg = [obstacle.x_units{end},obstacle.y_units{end}];
        verifyEqual(testCase,final_deg-first_deg, ...
            repmat(180*movingOnly.MovingVelocity_deg_s,size(first_deg,1),1),'AbsTol',1e-12);
        verifyGreaterThan(testCase,norm(movingOnly.MovingVelocity_deg_s),0);
        verifyTrue(testCase,inpolygon(movingOnly.MovingCenterAtStart_deg(1), ...
            movingOnly.MovingCenterAtStart_deg(2),first_deg(:,1),first_deg(:,2)));
        cross_deg2 = det([goals_deg(caseIndex,:)-starts_deg(caseIndex,:); ...
            movingOnly.MovingCenterAtStart_deg-starts_deg(caseIndex,:)]);
        verifyLessThan(testCase,abs(cross_deg2),1e-10);
    end
    verifySize(testCase,unique(starts_deg,'rows'),[80,2]);
    verifySize(testCase,unique(goals_deg,'rows'),[80,2]);
    verifyTrue(testCase,any(goals_deg(:,1)>starts_deg(:,1)) && any(goals_deg(:,1)<starts_deg(:,1)));
end

function testLocalRandomnessAndInputDrivenCases(testCase)
    previous = rng;
    a = createRandomAzimuthScenario(17,true);
    verifyEqual(testCase,rng,previous);
    createRandomAzimuthScenario(80,false);
    verifyEqual(testCase,a,createRandomAzimuthScenario(17,true));
    b = createRandomAzimuthScenario(17,true,42);
    verifyNotEqual(testCase,a.InitialState.position_units,b.InitialState.position_units);
end

function testRejectInvalidInputs(testCase)
    verifyError(testCase,@()createRandomAzimuthScenario(0), ...
        'MATLAB:expectedPositive');
    verifyError(testCase,@()createRandomAzimuthScenario(1,1), ...
        'MATLAB:invalidType');
end

function testSuiteReturnsValidatedCoreResults(testCase)
    results = exampleRandomAzimuthSuite([2,53],struct('PlotOutputs',false));
    verifySize(testCase,results,[2,2]);
    for k = 1:numel(results)
        assertTrue(testCase,results{k}.Success,results{k}.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(results{k}).Passed);
        verifyEqual(testCase,results{k}.ArrivalTime_s,180,'AbsTol',1e-8);
    end
    single = exampleRandomAzimuth(6,true,struct('PlotOutputs',false));
    verifyTrue(testCase,single.Success,single.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(single).Passed);
end
