function tests = testBoundedJerk
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testBoundedJerk.m')
% PURPOSE: Check exact jerk-limited timing and independent C2 validation.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based tests.
% UNITS: Coordinate units, seconds, and physical derivatives.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
    testCase.TestData.Result = exampleObstacleFree(struct('PlotOutputs',false,'Verbose',false));
end

function testExactBenchmarkAndJerkJumps(testCase)
    r = testCase.TestData.Result;
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.TrajectoryDuration_s,0.5+sqrt(16.25),'AbsTol',1e-8);
    verifyEqual(testCase,r.MotionLength_units,sqrt(20),'AbsTol',1e-10);
    verifyEqual(testCase,r.SolverDiagnostics.TrajectorySocpCount,0);
    jerk = r.Polynomial.jerkPower_units_s3;
    jumps = sum(jerk(1:end-1,:,:),3)-jerk(2:end,:,1);
    verifyGreaterThan(testCase,max(abs(jumps),[],'all'),1.9);
    verifyLessThanOrEqual(testCase,max(abs(r.jerk_units_s3),[],'all'),2+1e-8);
end

function testJerkLimitStillRequired(testCase)
    r = testCase.TestData.Result;
    r.Limits.maxJerk_units_s3(1) = 1.9;
    validation = obstacleAvoidance.validateTrajectory(r);
    verifyTrue(testCase,validation.InterSegmentContinuous);
    verifyFalse(testCase,validation.JerkWithinLimits);
    verifyFalse(testCase,validation.Passed);
end

function testAccelerationJumpRejected(testCase)
    r = testCase.TestData.Result;
    p = r.Polynomial;
    p.positionPower_units(2,1,3) = p.positionPower_units(2,1,3)+1e-3;
    degree = p.Degree;
    p.velocityPower_units_s = p.positionPower_units(:,:,2:end).*reshape(1:degree,1,1,[])./p.SegmentDuration_s;
    p.accelerationPower_units_s2 = p.velocityPower_units_s(:,:,2:end).*reshape(1:degree-1,1,1,[])./p.SegmentDuration_s;
    p.jerkPower_units_s3 = p.accelerationPower_units_s2(:,:,2:end).*reshape(1:degree-2,1,1,[])./p.SegmentDuration_s;
    r.Polynomial = p;
    [~,r.position_units,r.velocity_units_s,r.acceleration_units_s2,r.jerk_units_s3] = bmtpEngine.evaluatePolynomial(p,r.time_s);
    validation = obstacleAvoidance.validateTrajectory(r);
    verifyTrue(testCase,validation.DynamicsConsistent);
    verifyTrue(testCase,validation.SampledHistoriesMatched);
    verifyFalse(testCase,validation.InterSegmentContinuous);
    verifyFalse(testCase,validation.Passed);
end

function testProfileRegimesAndAxes(testCase)
    % Triangular acceleration, velocity-limited triangular acceleration,
    % acceleration-limited cruise, and unequal axis limits on a reverse chord.
    displacements = [0.1,0;4,0;12,0;-3,6];
    velocity = [2,2;0.25,2;2,2;4,2];
    acceleration = [1,1;10,10;1,1;2,1];
    jerk = [2,2;2,2;2,2;4,2];
    expected_s = [4*(0.1/4)^(1/3);4/0.25+2*sqrt(0.25/2);8.5;5.5];
    for k = 1:4
        initial = struct('time_s',7,'position_units',[1,-2]);
        goal = struct('time_s',40,'position_units',initial.position_units+displacements(k,:));
        limits = struct('maxVelocity_units_s',velocity(k,:), ...
            'maxAcceleration_units_s2',acceleration(k,:),'maxJerk_units_s3',jerk(k,:));
        r = planner([],initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
        verifyTrue(testCase,r.Success,r.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
        verifyEqual(testCase,r.TrajectoryDuration_s,expected_s(k),'AbsTol',1e-8);
        verifyEqual(testCase,r.MotionLength_units,norm(displacements(k,:)),'AbsTol',1e-8);
    end
end
