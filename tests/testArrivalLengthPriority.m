function tests = testArrivalLengthPriority
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testArrivalLengthPriority.m')
% PURPOSE: Reproduce long arrival and long path reports through the public core.
% INPUTS: MATLAB unit test framework; literal original sandbox geometry below.
% OUTPUTS: Independent validation, arrival, and continuous arc-length checks.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    testCase.TestData.Initial = struct('time_s',0, ...
        'position_units',[-92.48768720474419,14.668308372700778]);
    testCase.TestData.Goal = struct('time_s',180, ...
        'position_units',[49.645190471404135,32.036888129460245]);
    testCase.TestData.Limits = struct('xInterval_units',[-180,180],'yInterval_units',[-90,90], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
        'maxJerk_units_s3',[2.5,2.5]);
    % Original, unprotected vertices from pathtoolong.mat. Apply its margin once.
    vertices_units = [-61.803196301135785, 16.694642677656027;
        -62.197742970298435, 19.167068252743334;
        -63.354495274302195, 21.471002110764886;
        -65.194622455252755, 23.449434966727246;
        -67.592722886722285, 24.967539891011455;
        -70.38536998735448, 25.921860541814553;
        -73.382249472308786, 26.247361543873751;
        -76.379128957263106, 25.921860541814553;
        -79.171776057895286, 24.967539891011459;
        -81.56987648936483, 23.449434966727246;
        -83.410003670315376, 21.471002110764886;
        -84.566755974319136, 19.167068252743338;
        -84.961302643481787, 16.694642677656027;
        -84.566755974319136, 14.222217102568722;
        -83.410003670315376, 11.918283244547167;
        -81.56987648936483, 9.9398503885848104;
        -79.461252387174582, 5.1155895064830759;
        -76.379128957263106, 7.4674248134975016;
        -73.382249472308786, 7.1419238114383035;
        -70.38536998735448, 7.4674248134975016;
        -67.592722886722285, 8.4217454643005976;
        -65.19462245525277, 9.9398503885848069;
        -63.354495274302195, 11.918283244547162;
        -62.197742970298435, 14.222217102568713];
    testCase.TestData.Obstacle = obstacleAvoidance.obstacles.createObstacle('early detour', ...
        [0;180],vertices_units(:,1),vertices_units(:,2),0.2);
end

function testClearChordUsesLimitsInsteadOfMinimumJerkClock(testCase)
    data = testCase.TestData;
    result = planner([],data.Initial,data.Goal,data.Limits,struct('GoalTimeMode','earliestArrival'));
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    % The limiting x-axis has cruise speed 2, acceleration .75, jerk 2.5.
    % Its bang-jerk lower bound is d/v + v/a + a/j. C3 smoothing adds .06 s.
    lowerBound_s = (data.Goal.position_units(1)-data.Initial.position_units(1))/2+2/0.75+0.75/2.5;
    verifyEqual(testCase,result.ArrivalTime_s,lowerBound_s+0.06,'AbsTol',1e-8);
    verifyEqual(testCase,result.MotionLength_units, ...
        norm(data.Goal.position_units-data.Initial.position_units),'AbsTol',1e-8);
end

function testShortenEarlyDetourWithoutSpendingArrivalTime(testCase)
    data = testCase.TestData;
    options = struct('GoalTimeMode','earliestArrival');
    result = planner(data.Obstacle,data.Initial,data.Goal,data.Limits,options);
    verifyTrue(testCase,result.Success,result.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    verifyLessThan(testCase,result.ArrivalTime_s,74.094);
    verifyLessThan(testCase,result.MotionLength_units,148);
    refinement = result.SolverDiagnostics.PathLengthRefinement;
    verifyTrue(testCase,refinement.Accepted);
    verifyLessThan(testCase,refinement.ArrivalCost_s,1e-8);
    % Disabling the allowed trade keeps the same earliest-clock solution.
    options.PathLengthTimeAllowance_s = 0;
    strict = planner(data.Obstacle,data.Initial,data.Goal,data.Limits,options);
    verifyTrue(testCase,strict.Success,strict.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(strict).Passed);
    verifyEqual(testCase,strict.ArrivalTime_s,result.ArrivalTime_s,'AbsTol',1e-8);
    verifyEqual(testCase,strict.MotionLength_units,result.MotionLength_units,'AbsTol',1e-8);
end

function testArrivalTradeIsStrictlyLessThanHalfASecond(testCase)
    data = testCase.TestData;
    for allowance_s = [-0.01,0.5,1,NaN,Inf]
        options = struct('GoalTimeMode','earliestArrival','PathLengthTimeAllowance_s',allowance_s);
        verifyError(testCase,@() planner([],data.Initial,data.Goal,data.Limits,options), ...
            'planner:InvalidPathLengthTimeAllowance');
    end
end
