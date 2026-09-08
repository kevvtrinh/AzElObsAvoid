function tests = testStaticCorridor
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testStaticCorridor.m')
% PURPOSE: Check static corridor symmetry and independent source validation.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Physical and collision checks of complete public planner results.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
    testCase.TestData.Vertices_units = [-1,-1;1,-1;1,1;-1,1];
end

function testAxisDirectionAndTranslation(testCase)
    transformations = cat(3,eye(2),-eye(2),[0,1;1,0],[0,-1;-1,0]);
    for k = 1:size(transformations,3)
        transform = transformations(:,:,k); shift_units = [7,-3];
        vertices_units = testCase.TestData.Vertices_units*transform+shift_units;
        obstacle = obstacleAvoidance.obstacles.createObstacle('rectangle',[0;30], ...
            vertices_units(:,1),vertices_units(:,2),0.1);
        initial = struct('time_s',0,'position_units',[-4,0]*transform+shift_units);
        goal = struct('time_s',30,'position_units',[4,0]*transform+shift_units);
        limits = struct('xInterval_units',[-20,20],'yInterval_units',[-20,20], ...
            'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[2,2],'maxJerk_units_s3',[4,4]);
        result = planner(obstacle,initial,goal,limits,struct('GoalTimeMode','earliestArrival'));
        verifyTrue(testCase,result.Success,result.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
        verifyEqual(testCase,result.SolverDiagnostics.Identifier,"monotoneStaticCorridor");
        verifyEqual(testCase,result.TrajectoryDuration_s,5.5,'AbsTol',1e-8);
        verifyEqual(testCase,result.PlaneCertificate.AllPairCount, ...
            result.Polynomial.SegmentCount*numel(result.PlaneCertificate.Regions_units));
        % Changing the authoritative source must invalidate a cached corridor.
        blocker = obstacleAvoidance.obstacles.createObstacle('new blocker',[0;30], ...
            shift_units(1)+[-3;3;3;-3],shift_units(2)+[-3;-3;3;3],0);
        result.Inputs.obstacles = blocker;
        verifyFalse(testCase,obstacleAvoidance.validateTrajectory(result).Passed);
    end
end
