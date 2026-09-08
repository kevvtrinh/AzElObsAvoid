function tests = testPreparedGeometryReuse
%% Section 0: Header & Readme
% SYNTAX: tests = testPreparedGeometryReuse
% PURPOSE: Compare source-derived geometry reuse with the original query path.
% INPUTS: None.
% OUTPUTS: MATLAB function tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
end

function testAllGeometryAndActivityDecisionsMatchReference(testCase)
    square = [0 0; 4 0; 4 4; 0 4];
    concave = [0 0; 4 0; 4 1; 1 1; 1 4; 0 4];
    hole = [square; NaN NaN; 1 1; 1 3; 3 3; 3 1];
    disconnected = [square; NaN NaN; square + [8 0]];
    histories = {{square,square}, {concave,concave}, {hole,hole}, ...
        {disconnected,disconnected}, {square,square+[2 1]}, ...
        {square,1.5*square}, {concave,square}, {zeros(0,2),square}};
    for k = 1:numel(histories)
        vertices = histories{k};
        obstacle = obstacleAvoidance.obstacles.createObstacle("reuse", [7;11], ...
            {vertices{1}(:,1);vertices{2}(:,1)}, {vertices{1}(:,2);vertices{2}(:,2)}, 0);
        obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
        for time_s = [6 7 7.1 9 10.9 11 12]
            for geometryOnly = [false true]
                [shape, geometry] = obstacleAvoidance.obstacles.preparedShapeAtTime(obstacle,time_s,geometryOnly);
                [referenceShape, reference] = preparedShapeAtTimeReference(obstacle,time_s,geometryOnly);
                verifyEqual(testCase, geometry, reference);
                verifyEqual(testCase, shape, referenceShape);
            end
        end
    end
end

function testSourceChangesInvalidateAllCachedGeometry(testCase)
    obstacle = obstacleAvoidance.obstacles.createObstacle("changed", [0;4], [0;2;2;0], [0;0;2;2], 0.1);
    obstacle = obstacleAvoidance.obstacles.prepareObstacles(obstacle);
    for changeIndex = 1:4
        changed = obstacle;
        if changeIndex == 1
            changed.time_s = changed.time_s + 10;
        elseif changeIndex == 2
            changed.x_units{2} = changed.x_units{2} + 1;
        elseif changeIndex == 3
            changed.safetyMargin_units = 0.2;
        else
            changed.originalX_units{1} = changed.originalX_units{1} + 1;
        end
        fresh = rmfield(changed, 'InternalPreparation');
        changed = obstacleAvoidance.obstacles.prepareObstacles(changed);
        fresh = obstacleAvoidance.obstacles.prepareObstacles(fresh);
        verifyEqual(testCase, changed.InternalPreparation, fresh.InternalPreparation);
        verifyNotEqual(testCase, changed.InternalPreparation.SourceSnapshot, obstacle.InternalPreparation.SourceSnapshot);
    end
end

function testPublicValidatorRebuildsCallerSuppliedCaches(testCase)
    initial = struct('time_s',0,'position_units',[0 0]);
    goal = struct('time_s',6,'position_units',[4 0]);
    limits = struct('maxVelocity_units_s',[2 2], 'maxAcceleration_units_s2',[2 2], 'maxJerk_units_s3',[4 4]);
    result = obstacleAvoidance.planTrajectory([],initial,goal,limits,struct('GoalTimeMode',"fixedArrival"));
    verifyTrue(testCase,result.Success);
    blocked = obstacleAvoidance.obstacles.createObstacle("blocked",[0;6],[1;3;3;1],[-1;-1;1;1],0);
    far = obstacleAvoidance.obstacles.createObstacle("far",[0;6],[11;13;13;11],[-1;-1;1;1],0);
    blocked = obstacleAvoidance.obstacles.prepareObstacles(blocked);
    far = obstacleAvoidance.obstacles.prepareObstacles(far);
    snapshot = blocked.InternalPreparation.SourceSnapshot;
    blocked.InternalPreparation = far.InternalPreparation;
    blocked.InternalPreparation.SourceSnapshot = snapshot;
    check = obstacleAvoidance.validateTrajectory(result,blocked,result.Inputs.initialState,result.Inputs.goalState,result.Inputs.limits,result.Options);
    verifyFalse(testCase,check.Passed);
    verifyFalse(testCase,check.CollisionFree);
end
