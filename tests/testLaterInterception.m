function tests = testLaterInterception
%% Section 0: Header & Readme
% SYNTAX: tests = testLaterInterception
% PURPOSE: Preserve chronological interception after an early meeting is blocked.
% INPUTS: None; run from the implementation root being tested.
% OUTPUTS: MATLAB function tests.
% UNITS: Coordinate units and seconds.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    % Using the current implementation lets the same test establish the pinned
    % baseline before a replacement is installed on the refactor branch.
    plannerPath = which('planner');
    if isempty(plannerPath)
        root = fileparts(fileparts(mfilename('fullpath')));
    else
        root = fileparts(plannerPath);
    end
    addpath(root,fullfile(root,'trajectory'));
end

function testEarlyBlockedMeetingDoesNotEndTheSearch(testCase)
    limits = struct('maxVelocity_units_s',[2 2], ...
        'maxAcceleration_units_s2',[2 2], 'maxJerk_units_s3',[4 4]);
    relativeArrival_s = zeros(2,1);
    caseIndex = 0;
    for origin_s = [0 7]
        initial = struct('time_s',origin_s,'position_units',[-2 0]);
        target = struct('time_s',origin_s+[0;8], 'position_units',[2 0;2.4 0], 'InterpolationMethod',"linear");
        options = struct('ArrivalTimeTolerance_s',1e-4);
        goal = struct('time_s',origin_s+8,'targetMotion',target);
        unblocked = planner([],initial,goal,limits,options);
        verifyTrue(testCase,unblocked.Success);
        verifyLessThan(testCase,unblocked.Intercept.Time_s,origin_s+5);
        obstacle = obstacleAvoidance.obstacles.createObstacle("active through early meetings",origin_s+[0;5], ...
            [1;3;3;1],[-1;-1;1;1],0);
        [result,diagnosis] = planner(obstacle,initial,goal,limits,options);
        verifyTrue(testCase,result.Success,result.Message);
        verifyGreaterThan(testCase,result.Intercept.Time_s,origin_s+5);
        % The route must also enter the cleared region after deactivation;
        % target visibility at five seconds is not a feasible-arrival bound.
        verifyLessThanOrEqual(testCase,result.Intercept.Time_s,origin_s+8);
        caseIndex = caseIndex+1;
        relativeArrival_s(caseIndex) = result.Intercept.Time_s-origin_s;
        check = obstacleAvoidance.validateTrajectory(result);
        verifyTrue(testCase,check.Passed,check.Message);
        verifyGreaterThan(testCase,diagnosis.InterceptSearch.TrialCount,1);
    end
    verifyEqual(testCase,relativeArrival_s(1),relativeArrival_s(2),'AbsTol',1e-4);
end
