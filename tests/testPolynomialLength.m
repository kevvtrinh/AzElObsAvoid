function tests = testPolynomialLength
% Verify executable length and selection independently of display sampling.
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'trajectory'));
end

function testAdaptiveLengthAgreesWithIndependentIntegration(testCase)
    previous = rng(24719,'twister'); cleanup = onCleanup(@() rng(previous));
    for degree = [3 5 8]
        for scale = [1e-5 1 1e5]
            for trial = 1:20
                polynomial = struct('positionPower_units',scale*randn(4,2,degree+1));
                expected = referenceLength(polynomial);
                actual = bmtpEngine.measurePolynomialLength(polynomial);
                verifyEqual(testCase,actual,expected,'AbsTol',1e-9*max(1,expected));
                shifted = polynomial;
                shifted.positionPower_units(:,:,1) = shifted.positionPower_units(:,:,1)+1e9;
                verifyEqual(testCase,bmtpEngine.measurePolynomialLength(shifted),actual);
            end
        end
    end
end

function testReversalStationaryMotionAndBatchBoundary(testCase)
    polynomial.positionPower_units = zeros(1,2,3);
    polynomial.positionPower_units(1,1,:) = [0 1 -1];
    verifyEqual(testCase,bmtpEngine.measurePolynomialLength(polynomial),.5,'AbsTol',1e-11);
    polynomial.positionPower_units(:) = 0;
    verifyEqual(testCase,bmtpEngine.measurePolynomialLength(polynomial),0);
    polynomial.positionPower_units = zeros(2051,2,2);
    polynomial.positionPower_units(:,1,2) = 1;
    verifyEqual(testCase,bmtpEngine.measurePolynomialLength(polynomial),2051,'AbsTol',1e-10);
end

function testCurveRankingIsIndependentOfCoarseSamples(testCase)
    initial = struct('time_s',7,'position_units',[0 0],'velocity_units_s',[0 0],'acceleration_units_s2',[0 0]);
    options = obstacleAvoidance.input.resolvePlannerOptions();
    for step = [2 .002]
        summaries = repmat(struct('ValidationPassed',true,'ArrivalTime_s',8,'MotionLength_units',NaN),2,1);
        for index = 1:2
            controls = zeros(1,9,2);
            controls(1,:,1) = [0 0 0 .25 .5 .75 1 1 1];
            controls(1,4:6,2) = .8/index;
            polynomial = bmtpEngine.createPowerPolynomial(controls,1,7);
            candidate = bmtpEngine.createMotionRecord(struct(),initial,polynomial,[],step,"lengthTest");
            summaries(index).MotionLength_units = candidate.MotionLength_units;
            verifyEqual(testCase,candidate.MotionLength_units,referenceLength(polynomial),'AbsTol',1e-10);
            if step == 2
                verifyEqual(testCase,sum(vecnorm(diff(candidate.position_units),2,2)),1,'AbsTol',1e-12);
            end
        end
        for mode = ["fixedArrival","earliestArrival"]
            options.GoalTimeMode = mode;
            selection = obstacleAvoidance.planner.selectValidatedCandidate(summaries,options);
            verifyEqual(testCase,selection.SelectedCandidateIndex,2);
        end
        % Earliest arrival remains primary even when the earlier curve is longer.
        summaries(1).ArrivalTime_s = 7.99;
        selection = obstacleAvoidance.planner.selectValidatedCandidate(summaries,options);
        verifyEqual(testCase,selection.SelectedCandidateIndex,1);
    end
end

function testPlannerKeepsItsMotionWhenOnlySamplingChanges(testCase)
    initial = struct('time_s',7,'position_units',[-5 0]);
    goal = struct('time_s',19,'position_units',[5 0]);
    limits = struct('maxVelocity_units_s',[2 3],'maxAcceleration_units_s2',[1 2],'maxJerk_units_s3',[2 4],'xInterval_units',[-20 20],'yInterval_units',[-20 20]);
    obstacle = obstacleAvoidance.obstacles.createObstacle("sampling detour",7,[-.2;.2;.2;-.2],[-.3;-.3;.3;.3],0);
    for mode = ["earliestArrival","fixedArrival"]
        [coarse, coarseDiagnosis] = planner(obstacle,initial,goal,limits,struct('GoalTimeMode',mode,'SampleTime_s',.5));
        [fine, fineDiagnosis] = planner(obstacle,initial,goal,limits,struct('GoalTimeMode',mode,'SampleTime_s',.01));
        verifyTrue(testCase,coarse.Success,coarse.Message);
        verifyTrue(testCase,fine.Success,fine.Message);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(coarse).Passed);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(fine).Passed);
        verifyEqual(testCase,coarse.Polynomial,fine.Polynomial);
        verifyEqual(testCase,coarseDiagnosis.SelectedAttemptIndex,fineDiagnosis.SelectedAttemptIndex);
        verifyEqual(testCase,coarseDiagnosis.Attempts(coarseDiagnosis.SelectedAttemptIndex).MotionLength_units,fineDiagnosis.Attempts(fineDiagnosis.SelectedAttemptIndex).MotionLength_units);
    end
end

function length_units = referenceLength(polynomial)
    length_units = 0;
    for span = 1:size(polynomial.positionPower_units,1)
        coefficients = reshape(polynomial.positionPower_units(span,:,:),2,[]);
        derivative = coefficients(:,2:end).*(1:size(coefficients,2)-1);
        speed = @(tau) hypot(polyval(fliplr(derivative(1,:)),tau),polyval(fliplr(derivative(2,:)),tau));
        length_units = length_units+integral(speed,0,1,'AbsTol',1e-12,'RelTol',1e-12);
    end
end
