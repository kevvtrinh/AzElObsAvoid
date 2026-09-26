function tests = testPolynomialSubdivision
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testPolynomialSubdivision.m')
% PURPOSE: Check that proof subdivision preserves the exported motion
%   after endpoint/continuity repair, including unequal spans and nonmidpoint cuts.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Independent validation and physical p/v/a/jerk preservation checks.
% UNITS: Coordinate units and seconds.
tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'));
    initial = struct( ...
        'position_units',        [0, 0], ...
        'velocity_units_s',      [1, 4], ...
        'acceleration_units_s2', [0, -8], ...
        'time_s',                11);
    goal = struct( ...
        'position_units',        [1, 0], ...
        'velocity_units_s',      [1, -4], ...
        'acceleration_units_s2', [0, -8], ...
        'time_s',                13);
    limits = struct( ...
        'xInterval_units',          [-2, 2], ...
        'yInterval_units',          [-2, 2], ...
        'maxVelocity_units_s',      [10, 10], ...
        'maxAcceleration_units_s2', [20, 20], ...
        'maxJerk_units_s3',         [100, 100]);
    options = struct( ...
        'GoalTimeMode',                      "earliestArrival", ...
        'ConstraintTolerance',               1e-8, ...
        'CollisionClearanceTolerance_units', 1e-6, ...
        'ArrivalTimeTolerance_s',             1e-8);
    % This box is below the parabola [u, 4u(1-u)], but inside the first
    % segment's control hull. Selective subdivision is needed to prove clearance.
    box_units = [0.12, 0.405; 0.14, 0.405; 0.14, 0.41; 0.12, 0.41];
    request = struct( ...
        'InitialState',    initial, ...
        'GoalState',       goal, ...
        'Limits',          limits, ...
        'Options',         options, ...
        'MotionHorizon_s', 2, ...
        'Regions_units',   {{box_units}}, ...
        'Coverage',        struct('Passed', true, 'ExactRegionCount', 1));
    wholePower_units = zeros(1, 2, 6);
    wholePower_units(1, :, 2) = [1, 4];
    wholePower_units(1, :, 3) = [0, -4];
    wholeControls_units = squeeze(bmtpEngine.motion.powerToBernstein(wholePower_units));
    durations_s = [0.4; 0.1; 0.5];
    boundaries = [0; cumsum(durations_s)];
    controls_units = zeros(3, 6, 2);
    for segmentIndex = 1:3
        controls_units(segmentIndex, :, :) = bmtpEngine.motion.restrictBezier( ...
            wholeControls_units, boundaries(segmentIndex:segmentIndex + 1).');
    end
    testCase.TestData.Request = request;
    testCase.TestData.Motion = bmtpEngine.motion.createMotion( ...
        request, controls_units, durations_s, [], false(3, 1));
end

function testPreparedCurveSurvivesSelectiveSubdivision(testCase)
    initial=struct('position_units',[-2,1],'time_s',11);
    goal=struct('position_units',[2,-1],'time_s',15);
    limits=struct('xInterval_units',[-5,5],'yInterval_units',[-5,5], ...
        'maxVelocity_units_s',[10,10],'maxAcceleration_units_s2',[20,20], ...
        'maxJerk_units_s3',[100,100]);
    base=planner([],initial,goal,limits,struct('GoalTimeMode','fixedArrival'));
    seed=struct('position_units',[initial.position_units;goal.position_units], ...
        'tau',[0;1]);
    request=bmtpEngine.prepareRequest(seed, ...
        struct('regions_units', {cell(0,1)}, ...
        'coverage', struct('Passed',true,'ExactRegionCount',0)), ...
        struct('initialState', base.Inputs.initialState, ...
        'goalState', base.Inputs.goalState, ...
        'limits', base.Diagnostics.Limits, ...
        'options', base.Options));
    durations_s=[1.3;0.7;2]; breaks=[0;cumsum(durations_s)]/4;
    [~,roundoffReserve_units]=bmtpEngine.validation.createCoordinateTolerances(base.Diagnostics.Route_units, ...
        limits.xInterval_units,limits.yInterval_units);
    target_units=(1+2^20*eps)*request.Options.CollisionClearanceTolerance_units+roundoffReserve_units;
    for degree=[5,8]
        fraction=zeros(degree+1,1); coefficients=[10,-15,6];
        for k=0:degree
            for j=3:min(k,5)
                fraction(k+1)=fraction(k+1)+coefficients(j-2)*nchoosek(k,j)/nchoosek(degree,j);
            end
        end
        whole=initial.position_units+fraction.*(goal.position_units-initial.position_units);
        controls_units=zeros(3,degree+1,2);
        for span=1:3
            controls_units(span,:,:)=bmtpEngine.motion.restrictBezier(whole,breaks(span:span+1).');
        end
        % The quintic export repairs this residual. Later proof must
        % preserve that repaired curve instead of projecting these controls again.
        if degree==5, controls_units(2,1,1)=controls_units(2,1,1)+1e-6; end
        source=bmtpEngine.motion.createMotion(request,controls_units,durations_s);
        original=resultWithPreparedMotion(base,request,source,roundoffReserve_units,target_units);
        assertTrue(testCase,obstacleAvoidance.validateTrajectory(original).Passed);
        verifyTrue(testCase, all(isfinite(source.GivenPower_units), 'all'));
        verifyEqual(testCase, source.SourceSegmentIndex, repelem((1:3).', 2));
        changed = source;
        splitFractions = [0.31; 0.5; 0.73];
        for refinementIndex = 1:3
            splitSegment = changed.SourceSegmentIndex ~= 2;
            [~, changed.SegmentTime_s, changed.GivenPower_units, parentSegmentIndex] = ...
                bmtpEngine.motion.subdivideMotion(changed.ControlPoint_units, changed.SegmentTime_s, ...
                changed.GivenPower_units, splitSegment, splitFractions(changed.SourceSegmentIndex));
            changed.SourceSegmentIndex = changed.SourceSegmentIndex(parentSegmentIndex);
            changed.ControlPoint_units = bmtpEngine.motion.powerToBernstein(changed.GivenPower_units);
        end
        output=resultWithPreparedMotion(base,request,changed,roundoffReserve_units,target_units);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(output).Passed);
        sampleTime_s=linspace(initial.time_s,goal.time_s,401).';
        [~,p,v,a,j]=bmtpEngine.motion.evaluatePolynomial(original.Diagnostics.Polynomial,sampleTime_s);
        [~,splitP,splitV,splitA,splitJ]=bmtpEngine.motion.evaluatePolynomial(output.Diagnostics.Polynomial,sampleTime_s);
        verifyEqual(testCase,[splitP,splitV,splitA,splitJ],[p,v,a,j],'AbsTol',1e-9);
        verifyEqual(testCase, output.ArrivalTime_s, original.ArrivalTime_s);
        unchangedPieces = changed.SourceSegmentIndex == 2;
        verifyEqual(testCase, changed.GivenPower_units(unchangedPieces, :, :), ...
            source.GivenPower_units(source.SourceSegmentIndex == 2, :, :));
    end
end

function testSuppliedZeroCoefficientsSurviveSubdivision(testCase)
    coefficients_units = zeros(1, 2, 9);
    coefficients_units(1, :, 1) = [2, -1];
    coefficients_units(1, :, 2) = [-3, 4];
    coefficients_units(1, :, 3) = [0.5, -0.8];
    coefficients_units(1, :, 4) = [0.25, -0.2];
    controls_units = bmtpEngine.motion.powerToBernstein(coefficients_units);
    [splitControls_units, durations_s, splitCoefficients_units, parentSegmentIndex] = ...
        bmtpEngine.motion.subdivideMotion(controls_units, 2, coefficients_units, true, 0.37);
    verifyEqual(testCase, parentSegmentIndex, [1; 1]);
    verifyEqual(testCase, splitCoefficients_units(:, :, 5:end), zeros(2, 2, 5));
    polynomial = bmtpEngine.motion.createPowerPolynomial( ...
        splitControls_units, durations_s, 11, splitCoefficients_units);
    sampleTime_s = linspace(11, 13, 101).';
    fraction = (sampleTime_s - 11) / 2;
    [~, position, velocity, acceleration, jerk] = ...
        bmtpEngine.motion.evaluatePolynomial(polynomial, sampleTime_s);
    expectedPosition = [2, -1] + fraction .* [-3, 4] + fraction.^2 .* [0.5, -0.8] + ...
        fraction.^3 .* [0.25, -0.2];
    expectedVelocity = ([-3, 4] + fraction .* [1, -1.6] + fraction.^2 .* [0.75, -0.6]) / 2;
    expectedAcceleration = ([1, -1.6] + fraction .* [1.5, -1.2]) / 4;
    expectedJerk = repmat([1.5, -1.2] / 8, numel(fraction), 1);
    verifyEqual(testCase, [position, velocity, acceleration, jerk], ...
        [expectedPosition, expectedVelocity, expectedAcceleration, expectedJerk], 'AbsTol', 1e-12);

    % An unspecified axis must stay unspecified until initial preparation
    % converts its controls. Splitting must not manufacture coefficients.
    coefficients_units(1, 2, :) = NaN;
    [~, ~, splitCoefficients_units] = bmtpEngine.motion.subdivideMotion( ...
        controls_units, 2, coefficients_units, true, 0.37);
    verifyTrue(testCase, all(isnan(splitCoefficients_units(:, 2, :)), 'all'));

    % A mask, duration list or coefficient array that does not match the
    % segment count, or a split fraction outside (0, 1), is refused rather
    % than silently dropping segments.
    twoSegments_units = repmat(controls_units, 2, 1);
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(twoSegments_units, [2; 2], [], false), ...
        'subdivideMotion:InvalidInput');
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(twoSegments_units, 2, [], [false; false]), ...
        'subdivideMotion:InvalidInput');
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(twoSegments_units, [2; 2], coefficients_units, [true; false]), ...
        'subdivideMotion:InvalidInput');
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(twoSegments_units, [2; 2], [], [true; false], [1; 0.5]), ...
        'subdivideMotion:InvalidSplitProgress');
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(controls_units, 2, [], true, NaN), ...
        'subdivideMotion:InvalidSplitProgress');
    % A fraction that leaves one piece no time at all is refused too.
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(controls_units, realmin, [], true, realmin), ...
        'subdivideMotion:InvalidSplitProgress');
end

function testRefinementKeepsCurveTimingAndSourceSegments(testCase)
    source = testCase.TestData.Motion;
    assertTrue(testCase, source.Success);
    for obstacleMoves = [false, true]
        request = testCase.TestData.Request;
        if obstacleMoves
            request.Coverage.ActiveTimeInterval_s = [11, 13];
            request.Coverage.EndRegions_units = {request.Regions_units{1} + [0.01, 0]};
        end
        [firstCheck, savedPairChecks] = bmtpEngine.validation.checkFinalMotion(request, source, 1e-8, 1e-6);
        assertFalse(testCase, firstCheck.Passed);
        assertTrue(testCase, firstCheck.WorkspacePassed && firstCheck.DynamicsPassed && firstCheck.ContinuityPassed);
        [changed, finalCheck] = bmtpEngine.validation.checkMotionWithSubdivision( ...
            request, source, 1e-8, 1e-6, savedPairChecks);
        verifyTrue(testCase, finalCheck.Passed);
        verifyGreaterThanOrEqual(testCase, nnz(changed.SourceSegmentIndex == 1), 3);
        verifyEqual(testCase, changed.SourceSegmentIndex(end - 1:end), [2; 3]);
        verifyEqual(testCase, changed.GivenPower_units(end - 1:end, :, :), source.GivenPower_units(2:3, :, :));
        verifyGreaterThanOrEqual(testCase, finalCheck.CachedPairCount, 2);
        % The split polynomial sets required times; assigned times and final time stay fixed.
        requiredTime_s = bmtpEngine.motion.findRequiredPolynomialTime( ...
            changed.GivenPower_units, changed.SegmentTime_s, request.Limits);
        verifyEqual(testCase, changed.RequiredSegmentTime_s, requiredTime_s);
        verifyEqual(testCase, changed.MotionProof.SegmentTime_s, changed.SegmentTime_s);
        verifyEqual(testCase, changed.MotionProof.Passed, all(changed.SegmentTime_s >= requiredTime_s));
        verifyEqual(testCase, changed.MotionProof.MaximumViolation, max([0; requiredTime_s - changed.SegmentTime_s]));
        verifyEqual(testCase, changed.FinalTime_s, source.FinalTime_s);
        verifyEqual(testCase, changed.DilationScale, source.DilationScale);
        verifyEqual(testCase, changed.ContinuityProjectionDisplacement_units, ...
            source.ContinuityProjectionDisplacement_units);
        verifyEqual(testCase, accumarray(changed.SourceSegmentIndex, changed.SegmentTime_s), ...
            source.SegmentTime_s, 'AbsTol', 1e-14);

        original = bmtpEngine.motion.createPowerPolynomial(source.ControlPoint_units, ...
            source.SegmentTime_s, request.InitialState.time_s, source.GivenPower_units, source.FinalTime_s);
        subdivided = bmtpEngine.motion.createPowerPolynomial(changed.ControlPoint_units, ...
            changed.SegmentTime_s, request.InitialState.time_s, changed.GivenPower_units, changed.FinalTime_s);
        sampleTime_s = unique([linspace(11, source.FinalTime_s, 401).'; subdivided.SegmentStartTime_s]);
        [~, position, velocity, acceleration, jerk] = bmtpEngine.motion.evaluatePolynomial(original, sampleTime_s);
        [~, splitPosition, splitVelocity, splitAcceleration, splitJerk] = ...
            bmtpEngine.motion.evaluatePolynomial(subdivided, sampleTime_s);
        verifyEqual(testCase, [splitPosition, splitVelocity, splitAcceleration, splitJerk], ...
            [position, velocity, acceleration, jerk], 'AbsTol', 1e-9);
    end
end

function testOtherFailuresDoNotTriggerSubdivision(testCase)
    for failedCheck = ["WorkspacePassed", "DynamicsPassed", "ContinuityPassed"]
        request = testCase.TestData.Request;
        source = testCase.TestData.Motion;
        if failedCheck == "WorkspacePassed"
            request.Limits.xInterval_units = [-2, 0.9];
        elseif failedCheck == "DynamicsPassed"
            request.Limits.maxVelocity_units_s = [0.1, 0.1];
        else
            source.GivenPower_units(2, 1, 1) = source.GivenPower_units(2, 1, 1) + 0.01;
            source.ControlPoint_units = bmtpEngine.motion.powerToBernstein(source.GivenPower_units);
        end
        [changed, motionCheck] = bmtpEngine.validation.checkMotionWithSubdivision(request, source, 1e-8, 1e-6);
        verifyFalse(testCase, motionCheck.Passed);
        verifyFalse(testCase, motionCheck.(failedCheck));
        verifyEqual(testCase, changed, source);
    end
end

function testCollisionRemainsRejected(testCase)
    request = testCase.TestData.Request;
    % The same parabola now enters this box near u = 0.13. Refining its
    % representation must not repair or move the curve to obtain a proof.
    request.Regions_units = {[0.12, 0.42; 0.14, 0.42; 0.14, 0.5; 0.12, 0.5]};
    source = testCase.TestData.Motion;
    [changed, motionCheck] = bmtpEngine.validation.checkMotionWithSubdivision(request, source, 1e-8, 1e-6);
    verifyFalse(testCase, motionCheck.Passed);
    verifyLessThan(testCase, motionCheck.VerifiedPairCount, motionCheck.AllPairCount);
    verifyEqual(testCase, changed.FinalTime_s, source.FinalTime_s);
    verifyEqual(testCase, changed.DilationScale, source.DilationScale);
end

function result = resultWithPreparedMotion(baseResult, request, preparedMotion, ...
        roundoffReserve_units, target_units)
    % Put a constructed BMTP motion in the planner's nested record for validation.
    motionOutput = bmtpEngine.createMotionOutput(struct(), request, preparedMotion);
    result = baseResult;
    result.time_s                = motionOutput.time_s;
    result.position_units        = motionOutput.position_units;
    result.velocity_units_s      = motionOutput.velocity_units_s;
    result.acceleration_units_s2 = motionOutput.acceleration_units_s2;
    result.jerk_units_s3         = motionOutput.jerk_units_s3;
    result.ArrivalTime_s         = motionOutput.ArrivalTime_s;
    result.MotionLength_units    = motionOutput.MotionLength_units;
    result.Diagnostics.Polynomial                      = motionOutput.Polynomial;
    result.Diagnostics.TrajectoryDuration_s            = motionOutput.TrajectoryDuration_s;
    result.Diagnostics.IntegratedSquaredJerk_units2_s5 = motionOutput.IntegratedSquaredJerk_units2_s5;
    result.Diagnostics.MaximumConstraintViolation      = motionOutput.MaximumConstraintViolation;
    result.Diagnostics.SeparationProof = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, target_units);
    result.Diagnostics.Validation = struct("Passed", false, ...
        "Message", "Synthetic motion has not been validated.");
end
