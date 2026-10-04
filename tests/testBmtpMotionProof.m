function tests = testBmtpMotionProof
%% Section 0: Header & Readme
% SYNTAX: results = runtests('tests/testBmtpMotionProof.m')
% PURPOSE: Preserve BMTP curves, continuous separation proofs, and validation.
% INPUTS: MATLAB unit test framework.
% OUTPUTS: Function-based test results with independently named cases.
% UNITS: Coordinate units and seconds.

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(~)
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(root, fullfile(root, 'trajectory'), fullfile(root, 'examples'));
end

function testPolynomialSubdivisionContracts(testCase)
    fixture = quadraticFixture();
    runCases(testCase, { ...
        @preparedCurveSurvivesSelectiveSubdivision; @clearQuadraticRetainsProofDepth; ...
        @suppliedZeroCoefficientsSurviveSubdivision; ...
        @(testCase) refinementKeepsCurveTimingAndSourceSegments(testCase, fixture); ...
        @(testCase) otherFailuresDoNotTriggerSubdivision(testCase, fixture); ...
        @(testCase) collisionRemainsRejected(testCase, fixture)});
end

function testSeparatingPlaneContracts(testCase)
    runCases(testCase, { ...
        @movingPlaneProvesTranslation; @staticPlaneMatchesStationaryAffineRepresentation; ...
        @cachedMovingGeometryMatchesUncachedSearch; @interiorObstaclePlaneViolationRejected; ...
        @planeSearchUsesProvenProductHull; @batchMatchesScalarVerification; @malformedRegionIsRejected; ...
        @affinePlaneBlockConsolidation; @affinePlaneNeedsOneEndpointWeight; ...
        @generatedPlaneRowsMatchExactOmissionOracle; @retainedPlaneCannotHideAnotherViolatedPair});
end

function testProofReuseContracts(testCase)
    fixture = proofReuseFixture();
    runCases(testCase, { ...
        @finalProofRechecksNeighborDirections; @(testCase) unchangedCurveReusesCompleteProof(testCase, fixture); ...
        @(testCase) sourceAndCurveChangesInvalidateReuse(testCase, fixture)});
end

function testIndependentValidationContracts(testCase)
    result = validationFixture();
    runCases(testCase, { ...
        @alteredEndpointCoverageRejected; @(testCase) coverageMetadataCannotExcludeTheMotion(testCase, result); ...
        @(testCase) continuityProjectionMayOnlyAbsorbRoundoff(testCase, result); ...
        @(testCase) incompleteSuccessfulRecordFailsWithoutThrowing(testCase, result); ...
        @timeToleranceIsIndependentOfConstraintTolerance; @detourAndTampering});
end

function preparedCurveSurvivesSelectiveSubdivision(testCase)
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
        source = bmtpEngine.motion.createMotion(request, controls_units, durations_s);
        original=resultWithPreparedMotion(base,request,source,roundoffReserve_units,target_units);
        assertTrue(testCase,obstacleAvoidance.validateTrajectory(original).Passed);
        verifyTrue(testCase, all(isfinite(source.GivenPower_units), 'all'));
        verifyEqual(testCase, source.SourceSegmentIndex, (1:3).');
        changed = source;
        for refinementIndex = 1:3
            splitSegment = changed.SourceSegmentIndex ~= 2;
            [~, changed.SegmentTime_s, changed.GivenPower_units, parentSegmentIndex] = ...
                bmtpEngine.motion.subdivideMotion(changed.ControlPoint_units, changed.SegmentTime_s, ...
                changed.GivenPower_units, splitSegment);
            changed.SourceSegmentIndex = changed.SourceSegmentIndex(parentSegmentIndex);
            changed.ControlPoint_units = bmtpEngine.motion.powerToBernstein(changed.GivenPower_units);
        end
        output=resultWithPreparedMotion(base,request,changed,roundoffReserve_units,target_units);
        verifyTrue(testCase,obstacleAvoidance.validateTrajectory(output).Passed);
        sampleTime_s=linspace(initial.time_s,goal.time_s,401).';
        [~,p,v,a,j]=bmtpEngine.motion.evaluatePolynomial(original.Diagnostics.Polynomial,sampleTime_s);
        [~,splitP,splitV,splitA,splitJ]=bmtpEngine.motion.evaluatePolynomial( ...
            output.Diagnostics.Polynomial,sampleTime_s);
        verifyEqual(testCase,[splitP,splitV,splitA,splitJ],[p,v,a,j],'AbsTol',1e-9);
        verifyEqual(testCase, output.ArrivalTime_s, original.ArrivalTime_s);
        unchangedPieces = changed.SourceSegmentIndex == 2;
        verifyEqual(testCase, changed.GivenPower_units(unchangedPieces, :, :), ...
            source.GivenPower_units(source.SourceSegmentIndex == 2, :, :));
    end
end

function clearQuadraticRetainsProofDepth(testCase)
    % y = 10000 x (u - 1/3)^2 stays above this box, but its control hull
    % needs eleven on-demand halvings near u = 1/3 to prove clearance.
    scale_units = 1e4;
    vertexX_units = 1 / 3;
    initial = struct('position_units', [0, scale_units / 9], ...
        'velocity_units_s', [1, -2 * scale_units / 3], ...
        'acceleration_units_s2', [0, 2 * scale_units], 'time_s', 0);
    goal = struct('position_units', [1, 4 * scale_units / 9], ...
        'velocity_units_s', [1, 4 * scale_units / 3], ...
        'acceleration_units_s2', [0, 2 * scale_units], 'time_s', 1);
    limits = struct('xInterval_units', [-1, 2], 'yInterval_units', [-1, 5e3], ...
        'maxVelocity_units_s', [10, 2e4], 'maxAcceleration_units_s2', [10, 3e4], ...
        'maxJerk_units_s3', [100, 100]);
    base = planner([], initial, goal, limits);
    assertTrue(testCase, base.Success, base.Message);
    box_units = [vertexX_units - 1e-6, -0.000251; vertexX_units + 1e-6, -0.000251; ...
        vertexX_units + 1e-6, -0.00025; vertexX_units - 1e-6, -0.00025];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'box below quadratic', 0, box_units(:, 1), box_units(:, 2), 0);
    preparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstacle, [0, 1]);
    snapshot = obstacleAvoidance.obstacles.snapshot(preparedObstacles, 0);
    request = struct( ...
        'InitialState',    base.Inputs.initialState, ...
        'GoalState',       base.Inputs.goalState, ...
        'Limits',          base.Diagnostics.Limits, ...
        'Options',         base.Options, ...
        'MotionHorizon_s', 1, ...
        'Regions_units',   {snapshot.Regions_units}, ...
        'Coverage',        struct('Passed', true, 'ExactRegionCount', 1));
    power_units = zeros(1, 2, 6);
    power_units(1, :, 1) = [0, scale_units / 9];
    power_units(1, :, 2) = [1, -2 * scale_units / 3];
    power_units(1, :, 3) = [0, scale_units];
    controls_units = bmtpEngine.motion.powerToBernstein(power_units);
    [~, roundoffReserve_units] = bmtpEngine.validation.createCoordinateTolerances( ...
        [initial.position_units; goal.position_units], limits.xInterval_units, ...
        limits.yInterval_units, snapshot.Regions_units, cell(0, 1));
    target_units = (1 + 2 ^ 20 * eps) * ...
        request.Options.CollisionClearanceTolerance_units + roundoffReserve_units;
    [preparedMotion, proof] = bmtpEngine.evaluateCandidate( ...
        request, controls_units, 1, roundoffReserve_units, target_units);
    verifyTrue(testCase, proof.Passed);
    base.Inputs.obstacles = obstacle;
    base.Diagnostics.PreparedObstacles = preparedObstacles;
    output = resultWithPreparedMotion( ...
        base, request, preparedMotion, roundoffReserve_units, target_units);
    output.Diagnostics.SeparationProof = proof;
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(output).Passed);
end

function suppliedZeroCoefficientsSurviveSubdivision(testCase)
    coefficients_units = zeros(1, 2, 9);
    coefficients_units(1, :, 1) = [2, -1];
    coefficients_units(1, :, 2) = [-3, 4];
    coefficients_units(1, :, 3) = [0.5, -0.8];
    coefficients_units(1, :, 4) = [0.25, -0.2];
    controls_units = bmtpEngine.motion.powerToBernstein(coefficients_units);
    [splitControls_units, durations_s, splitCoefficients_units, parentSegmentIndex] = ...
        bmtpEngine.motion.subdivideMotion(controls_units, 2, coefficients_units, true);
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
        controls_units, 2, coefficients_units, true);
    verifyTrue(testCase, all(isnan(splitCoefficients_units(:, 2, :)), 'all'));

    % A mask, duration list or coefficient array that does not match the
    % segment count is refused rather than silently dropping segments.
    twoSegments_units = repmat(controls_units, 2, 1);
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(twoSegments_units, [2; 2], [], false), ...
        'subdivideMotion:InvalidInput');
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(twoSegments_units, 2, [], [false; false]), ...
        'subdivideMotion:InvalidInput');
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion( ...
        twoSegments_units, [2; 2], coefficients_units, [true; false]), ...
        'subdivideMotion:InvalidInput');
    % Even a midpoint cut must leave each half with positive time.
    verifyError(testCase, @() bmtpEngine.motion.subdivideMotion(controls_units, realmin * eps, [], true), ...
        'subdivideMotion:InvalidSplitProgress');
end

function refinementKeepsCurveTimingAndSourceSegments(testCase, fixture)
    source = fixture.Motion;
    assertTrue(testCase, source.Success);
    for obstacleMoves = [false, true]
        request = fixture.Request;
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

function otherFailuresDoNotTriggerSubdivision(testCase, fixture)
    for failedCheck = ["WorkspacePassed", "DynamicsPassed", "ContinuityPassed"]
        request = fixture.Request;
        source = fixture.Motion;
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

function collisionRemainsRejected(testCase, fixture)
    request = fixture.Request;
    % The same parabola now enters this box near u = 0.13. Refining its
    % representation must not repair or move the curve to obtain a proof.
    request.Regions_units = {[0.12, 0.42; 0.14, 0.42; 0.14, 0.5; 0.12, 0.5]};
    source = fixture.Motion;
    [changed, motionCheck] = bmtpEngine.validation.checkMotionWithSubdivision(request, source, 1e-8, 1e-6);
    verifyFalse(testCase, motionCheck.Passed);
    verifyLessThan(testCase, motionCheck.VerifiedPairCount, motionCheck.AllPairCount);
    verifyEqual(testCase, changed.FinalTime_s, source.FinalTime_s);
    verifyEqual(testCase, changed.DilationScale, source.DilationScale);
end

function movingPlaneProvesTranslation(testCase)
    first = [1.5,-0.5;2.5,-0.5;2.5,0.5;1.5,0.5];
    vertices = cat(3,first,first+[10,0]);
    controls = [(0:8)'*10/8,zeros(9,1)];
    plane = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-6,1e-8);
    verifyTrue(testCase,plane.Verified);
    verifyTrue(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
    verifyFalse(testCase,bmtpEngine.separation.verifySeparatingLine( ...
        plane,controls,[first;first+[10,0]],1e-8,1e-6).Verified);
end

function staticPlaneMatchesStationaryAffineRepresentation(testCase)
    vertices = [1.5,-0.5;2.5,-0.5;2.5,0.5;1.5,0.5];
    controls = [linspace(0,0.5,9).',zeros(9,1)];
    plane = struct('Normal',[1,0;0.75,0.5],'Offset_units',[-0.75,-0.75]);
    stationary = cat(3,vertices,vertices);
    checked = bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6);
    verifyTrue(testCase,checked.Verified);
    verifyEqual(testCase,checked,bmtpEngine.separation.verifySeparatingLine(plane,controls,stationary,1e-8,1e-6));
    controls(end,:) = [3,0];
    verifyFalse(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
    verifyFalse(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,stationary,1e-8,1e-6).Verified);
end

function cachedMovingGeometryMatchesUncachedSearch(testCase)
    angle = linspace(0,2*pi,18).';
    first = [3.1*cos(angle(1:end-1)),1.7*sin(angle(1:end-1))]+[-1.2,0.8];
    last = first.*[0.72,1.31]+[4.6,-2.4];
    parameter = linspace(0,1,9).';
    controls = [-5+12*parameter,2.8*sin(pi*parameter)-1.1*parameter];
    geometry = createPlaneGeometry(first,last);
    vertices = cat(3,first,last);
    uncached = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-5,1e-8);
    cached = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-5,1e-8,geometry);
    verifyEqual(testCase,cached.Verified,uncached.Verified);
    verifyEqual(testCase,cached.Normal,uncached.Normal,'AbsTol',64*eps);
    verifyEqual(testCase,cached.Offset_units,uncached.Offset_units,'AbsTol',64*eps);
    verifyEqual(testCase,cached.SignedGap_units,uncached.SignedGap_units,'AbsTol',256*eps);
end

function interiorObstaclePlaneViolationRejected(testCase)
    first = [0.9,-0.1;1.1,-0.1;1.1,0.1;0.9,0.1];
    vertices = cat(3,first,first-[2,0]);
    plane = struct('Normal',[1,0;-1,0],'Offset_units',[-0.2,-0.2]);
    verifyGreaterThan(testCase,min(first(:,1)-0.2),0.1);
    checked = bmtpEngine.separation.verifySeparatingLine(plane,zeros(9,2),vertices,1e-8,0.1);
    verifyFalse(testCase,checked.Verified);
    verifyLessThan(testCase,checked.SignedGap_units,0);
end

function planeSearchUsesProvenProductHull(testCase)
    controls = [ones(9,1),zeros(9,1)]; controls(5,2) = 2;
    vertices = [0.5,1.3;10,1.3;10,3;0.5,3];
    % The original control hull penetrates more in y than x. Its degree-nine
    % product hull is separated in y, so choosing by the original hull fails.
    plane = bmtpEngine.separation.solveSeparatingLine(controls,vertices,1e-6,1e-8);
    verifyTrue(testCase,plane.Verified);
    verifyGreaterThan(testCase,plane.SignedGap_units,0.18);
    verifyTrue(testCase,bmtpEngine.separation.verifySeparatingLine(plane,controls,vertices,1e-8,1e-6).Verified);
end

function batchMatchesScalarVerification(testCase)
    controlPoint_units = [-3, 0; -2, 0.2; -1, 0.1; 0, -0.1; 1, -0.2; 2, 0];
    firstRegions_units = { ...
        [4, -1; 5, -1; 4.5, 1]; ...
        [-1, 2; 1, 2; 1, 3; -1, 3]; ...
        [-0.5, -0.5; 0.5, -0.5; 0.7, 0; 0, 0.7; -0.7, 0]};
    shifts_units = {[0.4, 0.2]; [-0.2, 0.3]; [0.1, -0.1]};
    lastRegions_units = cellfun(@plus, firstRegions_units, shifts_units, ...
        'UniformOutput', false);
    roundoffReserve_units = 1e-8;
    target_units  = 1e-7;
    planes = repmat(bmtpEngine.separation.createEmptyPlane(), 1, numel(firstRegions_units));
    scalarPlanes = planes;
    for regionIndex = 1:numel(firstRegions_units)
        vertices_units = cat(3, ...
            firstRegions_units{regionIndex}, lastRegions_units{regionIndex});
        planes(regionIndex) = bmtpEngine.separation.solveSeparatingLine( ...
            controlPoint_units, vertices_units, target_units, roundoffReserve_units);
        scalarPlanes(regionIndex) = bmtpEngine.separation.verifySeparatingLine( ...
            planes(regionIndex), controlPoint_units, vertices_units, ...
            roundoffReserve_units, target_units);
    end
    batchPlanes = bmtpEngine.separation.verifyMovingSeparatingLines( ...
        planes, controlPoint_units, firstRegions_units, lastRegions_units, ...
        roundoffReserve_units, target_units);

    verifyEqual(testCase, [batchPlanes.Verified], [scalarPlanes.Verified]);
    verifyEqual(testCase, [batchPlanes.Offset_units], ...
        [scalarPlanes.Offset_units], 'AbsTol', 1e-12);
    verifyEqual(testCase, [batchPlanes.SignedGap_units], ...
        [scalarPlanes.SignedGap_units], 'AbsTol', 1e-12);
end

function malformedRegionIsRejected(testCase)
    plane = bmtpEngine.separation.createEmptyPlane();
    verifyError(testCase, @() bmtpEngine.separation.verifyMovingSeparatingLines( ...
        plane, zeros(6, 2), {zeros(0, 2)}, {zeros(0, 2)}, 1e-8, 1e-7), ...
        'bmtpEngine:InvalidMovingPlaneBatch');
end

function affinePlaneBlockConsolidation(testCase)
    limits=struct('xInterval_units',[-100,100],'yInterval_units',[-100,100]);
    source=plane([1,0;0.5,0.5],[-2,-2]);
    target=plane(0.5*source.Normal,[-1.1,-1.1]);
    reduced=bmtpEngine.separation.removeRedundantPlanes([source,target],limits,0.1,true);
    verifyTrue(testCase,reduced(1).Active);
    verifyFalse(testCase,reduced(2).Active);
end

function affinePlaneNeedsOneEndpointWeight(testCase)
    limits=struct('xInterval_units',[-10,10],'yInterval_units',[-10,10]);
    source=plane([1,0;1,0],[0,0]);
    differentEndpointScale=plane([0.5,0;0.75,0],[0,0]);
    reduced=bmtpEngine.separation.removeRedundantPlanes( ...
        [source,differentEndpointScale],limits,0,true);
    verifyTrue(testCase,all([reduced.Active]));
end

function generatedPlaneRowsMatchExactOmissionOracle(testCase)
    degree=5;
    segmentCount=2;
    regionCount=3;
    controlCount=segmentCount*(degree+1)*2;
    variableCount=controlCount+4;
    controls=reshape(linspace(-1.3,1.7,controlCount),controlCount,1);
    x=[controls;zeros(4,1)];
    template=struct('Active',true,'Normal',[1,0;0.8,0.6], ...
        'Offset_units',[-0.2,0.1],'TimeFraction',[0,1]);
    planes=repmat(template,segmentCount,regionCount);
    planes(1,2).Normal=[-1,0;-0.6,0.8];
    planes(1,2).Offset_units=[0.3,-0.1];
    planes(1,2).TimeFraction=[0.15,0.8];
    planes(1,3).Normal=[0,-1;0.4,-sqrt(0.84)];
    planes(1,3).Offset_units=[-0.05,0.2];
    planes(2,1).Normal=[0.6,0.8;1,0];
    planes(2,1).TimeFraction=[0.2,0.65];
    planes(2,2).Normal=[-0.8,0.6;-1,0];
    planes(2,2).Offset_units=[0.15,0.25];
    planes(2,3).Normal=[0,-1;-0.8,-0.6];
    planes(2,3).TimeFraction=[0.4,0.9];
    activePairs=true(segmentCount,regionCount);
    slackColumnByPair=zeros(size(activePairs));
    roundoffReserve_units=2e-4;
    [rows,bounds]=bmtpEngine.separation.createSelectedPlaneRows(planes,activePairs, ...
        degree,variableCount,slackColumnByPair,roundoffReserve_units);
    rowResidual=rows*x-bounds;
    pairResidual=reshape(max(reshape(rowResidual,degree+2,[]),[],1), ...
        regionCount,segmentCount).';
    [selectedPairs,maximumResidual]=bmtpEngine.separation.findViolatedPlanePairs( ...
        x,planes,activePairs,false(size(activePairs)),degree, ...
        slackColumnByPair,roundoffReserve_units,-1e9);
    expected=false(size(activePairs));
    for segmentIndex=1:segmentCount
        [~,regionIndex]=max(pairResidual(segmentIndex,:));
        expected(segmentIndex,regionIndex)=true;
    end
    verifyEqual(testCase,maximumResidual,max(pairResidual,[],'all'), ...
        'AbsTol',32*eps(max(1,abs(maximumResidual))));
    verifyEqual(testCase,selectedPairs,expected);
end

function retainedPlaneCannotHideAnotherViolatedPair(testCase)
    degree=5;
    variableCount=2*(degree+1)+4;
    x=zeros(variableCount,1);
    planes=repmat(struct('Active',true,'Normal',[1,0;1,0], ...
        'Offset_units',[0.25,0.25],'TimeFraction',[0,1]),1,2);
    planes(2).Offset_units=[0.5,0.5];
    activePairs=true(1,2);
    retainedPairs=[true,false];
    [selectedPairs,maximumResidual]=bmtpEngine.separation.findViolatedPlanePairs( ...
        x,planes,activePairs,retainedPairs,degree,zeros(1,2),0,1e-10);
    verifyEqual(testCase,selectedPairs,[false,true]);
    verifyEqual(testCase,maximumResidual,0.5,'AbsTol',1e-12);
end

function finalProofRechecksNeighborDirections(testCase)
    box = [-0.5,-0.5;0.5,-0.5;0.5,0.5;-0.5,0.5];
    request = struct('Regions_units',{{box}},'Coverage',struct('Passed',true), ...
        'InitialState',struct('time_s',0),'IsRest',true);
    points = [-2,0;-2,0;2,0;0,0];
    prepared = struct('ControlPoint_units',repmat(reshape(points,4,1,2),1,9,1), ...
        'SegmentTime_s',ones(4,1),'FinalTime_s',4);
    proof = bmtpEngine.validation.checkFinalMotion(request,prepared,1e-8,1e-6);
    verifyFalse(testCase,proof.Passed);
    verifyEqual(testCase,proof.VerifiedPairCount,3);
    verifyEqual(testCase,proof.ReusedPairCount,1);
    verifyTrue(testCase,proof.Planes(3).Verified);
    verifyFalse(testCase,proof.Planes(4).Verified);
    for k = 1:3
        checked = bmtpEngine.separation.verifySeparatingLine(proof.Planes(k), ...
            squeeze(prepared.ControlPoint_units(k,:,:)),box,1e-8,1e-6);
        verifyTrue(testCase,checked.Verified);
    end
end

function unchangedCurveReusesCompleteProof(testCase, fixture)
    request=fixture.Request; motion=fixture.Motion;
    first=fixture.FirstProof; cache=fixture.Cache;
    second=bmtpEngine.validation.checkFinalMotion(request,motion,1e-8,1e-7,cache);
    verifyTrue(testCase,first.Passed); verifyTrue(testCase,second.Passed);
    verifyEqual(testCase,second.CachedPairCount,second.AllPairCount);
    verifyEqual(testCase,second.Planes,first.Planes);
end

function sourceAndCurveChangesInvalidateReuse(testCase, fixture)
    request=fixture.Request; motion=fixture.Motion;
    cache=fixture.Cache;
    changed=request; changed.Regions_units={[-1,-1;1,-1;1,1;-1,1]};
    proof=bmtpEngine.validation.checkFinalMotion(changed,motion,1e-8,1e-7,cache);
    verifyFalse(testCase,proof.Passed); verifyEqual(testCase,proof.CachedPairCount,0);
    motion.ControlPoint_units(:,:,1)=2.5;
    proof=bmtpEngine.validation.checkFinalMotion(request,motion,1e-8,1e-7,cache);
    verifyFalse(testCase,proof.Passed); verifyEqual(testCase,proof.CachedPairCount,0);
end

function alteredEndpointCoverageRejected(testCase)
    x = [-0.5;0.5;0.5;-0.5]; y = [-0.5;-0.5;0.5;0.5];
    obstacle = obstacleAvoidance.obstacles.createObstacle('box',[0;12],{x;x},{y+2;y+3},0.1);
    r = planner(obstacle,struct('time_s',0,'position_units',[-4,0]), ...
        struct('time_s',12,'position_units',[4,0]),struct(),struct());
    verifyTrue(testCase,r.Success,r.Message);
    altered = r;
    altered.Diagnostics.SeparationProof.Coverage.EndRegions_units{1}(:,2) = ...
        altered.Diagnostics.SeparationProof.Coverage.EndRegions_units{1}(:,2)+1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r;
    altered.Diagnostics.SeparationProof.Coverage = rmfield( ...
        altered.Diagnostics.SeparationProof.Coverage,'EndRegions_units');
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
end

function coverageMetadataCannotExcludeTheMotion(testCase, result)
    assertTrue(testCase, result.Success, result.Message);
    % A record whose declared coverage ends at the start time carries an
    % empty dynamic proof; the straight motion crosses the square.
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);

    square_units = [-0.5 -0.5; 0.5 -0.5; 0.5 0.5; -0.5 0.5];
    obstacle     = obstacleAvoidance.obstacles.createObstacle('center square', 0, ...
        square_units(:, 1), square_units(:, 2), 0);
    tampered = result;
    tampered.Inputs.obstacles                     = obstacle;
    tampered.Diagnostics.TrajectoryCoverageEndTime_s          = result.Inputs.initialState.time_s;
    tampered.Diagnostics.SeparationProof.Coverage            = struct( ...
        'Passed',               true, ...
        'ExactRegionCount',     0, ...
        'ActiveTimeInterval_s', zeros(0, 2), ...
        'EndRegions_units',     {cell(0, 1)});
    validation = obstacleAvoidance.validateTrajectory(tampered);
    verifyFalse(testCase, validation.Passed);
    verifyFalse(testCase, validation.SeparationProofValid);
end

function continuityProjectionMayOnlyAbsorbRoundoff(testCase, result)
    assertTrue(testCase, result.Success, result.Message);
    % A solver motion converts with a roundoff-sized join repair; a
    % discontinuous input is rejected instead of being silently connected.
    coordinateScale_units = max(1, max(abs(result.position_units), [], 'all'));
    verifyLessThanOrEqual(testCase, result.Diagnostics.Polynomial.ContinuityProjectionDisplacement_units, ...
        1e-6 * coordinateScale_units);

    controls_units             = zeros(2, 6, 2);
    controls_units(2, :, 1)    = 10;
    polynomial = bmtpEngine.motion.createPowerPolynomial(controls_units, [1; 1], 0);
    verifyGreaterThan(testCase, polynomial.ContinuityProjectionDisplacement_units, 1);

    route_units = [result.Inputs.initialState.position_units; result.Inputs.goalState.position_units];
    seed        = struct('position_units', route_units, 'tau', [0; 1]);
    request     = bmtpEngine.prepareRequest(seed, ...
        struct('regions_units', {cell(0, 1)}, ...
        'coverage', struct('Passed', true, 'ExactRegionCount', 0)), ...
        struct('initialState', result.Inputs.initialState, ...
        'goalState', result.Inputs.goalState, ...
        'limits', result.Diagnostics.Limits, ...
        'options', result.Options));
    prepared = bmtpEngine.motion.createMotion(request, controls_units, [5; 5]);
    verifyFalse(testCase, prepared.Success);
    verifyEqual(testCase, prepared.TerminationReason, "continuityProjectionExceedsTolerance");
end

function incompleteSuccessfulRecordFailsWithoutThrowing(testCase, result)
    assertTrue(testCase, result.Success, result.Message);

    missingArrival = rmfield(result, 'ArrivalTime_s');
    validation     = obstacleAvoidance.validateTrajectory(missingArrival);
    verifyFalse(testCase, validation.Passed);
    verifyTrue(testCase, contains(validation.Message, "record"));

    missingOption = result;
    missingOption.Options = rmfield(missingOption.Options, 'ArrivalTimeTolerance_s');
    validation = obstacleAvoidance.validateTrajectory(missingOption);
    verifyFalse(testCase, validation.Passed);

    missingIntercept = result;
    missingIntercept.Inputs.goalState.targetMotion = struct( ...
        'time_s', [0; 10], 'position_units', [4 0; 4 0]);
    missingIntercept.Diagnostics = rmfield(missingIntercept.Diagnostics, 'Intercept');
    validation = obstacleAvoidance.validateTrajectory(missingIntercept);
    verifyFalse(testCase, validation.Passed);
end

function timeToleranceIsIndependentOfConstraintTolerance(testCase)
    initial = struct('time_s', 0, 'position_units', [-1, 0]);
    goal    = struct('time_s', 10, 'position_units', [1, 0]);
    limits  = struct( ...
        'xInterval_units',          [-5, 5], ...
        'yInterval_units',          [-5, 5], ...
        'maxVelocity_units_s',      [10, 10], ...
        'maxAcceleration_units_s2', [10, 10], ...
        'maxJerk_units_s3',         [10, 10]);
    looseTolerance      = 1e-3;
    tightTolerance      = 1e-8;
    crossedTolerances  = [looseTolerance, tightTolerance; tightTolerance, looseTolerance];
    expectedAcceptance = [false; true];
    clockOffset_s       = 1e-4;
    controlPoint_units  = zeros(1, 6, 2);
    controlPoint_units(1, :, 1) = [-1, -1, -1, 1, 1, 1];

    for settingIndex = 1:size(crossedTolerances, 1)
        options = struct( ...
            'GoalTimeMode',          'fixedArrival', ...
            'ConstraintTolerance',   crossedTolerances(settingIndex, 1), ...
            'ArrivalTimeTolerance_s', crossedTolerances(settingIndex, 2));
        baseResult = planner([], initial, goal, limits, options);
        assertTrue(testCase, baseResult.Success, baseResult.Message);
        assertTrue(testCase, obstacleAvoidance.validateTrajectory(baseResult).Passed);

        seed = struct( ...
            'position_units', [initial.position_units; goal.position_units], ...
            'tau',            [0; 1]);
        request = bmtpEngine.prepareRequest(seed, ...
            struct('regions_units', {cell(0, 1)}, ...
            'coverage', struct('Passed', true, 'ExactRegionCount', 0)), ...
            struct('initialState', baseResult.Inputs.initialState, ...
            'goalState', baseResult.Inputs.goalState, ...
            'limits', baseResult.Diagnostics.Limits, ...
            'options', baseResult.Options));
        preparedMotion = bmtpEngine.motion.createMotion(request, controlPoint_units, ...
            request.MotionHorizon_s + clockOffset_s);
        output = createProvenOutput(baseResult, request, preparedMotion);
        validation = obstacleAvoidance.validateTrajectory(output);

        verifyTrue(testCase, preparedMotion.Success);
        verifyEqual(testCase, preparedMotion.FitsRequestHorizon, expectedAcceptance(settingIndex));
        verifyEqual(testCase, validation.Passed, expectedAcceptance(settingIndex));
    end

    for constraintTolerance = [tightTolerance, looseTolerance]
        options = struct( ...
            'GoalTimeMode',          'fixedArrival', ...
            'ConstraintTolerance',   constraintTolerance, ...
            'ArrivalTimeTolerance_s', tightTolerance);
        result = planner([], initial, goal, limits, options);
        assertTrue(testCase, result.Success, result.Message);
        result.Inputs.goalState.time_s = result.Diagnostics.Polynomial.FinalTime_s + clockOffset_s;
        validation = obstacleAvoidance.validateTrajectory(result);
        verifyFalse(testCase, validation.Passed);
        verifyFalse(testCase, validation.EndpointStatesMatched);

        if constraintTolerance == looseTolerance
            altered = result;
            altered.Diagnostics.Polynomial.FinalTime_s = altered.Diagnostics.Polynomial.FinalTime_s + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SegmentTimingConsistent);

            altered = result;
            altered.Diagnostics.Polynomial.SegmentStartTime_s(1) = ...
                altered.Diagnostics.Polynomial.SegmentStartTime_s(1) + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SegmentTimingConsistent);
            verifyFalse(testCase, validation.EndpointStatesMatched);

            altered           = result;
            altered.time_s(1) = altered.time_s(1) + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SampledHistoriesMatched);

            altered             = result;
            altered.time_s(end) = altered.time_s(end) - clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SampledHistoriesMatched);

            altered = result;
            altered.Inputs.initialState.time_s = ...
                altered.Inputs.initialState.time_s + clockOffset_s;
            validation = obstacleAvoidance.validateTrajectory(altered);
            verifyFalse(testCase, validation.SeparationProofValid);
        end
    end
end

function detourAndTampering(testCase)
    options = planner();
    verifyEqual(testCase, options.GoalTimeMode, "fixedArrival");
    verifyFalse(testCase, isfield(options, "Success"));
    obstacle = struct("Name", "center block", ...
        "Vertices_units", [-1 -1; 1 -1; 1 1; -1 1], ...
        "SafetyMargin_units", 0.25);
    initial = struct("time_s", 0, "position_units", [-4 0]);
    goal = struct("time_s", 12, "position_units", [4 0]);
    limits = struct("xInterval_units", [-180 180], "yInterval_units", [-90 90], ...
        "maxVelocity_units_s", [2 2], "maxAcceleration_units_s2", [2 2], ...
        "maxJerk_units_s3", [4 4]);
    r = planner(obstacle, initial, goal, limits, options);
    verifyTrue(testCase,r.Success,r.Message);
    verifyTrue(testCase,obstacleAvoidance.validateTrajectory(r).Passed);
    verifyEqual(testCase,r.Diagnostics.VisibilityGraph.SearchKind,"initialSpatialSnapshot");
    verifyTrue(testCase,r.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    altered = r; altered.position_units(2,1) = altered.position_units(2,1)+0.1;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.Diagnostics.Polynomial.jerkPower_units_s3(1,1,1) = 1e4;
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
    altered = r; altered.Diagnostics.SeparationProof.Regions_units = {};
    verifyFalse(testCase,obstacleAvoidance.validateTrajectory(altered).Passed);
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

function output = createProvenOutput(baseResult, request, preparedMotion)
    % Build an adversarial validator fixture without stale planner decisions.
    roundoffReserve_units = baseResult.Diagnostics.SeparationProof.RoundoffReserve_units;
    target_units = baseResult.Diagnostics.SeparationProof.RequiredGap_units - roundoffReserve_units;
    motionOutput = bmtpEngine.createMotionOutput(struct(), request, preparedMotion);
    output = baseResult;
    output.time_s                = motionOutput.time_s;
    output.position_units        = motionOutput.position_units;
    output.velocity_units_s      = motionOutput.velocity_units_s;
    output.acceleration_units_s2 = motionOutput.acceleration_units_s2;
    output.jerk_units_s3         = motionOutput.jerk_units_s3;
    output.ArrivalTime_s         = motionOutput.ArrivalTime_s;
    output.MotionLength_units    = motionOutput.MotionLength_units;
    output.Diagnostics.Polynomial                      = motionOutput.Polynomial;
    output.Diagnostics.TrajectoryDuration_s            = motionOutput.TrajectoryDuration_s;
    output.Diagnostics.IntegratedSquaredJerk_units2_s5 = motionOutput.IntegratedSquaredJerk_units2_s5;
    output.Diagnostics.MaximumConstraintViolation      = motionOutput.MaximumConstraintViolation;
    output.Diagnostics.SeparationProof = bmtpEngine.validation.checkFinalMotion( ...
        request, preparedMotion, roundoffReserve_units, target_units);
    output.Diagnostics.Validation = struct("Passed", false, ...
        "Message", "Synthetic motion has not been validated.");
end

function geometry = createPlaneGeometry(first,last)
    edges = [diff([first;first(1,:)],1,1);diff([last;last(1,:)],1,1)];
    lengths = vecnorm(edges,2,2);
    edges = edges(lengths>0,:);
    lengths = lengths(lengths>0);
    normals = [-edges(:,2),edges(:,1)]./lengths;
    firstProjection = first*normals.';
    lastProjection = last*normals.';
    geometry = struct('PositiveNormals',normals, ...
        'FirstPositiveSupport_units',min(firstProjection,[],1), ...
        'LastPositiveSupport_units',min(lastProjection,[],1), ...
        'FirstNegativeSupport_units',-max(firstProjection,[],1), ...
        'LastNegativeSupport_units',-max(lastProjection,[],1));
end

function value=plane(normal,offset_units)
    value=struct('Active',true,'Verified',true,'ExitFlag',1, ...
        'Normal',normal,'Offset_units',offset_units,'SignedGap_units',1);
end

function fixture = quadraticFixture()
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
    fixture.Request = request;
    fixture.Motion = bmtpEngine.motion.createMotion( ...
        request, controls_units, durations_s);
end

function fixture = proofReuseFixture()
    fixture.Request=struct('Regions_units',{{[2,-1;3,-1;3,1;2,1]}}, ...
        'Coverage',struct('Passed',true),'InitialState',struct('time_s',0));
    fixture.Motion=struct('ControlPoint_units',zeros(2,6,2),'SegmentTime_s',[1;1], ...
        'FinalTime_s',2);
    [fixture.FirstProof, fixture.Cache] = bmtpEngine.validation.checkFinalMotion( ...
        fixture.Request, fixture.Motion, 1e-8, 1e-7);
end

function result = validationFixture()
    limits = struct( ...
        'xInterval_units',          [-6 6], ...
        'yInterval_units',          [-4 4], ...
        'maxVelocity_units_s',      [2 2], ...
        'maxAcceleration_units_s2', [2 2], ...
        'maxJerk_units_s3',         [4 4]);
    initial = struct('time_s', 0, 'position_units', [-4 0]);
    goal    = struct('time_s', 10, 'position_units', [4 0]);
    result  = planner([], initial, goal, limits, struct('GoalTimeMode', 'fixedArrival'));
end
