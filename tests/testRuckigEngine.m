function tests = testRuckigEngine
%% Section 0: Header & Readme
% SYNTAX
%   tests = testRuckigEngine
%**************************************************************************
% PURPOSE
%   - Protect the direct, self-contained Ruckig-derived trajectory engine.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Fixture position is in abstract coordinate units and time is seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Add the trajectory product folder for direct engine tests.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(repositoryRoot, "trajectory"));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testExactMotionMatchesIndependentMinimum(testCase)
    % Verify exact switching remains independently callable and certified.
    [initialState, terminalState, limits] = restToRestFixture();
    result           = ruckigEngine.solve(initialState, terminalState, limits, struct("SampleTime", 0.01));
    expectedDuration = 4 * nthroot(1 / 2, 3);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyEqual(testCase, result.Duration, expectedDuration, "AbsTol", 1e-10);
    verifyEqual(testCase, sum(vecnorm(diff(result.position), 2, 2)), sqrt(5), "AbsTol", 1e-10);

    % Omitting acceleration states and maximum jerk selects Ruckig's official
    % second-order interface, where acceleration is the switching control.
    accelerationInitialState  = rmfield(initialState, "acceleration");
    accelerationTerminalState = rmfield(terminalState, "acceleration");
    accelerationLimits        = rmfield(limits, "maximumJerk");
    accelerationLimits.maximumAcceleration = [1, 2];
    accelerationResult = ruckigEngine.solve(accelerationInitialState, accelerationTerminalState, accelerationLimits, struct("SampleTime", 0.01));
    verifyTrue(testCase, accelerationResult.Success, accelerationResult.Message);
    verifyEqual(testCase, accelerationResult.Duration, 2, "AbsTol", 1e-12);
    verifyEqual(testCase, accelerationResult.Inputs.limits.ControlOrder, 2);

    % The randomized case generator represents the same omitted interface with
    % NaNs, so both public spellings must select the identical control order.
    nanInitialState = initialState;
    nanInitialState.acceleration(:) = NaN;
    nanTerminalState = terminalState;
    nanTerminalState.acceleration(:) = NaN;
    nanLimits = limits;
    nanLimits.maximumAcceleration = accelerationLimits.maximumAcceleration;
    nanLimits.maximumJerk(:) = NaN;
    nanResult = ruckigEngine.solve(nanInitialState, nanTerminalState, nanLimits, struct("SampleTime", 0.01));
    verifyTrue(testCase, nanResult.Success, nanResult.Message);
    verifyEqual(testCase, nanResult.Duration, accelerationResult.Duration, "AbsTol", 1e-12);
    verifyEqual(testCase, nanResult.Inputs.limits.ControlOrder, 2);

    % A coordinate that is already at rest at its target remains idle while the
    % other axis determines the synchronized clock.
    stationaryInitialState = struct();
    stationaryInitialState.time         = 0;
    stationaryInitialState.position     = [0, 0];
    stationaryInitialState.velocity     = [0.1, 0];
    stationaryInitialState.acceleration = [0, 0];
    stationaryTerminalState = struct();
    stationaryTerminalState.position     = [2, 0];
    stationaryTerminalState.velocity     = [0, 0];
    stationaryTerminalState.acceleration = [0, 0];
    stationaryTerminalState.maximumTime  = 10;
    stationaryLimits = struct();
    stationaryLimits.maximumVelocity     = [2, 2];
    stationaryLimits.maximumAcceleration = [1, 1];
    stationaryLimits.maximumJerk         = [2, 2];
    stationaryResult = ruckigEngine.solve(stationaryInitialState, stationaryTerminalState, stationaryLimits, struct());
    verifyTrue(testCase, stationaryResult.Success, stationaryResult.Message);
    verifyTrue(testCase, stationaryResult.Validation.Passed, stationaryResult.Validation.Message);
    verifyEqual(testCase, stationaryResult.Diagnostics.Profile.AxisFamily(2), "stationary");
end

function testAsymmetricBoundsAreRejected(testCase)
    % Verify the exact engine identifies its unsupported derivative-bound family.
    [initialState, terminalState, limits] = restToRestFixture();
    limits.velocityLower = [-0.5, -20];
    limits.velocityUpper = [10, 20];
    result = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "unsupportedAsymmetricBounds");
end

function testSatisfiedPathConstraintIsCertified(testCase)
    % Verify a satisfied affine row participates in continuous certification.
    [initialState, terminalState, limits] = restToRestFixture();
    result = ruckigEngine.solve(initialState, terminalState, limits, struct(), pointConstraint());
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.ConstraintPassed);
end

function testViolatedPathConstraintIsReported(testCase)
    % Verify exact profile construction never hides an affine path violation.
    [initialState, terminalState, limits] = restToRestFixture();
    pathConstraints = pointConstraint();
    pathConstraints.LowerBound = 2;
    result = ruckigEngine.solve(initialState, terminalState, limits, struct(), pathConstraints);
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "pathConstraintViolation");
    verifyGreaterThan(testCase, result.Validation.MaximumInequalityViolation, 0);
end

function testIntervalPathConstraintUsesContinuousHull(testCase)
    % Verify one interval row constrains the complete projected subtrajectory.
    [initialState, terminalState, limits] = restToRestFixture();
    pathConstraints = struct();
    pathConstraints.Tau        = 0.2;
    pathConstraints.TauEnd     = 0.8;
    pathConstraints.Normal     = [1, 0];
    pathConstraints.LowerBound = 0.01;
    result = ruckigEngine.solve(initialState, terminalState, limits, struct(), pathConstraints);
    verifyTrue(testCase, result.Success, result.Message);
    pathConstraints.LowerBound = 0.9;
    violatingResult = ruckigEngine.solve(initialState, terminalState, limits, struct(), pathConstraints);
    verifyFalse(testCase, violatingResult.Success);
    verifyEqual(testCase, violatingResult.TerminationReason, "pathConstraintViolation");
end

function testIntervalPathConstraintSpansPolynomialSegments(testCase)
    % Detect a violation after the first boundary of one requested interval.
    [initialState, terminalState, limits] = restToRestFixture();
    pathConstraints = struct();
    pathConstraints.Tau        = 0.1;
    pathConstraints.TauEnd     = 0.8;
    pathConstraints.Normal     = [-1, 0];
    pathConstraints.LowerBound = -0.2;
    result = ruckigEngine.solve(initialState, terminalState, limits, struct(), pathConstraints);
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "pathConstraintViolation");
    verifyGreaterThan(testCase, result.Validation.MaximumInequalityViolation, 0);
end

function testFixedTimeBelowMinimumIsIdentified(testCase)
    % Verify an impossible fixed duration remains an expected engine failure.
    [initialState, terminalState, limits] = restToRestFixture();
    options = struct();
    options.TimeMode  = "fixed";
    options.FinalTime = 1;
    result = ruckigEngine.solve(initialState, terminalState, limits, options);
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "fixedTimeBelowMinimum");

    accelerationInitialState  = rmfield(initialState, "acceleration");
    accelerationTerminalState = rmfield(terminalState, "acceleration");
    accelerationLimits        = rmfield(limits, "maximumJerk");
    accelerationLimits.maximumAcceleration = [1, 2];
    accelerationResult = ruckigEngine.solve(accelerationInitialState, accelerationTerminalState, accelerationLimits, options);
    verifyFalse(testCase, accelerationResult.Success);
    verifyEqual(testCase, accelerationResult.TerminationReason, "fixedTimeBelowMinimum");
end

function testEarliestArrivalAcceptsExactAndInsideHorizon(testCase)
    % Accept profiles at the horizon and within one arrival tolerance before it.
    [initialState, terminalState, limits] = restToRestFixture();
    minimumDuration  = 4 * nthroot(1 / 2, 3);
    arrivalTolerance = 1e-6;
    options          = struct("ArrivalTimeTolerance", arrivalTolerance);
    terminalState.maximumTime = minimumDuration;
    exactResult = ruckigEngine.solve(initialState, terminalState, limits, options);
    terminalState.maximumTime = minimumDuration + arrivalTolerance / 2;
    insideResult = ruckigEngine.solve(initialState, terminalState, limits, options);
    verifyTrue(testCase, exactResult.Success, exactResult.Message);
    verifyTrue(testCase, insideResult.Success, insideResult.Message);

    % Instantaneous endpoint values can be inside every box while their required
    % bounded-jerk continuation is not. The minimum acceleration-canceling
    % excursion must be classified as physical infeasibility, not a missing
    % switching family.
    initialState.position     = 0;
    initialState.velocity     = 0.9;
    initialState.acceleration = 1;
    terminalState.position     = 2;
    terminalState.velocity     = 0;
    terminalState.acceleration = 0;
    limits.maximumVelocity     = 1;
    limits.maximumAcceleration = 2;
    limits.maximumJerk         = 1;
    infeasibleResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyFalse(testCase, infeasibleResult.Success);
    verifyEqual(testCase, infeasibleResult.TerminationReason, "kinematicallyInfeasibleBoundaryState");
end

function testWrapperRejectsUnsupportedArities(testCase)
    % Return the public InvalidCall error before referencing missing arguments.
    verifyError(testCase, @() ruckigEngine.solve(struct()), "ruckigEngine:InvalidCall");
    verifyError(testCase, @() ruckigEngine.solve(struct(), struct()), "ruckigEngine:InvalidCall");
end

function testColumnStatesAndScalarLimitsNormalizeIdentically(testCase)
    % Verify direct normalization is independent of orientation and expansion.
    [initialState, terminalState, limits] = restToRestFixture();
    rowResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    initialState.position     = initialState.position.';
    initialState.velocity     = initialState.velocity.';
    initialState.acceleration = initialState.acceleration.';
    terminalState.position     = terminalState.position.';
    terminalState.velocity     = terminalState.velocity.';
    terminalState.acceleration = terminalState.acceleration.';
    limits.maximumVelocity     = 20;
    limits.maximumAcceleration = 20;
    limits.maximumJerk         = 2;
    columnResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyTrue(testCase, rowResult.Success && columnResult.Success);
    verifyEqual(testCase, columnResult.Inputs.initialState.position, [0, 0]);
    verifyEqual(testCase, columnResult.Inputs.limits.maximumJerk, [2, 2]);
end

function testPolynomialEvaluationMatchesReturnedHistories(testCase)
    % Verify engine-owned reconstruction agrees at every published sample.
    [initialState, terminalState, limits] = restToRestFixture();
    result = ruckigEngine.solve(initialState, terminalState, limits, struct("SampleTime", 0.01));
    [time, position, velocity, acceleration, jerk] = ruckigEngine.internal.evaluatePolynomial(result.Polynomial, result.time);
    verifyEqual(testCase, time, result.time);
    verifyEqual(testCase, position, result.position, "AbsTol", 1e-12);
    verifyEqual(testCase, velocity, result.velocity, "AbsTol", 1e-12);
    verifyEqual(testCase, acceleration, result.acceleration, "AbsTol", 1e-12);
    verifyEqual(testCase, jerk, result.jerk, "AbsTol", 1e-12);

    % These one- and two-axis motions have Bernstein coefficients outside a
    % derivative limit even though their true stationary-point extrema are inside.
    % The hull may prove acceptance, but one coefficient must never reject them.
    [initialState, terminalState, limits] = createBernsteinAmbiguityFixture(158);
    oneAxisResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyTrue(testCase, oneAxisResult.Success, oneAxisResult.Message);
    [initialState, terminalState, limits] = createBernsteinAmbiguityFixture(79);
    twoAxisResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyTrue(testCase, twoAxisResult.Success, twoAxisResult.Message);

    % Preserve exact switch times when different axes contribute nearly equal
    % boundaries. Decimal rounding previously accumulated into a false endpoint
    % mismatch on this deterministic two-axis row.
    [initialState, terminalState, limits] = createSwitchTimePrecisionFixture();
    precisionResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyTrue(testCase, precisionResult.Success, precisionResult.Message);

    % Exercise the upstream ACC0_VEL family and a synchronization time that must
    % advance to the certified right edge of another axis's blocked interval.
    [initialState, terminalState, limits] = createSynchronizationCoverageFixture(17);
    accelerationVelocityResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyTrue(testCase, accelerationVelocityResult.Success, accelerationVelocityResult.Message);
    verifyEqual(testCase, accelerationVelocityResult.Duration, 5.5096668913746516, "AbsTol", 1e-10);
    [initialState, terminalState, limits] = createSynchronizationCoverageFixture(39);
    blockedIntervalResult = ruckigEngine.solve(initialState, terminalState, limits, struct());
    verifyTrue(testCase, blockedIntervalResult.Success, blockedIntervalResult.Message);
    verifyEqual(testCase, blockedIntervalResult.Duration, 193.97421939594636, "AbsTol", 1e-9);
    verifyTrue(testCase, any(isfinite(blockedIntervalResult.Diagnostics.Profile.BlockedInterval), "all"));
end

function testUnknownOptionsWarnOnceAndAreIgnored(testCase)
    % Verify one warning identifies every ignored direct-engine option.
    [initialState, terminalState, limits] = restToRestFixture();
    options = struct();
    options.UnknownOne = 1;
    options.UnknownTwo = 2;
    verifyWarning(testCase, @() ruckigEngine.solve(initialState, terminalState, limits, options), "ruckigEngine:UnknownOptions");
end

function testSameSignTransitNearBoundIsSolved(testCase)
    % A short motion that keeps acceleration +1 the whole way: velocity runs
    % from -0.95 to -0.7 inside the bound of 1, so it must solve, and holding
    % jerk at zero does it in exactly 0.25 s, so the earliest arrival can be
    % no later than that. The old terminal rule mirrored the initial one in
    % time and refused the request outright.
    initialState  = struct("time", 0, "position", 0, "velocity", -0.95, "acceleration", 1);
    terminalState = struct("position", -0.20625, "velocity", -0.7, "acceleration", 1, "maximumTime", 10);
    limits        = struct("maximumVelocity", 1, "maximumAcceleration", 2, "maximumJerk", 1);
    result = ruckigEngine.solve(initialState, terminalState, limits, struct("SampleTime", 0.01));
    verifyTrue(testCase, result.Success, result.Message);
    verifyLessThanOrEqual(testCase, result.Duration, 0.25 + 1e-9);
    verifyLessThanOrEqual(testCase, max(abs(result.velocity)), 1 + 1e-9);

    % Truly impossible states are still refused. Starting at v = 0.95 with
    % a = +1 must keep a positive forever (canceling it would push v to
    % 1.45), so a terminal acceleration of -1 is unreachable.
    blockedInitialState  = struct("time", 0, "position", 0, "velocity", 0.95, "acceleration", 1);
    blockedTerminalState = struct("position", 0.5, "velocity", 0.7, "acceleration", -1, "maximumTime", 10);
    refused = ruckigEngine.solve(blockedInitialState, blockedTerminalState, limits, struct("SampleTime", 0.01));
    verifyFalse(testCase, refused.Success);
    verifyEqual(testCase, refused.TerminationReason, "kinematicallyInfeasibleBoundaryState");
    verifySubstring(testCase, refused.Message, "initial acceleration cannot be canceled");

    % Arriving at v = -0.7 with a = +1 needs a prior velocity of -1.2, past
    % the bound, so it is only reachable from a start that keeps a positive
    % from even further down. Starting at rest cannot do it.
    restStart = struct("time", 0, "position", 0, "velocity", 0, "acceleration", 0);
    refused = ruckigEngine.solve(restStart, terminalState, limits, struct("SampleTime", 0.01));
    verifyFalse(testCase, refused.Success);
    verifyEqual(testCase, refused.TerminationReason, "kinematicallyInfeasibleBoundaryState");
    verifySubstring(testCase, refused.Message, "terminal acceleration cannot be reached");

    % Velocity sitting on its bound with acceleration still pushing outward
    % leaves the bound at once. The settled-velocity escape alone would let
    % it through (the end keeps the sign and settles further out), so the
    % rule also demands that velocity moves toward the end value.
    onBoundStart = struct("time", 0, "position", 0, "velocity", -1, "acceleration", -0.5);
    onBoundEnd   = struct("position", -0.3, "velocity", -0.6, "acceleration", -1, "maximumTime", 10);
    refused = ruckigEngine.solve(onBoundStart, onBoundEnd, limits, struct("SampleTime", 0.01));
    verifyFalse(testCase, refused.Success);
    verifyEqual(testCase, refused.TerminationReason, "kinematicallyInfeasibleBoundaryState");

    % A tiny but real velocity change must not be refused by the monotone
    % condition: the tolerance has to sit on the permissive side. Holding
    % jerk at zero for one microsecond at a velocity bound of 1e9 is valid.
    tinyStep    = 1e-6;
    tinyStart   = struct("time", 0, "position", 0, "velocity", 1e9 - 0.25, "acceleration", 1);
    tinyEnd     = struct("position", tinyStart.velocity * tinyStep + tinyStep ^ 2 / 2, ...
        "velocity", tinyStart.velocity + tinyStep, "acceleration", 1, "maximumTime", 10);
    tinyLimits  = struct("maximumVelocity", 1e9, "maximumAcceleration", 2, "maximumJerk", 1);
    tinyResult  = ruckigEngine.solve(tinyStart, tinyEnd, tinyLimits, struct("SampleTime", 1e-7));
    verifyNotEqual(testCase, tinyResult.TerminationReason, "kinematicallyInfeasibleBoundaryState");
end

function testMissingFamilyRegressionsReachGoal(testCase)
    % Cases 561, 753, and 999 previously ended as unsupported even though
    % every axis had a minimum-time profile.
    requests = createMissingFamilyRegressionFixtures();
    for requestIndex = 1:numel(requests)
        request = requests{requestIndex};
        result = ruckigEngine.solve(request.initialState, request.terminalState, ...
            request.limits, struct("SampleTime", 0.01));
        verifyTrue(testCase, result.Success, result.Message);
        verifyEqual(testCase, result.TerminationReason, "goalReached");
        verifyPlainMotionLimits(testCase, result, request.initialState, ...
            request.terminalState, request.limits);
    end
end

function testNewFixedTimeFamiliesAreSelected(testCase)
    % Exercise one public fixed-time request for each newly ported Step-2
    % family: ACC0_ACC1, ACC0, ACC1, NONE, and NONE_SMOOTH.
    fixtures = createFixedTimeFamilyFixtures();
    for fixtureIndex = 1:numel(fixtures)
        fixture = fixtures{fixtureIndex};
        options = struct("TimeMode", "fixed", "FinalTime", fixture.duration);
        result = ruckigEngine.solve(fixture.initialState, fixture.terminalState, ...
            fixture.limits, options);
        verifyTrue(testCase, result.Success, result.Message);
        verifyEqual(testCase, result.TerminationReason, "goalReached");
        verifyTrue(testCase, any(result.Diagnostics.Profile.AxisFamily == ...
            fixture.expectedFamily), sprintf("Expected fixed-time family %s.", ...
            fixture.expectedFamily));
    end
end

function [initialState, terminalState, limits] = restToRestFixture()
    % Create an exact two-axis request with one shared scalar progress law.
    initialState = struct();
    initialState.time         = 0;
    initialState.position     = [0, 0];
    initialState.velocity     = [0, 0];
    initialState.acceleration = [0, 0];
    terminalState = struct();
    terminalState.position     = [1, -2];
    terminalState.velocity     = [0, 0];
    terminalState.acceleration = [0, 0];
    terminalState.maximumTime  = 10;
    limits = struct();
    limits.maximumVelocity     = [10, 20];
    limits.maximumAcceleration = [10, 20];
    limits.maximumJerk         = [1, 2];
end

function requests = createMissingFamilyRegressionFixtures()
    % Preserve the full-precision seed-7 rows supplied for cases 561, 753, 999.
    requests = cell(1, 3);
    requests{1} = createRequest( ...
        [-0.041780847816605918, -0.0087449872615916139], ...
        [-2.981180516123966, -3.3661432115845331], ...
        [-0.84707234382644092, 0.97634450797024197], ...
        [0.048980982937721691, -0.076996588586742423], ...
        [5.5724298422119816, 4.1597893808079025], ...
        [2.482306881306366, 0.0016163314022269346], ...
        11.242510368355205, 3.6380569910018927, 1.524709904121708);
    requests{2} = createRequest( ...
        [1.2895646368540872, 3.1008710481508497], ...
        [-10.480416863111659, -0.68510656400336922], ...
        [0.68289338931004284, -0.040657599135815854], ...
        [1.8410687177251601, -2.2725822721717903], ...
        [11.308266604555435, 0.34400622981032369], ...
        [1.3053726969271988, -0.098122579668354734], ...
        [13.289401573103625, 0.86610398174652792], ...
        [2.5616785953113204, 0.15964962435184071], ...
        [0.78138477499646553, 14.145588402517298]);
    requests{3} = createRequest( ...
        [-37.936724835501124, -17.678489976103517, 38.378441641865763], ...
        [1.709251938245296, 0.50480862223809164, -20.247370422886892], ...
        [-0.099222748431344363, -0.98464461277436233, -1.3363162229240031], ...
        [-6.2764186922895595, -1.6491843938993009, -27.810018583501453], ...
        [1.7773045110673047, 0.020951498008939518, 16.451903600877124], ...
        [-0.034218956510029312, -1.1493740375075634, 1.0129552967382338], ...
        [2.8969260286573735, 2.3208369604649608, 24.458085228544924], ...
        [0.20905580255043207, 1.337807647406366, 2.0281330875355295], ...
        [69.719390421139764, 0.6456688118741496, 9.7869421998389807]);
end

function fixtures = createFixedTimeFamilyFixtures()
    % Use deterministic requests whose selected diagnostic names identify the
    % exact newly ported family, rather than only reaching a nearby boundary.
    fixtures = cell(1, 5);
    fixtures{1} = createFixedFixture(createRequest(0, 0, 0, 3.75, 0, 0, ...
        100, 1, 2), 5, "synchronizedAccelerationBounds");
    fixtures{2} = createFixedFixture(createRequest( ...
        -2.3921587742199097, -0.67186199163658789, -0.55280533314062097, ...
        -0.12526539287556382, 0.065395024161649079, -0.46364163014167309, ...
        1.8292924370975656, 0.89366341409209404, 6.2440713837758359), ...
        6.8336098705046009, "synchronizedInitialAcceleration");
    fixtures{3} = createFixedFixture(createRequest( ...
        -2.8093810258219403, 0.13235099155231486, 0.038259211295983184, ...
        -3.0852564463087777, -1.8667752730902001, 0.34414599351730812, ...
        4.0746794875158709, 0.56177611138523642, 5.8008473145496193), ...
        32.518161498262607, "synchronizedTerminalAcceleration");
    regressionRequests = createMissingFamilyRegressionFixtures();
    fixtures{4} = createFixedFixture(regressionRequests{2}, ...
        11.639036828787253, "synchronizedNoLimit");
    fixtures{5} = createFixedFixture(createRequest( ...
        -7.3565379950544347, 0.83121010666569217, 0.099910874201434532, ...
        2.9034822593730993, 0.14669077674100778, 0.040334768761807749, ...
        0.99509433858469998, 0.55947402413775393, 9.0868891010266708), ...
        23.11605281805911, "synchronizedSmoothNoLimit");
end

function request = createRequest(initialPosition, initialVelocity, initialAcceleration, ...
        terminalPosition, terminalVelocity, terminalAcceleration, ...
        maximumVelocity, maximumAcceleration, maximumJerk)
    % Create one normalized-shape request without calling engine helpers.
    request = struct();
    request.initialState = struct("time", 0, "position", initialPosition, ...
        "velocity", initialVelocity, "acceleration", initialAcceleration);
    request.terminalState = struct("position", terminalPosition, ...
        "velocity", terminalVelocity, "acceleration", terminalAcceleration, ...
        "maximumTime", 1e6);
    request.limits = struct("maximumVelocity", maximumVelocity, ...
        "maximumAcceleration", maximumAcceleration, "maximumJerk", maximumJerk);
end

function fixture = createFixedFixture(request, duration, expectedFamily)
    % Add the fixed clock and expected diagnostic to one request fixture.
    fixture = request;
    fixture.duration       = duration;
    fixture.expectedFamily = expectedFamily;
end

function verifyPlainMotionLimits(testCase, result, initialState, terminalState, limits)
    % Check endpoints and continuous derivative extrema from plain polynomial
    % arithmetic. This intentionally does not call the engine validator.
    polynomial = result.Polynomial;
    dimensionCount = numel(initialState.position);
    maximumVelocity     = reshape(expandLimit(limits.maximumVelocity, dimensionCount), 1, []);
    maximumAcceleration = reshape(expandLimit(limits.maximumAcceleration, dimensionCount), 1, []);
    maximumJerk         = reshape(expandLimit(limits.maximumJerk, dimensionCount), 1, []);
    velocityPeak        = zeros(1, dimensionCount);
    accelerationPeak    = zeros(1, dimensionCount);
    jerkPeak            = zeros(1, dimensionCount);
    for segmentIndex = 1:polynomial.SegmentCount
        for dimensionIndex = 1:dimensionCount
            velocityCoefficient = reshape(polynomial.velocityPower_units_s( ...
                segmentIndex, dimensionIndex, :), 1, []);
            accelerationCoefficient = reshape(polynomial.accelerationPower_units_s2( ...
                segmentIndex, dimensionIndex, :), 1, []);
            jerkValue = polynomial.jerkPower_units_s3(segmentIndex, dimensionIndex, 1);
            velocityValues = [velocityCoefficient(1), sum(velocityCoefficient)];
            if velocityCoefficient(3) ~= 0
                stationaryTau = -velocityCoefficient(2) / (2 * velocityCoefficient(3));
                if stationaryTau > 0 && stationaryTau < 1
                    velocityValues(end + 1) = velocityCoefficient(1) + ...
                        stationaryTau * (velocityCoefficient(2) + ...
                        stationaryTau * velocityCoefficient(3)); %#ok<AGROW>
                end
            end
            velocityPeak(dimensionIndex) = max(velocityPeak(dimensionIndex), ...
                max(abs(velocityValues)));
            accelerationPeak(dimensionIndex) = max(accelerationPeak(dimensionIndex), ...
                max(abs([accelerationCoefficient(1), sum(accelerationCoefficient)])));
            jerkPeak(dimensionIndex) = max(jerkPeak(dimensionIndex), abs(jerkValue));
        end
    end
    tolerance = 1e-8;
    verifyTrue(testCase, all(velocityPeak <= maximumVelocity + ...
        tolerance * max(1, maximumVelocity)));
    verifyTrue(testCase, all(accelerationPeak <= maximumAcceleration + ...
        tolerance * max(1, maximumAcceleration)));
    verifyTrue(testCase, all(jerkPeak <= maximumJerk + ...
        tolerance * max(1, maximumJerk)));
    verifyEqual(testCase, result.position(1, :), initialState.position, ...
        "AbsTol", tolerance);
    verifyEqual(testCase, result.position(end, :), terminalState.position, ...
        "AbsTol", tolerance * max(1, max(abs(terminalState.position))));
    verifyEqual(testCase, result.velocity(end, :), terminalState.velocity, ...
        "AbsTol", tolerance * max(1, max(abs(terminalState.velocity))));
    verifyEqual(testCase, result.acceleration(end, :), terminalState.acceleration, ...
        "AbsTol", tolerance * max(1, max(abs(terminalState.acceleration))));
end

function expanded = expandLimit(limitValue, dimensionCount)
    % Expand a scalar test limit the same way the public input contract does.
    if isscalar(limitValue)
        expanded = repmat(limitValue, 1, dimensionCount);
    else
        expanded = limitValue;
    end
end

function [initialState, terminalState, limits] = createSwitchTimePrecisionFixture()
    % Reproduce a deterministic row sensitive to perturbing exact switch times.
    initialState = struct();
    initialState.time         = 0;
    initialState.position     = [6.132921190223791, -1.3342714518861754];
    initialState.velocity     = [0.3923098688068975, 35.447623188174276];
    initialState.acceleration = [0.36414099344783424, 0.36500554264394841];
    terminalState = struct();
    terminalState.position     = [6.6842357462629893, -1.1433546756210493];
    terminalState.velocity     = [-0.49336870197687271, 50.420219344149238];
    terminalState.acceleration = [0.037298949880907178, 0.13965076056096573];
    terminalState.maximumTime  = 1e4;
    limits = struct();
    limits.maximumVelocity     = [0.55818725449841133, 58.492559202119168];
    limits.maximumAcceleration = [0.60846798580561656, 0.53437404655703324];
    limits.maximumJerk         = [38.803908282921128, 0.31031934135500161];
end

function pathConstraints = pointConstraint()
    % Require a midpoint projection to remain on one side of x=-1.
    pathConstraints = struct();
    pathConstraints.Tau        = 0.5;
    pathConstraints.TauEnd     = 0.5;
    pathConstraints.Normal     = [1, 0];
    pathConstraints.LowerBound = -1;
end

function [initialState, terminalState, limits] = createSynchronizationCoverageFixture(caseIndex)
    % Reproduce general fixed-time families and blocked-interval synchronization.
    if caseIndex == 17
        initialPosition      = [-0.2946943284974039, 0.72717803454408725];
        terminalPosition     = [-0.15161472425184003, -0.39190486277543068];
        initialVelocity      = [0.35732941916323346, -0.26539008381367046];
        terminalVelocity     = [0.23069026127896586, -0.23917087977634027];
        initialAcceleration  = [-0.098603180970299076, 0.59933446756194153];
        terminalAcceleration = [-0.13405652283367087, -0.08568594494596285];
        maximumVelocity      = [0.53600716215012612, 0.94808197764359758];
        maximumAcceleration  = [0.19781524135065387, 0.72654405607998696];
        maximumJerk          = [35.898279505276207, 10.279050618472136];
    else
        initialPosition      = [4.245942479114734, -3.1033145813540557];
        terminalPosition     = [37.287973465236455, 289.58343302712228];
        initialVelocity      = [-0.0677236254980743, 7.8965033421377004];
        terminalVelocity     = [0.35442016595613374, 10.706199226974498];
        initialAcceleration  = [0.04965017779420116, 0.032692862375515691];
        terminalAcceleration = [-0.39986819658982919, 0.1082269739181822];
        maximumVelocity      = [0.92176274290673732, 14.019938991072353];
        maximumAcceleration  = [0.77755769120321516, 0.1619937711031128];
        maximumJerk          = [11.131650037621547, 24.178279381637786];
    end
    initialState = struct("time", 0, "position", initialPosition, ...
        "velocity", initialVelocity, "acceleration", initialAcceleration);
    terminalState = struct("position", terminalPosition, "velocity", terminalVelocity, ...
        "acceleration", terminalAcceleration, "maximumTime", 1e6);
    limits = struct("maximumVelocity", maximumVelocity, ...
        "maximumAcceleration", maximumAcceleration, ...
        "maximumJerk", maximumJerk);
end

function [initialState, terminalState, limits] = createBernsteinAmbiguityFixture(caseIndex)
    % Reproduce deterministic random rows that require exact extrema after hulls.
    if caseIndex == 158
        initialPosition      = -5.9737446739127993;
        terminalPosition     = 30.22900307488564;
        initialVelocity      = -0.59788036120734234;
        terminalVelocity     = -0.78962791395163801;
        initialAcceleration  = 1.3813665975555351;
        terminalAcceleration = 1.2478598039919828;
        maximumVelocity      = 3.2730336318134627;
        maximumAcceleration  = 10.91244529127944;
        maximumJerk          = 0.3478901862576031;
    else
        initialPosition      = [5.0273423808170312, 9.5040486395833934];
        terminalPosition     = [533.95549673188123, 11.418723153650706];
        initialVelocity      = [0.46491707447445291, 2.1870457828960146];
        terminalVelocity     = [0.56144188614133828, -9.8252155070793776];
        initialAcceleration  = [-0.53736227789276114, 0.46287605322962316];
        terminalAcceleration = [0.42529459081166443, -0.447275188394678];
        maximumVelocity      = [0.76360013640328273, 52.053996939222763];
        maximumAcceleration  = [0.62002210157176507, 0.55748567640703373];
        maximumJerk          = [0.14507271549207201, 0.12227454988214717];
    end
    initialState = struct("time", 0, "position", initialPosition, ...
        "velocity", initialVelocity, "acceleration", initialAcceleration);
    terminalState = struct("position", terminalPosition, ...
        "velocity", terminalVelocity, ...
        "acceleration", terminalAcceleration, "maximumTime", 1e6);
    limits = struct("maximumVelocity", maximumVelocity, ...
        "maximumAcceleration", maximumAcceleration, ...
        "maximumJerk", maximumJerk);
end
