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
result = planTrajRuckig(initialState, terminalState, limits, ...
    struct("SampleTime", 0.01));
expectedDuration = 4 * nthroot(1 / 2, 3);
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
verifyEqual(testCase, result.Duration, expectedDuration, "AbsTol", 1e-10);
verifyEqual(testCase, ...
    sum(vecnorm(diff(result.position), 2, 2)), sqrt(5), ...
    "AbsTol", 1e-10);

% Omitting acceleration states and maximum jerk selects Ruckig's official
% second-order interface, where acceleration is the switching control.
accelerationInitialState = rmfield(initialState, "acceleration");
accelerationTerminalState = rmfield(terminalState, "acceleration");
accelerationLimits = rmfield(limits, "maximumJerk");
accelerationLimits.maximumAcceleration = [1, 2];
accelerationResult = ruckigEngine.solve( ...
    accelerationInitialState, accelerationTerminalState, ...
    accelerationLimits, struct("SampleTime", 0.01));
verifyTrue(testCase, accelerationResult.Success, ...
    accelerationResult.Message);
verifyEqual(testCase, accelerationResult.Duration, 2, "AbsTol", 1e-12);
verifyEqual(testCase, ...
    accelerationResult.Inputs.limits.ControlOrder, 2);

% The randomized case generator represents the same omitted interface with
% NaNs, so both public spellings must select the identical control order.
nanInitialState = initialState;
nanInitialState.acceleration(:) = NaN;
nanTerminalState = terminalState;
nanTerminalState.acceleration(:) = NaN;
nanLimits = limits;
nanLimits.maximumAcceleration = accelerationLimits.maximumAcceleration;
nanLimits.maximumJerk(:) = NaN;
nanResult = ruckigEngine.solve( ...
    nanInitialState, nanTerminalState, nanLimits, ...
    struct("SampleTime", 0.01));
verifyTrue(testCase, nanResult.Success, nanResult.Message);
verifyEqual(testCase, nanResult.Duration, accelerationResult.Duration, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, nanResult.Inputs.limits.ControlOrder, 2);

% A coordinate that is already at rest at its target remains idle while the
% other axis determines the synchronized clock.
stationaryInitialState = struct( ...
    "time", 0, "position", [0, 0], ...
    "velocity", [0.1, 0], "acceleration", [0, 0]);
stationaryTerminalState = struct( ...
    "position", [2, 0], "velocity", [0, 0], ...
    "acceleration", [0, 0], "maximumTime", 10);
stationaryLimits = struct( ...
    "maximumVelocity", [2, 2], ...
    "maximumAcceleration", [1, 1], ...
    "maximumJerk", [2, 2]);
stationaryResult = ruckigEngine.solve( ...
    stationaryInitialState, stationaryTerminalState, ...
    stationaryLimits, struct());
verifyTrue(testCase, stationaryResult.Success, stationaryResult.Message);
verifyTrue(testCase, stationaryResult.Validation.Passed, ...
    stationaryResult.Validation.Message);
verifyEqual(testCase, stationaryResult.Diagnostics.Profile.AxisFamily(2), ...
    "stationary");
end

function testAsymmetricBoundsAreRejected(testCase)
% Verify the exact engine identifies its unsupported derivative-bound family.
[initialState, terminalState, limits] = restToRestFixture();
limits.velocityLower = [-0.5, -20];
limits.velocityUpper = [10, 20];
result = ruckigEngine.solve(initialState, terminalState, limits, struct());
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "unsupportedAsymmetricBounds");
end

function testSatisfiedPathConstraintIsCertified(testCase)
% Verify a satisfied affine row participates in continuous certification.
[initialState, terminalState, limits] = restToRestFixture();
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pointConstraint());
verifyTrue(testCase, result.Success, result.Message);
verifyTrue(testCase, result.Validation.ConstraintPassed);
end

function testViolatedPathConstraintIsReported(testCase)
% Verify exact profile construction never hides an affine path violation.
[initialState, terminalState, limits] = restToRestFixture();
pathConstraints = pointConstraint();
pathConstraints.LowerBound = 2;
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pathConstraints);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "pathConstraintViolation");
verifyGreaterThan(testCase, ...
    result.Validation.MaximumInequalityViolation, 0);
end

function testIntervalPathConstraintUsesContinuousHull(testCase)
% Verify one interval row constrains the complete projected subtrajectory.
[initialState, terminalState, limits] = restToRestFixture();
pathConstraints = struct( ...
    "Tau", 0.2, ...
    "TauEnd", 0.8, ...
    "Normal", [1, 0], ...
    "LowerBound", 0.01);
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pathConstraints);
verifyTrue(testCase, result.Success, result.Message);
pathConstraints.LowerBound = 0.9;
violatingResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct(), pathConstraints);
verifyFalse(testCase, violatingResult.Success);
verifyEqual(testCase, violatingResult.TerminationReason, ...
    "pathConstraintViolation");
end

function testIntervalPathConstraintSpansPolynomialSegments(testCase)
% Detect a violation after the first boundary of one requested interval.
[initialState, terminalState, limits] = restToRestFixture();
pathConstraints = struct( ...
    "Tau", 0.1, "TauEnd", 0.8, ...
    "Normal", [-1, 0], "LowerBound", -0.2);
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct(), pathConstraints);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, ...
    "pathConstraintViolation");
verifyGreaterThan(testCase, ...
    result.Validation.MaximumInequalityViolation, 0);
end

function testFixedTimeBelowMinimumIsIdentified(testCase)
% Verify an impossible fixed duration remains an expected engine failure.
[initialState, terminalState, limits] = restToRestFixture();
options = struct("TimeMode", "fixed", "FinalTime", 1);
result = ruckigEngine.solve(initialState, terminalState, limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "fixedTimeBelowMinimum");

accelerationInitialState = rmfield(initialState, "acceleration");
accelerationTerminalState = rmfield(terminalState, "acceleration");
accelerationLimits = rmfield(limits, "maximumJerk");
accelerationLimits.maximumAcceleration = [1, 2];
accelerationResult = ruckigEngine.solve( ...
    accelerationInitialState, accelerationTerminalState, ...
    accelerationLimits, options);
verifyFalse(testCase, accelerationResult.Success);
verifyEqual(testCase, accelerationResult.TerminationReason, ...
    "fixedTimeBelowMinimum");
end

function testEarliestArrivalAcceptsExactAndInsideHorizon(testCase)
% Accept profiles at the horizon and within one arrival tolerance before it.
[initialState, terminalState, limits] = restToRestFixture();
minimumDuration = 4 * nthroot(1 / 2, 3);
arrivalTolerance = 1e-6;
options = struct("ArrivalTimeTolerance", arrivalTolerance);
terminalState.maximumTime = minimumDuration;
exactResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, options);
terminalState.maximumTime = minimumDuration + arrivalTolerance / 2;
insideResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, options);
verifyTrue(testCase, exactResult.Success, exactResult.Message);
verifyTrue(testCase, insideResult.Success, insideResult.Message);

% Instantaneous endpoint values can be inside every box while their required
% bounded-jerk continuation is not. The minimum acceleration-canceling
% excursion must be classified as physical infeasibility, not a missing
% switching family.
initialState.position = 0;
initialState.velocity = 0.9;
initialState.acceleration = 1;
terminalState.position = 2;
terminalState.velocity = 0;
terminalState.acceleration = 0;
limits.maximumVelocity = 1;
limits.maximumAcceleration = 2;
limits.maximumJerk = 1;
infeasibleResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyFalse(testCase, infeasibleResult.Success);
verifyEqual(testCase, infeasibleResult.TerminationReason, ...
    "kinematicallyInfeasibleBoundaryState");
end

function testWrapperRejectsUnsupportedArities(testCase)
% Return the public InvalidCall error before referencing missing arguments.
verifyError(testCase, @() planTrajRuckig(struct()), ...
    "planTrajRuckig:InvalidCall");
verifyError(testCase, @() planTrajRuckig(struct(), struct()), ...
    "planTrajRuckig:InvalidCall");
end

function testColumnStatesAndScalarLimitsNormalizeIdentically(testCase)
% Verify direct normalization is independent of orientation and expansion.
[initialState, terminalState, limits] = restToRestFixture();
rowResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
initialState.position = initialState.position.';
initialState.velocity = initialState.velocity.';
initialState.acceleration = initialState.acceleration.';
terminalState.position = terminalState.position.';
terminalState.velocity = terminalState.velocity.';
terminalState.acceleration = terminalState.acceleration.';
limits.maximumVelocity = 20;
limits.maximumAcceleration = 20;
limits.maximumJerk = 2;
columnResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyTrue(testCase, rowResult.Success && columnResult.Success);
verifyEqual(testCase, columnResult.Inputs.initialState.position, [0, 0]);
verifyEqual(testCase, columnResult.Inputs.limits.maximumJerk, [2, 2]);
end

function testPolynomialEvaluationMatchesReturnedHistories(testCase)
% Verify engine-owned reconstruction agrees at every published sample.
[initialState, terminalState, limits] = restToRestFixture();
result = ruckigEngine.solve(initialState, terminalState, limits, ...
    struct("SampleTime", 0.01));
[time, position, velocity, acceleration, jerk] = ...
    ruckigEngine.internal.evaluatePolynomial( ...
    result.Polynomial, result.time);
verifyEqual(testCase, time, result.time);
verifyEqual(testCase, position, result.position, "AbsTol", 1e-12);
verifyEqual(testCase, velocity, result.velocity, "AbsTol", 1e-12);
verifyEqual(testCase, acceleration, result.acceleration, "AbsTol", 1e-12);
verifyEqual(testCase, jerk, result.jerk, "AbsTol", 1e-12);

% These one- and two-axis motions have Bernstein coefficients outside a
% derivative limit even though their true stationary-point extrema are inside.
% The hull may prove acceptance, but one coefficient must never reject them.
[initialState, terminalState, limits] = ...
    createBernsteinAmbiguityFixture(158);
oneAxisResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyTrue(testCase, oneAxisResult.Success, oneAxisResult.Message);
[initialState, terminalState, limits] = ...
    createBernsteinAmbiguityFixture(79);
twoAxisResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyTrue(testCase, twoAxisResult.Success, twoAxisResult.Message);

% Preserve exact switch times when different axes contribute nearly equal
% boundaries. Decimal rounding previously accumulated into a false endpoint
% mismatch on this deterministic two-axis row.
[initialState, terminalState, limits] = ...
    createSwitchTimePrecisionFixture();
precisionResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyTrue(testCase, precisionResult.Success, precisionResult.Message);

% Exercise the upstream ACC0_VEL family and a synchronization time that must
% advance to the certified right edge of another axis's blocked interval.
[initialState, terminalState, limits] = ...
    createSynchronizationCoverageFixture(17);
accelerationVelocityResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyTrue(testCase, accelerationVelocityResult.Success, ...
    accelerationVelocityResult.Message);
verifyEqual(testCase, accelerationVelocityResult.Duration, ...
    5.5096668913746516, "AbsTol", 1e-10);
[initialState, terminalState, limits] = ...
    createSynchronizationCoverageFixture(39);
blockedIntervalResult = ruckigEngine.solve( ...
    initialState, terminalState, limits, struct());
verifyTrue(testCase, blockedIntervalResult.Success, ...
    blockedIntervalResult.Message);
verifyEqual(testCase, blockedIntervalResult.Duration, ...
    193.97421939594636, "AbsTol", 1e-9);
verifyTrue(testCase, any(isfinite( ...
    blockedIntervalResult.Diagnostics.Profile.BlockedInterval), "all"));
end

function testUnknownOptionsWarnOnceAndAreIgnored(testCase)
% Verify one warning identifies every ignored direct-engine option.
[initialState, terminalState, limits] = restToRestFixture();
options = struct("UnknownOne", 1, "UnknownTwo", 2);
verifyWarning(testCase, @() ruckigEngine.solve( ...
    initialState, terminalState, limits, options), ...
    "ruckigEngine:UnknownOptions");
end

function [initialState, terminalState, limits] = restToRestFixture()
% Create an exact two-axis request with one shared scalar progress law.
initialState = struct( ...
    "time", 0, ...
    "position", [0, 0], ...
    "velocity", [0, 0], ...
    "acceleration", [0, 0]);
terminalState = struct( ...
    "position", [1, -2], ...
    "velocity", [0, 0], ...
    "acceleration", [0, 0], ...
    "maximumTime", 10);
limits = struct( ...
    "maximumVelocity", [10, 20], ...
    "maximumAcceleration", [10, 20], ...
    "maximumJerk", [1, 2]);
end

function [initialState, terminalState, limits] = ...
        createSwitchTimePrecisionFixture()
% Reproduce a deterministic row sensitive to perturbing exact switch times.
initialState = struct( ...
    "time", 0, ...
    "position", [6.132921190223791, -1.3342714518861754], ...
    "velocity", [0.3923098688068975, 35.447623188174276], ...
    "acceleration", [0.36414099344783424, 0.36500554264394841]);
terminalState = struct( ...
    "position", [6.6842357462629893, -1.1433546756210493], ...
    "velocity", [-0.49336870197687271, 50.420219344149238], ...
    "acceleration", [0.037298949880907178, 0.13965076056096573], ...
    "maximumTime", 1e4);
limits = struct( ...
    "maximumVelocity", [0.55818725449841133, 58.492559202119168], ...
    "maximumAcceleration", [0.60846798580561656, 0.53437404655703324], ...
    "maximumJerk", [38.803908282921128, 0.31031934135500161]);
end

function pathConstraints = pointConstraint()
% Require a midpoint projection to remain on one side of x=-1.
pathConstraints = struct( ...
    "Tau", 0.5, ...
    "TauEnd", 0.5, ...
    "Normal", [1, 0], ...
    "LowerBound", -1);
end

function [initialState, terminalState, limits] = ...
        createSynchronizationCoverageFixture(caseIndex)
% Reproduce general fixed-time families and blocked-interval synchronization.
if caseIndex == 17
    initialPosition = [-0.2946943284974039, 0.72717803454408725];
    terminalPosition = [-0.15161472425184003, -0.39190486277543068];
    initialVelocity = [0.35732941916323346, -0.26539008381367046];
    terminalVelocity = [0.23069026127896586, -0.23917087977634027];
    initialAcceleration = [-0.098603180970299076, 0.59933446756194153];
    terminalAcceleration = [-0.13405652283367087, -0.08568594494596285];
    maximumVelocity = [0.53600716215012612, 0.94808197764359758];
    maximumAcceleration = [0.19781524135065387, 0.72654405607998696];
    maximumJerk = [35.898279505276207, 10.279050618472136];
else
    initialPosition = [4.245942479114734, -3.1033145813540557];
    terminalPosition = [37.287973465236455, 289.58343302712228];
    initialVelocity = [-0.0677236254980743, 7.8965033421377004];
    terminalVelocity = [0.35442016595613374, 10.706199226974498];
    initialAcceleration = [0.04965017779420116, 0.032692862375515691];
    terminalAcceleration = [-0.39986819658982919, 0.1082269739181822];
    maximumVelocity = [0.92176274290673732, 14.019938991072353];
    maximumAcceleration = [0.77755769120321516, 0.1619937711031128];
    maximumJerk = [11.131650037621547, 24.178279381637786];
end
initialState = struct( ...
    "time", 0, "position", initialPosition, ...
    "velocity", initialVelocity, "acceleration", initialAcceleration);
terminalState = struct( ...
    "position", terminalPosition, "velocity", terminalVelocity, ...
    "acceleration", terminalAcceleration, "maximumTime", 1e6);
limits = struct( ...
    "maximumVelocity", maximumVelocity, ...
    "maximumAcceleration", maximumAcceleration, ...
    "maximumJerk", maximumJerk);
end

function [initialState, terminalState, limits] = ...
        createBernsteinAmbiguityFixture(caseIndex)
% Reproduce deterministic random rows that require exact extrema after hulls.
if caseIndex == 158
    initialPosition = -5.9737446739127993;
    terminalPosition = 30.22900307488564;
    initialVelocity = -0.59788036120734234;
    terminalVelocity = -0.78962791395163801;
    initialAcceleration = 1.3813665975555351;
    terminalAcceleration = 1.2478598039919828;
    maximumVelocity = 3.2730336318134627;
    maximumAcceleration = 10.91244529127944;
    maximumJerk = 0.3478901862576031;
else
    initialPosition = [5.0273423808170312, 9.5040486395833934];
    terminalPosition = [533.95549673188123, 11.418723153650706];
    initialVelocity = [0.46491707447445291, 2.1870457828960146];
    terminalVelocity = [0.56144188614133828, -9.8252155070793776];
    initialAcceleration = [-0.53736227789276114, 0.46287605322962316];
    terminalAcceleration = [0.42529459081166443, -0.447275188394678];
    maximumVelocity = [0.76360013640328273, 52.053996939222763];
    maximumAcceleration = [0.62002210157176507, 0.55748567640703373];
    maximumJerk = [0.14507271549207201, 0.12227454988214717];
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
