function tests = testBmtpEngine
%% Section 0: Header & Readme
% SYNTAX
%   tests = testBmtpEngine
%**************************************************************************
% PURPOSE
%   - Verify the independent BMTP trajectory engine and its stable failures.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds. Derivatives use deg/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
% Add only the repository and independent trajectory-engine parents.
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
testCase.TestData.InitialState = createState(0, [0 0]);
testCase.TestData.GoalState = createState(10, [4 2]);
testCase.TestData.Limits = createLimits();
testCase.TestData.Options = createOptions("earliestArrival");
end

function testDirectMotionReachesRestEndpoint(testCase)
% Check exact synchronized construction and polynomial reconstruction.
initialState = testCase.TestData.InitialState;
goalState = testCase.TestData.GoalState;
result = bmtpEngine.createDirectMotion(initialState, goalState, ...
    testCase.TestData.Limits, testCase.TestData.Options);
verifyTrue(testCase, result.Success, result.Message);
verifyLessThan(testCase, result.FinalTime_s, goalState.time_s);
verifyEqual(testCase, result.position_deg(1, :), ...
    initialState.position_deg, "AbsTol", 1e-12);
verifyEqual(testCase, result.position_deg(end, :), ...
    goalState.position_deg, "AbsTol", 1e-9);
verifyEqual(testCase, result.velocity_deg_s(end, :), [0 0], ...
    "AbsTol", 1e-9);
verifyEqual(testCase, result.acceleration_deg_s2(end, :), [0 0], ...
    "AbsTol", 1e-9);
end

function testFixedTimeBelowMinimumReturnsStableFailure(testCase)
% Keep physical infeasibility visible without throwing or clipping motion.
options = createOptions("fixedArrival");
goalState = testCase.TestData.GoalState;
goalState.time_s = 0.1;
result = bmtpEngine.createDirectMotion( ...
    testCase.TestData.InitialState, goalState, ...
    testCase.TestData.Limits, options);
verifyFalse(testCase, result.Success);
verifyEqual(testCase, result.TerminationReason, "fixedTimeBelowMinimum");
verifyTrue(testCase, isempty(result.time_s));
verifyTrue(testCase, isfield(result, "Polynomial"));
end

function testEventWordReconstructionIsExact(testCase)
% Exercise piecewise-constant jerk integration independently of the planner.
initialState = createState(0, [0 0]);
[motion, terminalState] = bmtpEngine.createMotionRecord( ...
    struct(), initialState, [0; 0.5; 1], [1 0; -1 0], ...
    0.05, "unitEventWord");
verifyEqual(testCase, terminalState.position_deg, [0.125 0], ...
    "AbsTol", 1e-12);
verifyEqual(testCase, terminalState.velocity_deg_s, [0.25 0], ...
    "AbsTol", 1e-12);
verifyEqual(testCase, terminalState.acceleration_deg_s2, [0 0], ...
    "AbsTol", 1e-12);
verifyEqual(testCase, motion.position_deg(end, :), ...
    terminalState.position_deg, "AbsTol", 1e-12);
end

function testOffsetSplinePreservesBaseClockAndEndpoints(testCase)
% Verify the extracted quintic generator composes without changing arrival.
baseMotion = bmtpEngine.createDirectMotion( ...
    testCase.TestData.InitialState, testCase.TestData.GoalState, ...
    testCase.TestData.Limits, testCase.TestData.Options);
midpointTime_s = 0.5 * (baseMotion.time_s(1) + baseMotion.FinalTime_s);
motion = bmtpEngine.createOffsetSplineMotion( ...
    baseMotion, [baseMotion.time_s(1); midpointTime_s; ...
    baseMotion.FinalTime_s], [0; 0.2; 0], 2, ...
    testCase.TestData.InitialState, 0.02, "unitOffsetSpline");
verifyEqual(testCase, motion.FinalTime_s, baseMotion.FinalTime_s, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, motion.position_deg([1 end], :), ...
    baseMotion.position_deg([1 end], :), "AbsTol", 1e-9);
verifyEqual(testCase, motion.velocity_deg_s([1 end], :), zeros(2), ...
    "AbsTol", 1e-8);
verifyEqual(testCase, motion.acceleration_deg_s2([1 end], :), zeros(2), ...
    "AbsTol", 1e-8);
end

function testProgressPolynomialPreservesExactClockAndRestEndpoints(testCase)
% Exercise the degree-15 progress composition without planner dependencies.
initialState = testCase.TestData.InitialState;
goalState = testCase.TestData.GoalState;
baseMotion = bmtpEngine.createDirectMotion( ...
    initialState, goalState, testCase.TestData.Limits, ...
    testCase.TestData.Options);
motion = bmtpEngine.createProgressPolynomialMotion( ...
    baseMotion, initialState, goalState, 1, 0.2, 0.02, ...
    "unitProgressPolynomial");
verifyEqual(testCase, motion.Polynomial.Degree, 15);
verifyEqual(testCase, motion.FinalTime_s, baseMotion.FinalTime_s, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, motion.position_deg([1 end], :), ...
    [initialState.position_deg; goalState.position_deg], "AbsTol", 1e-9);
verifyEqual(testCase, motion.velocity_deg_s([1 end], :), zeros(2), ...
    "AbsTol", 1e-8);
verifyEqual(testCase, motion.acceleration_deg_s2([1 end], :), zeros(2), ...
    "AbsTol", 1e-8);
verifyTrue(testCase, isfinite(motion.IntegratedSquaredJerk_deg2_s5));
verifyGreaterThan(testCase, motion.MotionLength_deg, ...
    baseMotion.MotionLength_deg);
end

function testOneSidedProgressBasisPreservesClockAndSide(testCase)
% Compose a nonalternating basis without changing the direct physical clock.
initialState = testCase.TestData.InitialState;
goalState = testCase.TestData.GoalState;
baseMotion = bmtpEngine.createDirectMotion( ...
    initialState, goalState, testCase.TestData.Limits, ...
    testCase.TestData.Options);
% u(1-u)^2 has unit peak at u=1/3 after multiplication by 27/4.
basisPower = 27 / 4 * [0, 1, -2, 1];
motion = bmtpEngine.createProgressPolynomialMotion( ...
    baseMotion, initialState, goalState, 1, 0.2, 0.02, ...
    "unitOneSidedProgressPolynomial", basisPower);
progress = (motion.position_deg(:, 1) - initialState.position_deg(1)) / ...
    (goalState.position_deg(1) - initialState.position_deg(1));
directLateral_deg = initialState.position_deg(2) + ...
    (goalState.position_deg(2) - initialState.position_deg(2)) * progress;
offset_deg = motion.position_deg(:, 2) - directLateral_deg;

verifyEqual(testCase, motion.Polynomial.Degree, 9);
verifyEqual(testCase, motion.FinalTime_s, baseMotion.FinalTime_s, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, motion.position_deg([1 end], :), ...
    [initialState.position_deg; goalState.position_deg], "AbsTol", 1e-9);
verifyGreaterThanOrEqual(testCase, min(offset_deg), -1e-10);
verifyGreaterThan(testCase, max(offset_deg), 0.19);
end

function testStaticRegionSolverReturnsPlaneWitness(testCase)
% Exercise the generic numeric-region SOCP without obstacle package inputs.
initialState = createState(0, [-2 0]);
goalState = createState(10, [2 0]);
seed = struct("Index", 1, "Source", "unitDirect", ...
    "tau", [0; 1], "position_deg", [-2 0; 2 0], ...
    "CorridorBoundary_deg", zeros(0, 2));
regions_deg = { [3 3; 4 3; 4 4; 3 4] };
coverage = struct("Passed", true, "RegionCount", 1);
[motion, diagnostics] = bmtpEngine.solve( ...
    seed, regions_deg, coverage, initialState, goalState, ...
    testCase.TestData.Limits, testCase.TestData.Options);
verifyTrue(testCase, motion.Success, motion.Message);
verifyTrue(testCase, diagnostics.Accepted);
verifyTrue(testCase, motion.PlaneCertificate.Passed);
verifyEqual(testCase, motion.position_deg([1 end], :), ...
    [initialState.position_deg; goalState.position_deg], "AbsTol", 1e-8);
end

function testTimedRegionAppliesOnlyToOverlappingSpans(testCase)
% Certify a structurally different moving-polygon time-cell assignment.
initialState = createState(0, [-4 0]);
goalState = createState(20, [4 0]);
seed = struct("Index", 1, "Source", "unitTimedRoute", ...
    "tau", [0; 1], "position_deg", [-4 0; 4 0], ...
    "CorridorBoundary_deg", zeros(0, 2));
regions_deg = {[2 -1; 3 -1; 3 1; 2 1]};
coverage = struct( ...
    "Passed", true, "RegionCount", 1, "ExactRegionCount", 1, ...
    "RegionActiveTauInterval", [0 0.25], "TimedSegmentCount", 4);
options = createOptions("fixedArrival");
[motion, diagnostics] = bmtpEngine.solve( ...
    seed, regions_deg, coverage, initialState, goalState, ...
    testCase.TestData.Limits, options);
verifyTrue(testCase, motion.Success, motion.Message);
verifyTrue(testCase, diagnostics.Accepted);
verifyEqual(testCase, motion.PlaneCertificate.Kind, "timeCellDegreeOne");
verifyEqual(testCase, motion.PlaneCertificate.AllPairCount, 2);
verifyEqual(testCase, ...
    sum(motion.PlaneCertificate.RegionActiveBySegment, 1), 2);
verifyTrue(testCase, motion.PlaneCertificate.Passed);
end

function testPublicWrapperAcceptsDocumentedArities(testCase)
% Keep both BMTP wrapper forms as direct engine dispatches.
initialState = createState(0, [-2 0]);
goalState = createState(10, [2 0]);
seed = struct("Index", 1, "Source", "unitDirect", ...
    "tau", [0; 1], "position_deg", [-2 0; 2 0], ...
    "CorridorBoundary_deg", zeros(0, 2));
regions_deg = {[3 3; 4 3; 4 4; 3 4]};
coverage = struct("Passed", true, "RegionCount", 1);
[candidate, diagnostics, restart] = planTrajBmtp( ...
    seed, regions_deg, coverage, initialState, goalState, ...
    testCase.TestData.Limits, testCase.TestData.Options);
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyTrue(testCase, diagnostics.Accepted);
verifyTrue(testCase, isstruct(restart) && isscalar(restart));
[candidate, diagnostics, restart] = planTrajBmtp( ...
    seed, regions_deg, coverage, initialState, goalState, ...
    testCase.TestData.Limits, testCase.TestData.Options, struct());
verifyTrue(testCase, candidate.Success, candidate.Message);
verifyTrue(testCase, diagnostics.Accepted);
verifyTrue(testCase, isstruct(restart) && isscalar(restart));
end

function testPublicWrapperAcceptsReturnedRestart(testCase)
% Feed a certified cold-solve restart through the documented eight-input form.
initialState = createState(0, [-2 0]);
goalState = createState(10, [2 0]);
seed = struct("Index", 1, "Source", "unitDirect", ...
    "tau", [0; 1], "position_deg", [-2 0; 2 0], ...
    "CorridorBoundary_deg", zeros(0, 2));
regions_deg = {[3 3; 4 3; 4 4; 3 4]};
coverage = struct("Passed", true, "RegionCount", 1);
options = createOptions("fixedArrival");
[coldCandidate, coldDiagnostics, restart] = planTrajBmtp( ...
    seed, regions_deg, coverage, initialState, goalState, ...
    testCase.TestData.Limits, options);
[warmCandidate, warmDiagnostics] = planTrajBmtp( ...
    seed, regions_deg, coverage, initialState, goalState, ...
    testCase.TestData.Limits, options, restart);

verifyTrue(testCase, coldCandidate.Success, coldCandidate.Message);
verifyTrue(testCase, coldDiagnostics.Accepted);
verifyTrue(testCase, warmCandidate.Success, warmCandidate.Message);
verifyTrue(testCase, warmDiagnostics.Accepted);
verifyEqual(testCase, warmCandidate.position_deg([1 end], :), ...
    coldCandidate.position_deg([1 end], :), "AbsTol", 1e-8);
verifyEqual(testCase, warmCandidate.MotionDuration_s, ...
    coldCandidate.MotionDuration_s, "AbsTol", 1e-8);
end

function testPublicWrapperRejectsUnsupportedArities(testCase)
% Return InvalidCall before the wrapper references absent input arguments.
verifyError(testCase, @() planTrajBmtp(), "planTrajBmtp:InvalidCall");
verifyError(testCase, @() planTrajBmtp( ...
    struct(), struct(), struct(), struct(), struct(), struct()), ...
    "planTrajBmtp:InvalidCall");
end

function testOversizedWarmRouteUsesBoundedSegmentCount(testCase)
% Keep proposal waypoint density from inflating the conic decision dimension.
initialState = createState(0, [-2 0]);
goalState = createState(10, [2 0]);
position_deg = [linspace(-2, 2, 31).', zeros(31, 1)];
seed = struct("Index", 1, "Source", "denseUnitWarmRoute", ...
    "tau", linspace(0, 1, 31).', "position_deg", position_deg, ...
    "CorridorBoundary_deg", zeros(0, 2));
regions_deg = {[3 3; 4 3; 4 4; 3 4]};
grouping = struct("Applied", true);
coverage = struct( ...
    "Passed", true, "RegionCount", 1, ...
    "ConservativeGrouping", grouping);
[motion, diagnostics] = bmtpEngine.solve( ...
    seed, regions_deg, coverage, initialState, goalState, ...
    testCase.TestData.Limits, testCase.TestData.Options);
verifyTrue(testCase, motion.Success, motion.Message);
verifyTrue(testCase, diagnostics.WarmRouteResampled);
verifyEqual(testCase, diagnostics.OriginalSeedSegmentCount, 30);
verifyEqual(testCase, diagnostics.OptimizerSpanCount, 20);
end

function state = createState(time_s, position_deg)
% Create one normalized rest state used by engine-only tests.
state = struct("time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", zeros(size(position_deg)), ...
    "acceleration_deg_s2", zeros(size(position_deg)));
end

function limits = createLimits()
% Create symmetric two-axis physical and workspace limits.
limits = struct( ...
    "azimuthInterval_deg", [-10 10], ...
    "elevationInterval_deg", [-10 10], ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [2 2]);
end

function options = createOptions(goalTimeMode)
% Create the complete small engine option contract without planner defaults.
options = struct( ...
    "GoalTimeMode", string(goalTimeMode), ...
    "SampleTime_s", 0.02, ...
    "ConstraintTolerance", 1e-7, ...
    "CollisionClearanceTolerance_deg", 1e-7, ...
    "ArrivalTimeTolerance_s", 1e-7, ...
    "EnablePlaneReuse", false, ...
    "PlaneReuseImprovementTolerance_s", 1e-7, ...
    "AllowAzimuthWrapping", false, ...
    "MaximumNlpIterations", 100, ...
    "MaximumSolverTime_s", 10);
end
