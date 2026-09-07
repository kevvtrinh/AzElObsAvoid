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
%   - Position is coordinate units and time is seconds. Derivatives use units/s powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Add only the repository and independent trajectory-engine parents.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
    testCase.TestData.InitialState = createState(0, [0 0]);
    testCase.TestData.GoalState    = createState(10, [4 2]);
    testCase.TestData.Limits       = createLimits();
    testCase.TestData.Options      = createOptions("earliestArrival");
end

function testDirectMotionReachesRestEndpoint(testCase)
    % Check exact synchronized construction and polynomial reconstruction.
    initialState = testCase.TestData.InitialState;
    goalState    = testCase.TestData.GoalState;
    result       = bmtpEngine.createDirectMotion(initialState, goalState, testCase.TestData.Limits, testCase.TestData.Options);
    verifyTrue(testCase, result.Success, result.Message);
    verifyLessThan(testCase, result.ArrivalTime_s, goalState.time_s);
    verifyEqual(testCase, result.position_units(1, :), initialState.position_units, "AbsTol", 1e-12);
    verifyEqual(testCase, result.position_units(end, :), goalState.position_units, "AbsTol", 1e-9);
    verifyEqual(testCase, result.velocity_units_s(end, :), [0 0], "AbsTol", 1e-9);
    verifyEqual(testCase, result.acceleration_units_s2(end, :), [0 0], "AbsTol", 1e-9);
end

function testFixedTimeBelowMinimumReturnsStableFailure(testCase)
    % Keep physical infeasibility visible without throwing or clipping motion.
    options   = createOptions("fixedArrival");
    goalState = testCase.TestData.GoalState;
    goalState.time_s = 0.1;
    result = bmtpEngine.createDirectMotion(testCase.TestData.InitialState, goalState, testCase.TestData.Limits, options);
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "fixedTimeBelowMinimum");
    verifyTrue(testCase, isempty(result.time_s));
    verifyTrue(testCase, isfield(result, "Polynomial"));
end

function testEventWordReconstructionIsExact(testCase)
    % Exercise piecewise-constant jerk integration independently of the planner.
    initialState = createState(0, [0 0]);
    [motion, terminalState] = bmtpEngine.createMotionRecord(struct(), initialState, [0; 0.5; 1], [1 0; -1 0], 0.05, "unitEventWord");
    verifyEqual(testCase, terminalState.position_units, [0.125 0], "AbsTol", 1e-12);
    verifyEqual(testCase, terminalState.velocity_units_s, [0.25 0], "AbsTol", 1e-12);
    verifyEqual(testCase, terminalState.acceleration_units_s2, [0 0], "AbsTol", 1e-12);
    verifyEqual(testCase, motion.position_units(end, :), terminalState.position_units, "AbsTol", 1e-12);
end

function testOffsetSplinePreservesBaseClockAndEndpoints(testCase)
    % Verify the extracted quintic generator composes without changing arrival.
    baseMotion     = bmtpEngine.createDirectMotion(testCase.TestData.InitialState, testCase.TestData.GoalState, testCase.TestData.Limits, testCase.TestData.Options);
    midpointTime_s = 0.5 * (baseMotion.time_s(1) + baseMotion.ArrivalTime_s);
    motion         = bmtpEngine.createOffsetSplineMotion(baseMotion, [baseMotion.time_s(1); midpointTime_s; baseMotion.ArrivalTime_s], [0; 0.2; 0], 2, testCase.TestData.InitialState, 0.02, "unitOffsetSpline");
    verifyEqual(testCase, motion.ArrivalTime_s, baseMotion.ArrivalTime_s, "AbsTol", 1e-12);
    verifyEqual(testCase, motion.position_units([1 end], :), baseMotion.position_units([1 end], :), "AbsTol", 1e-9);
    verifyEqual(testCase, motion.velocity_units_s([1 end], :), zeros(2), "AbsTol", 1e-8);
    verifyEqual(testCase, motion.acceleration_units_s2([1 end], :), zeros(2), "AbsTol", 1e-8);
end

function testStaticRegionSolverReturnsPlaneWitness(testCase)
    % Exercise the generic numeric-region SOCP without obstacle package inputs.
    initialState = createState(0, [-2 0]);
    goalState    = createState(10, [2 0]);
    seed         = struct();
    seed.Index                = 1;
    seed.Source               = "unitDirect";
    seed.tau                  = [0; 1];
    seed.position_units         = [-2 0; 2 0];
    seed.ObstacleEnvelope_units = zeros(0, 2);
    regions_units = { [3 3; 4 3; 4 4; 3 4] };
    coverage    = struct();
    coverage.Passed      = true;
    coverage.RegionCount = 1;
    [motion, diagnostics] = bmtpEngine.solve(seed, regions_units, coverage, initialState, goalState, testCase.TestData.Limits, testCase.TestData.Options);
    verifyTrue(testCase, motion.Success, motion.Message);
    verifyTrue(testCase, diagnostics.Accepted);
    verifyTrue(testCase, motion.PlaneCertificate.Passed);
    verifyFalse(testCase, any(isfield(motion, {'FinalTime_s', 'MotionDuration_s'})));
    verifyTrue(testCase, all(isfield(motion, {'ArrivalTime_s', 'TrajectoryDuration_s'})));
    verifyGreaterThan(testCase, motion.PlaneCertificate.AnalyticPairCount, 0);
    verifyEqual(testCase, motion.PlaneCertificate.ReusedPairCount + motion.PlaneCertificate.AnalyticPairCount + motion.PlaneCertificate.ConicPairCount, motion.PlaneCertificate.AllPairCount);
    verifyEqual(testCase, motion.position_units([1 end], :), [initialState.position_units; goalState.position_units], "AbsTol", 1e-8);
    verifyTrue(testCase, diagnostics.TravelRefinementAttempted);
    verifyLessThanOrEqual(testCase, diagnostics.TravelRefinementFinalLength_units, diagnostics.TravelRefinementInitialLength_units);
    verifyEqual(testCase, diagnostics.TravelRefinementFinalDuration_s, diagnostics.TravelRefinementInitialDuration_s, "AbsTol", 1e-9);
end

function testTimedRegionAppliesOnlyToOverlappingSpans(testCase)
    % Certify a structurally different moving-polygon time-cell assignment.
    initialState = createState(0, [-4 0]);
    goalState    = createState(20, [4 0]);
    seed         = struct();
    seed.Index                = 1;
    seed.Source               = "unitTimedRoute";
    seed.tau                  = [0; 1];
    seed.position_units         = [-4 0; 4 0];
    seed.ObstacleEnvelope_units = zeros(0, 2);
    regions_units = {[2 -1; 3 -1; 3 1; 2 1]};
    coverage    = struct();
    coverage.Passed                  = true;
    coverage.RegionCount             = 1;
    coverage.ExactRegionCount        = 1;
    coverage.RegionActiveTauInterval = [0 0.25];
    coverage.TimedSegmentCount       = 4;
    options = createOptions("fixedArrival");
    [motion, diagnostics] = bmtpEngine.solve(seed, regions_units, coverage, initialState, goalState, testCase.TestData.Limits, options);
    verifyTrue(testCase, motion.Success, motion.Message);
    verifyTrue(testCase, diagnostics.Accepted);
    verifyEqual(testCase, motion.PlaneCertificate.Kind, "timeCellDegreeOne");
    verifyEqual(testCase, motion.PlaneCertificate.AllPairCount, 2);
    verifyEqual(testCase, sum(motion.PlaneCertificate.RegionActiveBySegment, 1), 2);
    verifyTrue(testCase, motion.PlaneCertificate.Passed);
    verifyEqual(testCase, motion.PlaneCertificate.ReusedPairCount + motion.PlaneCertificate.AnalyticPairCount + motion.PlaneCertificate.ConicPairCount, motion.PlaneCertificate.AllPairCount);
end

function testRedundantWarmRouteVerticesDoNotChangeRepresentation(testCase)
    % Make identical polylines produce one identical distance-balanced warm route.
    initialState = createState(0, [-2 0]);
    goalState    = createState(10, [2 0]);
    position_units = [linspace(-2, 2, 31).', zeros(31, 1)];
    denseSeed    = struct("Index", 1, "Source", "denseUnitWarmRoute", ...
        "tau", linspace(0, 1, 31).', "position_units", position_units, ...
        "ObstacleEnvelope_units", zeros(0, 2));
    sparseSeed = denseSeed;
    sparseSeed.Source       = "sparseUnitWarmRoute";
    sparseSeed.tau          = [0; 1];
    sparseSeed.position_units = position_units([1 end], :);
    regions_units = {[3 3; 4 3; 4 4; 3 4]};
    coverage    = struct();
    coverage.Passed      = true;
    coverage.RegionCount = 1;
    denseRequest    = bmtpEngine.createSolveRequest(denseSeed, regions_units, coverage, initialState, goalState, testCase.TestData.Limits, testCase.TestData.Options);
    sparseRequest   = bmtpEngine.createSolveRequest(sparseSeed, regions_units, coverage, initialState, goalState, testCase.TestData.Limits, testCase.TestData.Options);
    denseWarmStart  = bmtpEngine.createWarmStart(denseRequest);
    sparseWarmStart = bmtpEngine.createWarmStart(sparseRequest);
    verifyEqual(testCase, denseWarmStart.Route_units, sparseWarmStart.Route_units, "AbsTol", 1e-12);
    verifyEqual(testCase, denseWarmStart.ControlPoint_units, sparseWarmStart.ControlPoint_units, "AbsTol", 1e-12);
    verifyEqual(testCase, denseWarmStart.SegmentTime_s, sparseWarmStart.SegmentTime_s, "AbsTol", 1e-12);

    [motion, diagnostics] = bmtpEngine.solve(denseSeed, regions_units, coverage, initialState, goalState, testCase.TestData.Limits, testCase.TestData.Options);
    verifyTrue(testCase, motion.Success, motion.Message);
    verifyTrue(testCase, diagnostics.WarmRouteResampled);
    verifyEqual(testCase, diagnostics.OriginalSeedSegmentCount, 30);
    verifyEqual(testCase, diagnostics.OptimizerSpanCount, 3);
end

function state = createState(time_s, position_units)
    % Create one normalized rest state used by engine-only tests.
    state = struct("time_s", time_s, "position_units", position_units, ...
        "velocity_units_s", zeros(size(position_units)), ...
        "acceleration_units_s2", zeros(size(position_units)));
end

function limits = createLimits()
    % Create symmetric two-axis physical and workspace limits.
    limits = struct();
    limits.xInterval_units    = [-10 10];
    limits.yInterval_units  = [-10 10];
    limits.maxVelocity_units_s      = [2 2];
    limits.maxAcceleration_units_s2 = [1 1];
    limits.maxJerk_units_s3         = [2 2];
end

function options = createOptions(goalTimeMode)
    % Create the complete small engine option contract without planner defaults.
    options = struct("GoalTimeMode", string(goalTimeMode), ...
        "SampleTime_s", 0.02, ...
        "ConstraintTolerance", 1e-7, ...
        "CollisionClearanceTolerance_units", 1e-7, ...
        "ArrivalTimeTolerance_s", 1e-7, ...
        "WrapX", false, "WrapY", false);
end
