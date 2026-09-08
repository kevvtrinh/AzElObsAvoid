function tests = testPlannerLimits
%% Section 0: Header & Readme
% SYNTAX
%   tests = testPlannerLimits
% PURPOSE
%   Verify combined derivative magnitudes across public planner boundaries.
% INPUTS
%   None.
% OUTPUTS
%   Deterministic MATLAB function tests.
% UNITS
%   Coordinate units, seconds, and their motion derivatives.

%% Section 1: Register Tests
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
    addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'), fullfile(repositoryRoot, 'examples'), fullfile(repositoryRoot, 'sandbox'), fullfile(repositoryRoot, 'offlinesandbox'));
    testCase.TestData.Fixtures = testSupport.plannerFixtures();
end

function testCombinedPlanningMatchesExplicitAxisLimits(testCase)
    % The same physical axis allocation must produce the same motion in both modes.
    fixtures = testCase.TestData.Fixtures;
    initial  = fixtures.State(3, [0 0], [0 0], [0 0]);
    goal     = fixtures.State(33, [8 3], [0 0], [0 0]);
    combined = fixtures.PhysicalLimits(2, 1, 2.5);
    separate = fixtures.PhysicalLimits([2 2] / sqrt(2), [1 1] / sqrt(2), [2.5 2.5] / sqrt(2));
    for mode = ["earliestArrival", "fixedArrival"]
        options = struct("GoalTimeMode", mode);
        result  = planner([], initial, goal, combined, options);
        reference = planner([], initial, goal, separate, options);
        assertTrue(testCase, result.Success, result.Message);
        assertTrue(testCase, reference.Success, reference.Message);
        verifyEqual(testCase, result.Inputs.limits, separate);
        verifyEqual(testCase, result.Polynomial, reference.Polynomial);
        verifyEqual(testCase, result.TrajectoryDuration_s, reference.TrajectoryDuration_s);
        validation = obstacleAvoidance.validateTrajectory(result);
        explicitValidation = obstacleAvoidance.validateTrajectory(result, [], initial, goal, combined, result.Options);
        verifyTrue(testCase, validation.Passed, validation.Message);
        verifyTrue(testCase, explicitValidation.Passed, explicitValidation.Message);
        verifyLessThanOrEqual(testCase, max(vecnorm(result.velocity_units_s, 2, 2)), combined.maxVelocity_units_s + 1e-6);
        verifyLessThanOrEqual(testCase, max(vecnorm(result.acceleration_units_s2, 2, 2)), combined.maxAcceleration_units_s2 + 1e-6);
        verifyLessThanOrEqual(testCase, max(vecnorm(result.jerk_units_s3, 2, 2)), combined.maxJerk_units_s3 + 1e-6);
    end
end

function testSeparateColumnsAndWorkspaceRemainUnscaled(testCase)
    % Preserve unequal axis limits, numeric normalization, and workspace defaults.
    fixtures = testCase.TestData.Fixtures;
    limits   = fixtures.PhysicalLimits(single([2; 3]), uint16([1; 4]), [5; 6]);
    limits.xInterval_units   = [-20; 40];
    limits.yInterval_units = [];
    resolved = obstacleAvoidance.input.normalizePlannerLimits(limits);
    verifyEqual(testCase, resolved.maxVelocity_units_s, [2 3]);
    verifyEqual(testCase, resolved.maxAcceleration_units_s2, [1 4]);
    verifyEqual(testCase, resolved.maxJerk_units_s3, [5 6]);
    verifyEqual(testCase, resolved.xInterval_units, [-20 40]);
    verifyEqual(testCase, resolved.yInterval_units, [-90 90]);
    verifyEqual(testCase, obstacleAvoidance.input.normalizePlannerLimits(resolved), resolved);
    combined = fixtures.PhysicalLimits(single(2), uint16(1), 2.5);
    combined = rmfield(combined, {'xInterval_units', 'yInterval_units'});
    resolved = obstacleAvoidance.input.normalizePlannerLimits(combined);
    verifyEqual(testCase, resolved.maxVelocity_units_s, [2 2] / sqrt(2));
    verifyEqual(testCase, resolved.maxAcceleration_units_s2, [1 1] / sqrt(2));
    verifyEqual(testCase, resolved.maxJerk_units_s3, [2.5 2.5] / sqrt(2));
    verifyEqual(testCase, resolved.xInterval_units, [-180 180]);
    verifyEqual(testCase, resolved.yInterval_units, [-90 90]);
    verifyEqual(testCase, obstacleAvoidance.input.normalizePlannerLimits(resolved), resolved);
end

function testAllMixedFormsRejectedAtPublicBoundaries(testCase)
    % Exercise all six ways of mixing scalar and vector derivative limits.
    fixtures = testCase.TestData.Fixtures;
    initial  = fixtures.State(0, [0 0], [0 0], [0 0]);
    goal     = fixtures.State(30, [8 3], [0 0], [0 0]);
    target   = struct("time_s", [0; 30], "position_units", [6 2; 9 5]);
    names    = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];
    options  = planner();
    for mask = 1:6
        limits = fixtures.PhysicalLimits(2, 1, 2.5);
        for fieldIndex = 1:3
            if bitget(mask, fieldIndex)
                limits.(names(fieldIndex)) = repmat(limits.(names(fieldIndex)), 1, 2);
            end
        end
        verifyError(testCase, @() planner([], initial, goal, limits), "planner:MixedLimitModes");
        verifyError(testCase, @() planner([], initial, struct("time_s", 30, "targetMotion", target), limits), "planner:MixedLimitModes");
        verifyError(testCase, @() obstacleAvoidance.validateTrajectory(struct(), [], initial, goal, limits, options), "planner:MixedLimitModes");
    end
end

function testCombinedInterceptMatchesExplicitAxisLimits(testCase)
    % Earliest interception must normalize before computing its exact search clock.
    fixtures = testCase.TestData.Fixtures;
    initial  = fixtures.State(0, [0 0], [0 0], [0 0]);
    target   = struct("time_s", [0; 30], "position_units", [6 2; 9 5]);
    combined = fixtures.PhysicalLimits(2, 1, 2.5);
    separate = fixtures.PhysicalLimits([2 2] / sqrt(2), [1 1] / sqrt(2), [2.5 2.5] / sqrt(2));
    for mode = ["earliest", "specifiedTime"]
        options = struct("GoalTimeMode", "earliestArrival");
        goal = struct("time_s", 60, "targetMotion", target);
        if mode == "specifiedTime"
            options.GoalTimeMode = "fixedArrival";
            goal.time_s = 15;
        end
        [result, diagnosis] = planner([], initial, goal, combined, options);
        reference = planner([], initial, goal, separate, options);
        assertTrue(testCase, result.Success, result.Message);
        assertTrue(testCase, reference.Success, reference.Message);
        verifyEqual(testCase, result.Inputs.limits, separate);
        verifyEqual(testCase, result.Intercept.Time_s, reference.Intercept.Time_s);
        verifyEqual(testCase, result.Polynomial, reference.Polynomial);
        validation = obstacleAvoidance.validateTrajectory(result);
        verifyTrue(testCase, validation.Passed, validation.Message);
        if mode == "earliest"
            search = diagnosis.InterceptSearch;
            verifyEqual(testCase, search.Policy, "completePiecewisePolynomialDirect");
        end
    end
end

function testEndpointAboveAllocatedShareReturnsExpectedFailure(testCase)
    % A combined magnitude is not duplicated as the full limit on each axis.
    fixtures = testCase.TestData.Fixtures;
    initial  = fixtures.State(0, [0 0], [1.5 0], [0 0]);
    goal     = fixtures.State(30, [8 3], [0 0], [0 0]);
    limits   = fixtures.PhysicalLimits(2, 1, 2.5);
    result   = planner([], initial, goal, limits);
    verifyFalse(testCase, result.Success);
    verifyEqual(testCase, result.TerminationReason, "dynamicEndpointInfeasible");
    verifyEqual(testCase, result.Inputs.limits.maxVelocity_units_s, [2 2] / sqrt(2));
end

function testCombinedValidationChecksEveryDerivative(testCase)
    % The analytic motion reaches v=2, a=2, j=1. Each scalar cap below those
    % equal-share requirements must fail even though duplicating it would pass.
    fixtures = testCase.TestData.Fixtures;
    motion   = fixtures.ConstantJerkTrajectory(2);
    initial  = fixtures.State(0, [0 0], [0 0], [0 0]);
    goal     = fixtures.State(2, [4/3 0], [2 0], [2 0]);
    options  = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "fixedArrival"));
    names    = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"];
    checks   = ["VelocityWithinLimits", "AccelerationWithinLimits", "JerkWithinLimits"];
    caps     = [2.5 2.5 1.2];
    for fieldIndex = 1:3
        limits = fixtures.PhysicalLimits(100, 100, 100);
        limits.(names(fieldIndex)) = caps(fieldIndex);
        validation = obstacleAvoidance.validateTrajectory(motion, [], initial, goal, limits, options);
        verifyFalse(testCase, validation.Passed);
        verifyFalse(testCase, validation.(checks(fieldIndex)));
        otherChecks = checks(checks ~= checks(fieldIndex));
        for checkName = otherChecks
            verifyTrue(testCase, validation.(checkName));
        end
    end
end

function testCombinedVelocityViolationBetweenSamplesIsRejected(testCase)
    % Zero endpoint velocity samples cannot conceal the interior unit peak.
    fixtures = testCase.TestData.Fixtures;
    motion   = fixtures.InteriorVelocityPeakTrajectory();
    initial  = fixtures.State(0, [0 0], [0 0], [4 0]);
    goal     = fixtures.State(1, [2/3 0], [0 0], [-4 0]);
    limits   = fixtures.PhysicalLimits(1.1, 10, 20);
    options  = obstacleAvoidance.input.resolvePlannerOptions(struct("GoalTimeMode", "fixedArrival"));
    validation = obstacleAvoidance.validateTrajectory(motion, [], initial, goal, limits, options);
    verifyEqual(testCase, motion.velocity_units_s, zeros(2, 2));
    verifyFalse(testCase, validation.Passed);
    verifyFalse(testCase, validation.VelocityWithinLimits);
    verifyTrue(testCase, validation.AccelerationWithinLimits);
    verifyTrue(testCase, validation.JerkWithinLimits);
end

function testMalformedLimitsAndScalarPositionIntervalsAreRejected(testCase)
    fixtures = testCase.TestData.Fixtures;
    limits   = fixtures.PhysicalLimits(2, 1, 2.5);
    verifyError(testCase, @() obstacleAvoidance.input.normalizePlannerLimits(rmfield(limits, 'maxJerk_units_s3')), "planner:InvalidLimits");
    limits.maxVelocity_units_s = [1 2 3];
    verifyError(testCase, @() obstacleAvoidance.input.normalizePlannerLimits(limits), "planner:InvalidLimits");
    limits = fixtures.PhysicalLimits(2, 1, 2.5);
    limits.xInterval_units = 180;
    verifyError(testCase, @() obstacleAvoidance.input.normalizePlannerLimits(limits), "MATLAB:planner:incorrectNumel");
end

function testExampleJerkOverrideCannotHideMixedLimitForms(testCase)
    verifyError(testCase, @() resolveExampleOptions(struct("MaxJerk_units_s3", 2)), "resolveExampleOptions:InvalidMaxJerk");
    [~, displayOptions] = resolveExampleOptions(struct("MaxJerk_units_s3", [2; 3]));
    verifyEqual(testCase, displayOptions.MaxJerk_units_s3, [2 3]);
end

function testNativeSandboxResolvesCombinedOverridesBeforeGraphics(testCase)
    % The controls receive normalized components, and partial mixed overrides fail.
    overrides = struct();
    overrides.FigureVisible          = "off";
    overrides.MaxVelocity_units_s      = 2;
    overrides.MaxAcceleration_units_s2 = 1;
    overrides.MaxJerk_units_s3         = 2.5;
    sandbox = obstacleAvoidanceSandbox(overrides);
    testCase.addTeardown(@() close(sandbox.FigureHandle));
    verifyEqual(testCase, sandbox.Options.MaxVelocity_units_s, [2 2] / sqrt(2));
    verifyEqual(testCase, sandbox.Options.MaxAcceleration_units_s2, [1 1] / sqrt(2));
    verifyEqual(testCase, sandbox.Options.MaxJerk_units_s3, [2.5 2.5] / sqrt(2));
    verifyEqual(testCase, sandbox.Options.WorkspaceXInterval_units, [-180 180]);
    controls = sandbox.GoalMode.GraphicsHandles.Controls;
    for name = ["VelocityHandles", "AccelerationHandles", "JerkHandles"]
        displayed = [str2double(get(controls.(name).FirstHandle, 'String')), str2double(get(controls.(name).SecondHandle, 'String'))];
        verifyEqual(testCase, displayed(1), displayed(2));
        if name == "VelocityHandles"
            verifyEqual(testCase, displayed, sandbox.Options.MaxVelocity_units_s);
        elseif name == "AccelerationHandles"
            verifyEqual(testCase, displayed, sandbox.Options.MaxAcceleration_units_s2);
        else
            verifyEqual(testCase, displayed, sandbox.Options.MaxJerk_units_s3);
        end
    end
    overrides.MaxJerk_units_s3 = [2.5 2.5];
    verifyError(testCase, @() obstacleAvoidanceSandbox(overrides), "planner:MixedLimitModes");
end

function testOfflineJsonAcceptsCombinedLimitsAndRejectsMixing(testCase)
    % Exercise the actual file adapter and its independently validated result.
    fixtures = testCase.TestData.Fixtures;
    request  = struct();
    request.schemaVersion = "offlineSandboxRequest/v1";
    request.requestId     = "combined-limit-regression";
    request.obstacles     = [];
    request.initialState  = fixtures.State(0, [0 0], [0 0], [0 0]);
    request.goalState     = fixtures.State(30, [8 3], [0 0], [0 0]);
    request.limits        = fixtures.PhysicalLimits(2, 1, 2.5);
    request.options       = struct();
    requestPath = string(tempname) + ".json";
    resultPath  = string(tempname) + ".json";
    testCase.addTeardown(@() deleteJsonFiles(requestPath, resultPath));
    writeJson(requestPath, request);
    [response, bundle] = offlineSandbox.runPlanningRequest(requestPath, resultPath);
    verifyTrue(testCase, response.result.Success, response.result.Message);
    verifyTrue(testCase, response.validation.Passed, response.validation.Message);
    verifyEqual(testCase, bundle.PlannerInputs.limits.maxVelocity_units_s, [2 2] / sqrt(2));
    verifyEqual(testCase, bundle.PlannerInputs.limits.maxAcceleration_units_s2, [1 1] / sqrt(2));
    verifyEqual(testCase, bundle.PlannerInputs.limits.maxJerk_units_s3, [2.5 2.5] / sqrt(2));
    request.limits.maxAcceleration_units_s2 = [1 1];
    writeJson(requestPath, request);
    verifyError(testCase, @() offlineSandbox.runPlanningRequest(requestPath, resultPath), "planner:MixedLimitModes");
end

function writeJson(path, request)
    fileIdentifier = fopen(path, 'w');
    cleanup = onCleanup(@() fclose(fileIdentifier));
    fprintf(fileIdentifier, '%s', jsonencode(request));
end

function deleteJsonFiles(varargin)
    for fileIndex = 1:nargin
        if isfile(varargin{fileIndex})
            delete(varargin{fileIndex});
        end
    end
end
