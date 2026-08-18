function tests = testContinuousJerkRetimer
%% Section 0: Header & Readme
% SYNTAX
%   tests = testContinuousJerkRetimer
%**************************************************************************
% PURPOSE
%   - Verify continuous-sine, modified-sigmoid, and compatibility profiles.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (MATLAB function-test array)
%       Deterministic continuous-jerk retimer tests.
%**************************************************************************
% UNITS
%   - Position is degrees and derivatives use seconds.
%**************************************************************************

%% Section 1: Create The Function-Test Suite

root = fileparts(fileparts(mfilename("fullpath")));
addpath(root);
tests = functiontests(localfunctions);
end

%% Section 2: Local Test Cases

function testModifiedSigmoidHasContinuousTangentialSnap(testCase)
% The paper's sigmoid must have continuous jerk and tangential snap.
[route_deg, obstacleField, initialState, goalState, limits] = ...
    retimerFixture();
limits.maxTangentialSnap_deg_s4 = 40;
options = planAzElMotion();
options.CollisionTimePaddingSamples = 0;
options.JerkProfileType = "continuoussigmoid";

[~, timedPath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, options, 0);

testCase.verifyTrue(timedPath.Success, timedPath.Message);
testCase.verifyTrue(timedPath.ConstraintDiagnostics.ContinuousJerkCertified);
testCase.verifyTrue( ...
    timedPath.ConstraintDiagnostics.ContinuousTangentialSnapCertified);
testCase.verifyGreaterThan( ...
    timedPath.ConstraintDiagnostics.PeakTangentialSnap_deg_s4, 0);
testCase.verifyEqual(timedPath.RetimerType, ...
    "certifiedQuadratureSpatialContinuousSigmoidJerk");
profiles = timedPath.SegmentProfiles;
for profileIndex = 1:numel(profiles)
    profile = profiles(profileIndex);
    testCase.verifyEqual(profile.JerkProfileType, "continuoussigmoid");
    testCase.verifyTrue(all(profile.PhaseLaw == "sigmoidRise" | ...
        profile.PhaseLaw == "sigmoidFall" | ...
        profile.PhaseLaw == "constant"));
    hasSigmoidRamp = any(profile.PhaseLaw == "sigmoidRise");
    if hasSigmoidRamp
        testCase.verifyEqual(numel(profile.PhaseLaw), 15);
    else
        testCase.verifyEqual(profile.PhaseJerk_deg_s3, ...
            zeros(size(profile.PhaseJerk_deg_s3)), "AbsTol", 1e-12);
    end
end
end

function testModifiedSigmoidRespectsTangentialSnapLimit(testCase)
% A finite tangential snap limit must lengthen and certify the profile.
[route_deg, obstacleField, initialState, goalState, limits] = ...
    retimerFixture();
options = planAzElMotion();
options.CollisionTimePaddingSamples = 0;
options.JerkProfileType = "continuoussigmoid";

limits.maxTangentialSnap_deg_s4 = 1000;
[~, loosePath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, options, 0);
limits.maxTangentialSnap_deg_s4 = 12;
[~, boundedPath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, options, 0);

testCase.verifyTrue(boundedPath.Success, boundedPath.Message);
testCase.verifyTrue( ...
    boundedPath.ConstraintDiagnostics.TangentialSnapConstrained);
testCase.verifyTrue( ...
    boundedPath.ConstraintDiagnostics.TangentialSnapSatisfied);
testCase.verifyLessThanOrEqual( ...
    boundedPath.ConstraintDiagnostics.PeakTangentialSnap_deg_s4, ...
    limits.maxTangentialSnap_deg_s4 + 1e-10);
testCase.verifyGreaterThan(boundedPath.MinimumMotionDuration_s, ...
    loosePath.MinimumMotionDuration_s);
end

function testModifiedSigmoidQuadratureMatchesAdaptiveIntegral(testCase)
% Fixed quadrature must match an independent adaptive phase integral.
[route_deg, obstacleField, initialState, goalState, limits] = ...
    retimerFixture();
limits.maxTangentialSnap_deg_s4 = 40;
options = planAzElMotion();
options.CollisionTimePaddingSamples = 0;
options.JerkProfileType = "continuoussigmoid";
[~, timedPath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, options, 0);

testCase.verifyTrue(timedPath.Success, timedPath.Message);
profiles = timedPath.SegmentProfiles;
for profileIndex = 1:numel(profiles)
    profile = profiles(profileIndex);
    phaseCount = numel(profile.PhaseLaw);
    for phaseIndex = 1:phaseCount
        law = profile.PhaseLaw(phaseIndex);
        if law ~= "sigmoidRise" && law ~= "sigmoidFall"
            continue;
        end
        if law == "sigmoidRise"
            shape = @(normalizedTime) referenceModifiedLogistic( ...
                normalizedTime);
        else
            shape = @(normalizedTime) 1 - referenceModifiedLogistic( ...
                normalizedTime);
        end
        accelerationIntegral = integral(shape, 0, 1, ...
            "AbsTol", 1e-13, "RelTol", 1e-13);
        velocityIntegral = integral( ...
            @(normalizedTime) (1 - normalizedTime) .* ...
            shape(normalizedTime), 0, 1, ...
            "AbsTol", 1e-13, "RelTol", 1e-13);
        positionIntegral = 0.5 * integral( ...
            @(normalizedTime) (1 - normalizedTime).^2 .* ...
            shape(normalizedTime), 0, 1, ...
            "AbsTol", 1e-13, "RelTol", 1e-13);

        duration_s = profile.PhaseDuration_s(phaseIndex);
        jerk_deg_s3 = profile.PhaseJerk_deg_s3(phaseIndex);
        startPosition_deg = profile.PhaseStartPosition_deg(phaseIndex);
        startSpeed_deg_s = profile.PhaseStartSpeed_deg_s(phaseIndex);
        startAcceleration_deg_s2 = ...
            profile.PhaseStartAcceleration_deg_s2(phaseIndex);
        expectedAcceleration_deg_s2 = startAcceleration_deg_s2 + ...
            jerk_deg_s3 * duration_s * accelerationIntegral;
        expectedSpeed_deg_s = startSpeed_deg_s + ...
            startAcceleration_deg_s2 * duration_s + ...
            jerk_deg_s3 * duration_s^2 * velocityIntegral;
        expectedPosition_deg = startPosition_deg + ...
            startSpeed_deg_s * duration_s + ...
            0.5 * startAcceleration_deg_s2 * duration_s^2 + ...
            jerk_deg_s3 * duration_s^3 * positionIntegral;
        if phaseIndex < phaseCount
            actualPosition_deg = ...
                profile.PhaseStartPosition_deg(phaseIndex + 1);
            actualSpeed_deg_s = ...
                profile.PhaseStartSpeed_deg_s(phaseIndex + 1);
            actualAcceleration_deg_s2 = ...
                profile.PhaseStartAcceleration_deg_s2(phaseIndex + 1);
        else
            actualPosition_deg = profile.Length_deg;
            actualSpeed_deg_s = profile.EndSpeed_deg_s;
            actualAcceleration_deg_s2 = 0;
        end
        testCase.verifyEqual(actualPosition_deg, expectedPosition_deg, ...
            "AbsTol", 2e-11);
        testCase.verifyEqual(actualSpeed_deg_s, expectedSpeed_deg_s, ...
            "AbsTol", 2e-11);
        testCase.verifyEqual(actualAcceleration_deg_s2, ...
            expectedAcceleration_deg_s2, "AbsTol", 2e-11);
    end
end
end

function testContinuousSineJerkIsCertified(testCase)
% The sine family must produce finite, bounded, continuous scalar jerk.
[route_deg, obstacleField, initialState, goalState, limits] = ...
    retimerFixture();
options = planAzElMotion();
options.CollisionTimePaddingSamples = 0;
options.JerkProfileType = "continuoussine";

[~, timedPath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, options, 0);

testCase.verifyTrue(timedPath.Success, timedPath.Message);
testCase.verifyTrue(timedPath.ConstraintDiagnostics.FiniteJerkCertified);
testCase.verifyTrue(timedPath.ConstraintDiagnostics.ContinuousJerkCertified);
testCase.verifyEqual(timedPath.RetimerType, ...
    "certifiedAnalyticSpatialContinuousSineJerk");
testCase.verifyTrue(all(isfinite(timedPath.jerk_deg_s3), "all"));
testCase.verifyLessThanOrEqual(max(abs(timedPath.jerk_deg_s3), [], 1), ...
    limits.maxJerk_deg_s3 + 1e-10);
testCase.verifyFalse(any(queryAzElTimedPathCollision( ...
    obstacleField, timedPath.time_s, timedPath.position_deg)));

profiles = timedPath.SegmentProfiles;
for profileIndex = 1:numel(profiles)
    profile = profiles(profileIndex);
    testCase.verifyEqual(profile.JerkProfileType, "continuoussine");
    constantPhase = profile.PhaseLaw == "constant";
    testCase.verifyEqual(profile.PhaseJerk_deg_s3(constantPhase), ...
        zeros(nnz(constantPhase), 1), "AbsTol", 1e-12);
end
end

function testPiecewiseConstantCompatibilityMode(testCase)
% The previous profile must remain selectable and keep its status honest.
[route_deg, obstacleField, initialState, goalState, limits] = ...
    retimerFixture();
continuousOptions = planAzElMotion();
continuousOptions.CollisionTimePaddingSamples = 0;
legacyOptions = continuousOptions;
legacyOptions.JerkProfileType = "piecewiseconstant";

[~, continuousPath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, ...
    continuousOptions, 0);
[~, legacyPath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, ...
    legacyOptions, 0);

testCase.verifyTrue(continuousPath.Success, continuousPath.Message);
testCase.verifyTrue(legacyPath.Success, legacyPath.Message);
testCase.verifyFalse(legacyPath.ConstraintDiagnostics.ContinuousJerkCertified);
testCase.verifyEqual(legacyPath.ConstraintDiagnostics.JerkProfileType, ...
    "piecewiseconstant");
testCase.verifyGreaterThanOrEqual( ...
    continuousPath.MinimumMotionDuration_s, ...
    legacyPath.MinimumMotionDuration_s - 1e-10);
end

function testUnlimitedJerkDoesNotRequireSnapHistory(testCase)
% An acceleration-only profile must not fail because snap is unavailable.
[route_deg, obstacleField, initialState, goalState, limits] = ...
    retimerFixture();
limits.maxJerk_deg_s3 = [Inf Inf];
options = planAzElMotion();
options.CollisionTimePaddingSamples = 0;

[~, timedPath] = azElInternal.retimeAzElSpatialPath( ...
    route_deg, obstacleField, initialState, goalState, limits, options, 0);

testCase.verifyTrue(timedPath.Success, timedPath.Message);
diagnostics = timedPath.ConstraintDiagnostics;
testCase.verifyFalse(diagnostics.TangentialSnapConstrained);
testCase.verifyTrue(diagnostics.TangentialSnapSatisfied);
testCase.verifyTrue(isnan(diagnostics.PeakTangentialSnap_deg_s4));
testCase.verifyTrue(isnan(diagnostics.TangentialSnapMargin_deg_s4));
end

function testJerkProfileOptionDefaultAndValidation(testCase)
% The public option must use one default and reject unsupported values.
options = planAzElMotion();
testCase.verifyEqual(options.JerkProfileType, "continuousSine");

initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct("time_s", 10, "position_deg", [1 0]);
limits = struct( ...
    "maxVelocity_deg_s", [1 1], ...
    "maxAcceleration_deg_s2", [1 1], ...
    "maxJerk_deg_s3", [1 1]);
testCase.verifyError(@() planAzElMotion( ...
    [], initialState, goalState, limits, ...
    struct("JerkProfileType", "unsupported")), ...
    "planAzElMotion:InvalidJerkProfileType");

limits.maxTangentialSnap_deg_s4 = 10;
testCase.verifyError(@() planAzElMotion( ...
    [], initialState, goalState, limits, ...
    struct("JerkProfileType", "piecewiseConstant")), ...
    "planAzElMotion:UnsupportedSnapProfile");

limits = rmfield(limits, "maxTangentialSnap_deg_s4");
testCase.verifyError(@() planAzElMotion( ...
    [], initialState, goalState, limits, ...
    struct("JerkProfileType", "continuousSigmoid")), ...
    "planAzElMotion:SigmoidRequiresSnapLimit");
end

%% Section 3: Local Functions

function [route_deg, obstacleField, initialState, goalState, limits] = ...
        retimerFixture()
% PURPOSE
%   - Return one rounded-corner finite-jerk request without obstacles.
route_deg = [0 0; 10 0; 10 10];
obstacleField = buildAzElTimeObstacleField([]);
initialState = struct( ...
    "time_s", 0, "position_deg", route_deg(1, :), ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
goalState = struct( ...
    "time_s", 100, "position_deg", route_deg(end, :), ...
    "velocity_deg_s", [0 0], "acceleration_deg_s2", [0 0]);
limits = struct( ...
    "maxVelocity_deg_s", [5 4], ...
    "maxAcceleration_deg_s2", [3 2], ...
    "maxJerk_deg_s3", [2.5 2.5]);
end

function value = referenceModifiedLogistic(normalizedTime)
% PURPOSE
%   - Evaluate the paper law independently for adaptive integration.
value = zeros(size(normalizedTime));
value(normalizedTime >= 1) = 1;
interior = normalizedTime > 0 & normalizedTime < 1;
interiorTime = normalizedTime(interior);
logit = sqrt(3) / 2 * (1 ./ (1 - interiorTime) - 1 ./ interiorTime);
value(interior) = 1 ./ (1 + exp(-logit));
end
