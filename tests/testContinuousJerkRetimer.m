function tests = testContinuousJerkRetimer
%% Section 0: Header & Readme
% SYNTAX
%   tests = testContinuousJerkRetimer
%**************************************************************************
% PURPOSE
%   - Verify continuous-sine and compatibility finite-jerk profiles.
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
