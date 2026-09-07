function tests = testSandboxRouteEconomy
%% Section 0: Header & Readme
% SYNTAX
%   tests = testSandboxRouteEconomy
%**************************************************************************
% PURPOSE
%   - Guard sandbox-style static and moving obstacle requests against
%     excessive joint travel and repeated lateral reversals.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function-test array)
%       Deterministic route-economy regression cases.
%**************************************************************************
% UNITS
%   - Position and accumulated two-axis travel are in coordinate units. Time is in
%     seconds, and derivative limits use coordinate units per second and its powers.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Add the repository and trajectory engine used by the public planner.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
    testCase.TestData.Request = createRequest();
end

function testSavedLongPath(testCase)
    % Use current options with the saved scene and preserve its shorter path.
    verifySavedRoute(testCase, "pathtoolong", 233.058989023, 117.744031226);
end

function testSavedLongPathTwo(testCase)
    % A differently placed obstacle and oblique endpoint chord exercise the gate.
    verifySavedRoute(testCase, "pathtoolong2", 242.064492067, 117.250241772);
end

function verifySavedRoute(testCase, name, baselineLength_units, baselineDuration_s)
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root, "offlinesandbox"));
    outputDirectory = tempname;
    mkdir(outputDirectory);
    cleanup         = onCleanup(@() rmdir(outputDirectory, 's')); %#ok<NASGU>
    loaded          = load(fullfile(root, "Rogue Examples", name + ".mat"), "diagnosisBundle");
    diagnosisBundle = loaded.diagnosisBundle;
    diagnosisBundle.PlannerOptions.GoalTimeMode = "earliestArrival";
    fixturePath = fullfile(outputDirectory, "scene.mat");
    save(fixturePath, "diagnosisBundle");
    [~, bundle] = offlineSandbox.replayDiagnosisBundle(fixturePath, fullfile(outputDirectory, "response.json"));
    result          = bundle.Result;
    resultDiagnosis = bundle.Diagnosis;
    validation      = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyLessThan(testCase, motionLength(result), baselineLength_units - 1);
    refinement = resultDiagnosis.PathRefinement;
    verifyGreaterThan(testCase, testSupport.diagnosisValue(refinement, "TravelRefinement.AcceptedCount"), 0);
    verifyLessThan(testCase, testSupport.diagnosisValue(refinement, "TravelRefinement.FinalLength_units"), testSupport.diagnosisValue(refinement, "TravelRefinement.InitialLength_units"));
    verifyLessThanOrEqual(testCase, result.TrajectoryDuration_s, baselineDuration_s + 1e-7);
end

function testStaticCircleHasOneEconomicalDetour(testCase)
    % Compare joint travel with the exact tangent-and-arc geometric lower bound.
    request          = testCase.TestData.Request;
    circleRadius_units = 2;
    safetyMargin_units = 0.25;
    angle_rad        = linspace(0, 2 * pi, 49).';
    angle_rad(end) = [];
    circle_units = circleRadius_units * [cos(angle_rad), sin(angle_rad)];
    obstacles  = obstacleAvoidance.obstacles.createObstacle("static circle", [0; request.goalState.time_s], circle_units(:, 1), circle_units(:, 2), safetyMargin_units);

    [result, resultDiagnosis] = runAndValidate(testCase, obstacles, request);
    protectedRadius_units     = circleRadius_units + safetyMargin_units;
    halfChord_units           = 0.5 * norm(request.goalState.position_units - request.initialState.position_units);
    tangentLength_units       = sqrt(halfChord_units^2 - protectedRadius_units^2);
    arcAngle_rad            = 2 * asin(protectedRadius_units / halfChord_units);
    geometricLowerBound_units = 2 * tangentLength_units + protectedRadius_units * arcAngle_rad;

    verifyLessThanOrEqual(testCase, motionLength(result), 1.01 * geometricLowerBound_units, "The circle detour exceeds the tangent-and-arc lower bound by over 1%%.");
    verifyLessThanOrEqual(testCase, lateralReversalCount(result), 1, "The circle detour repeatedly reverses its lateral joint motion.");
    details = resultDiagnosis.PathRefinement;
    indices = endsWith(details.Field, ".BoundaryRefinementCount");
    counts  = cell2mat(details.Value(indices));
    verifyGreaterThan(testCase, sum(counts), 0, "The fixed-clock clearance boundary was not refined.");
end

function testIrregularStaticObstacleAvoidsRepeatedJointMotion(testCase)
    % Exercise a concave asymmetric outline that is unlike the circular case.
    request   = testCase.TestData.Request;
    star_units  = createIrregularStar();
    obstacles = obstacleAvoidance.obstacles.createObstacle("irregular static obstacle", [0; request.goalState.time_s], star_units(:, 1), star_units(:, 2), 0.25);

    result             = runAndValidate(testCase, obstacles, request);
    directDistance_units = norm(request.goalState.position_units - request.initialState.position_units);
    verifyLessThanOrEqual(testCase, motionLength(result), 1.04 * directDistance_units, "The irregular static detour adds over 4%% joint travel.");
    verifyLessThanOrEqual(testCase, lateralReversalCount(result), 1, "The irregular static detour repeatedly reverses lateral motion.");
end

function testMovingIrregularObstacleAvoidsRepeatedJointMotion(testCase)
    % Move the same concave outline across the route during the earliest motion.
    request    = testCase.TestData.Request;
    start_units  = createIrregularStar();
    finish_units = start_units + [0 3];
    obstacles  = obstacleAvoidance.obstacles.createObstacle("irregular moving obstacle", [0; 10; request.goalState.time_s], {start_units(:, 1); finish_units(:, 1); finish_units(:, 1)}, {start_units(:, 2); finish_units(:, 2); finish_units(:, 2)}, 0.25);

    result             = runAndValidate(testCase, obstacles, request);
    directDistance_units = norm(request.goalState.position_units - request.initialState.position_units);
    verifyLessThanOrEqual(testCase, motionLength(result), 1.02 * directDistance_units, "The moving-obstacle detour adds over 2%% joint travel.");
    verifyLessThanOrEqual(testCase, lateralReversalCount(result), 1, "The moving-obstacle detour repeatedly reverses lateral motion.");
end

function request = createRequest()
    % Keep one deterministic sandbox-scale request shared by all obstacle shapes.
    request = struct();
    request.initialState = struct("time_s", 0, "position_units", [-8 0]);
    request.goalState    = struct("time_s", 20, "position_units", [8 0]);
    request.limits       = struct("xInterval_units", [-10 10], ...
        "yInterval_units", [-6 6], ...
        "maxVelocity_units_s", [3 3], ...
        "maxAcceleration_units_s2", [2 2], ...
        "maxJerk_units_s3", [4 4]);
    request.options = struct("GoalTimeMode", "earliestArrival", ...
        "MaximumSeedCount", 5, ...
        "MaximumTimeLayerCount", 17, ...
        "SampleTime_s", 0.05);
end

function star_units = createIrregularStar()
    % Alternate radii and scale one axis to create a concave asymmetric outline.
    angle_rad  = (0:9).' * (2 * pi / 10) + pi / 2;
    radius_units = repmat([2.4; 1.1], 5, 1);
    star_units   = [radius_units .* cos(angle_rad), ...
        0.7 * radius_units .* sin(angle_rad)];
end

function [result, resultDiagnosis] = runAndValidate(testCase, obstacles, request)
    % Require the public planner and independent validator to agree on success.
    [result, resultDiagnosis] = obstacleAvoidance.planTrajectory(obstacles, request.initialState, request.goalState, request.limits, request.options);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, validation.Passed, validation.Message);
    verifyTrue(testCase, validation.CollisionFree);
    verifyTrue(testCase, validation.VelocityWithinLimits);
    verifyTrue(testCase, validation.AccelerationWithinLimits);
    verifyTrue(testCase, validation.JerkWithinLimits);
end

function count = lateralReversalCount(result)
    % Ignore sampled derivative roundoff before counting meaningful reversals.
    lateralVelocity_units_s   = result.velocity_units_s(:, 2);
    velocityTolerance_units_s = 1e-8 * max(1, max(abs(lateralVelocity_units_s)));
    significantSign         = sign(lateralVelocity_units_s(abs(lateralVelocity_units_s) > velocityTolerance_units_s));
    count                   = sum(diff(significantSign) ~= 0);
end

function length_units = motionLength(result)
    % Measure the returned position history without requiring a duplicate field.
    length_units = obstacleAvoidance.geometry.routeLength(result.position_units);
end
