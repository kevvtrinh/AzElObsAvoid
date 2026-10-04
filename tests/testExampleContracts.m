function tests = testExampleContracts
% Preserve example geometry, random factory corpora, and validated motions.
% Run with runtests('tests/testExampleContracts.m').
tests = functiontests(localfunctions);
end

function setupOnce(~)
    rootFolder = fileparts(fileparts(mfilename('fullpath')));
    addpath(rootFolder, fullfile(rootFolder, 'trajectory'), fullfile(rootFolder, 'examples'));
end

function testReducedOutlineIsSupportedAndValid(testCase)
    result = exampleMovingDeformingUSOutlineVisibility(struct('PlotOutputs', false, 'Verbose', false));
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.TerminationReason, "goalReached");
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    % The supplied outline is the 100-vertex reduction of the 14613-vertex
    % source ring at every sample; the planner treats it exactly.
    outline = result.Diagnostics.PreparedObstacles(1);
    verifyEqual(testCase, cellfun(@numel, outline.originalX_units), 100 * ones(numel(outline.time_s), 1));
    verifyEqual(testCase, string(outline.vertexCorrespondence), "sourceIndex");
    preparation = outline.InternalPreparation;
    verifyTrue(testCase, all(preparation.IntervalPrepared));
    verifyFalse(testCase, any(preparation.IntervalIsUnsupported));
    verifyFalse(testCase, any(preparation.IntervalUsesEndpointHull));
    for intervalIndex = find(preparation.IntervalUsesMovingCells).'
        enclosure = preparation.IntervalUnionShapes{intervalIndex};
        endpointShapes = preparation.SampleShapes(intervalIndex:intervalIndex + 1);
        uncoveredArea_units2 = cellfun(@(shape) area(subtract(shape, enclosure)), endpointShapes);
        verifyLessThanOrEqual(testCase, max(uncoveredArea_units2), ...
            4096 * eps(max(1, max(cellfun(@area, preparation.SampleShapes)))));
    end
    % Arrival and length pins belong to the owner's 24-example exact gate.
    verifyGreaterThan(testCase, result.Diagnostics.SeparationProof.MinimumSignedGap_units, 0);
end

function testDenseMovingGeometryContracts(testCase)
    runCases(testCase, {@movingObstacle220Quality});
    [obstacle, initial, goal, limits] = createVietnamBoundaryScenario();
    runCases(testCase, { ...
        @(testCase) sourceAndInterpolation(testCase, obstacle, initial, goal, limits)});
    % The unrelated motion and raw-source checks run before preparation.
    prepared = obstacleAvoidance.obstacles.prepareObstacles(obstacle, [initial.time_s, goal.time_s]);
    runCases(testCase, { ...
        @(testCase) preparationRetainsSourceAndCanonicalizesTwoSpans(testCase, obstacle, prepared); ...
        @(testCase) exactDeformationIsNeverReplacedByAConvexHull(testCase, initial, goal, limits, prepared)});
end

function testRandomScenarioFactoryContracts(testCase)
    previousRandomState = rng;
    restoreRandomState = onCleanup(@() rng(previousRandomState));
    runCases(testCase, { ...
        @eightyDistinctPairedSlews; ...
        @localRandomnessAndInputDrivenCases; ...
        @rejectInvalidInputs; ...
        @exampleResolverForwardsCandidateBudget; ...
        @generatorIsDeterministicAndLocallyRandom; ...
        @rejectsInvalidGeneratorInputs});
end

function testRandomAzimuthPlannerContracts(testCase)
    runCases(testCase, { ...
        @suiteReturnsValidatedCoreResults; ...
        @fixedArrivalCascadeRepairsSpatialSeedFailures});
end

function testRandomInterceptionPlannerContracts(testCase)
    runCases(testCase, { ...
        @staticZonesWithMovingGoal; ...
        @movingObstaclesWithMovingGoal; ...
        @fixedMovingTargetTimedSearchMatchesTargetDerivatives});
end

function sourceAndInterpolation(testCase, obstacle, initial, goal, limits)
    verifyEqual(testCase, obstacle.time_s, (2770:0.25:3000).');
    verifyEqual(testCase, cellfun(@numel, obstacle.x_units), 275 * ones(921, 1));
    verifyEqual(testCase, cellfun(@numel, obstacle.y_units), 275 * ones(921, 1));
    verifyEqual(testCase, obstacle.x_units{1}([1, end]), [71.2030949368775; 71.1630137217925]);
    verifyEqual(testCase, obstacle.y_units{end}([1, end]), [44.5387196138512; 44.6152394585970]);
    verifyEqual(testCase, obstacle.x_units, obstacle.originalX_units);
    verifyEqual(testCase, obstacle.y_units, obstacle.originalY_units);
    for index = [2, 281, 560, 562, 741, 920]
        if index <= 561
            endpoints = [1, 561];
        else
            endpoints = [561, 921];
        end
        fraction = (index - endpoints(1)) / diff(endpoints);
        for coordinate = ["x_units", "y_units"]
            values = obstacle.(coordinate);
            expected = (1 - fraction) * values{endpoints(1)} + fraction * values{endpoints(2)};
            verifyEqual(testCase, values{index}, expected, 'AbsTol', 3e-14);
        end
    end
    verifyEqual(testCase, initial.position_units, [80, 0]);
    verifyEqual(testCase, goal.position_units, [0, 80]);
    verifyEqual(testCase, goal.time_s - initial.time_s, 230);
    verifyEqual(testCase, limits.maxVelocity_units_s, [2, 2]);
end

function preparationRetainsSourceAndCanonicalizesTwoSpans(testCase, obstacle, prepared)
    preparation = prepared.InternalPreparation;
    verifyTrue(testCase, all(preparation.IntervalPrepared));
    acceptedSpanSampleIndices = unique( ...
        [preparation.SpanStartSampleIndex, preparation.SpanEndSampleIndex], 'rows');
    verifyEqual(testCase, prepared.time_s(acceptedSpanSampleIndices), [2770, 2910; 2910, 3000]);
    verifyEqual(testCase, prepared.x_units, obstacle.x_units);
    verifyEqual(testCase, prepared.y_units, obstacle.y_units);
    % Each source anchor has 280 vertices after its exact closing copy is
    % removed. The repaired history keeps 275 supplied vertices per sample.
    source = readtable(fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
        'examples', 'data', 'vietnamBoundaryPoints.csv'));
    anchorTimes_s = unique(source.time_s);
    for anchorTime_s = anchorTimes_s.'
        rows = source(source.time_s == anchorTime_s, :);
        sourceVertices = [rows.az_deg, rows.el_deg];
        if isequal(sourceVertices(1, :), sourceVertices(end, :))
            sourceVertices(end, :) = [];
        end
        verifyEqual(testCase, size(sourceVertices, 1), 280);
    end
    verifyEqual(testCase, 280 - cellfun(@numel, obstacle.x_units), 5 * ones(921, 1));
    verifyEqual(testCase, 280 - cellfun(@numel, obstacle.originalX_units), 5 * ones(921, 1));
end

function exactDeformationIsNeverReplacedByAConvexHull(testCase, initial, goal, limits, prepared)
    verifyTrue(testCase, all(prepared.InternalPreparation.IntervalHasExactPartition));
    verifyFalse(testCase, any(prepared.InternalPreparation.IntervalUsesMovingCells));
    verifyFalse(testCase, any(prepared.InternalPreparation.IntervalIsUnsupported));
    verifyFalse(testCase, any(prepared.InternalPreparation.IntervalUsesEndpointHull));
    cells = obstacleAvoidance.obstacles.createTimeCells(prepared, 2770, 3000);
    verifyEqual(testCase, unique(cells.ActiveTimeInterval_s, 'rows'), [2770, 2910; 2910, 3000]);
    result = planner(prepared, initial, goal, limits, struct('GoalTimeMode', 'fixedArrival', 'WrapX', false));
    verifyTrue(testCase, result.Success, result.Message);
    verifyEqual(testCase, result.ArrivalTime_s, 3000);
    verifyLessThan(testCase, abs(result.MotionLength_units / 113.153 - 1), 0.01);
    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, validation.Passed, validation.Message);
end

function movingObstacle220Quality(testCase)
    result = exampleMovingObstacle220(struct('PlotOutputs', false, 'Verbose', false));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
    verifyEqual(testCase, result.Diagnostics.TrajectoryDuration_s, 230, 'AbsTol', 1e-10);
    % Length pin belongs to the owner's 24-example exact gate.
    verifyGreaterThan(testCase, result.Diagnostics.SeparationProof.MinimumSignedGap_units, 0);
    verifyTrue(testCase, result.Diagnostics.VisibilityGraph.GraphIsFullyEnumerated);
    verifyGreaterThan(testCase, result.Diagnostics.SeparationProof.CachedGeometryPairCount, 0);
    verifyGreaterThan(testCase, result.Diagnostics.SolverDiagnostics.FullPlaneUpdateSkippedCount, 0);
    verifyGreaterThan(testCase, result.Diagnostics.SolverDiagnostics.ConstraintRowPairVerificationCount, 0);
    verifyEqual(testCase, result.Diagnostics.SolverDiagnostics.ExistingPlanePairVerificationCount, 0);
    verifyTrue(testCase, result.Diagnostics.SolverDiagnostics.ConstraintGenerationComplete);
end

function eightyDistinctPairedSlews(testCase)
    starts_deg = zeros(80, 2);
    goals_deg = starts_deg;
    for caseIndex = 1:80
        movingOnly = createRandomAzimuthScenario(caseIndex, false);
        withStatic = createRandomAzimuthScenario(caseIndex, true);
        starts_deg(caseIndex, :) = movingOnly.InitialState.position_units;
        goals_deg(caseIndex, :) = movingOnly.GoalState.position_units;
        verifyGreaterThanOrEqual(testCase, abs(goals_deg(caseIndex, 1) - starts_deg(caseIndex, 1)), 80);
        verifyEqual(testCase, movingOnly.InitialState, withStatic.InitialState);
        verifyEqual(testCase, movingOnly.GoalState, withStatic.GoalState);
        verifyEqual(testCase, movingOnly.Obstacles, withStatic.Obstacles(1));
        verifyNumElements(testCase, withStatic.Obstacles, 2);
        obstacle = movingOnly.Obstacles;
        first_deg = [obstacle.x_units{1}, obstacle.y_units{1}];
        final_deg = [obstacle.x_units{end}, obstacle.y_units{end}];
        verifyEqual(testCase, final_deg - first_deg, ...
            repmat(180 * movingOnly.MovingVelocity_deg_s, size(first_deg, 1), 1), 'AbsTol', 1e-12);
        verifyGreaterThan(testCase, norm(movingOnly.MovingVelocity_deg_s), 0);
        verifyTrue(testCase, inpolygon(movingOnly.MovingCenterAtStart_deg(1), ...
            movingOnly.MovingCenterAtStart_deg(2), first_deg(:, 1), first_deg(:, 2)));
        cross_deg2 = det([goals_deg(caseIndex, :) - starts_deg(caseIndex, :); ...
            movingOnly.MovingCenterAtStart_deg - starts_deg(caseIndex, :)]);
        verifyLessThan(testCase, abs(cross_deg2), 1e-10);
    end
    verifySize(testCase, unique(starts_deg, 'rows'), [80, 2]);
    verifySize(testCase, unique(goals_deg, 'rows'), [80, 2]);
    verifyTrue(testCase, any(goals_deg(:, 1) > starts_deg(:, 1)) && any(goals_deg(:, 1) < starts_deg(:, 1)));
end

function localRandomnessAndInputDrivenCases(testCase)
    previous = rng;
    a = createRandomAzimuthScenario(17, true);
    verifyEqual(testCase, rng, previous);
    createRandomAzimuthScenario(80, false);
    verifyEqual(testCase, a, createRandomAzimuthScenario(17, true));
    b = createRandomAzimuthScenario(17, true, 42);
    verifyNotEqual(testCase, a.InitialState.position_units, b.InitialState.position_units);
end

function rejectInvalidInputs(testCase)
    verifyError(testCase, @() createRandomAzimuthScenario(0), 'MATLAB:expectedPositive');
    verifyError(testCase, @() createRandomAzimuthScenario(1, 1), 'MATLAB:invalidType');
end

function exampleResolverForwardsCandidateBudget(testCase)
    [options, ~] = resolveExampleOptions( ...
        struct('MaxArrivalCandidates', 17), struct());
    verifyEqual(testCase, options.MaxArrivalCandidates, 17);
end

function generatorIsDeterministicAndLocallyRandom(testCase)
    previousRandomState = rng;
    caseCount = 40;
    staticStarts_units = zeros(caseCount, 2);
    movingStarts_units = zeros(caseCount, 2);
    for caseIndex = 1:caseCount
        staticScenario = createRandomInterceptionScenario( ...
            caseIndex, 'staticZone', 941207);
        movingScenario = createRandomInterceptionScenario( ...
            caseIndex, 'movingObstacles', 941207);
        verifyEqual(testCase, staticScenario, createRandomInterceptionScenario(caseIndex, 'staticZone', 941207));
        verifyEqual(testCase, movingScenario, createRandomInterceptionScenario(caseIndex, 'movingObstacles', 941207));
        staticStarts_units(caseIndex, :) = staticScenario.InitialState.position_units;
        movingStarts_units(caseIndex, :) = movingScenario.InitialState.position_units;
    end
    verifyEqual(testCase, rng, previousRandomState);
    verifySize(testCase, unique(staticStarts_units, 'rows'), [caseCount, 2]);
    verifySize(testCase, unique(movingStarts_units, 'rows'), [caseCount, 2]);
    alternateSeedScenario = createRandomInterceptionScenario( ...
        17, 'staticZone', 941207 + 1);
    verifyNotEqual(testCase, alternateSeedScenario.InitialState.position_units, staticStarts_units(17, :));
    verifyEqual(testCase, rng, previousRandomState);
end

function rejectsInvalidGeneratorInputs(testCase)
    verifyError(testCase, @() createRandomInterceptionScenario(0, 'staticZone'), 'MATLAB:expectedPositive');
    verifyError(testCase, @() createRandomInterceptionScenario(1, 'unknown'), ...
        'createRandomInterceptionScenario:InvalidFamily');
end

function suiteReturnsValidatedCoreResults(testCase)
    results = exampleRandomAzimuthSuite([2, 53], struct('PlotOutputs', false));
    verifySize(testCase, results, [2, 2]);
    for k = 1:numel(results)
        assertTrue(testCase, results{k}.Success, results{k}.Message);
        verifyTrue(testCase, obstacleAvoidance.validateTrajectory(results{k}).Passed);
        verifyEqual(testCase, results{k}.ArrivalTime_s, 180, 'AbsTol', 1e-8);
    end
    single = exampleRandomAzimuth(6, true, struct('PlotOutputs', false));
    verifyTrue(testCase, single.Success, single.Message);
    verifyTrue(testCase, obstacleAvoidance.validateTrajectory(single).Passed);
end

function fixedArrivalCascadeRepairsSpatialSeedFailures(testCase)
    caseIndices = [26, 36, 62, 62];
    withStatic = [true, true, false, true];
    expectedSources = ["arrivalSpatialSnapshot", "timeExpandedVisibilityGraph", ...
        "timeExpandedVisibilityGraph", "arrivalSpatialSnapshot"];
    overrides = struct('PlotOutputs', false, 'Verbose', false);
    for caseNumber = 1:numel(caseIndices)
        result = exampleRandomAzimuth(caseIndices(caseNumber), ...
            withStatic(caseNumber), overrides);
        verifyTrue(testCase, result.Success, result.Message);
        verifyTrue(testCase, obstacleAvoidance.validateTrajectory(result).Passed);
        verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, expectedSources(caseNumber));
        verifyGreaterThanOrEqual(testCase, numel(result.Diagnostics.Attempts), 2);
        verifyTrue(testCase, result.Diagnostics.Attempts(1).IsShortcut);
        verifyTrue(testCase, result.Diagnostics.Attempts(1).NextAttemptAllowed);
        snapshotAttempts = result.Diagnostics.Attempts([result.Diagnostics.Attempts.IsShortcut]);
        verifyEqual(testCase, [snapshotAttempts.IterationLimit], 2 * ones(1, numel(snapshotAttempts)));
        verifyEqual(testCase, result.ArrivalTime_s, 180, 'AbsTol', 1e-10);
    end
end

function staticZonesWithMovingGoal(testCase)
    for caseIndex = 1:12
        scenario = createRandomInterceptionScenario( ...
            caseIndex, 'staticZone', 941207);
        label = "static-zone case " + caseIndex;
        verifyTargetOccupancyPattern(testCase, scenario, label);
        result = verifyPlannerOutcome(testCase, scenario, label);
        if caseIndex == 8
            verifyGreaterThan(testCase, numel(result.Diagnostics.Attempts), 1);
            verifyEqual(testCase, result.Diagnostics.Attempts(1).FailureStage, "timing");
            verifyEqual(testCase, result.Diagnostics.Attempts(1).FailureKind, "kinematicCertificateUnavailable");
            verifyTrue(testCase, result.Diagnostics.Attempts(1).NextMethodAllowed);
            verifyTrue(testCase, any([result.Diagnostics.Attempts.Selected]));
        end
    end
end

function movingObstaclesWithMovingGoal(testCase)
    for caseIndex = 1:12
        scenario = createRandomInterceptionScenario( ...
            caseIndex, 'movingObstacles', 941207);
        label = "moving-obstacle case " + caseIndex;
        verifyMovingGeometry(testCase, scenario, label);
        verifyTargetOccupancyPattern(testCase, scenario, label);
        if scenario.DirectRouteShouldBeBlocked
            verifyTrue(testCase, directRouteIsBlocked(scenario), ...
                label + " did not block its timed direct route.");
        end
        verifyPlannerOutcome(testCase, scenario, label);
    end
end

function fixedMovingTargetTimedSearchMatchesTargetDerivatives(testCase)
    % Pin the physical failure that motivated the core eligibility change.
    % The spatial seed is unavailable; the one exact timed route waits for
    % the crossing obstacle and must retain both matched target derivatives.
    scenario = timedNextMethodRegressionScenario();
    label = "fixed moving-target timed search";
    verifyMovingGeometry(testCase, scenario, label);
    verifyTargetOccupancyPattern(testCase, scenario, label);
    verifyTrue(testCase, directRouteIsBlocked(scenario), ...
        label + ": timed direct route is unexpectedly clear.");
    result = verifyPlannerOutcome(testCase, scenario, label);

    verifyEqual(testCase, result.Diagnostics.VisibilityGraph.SearchKind, ...
        "timeExpandedVisibilityGraph", ...
        label + ": timed route was not selected.");
    verifyGreaterThanOrEqual(testCase, numel(result.Diagnostics.Attempts), 2, ...
        label + ": rejected spatial-guide evidence is missing.");
    verifyTrue(testCase, any([result.Diagnostics.Attempts(1:end - 1).NextAttemptAllowed]), ...
        label + ": no rejected guide admitted the timed search.");
    verifyEqual(testCase, result.Diagnostics.Attempts(end).Kind, "timedVisibility");
    [~, targetVelocity_units_s, targetAcceleration_units_s2] = ...
        obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, scenario.GoalState.time_s);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetVelocity_units_s, ...
        targetVelocity_units_s, 'AbsTol', 1e-9);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetAcceleration_units_s2, ...
        targetAcceleration_units_s2, 'AbsTol', 1e-9);
    verifyEqual(testCase, result.velocity_units_s(end, :), targetVelocity_units_s, 'AbsTol', 1e-8);
    verifyEqual(testCase, result.acceleration_units_s2(end, :), targetAcceleration_units_s2, 'AbsTol', 1e-8);
end

function result = verifyPlannerOutcome(testCase, scenario, label)
    % Check the public stable outcome and independently prove every success.
    result = planner(scenario.Obstacles, scenario.InitialState, ...
        scenario.GoalState, scenario.Limits, scenario.Options);
    verifyEqual(testCase, result.Success, scenario.ExpectedSuccess, ...
        label + ": " + result.Message);
    % TerminationReason stays part of the public stable outcome, so the
    % expected value is derived here rather than carried on the scenario.
    expectedReason = "endpointBlocked";
    if scenario.ExpectedSuccess
        expectedReason = "goalReached";
    end
    verifyEqual(testCase, string(result.TerminationReason), expectedReason, ...
        label + ": unexpected termination.");
    if result.Success ~= scenario.ExpectedSuccess
        return
    end
    if ~scenario.ExpectedSuccess
        verifyEmpty(testCase, result.time_s, label + ": failure returned a motion.");
        return
    end

    validation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, validation.Passed, label + ": " + validation.Message);
    expectedTarget_units = obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, result.Diagnostics.Intercept.Time_s);
    verifyEqual(testCase, result.Diagnostics.Intercept.TargetPosition_units, ...
        expectedTarget_units, 'AbsTol', 1e-8, ...
        label + ": intercept did not match the target.");
    targetIsOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        scenario.Obstacles, expectedTarget_units(1), expectedTarget_units(2), ...
        result.Diagnostics.Intercept.Time_s);
    verifyFalse(testCase, targetIsOccupied, ...
        label + ": successful intercept remained inside an obstacle.");
    verifyGreaterThan(testCase, norm(diff( ...
        scenario.GoalState.targetMotion.position_units, 1, 1)), 0.5, ...
        label + ": target did not move materially.");

    if string(scenario.Options.GoalTimeMode) == "earliestArrival"
        verifyLessThan(testCase, result.Diagnostics.Intercept.Time_s, scenario.GoalState.time_s, ...
            label + ": earliest search only returned the horizon.");
        verifyTrue(testCase, isfield(result.Diagnostics, 'TemporalSearch'), ...
            label + ": arrival-time search diagnostics are missing.");
    else
        verifyEqual(testCase, result.Diagnostics.Intercept.Time_s, scenario.GoalState.time_s, ...
            'AbsTol', 1e-8, label + ": fixed intercept time changed.");
    end
end

function verifyTargetOccupancyPattern(testCase, scenario, label)
    % Probe the target independently over its complete declared history.
    sampleCount = 401;
    sampleTime_s = linspace(0, scenario.GoalState.time_s, sampleCount).';
    target_units = obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, sampleTime_s);
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        scenario.Obstacles, target_units(:, 1), target_units(:, 2), sampleTime_s);
    transitionCount = nnz(diff(occupied) ~= 0);

    switch scenario.TargetOccupancyPattern
        case "outsideInsideOutside"
            verifyFalse(testCase, occupied(1), label + ": target started occupied.");
            verifyTrue(testCase, any(occupied), label + ": target never entered the zone.");
            verifyFalse(testCase, occupied(end), label + ": target did not leave the zone.");
            verifyGreaterThanOrEqual(testCase, transitionCount, 2, ...
                label + ": expected entry and exit transitions.");
        case "insideOutside"
            verifyTrue(testCase, occupied(1), label + ": target did not start occupied.");
            verifyFalse(testCase, occupied(end), label + ": target did not exit.");
            verifyGreaterThanOrEqual(testCase, transitionCount, 1, ...
                label + ": expected one exit transition.");
        case "outsideInside"
            verifyFalse(testCase, occupied(1), label + ": target started occupied.");
            verifyTrue(testCase, occupied(end), label + ": target did not finish occupied.");
            verifyGreaterThanOrEqual(testCase, transitionCount, 1, ...
                label + ": expected one entry transition.");
        case "outsideOutside"
            verifyFalse(testCase, occupied(1), label + ": target started occupied.");
            verifyFalse(testCase, occupied(end), label + ": target ended occupied.");
        otherwise
            verifyFail(testCase, label + ": unknown occupancy expectation.");
    end
end

function verifyMovingGeometry(testCase, scenario, label)
    % Every obstacle in this family must translate between its endpoint slices.
    for obstacleIndex = 1:numel(scenario.Obstacles)
        obstacle = scenario.Obstacles(obstacleIndex);
        verifyGreaterThan(testCase, numel(obstacle.time_s), 1, ...
            label + ": obstacle history is static.");
        start_units = [obstacle.x_units{1}, obstacle.y_units{1}];
        end_units = [obstacle.x_units{end}, obstacle.y_units{end}];
        verifyGreaterThan(testCase, max(abs(end_units - start_units), [], 'all'), ...
            0.5, label + ": obstacle did not move materially.");
    end
end

function blocked = directRouteIsBlocked(scenario)
    % Sample only to assert fixture construction; planner acceptance remains exact.
    sampleCount = 401;
    sampleTime_s = linspace(0, scenario.GoalState.time_s, sampleCount).';
    goal_units = obstacleAvoidance.input.targetPositionAtTime( ...
        scenario.GoalState.targetMotion, scenario.GoalState.time_s);
    fraction = sampleTime_s / scenario.GoalState.time_s;
    route_units = scenario.InitialState.position_units + fraction .* ...
        (goal_units - scenario.InitialState.position_units);
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        scenario.Obstacles, route_units(:, 1), route_units(:, 2), sampleTime_s);
    blocked = any(occupied);
end

function scenario = timedNextMethodRegressionScenario()
    % Freeze one general moving-target/moving-obstacle factory regression.
    missionEndTime_s = 12.6824677531027;
    startVertices_units = [ ...
        -2.41212665955762, -3.10756783587549; ...
        -1.26869088471832, -3.85354373008669; ...
        -0.11574128448923, -2.08629651778552; ...
        -1.25917705932853, -1.34032062357432];
    endVertices_units = [ ...
        0.451174172683163, 1.28131474212419; ...
        1.59460994752247, 0.535338847912989; ...
        2.74755954775155, 2.30258606021416; ...
        1.60412377291225, 3.04856195442535];
    obstacle = obstacleAvoidance.obstacles.createObstacle( ...
        'Crossing rectangle', [0; missionEndTime_s], ...
        {startVertices_units(:, 1); endVertices_units(:, 1)}, ...
        {startVertices_units(:, 2); endVertices_units(:, 2)}, ...
        0.06, struct('vertexCorrespondence', 'sourceIndex'));

    initialState = struct( ...
        'time_s', 0, ...
        'position_units', [-5.67556230729695, 3.70483299936472], ...
        'velocity_units_s', [0, 0], ...
        'acceleration_units_s2', [0, 0]);
    targetStart_units = [3.56626840907745, -2.18951624292289];
    targetEnd_units = [5.47468754926656, -3.74372919576700];
    targetDisplacement_units = targetEnd_units - targetStart_units;
    targetMotion = struct( ...
        'time_s', [0; missionEndTime_s / 2; missionEndTime_s], ...
        'position_units', [targetStart_units; ...
            targetStart_units + 0.4 * targetDisplacement_units; ...
            targetEnd_units], ...
        'InterpolationMethod', 'pchip');
    goalState = struct('time_s', missionEndTime_s, 'targetMotion', targetMotion);
    limits = struct( ...
        'xInterval_units', [-20, 20], ...
        'yInterval_units', [-20, 20], ...
        'maxVelocity_units_s', [2.5, 2.5], ...
        'maxAcceleration_units_s2', [1.2, 1.2], ...
        'maxJerk_units_s3', [2.5, 2.5]);
    options = struct( ...
        'GoalTimeMode', 'fixedArrival', ...
        'SampleTime_s', 0.1, ...
        'TemporalResolution_s', 1, ...
        'MaxArrivalTrials', 16, ...
        'MatchTargetVelocity', true, ...
        'MatchTargetAcceleration', true);
    scenario = struct( ...
        'Obstacles', obstacle, ...
        'InitialState', initialState, ...
        'GoalState', goalState, ...
        'Limits', limits, ...
        'Options', options, ...
        'ExpectedSuccess', true, ...
        'TargetOccupancyPattern', "outsideOutside", ...
        'DirectRouteShouldBeBlocked', true);
end
