function tests = testExampleInvariants
%% Section 0: Header & Readme
% SYNTAX
%   tests = testExampleInvariants
%**************************************************************************
% PURPOSE
%   - Protect reviewed physical requirements of maintained examples.
%   - Verify shared example source contracts from one inventory.
%   - Execute representative static and mixed dynamic examples headlessly.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - tests (matlab.unittest function test array)
%**************************************************************************
% UNITS
%   - Test positions are coordinate units and test times are seconds.
%**************************************************************************
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    % Locate production, trajectory, and maintained example entry points once.
    repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
    addpath(repositoryRoot);
    addpath(fullfile(repositoryRoot, "trajectory"));
    addpath(fullfile(repositoryRoot, "examples"));
    testCase.TestData.RepositoryRoot = repositoryRoot;
end

function testPhysicalRequirementHashes(testCase)
    % Protect reviewed inputs and the native U.S. deformation from silent drift.
    relativePaths = [ ...
        "examples/exampleStaticUShapedObstacle.m", ...
        "examples/exampleMovingDeformingUSOutlineVisibility.m", ...
        "examples/private/createContiguousUSObstacle.m"];
    expectedHashes = [ ...
        "f23739dd1d3276e941967fc29d05616b51b04f8379efe81604ac025bbb437c31", ...
        "dae1d7e27639e30865e7d54c5f1b227687f714bf683fa65fa9a767b40fd0654e", ...
        "3b617f0435872e2c7b2cb79b9ab20aadde5bdf7b8819fa547ab08eeff33fb7b8"];
    % Exercise each requirement covered by this regression.
    for requirementIndex = 1:numel(relativePaths)
        requirementPath = fullfile(testCase.TestData.RepositoryRoot, relativePaths(requirementIndex));
        sourceText      = string(fileread(requirementPath));
        sourceText      = replace(sourceText, [string(char([13 10])) string(char(13))], newline);
        if requirementIndex < numel(relativePaths)
            sourceText = extractBefore(extractAfter(sourceText, "%% Section 1:"), "%% Section 6:");
        end
        hashEngine = java.security.MessageDigest.getInstance("SHA-256");
        hashEngine.update(unicode2native(char(sourceText), "UTF-8"));
        hashBytes  = mod(double(hashEngine.digest()), 256);
        actualHash = lower(string(reshape(dec2hex(hashBytes, 2).', 1, [])));
        verifyEqual(testCase, actualHash, expectedHashes(requirementIndex), "The reviewed example requirement changed in " + relativePaths(requirementIndex) + ".");
    end
end

function testMaintainedExampleSourceContracts(testCase)
    % Verify shared source contracts from one maintained-example inventory.
    allExampleNames = [ ...
        "exampleAlternatingSlalom", "exampleVietnamKeepoutSlew", ...
        "exampleDenseConcaveObstacle", "exampleFourAcceleratingCircles", ...
        "exampleInterceptMovingTargetAtSetTime", ...
        "exampleInterceptMovingTargetEarliest", ...
        "exampleMovingBarrierWait", "exampleMovingCircleNoWrap", ...
        "exampleMovingDeformingUSOutlineVisibility", ...
        "exampleMovingRotatingObstacleField", ...
        "exampleNoPath", "exampleObstacleFree", ...
        "exampleOpeningUShapedObstacle", ...
        "exampleStraightTargetAlternatingOcclusion", ...
        "exampleTargetExitsObstacle", ...
        "exampleTwoOpposingUVisibilityGraph", ...
        "exampleStaticUShapedObstacle", "exampleUSOutlineExtremeVisibility"];
    fixedArrivalExamples = [ ...
        "exampleFourAcceleratingCircles", ...
        "exampleVietnamKeepoutSlew", ...
        "exampleInterceptMovingTargetAtSetTime", ...
        "exampleInterceptMovingTargetEarliest", ...
        "exampleStraightTargetAlternatingOcclusion", ...
        "exampleTargetExitsObstacle"];
    earliestArrivalExamples = setdiff(allExampleNames, fixedArrivalExamples, "stable");

    % Exercise each example name covered by this regression.
    for exampleName = allExampleNames
        examplePath = fullfile(testCase.TestData.RepositoryRoot, "examples", exampleName + ".m");
        sourceText  = string(fileread(examplePath));
        routedMatch = regexp(sourceText, '(?:"maxJerk_units_s3"\s*,\s*\w+\.MaxJerk_units_s3|maxJerk_units_s3\s*=\s*displayOptions\.MaxJerk_units_s3)', 'once');
        verifyNotEmpty(testCase, routedMatch, exampleName + " must route MaxJerk_units_s3 into limits.");
        addedField = regexp(sourceText, '(?m)^\s*result\.[A-Za-z]\w*\s*=', 'once');
        verifyEmpty(testCase, addedField, exampleName + " must not append fields to the planner result.");
    end
    % Exercise each example name covered by this regression.
    for exampleName = earliestArrivalExamples
        examplePath = fullfile(testCase.TestData.RepositoryRoot, "examples", exampleName + ".m");
        sourceText  = fileread(examplePath);
        policyMatch = regexp(sourceText, '"GoalTimeMode"\s*,\s*"earliestArrival"', 'once');
        verifyNotEmpty(testCase, policyMatch, exampleName + " must state GoalTimeMode=earliestArrival.");
    end
    slalomPath = fullfile(testCase.TestData.RepositoryRoot, "examples", "exampleAlternatingSlalom.m");
    slalomText = fileread(slalomPath);
    boundMatch = regexp(slalomText, '"yInterval_units"\s*,\s*\[-5\s+5\]', 'once');
    verifyNotEmpty(testCase, boundMatch, "The slalom y interval must be [-5 5] coordinate units.");
end

function testExampleResolverMaterializesPlannerDefaults(testCase)
    % Verify examples materialize public defaults and separate display controls.
    [defaultOptions, defaultDisplayOptions] = resolveExampleOptions(struct("PlotOutputs", false), struct("MaximumSeedCount", 2));
    verifyFalse(testCase, isfield(defaultOptions, "Verbose"));
    verifyTrue(testCase, defaultDisplayOptions.Verbose);
    verifyEqual(testCase, defaultOptions.MaximumSeedCount, 2);
    verifyFalse(testCase, isfield(defaultOptions, "PlannerMethod"));
    verifyFalse(testCase, isfield(defaultOptions, "CollocationSegmentCount"));
    verifyFalse(testCase, isfield(defaultOptions, "MotionMethod"));

    [hs3Options, displayOptions] = resolveExampleOptions(struct("Verbose", false), struct("MaximumSeedCount", 5));
    verifyFalse(testCase, isfield(hs3Options, "Verbose"));
    verifyFalse(testCase, displayOptions.Verbose);
    verifyFalse(testCase, isfield(hs3Options, "PlannerMethod"));
    verifyEqual(testCase, hs3Options.MaximumSeedCount, 5);
    verifyFalse(testCase, isfield(hs3Options, "MotionMethod"));
end

function testExampleResolverRejectsRetiredPlannerOptions(testCase)
    % Discard obsolete planner fields at the example boundary as unknown inputs.
    retiredNames = ["PerSeedWorkBudgetMultiplier", ...
        "SeedClusterDistance_units", "MaximumNlpIterations", ...
        "CollocationSegmentCount", "EnablePlaneReuse", ...
        "PlaneReuseImprovementTolerance_s", "WaypointWarmStartMode", ...
        "RequestedWaypointWarmStartMode", "IsWaypointWarmStartAvailable"];
    retiredOptions = struct();
    retiredOptions.PerSeedWorkBudgetMultiplier      = 3;
    retiredOptions.SeedClusterDistance_units          = 2;
    retiredOptions.MaximumNlpIterations             = 5;
    retiredOptions.CollocationSegmentCount          = 6;
    retiredOptions.EnablePlaneReuse                 = false;
    retiredOptions.PlaneReuseImprovementTolerance_s = 1e-7;
    retiredOptions.WaypointWarmStartMode            = "passThrough";
    retiredOptions.RequestedWaypointWarmStartMode   = "none";
    retiredOptions.IsWaypointWarmStartAvailable     = true;
    retiredOptions.PlotOutputs                      = false;
    verifyWarning(testCase, @() resolveExampleOptions(retiredOptions, struct()), "resolveExampleOptions:UnknownOptions");
    warningState   = warning("off", "resolveExampleOptions:UnknownOptions");
    warningCleanup = onCleanup(@() warning(warningState));
    [plannerOptions, ~] = resolveExampleOptions(retiredOptions, struct());
    % Exercise each field name covered by this regression.
    for fieldName = retiredNames
        verifyFalse(testCase, isfield(plannerOptions, fieldName));
    end
end

function testVietnamKeepoutSlewRunsHeadlessly(testCase)
    % Execute the maintained timed example and protect its physical answer.
    [result, resultDiagnosis] = exampleVietnamKeepoutSlew(struct("PlotOutputs", false, "FigureVisible", "off"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    independentValidation = obstacleAvoidance.validateTrajectory(result);
    verifyTrue(testCase, independentValidation.Passed, independentValidation.Message);
    verifyEqual(testCase, result.TerminationReason, "goalReached");
    verifyEqual(testCase, result.TrajectoryDuration_s, 30, "AbsTol", 1e-12);
    verifyEqual(testCase, obstacleAvoidance.geometry.routeLength(result.Route_units), 17.297991315813945, "AbsTol", 1e-9);
    verifyEqual(testCase, obstacleAvoidance.geometry.routeLength(result.position_units), 17.305374620918947, "AbsTol", 1e-9);
    summary = resultDiagnosis.Attempts(resultDiagnosis.SelectedAttemptIndex);
    verifyEqual(testCase, summary.MotionLength_units, 17.305374620918947, "AbsTol", 1e-9);
    verifyEqual(testCase, resultDiagnosis.Search.MotionEdgeCount, 29231);
    verifyEqual(testCase, resultDiagnosis.Search.WaitEdgeCount, 5618);
    verifyEqual(testCase, resultDiagnosis.Search.RejectedTransitionCount, 2859775);
    verifyEqual(testCase, resultDiagnosis.Search.GoalCostBoundRejectionCount, 214274);
    verifyEqual(testCase, resultDiagnosis.Search.CandidateBatchSplitCount, 0);
    selectedDetails = resultDiagnosis.SolverDetails(resultDiagnosis.SolverDetails.Attempt == resultDiagnosis.SelectedAttemptIndex, :);
    verifyFalse(testCase, any(startsWith(selectedDetails.Field, "TimedBmtp.SolverDiagnostics")));
    verifyTrue(testCase, any(selectedDetails.Field == "TimedBmtp.FullObstacleValidation.Passed"));
end

function testVietnamInputHash(testCase)
    % Keep the maintained physical request byte-for-byte unchanged.
    inputPath = fullfile(testCase.TestData.RepositoryRoot, "examples", "data", "vietnamKeepoutSlewInput.mat");
    fileIdentifier = fopen(inputPath, "r");
    verifyGreaterThan(testCase, fileIdentifier, 0, "The maintained Vietnam input could not be opened.");
    fileCleanup = onCleanup(@() fclose(fileIdentifier));
    inputBytes = fread(fileIdentifier, Inf, "*uint8");
    hashEngine = java.security.MessageDigest.getInstance("SHA-256");
    hashEngine.update(inputBytes);
    hashBytes = mod(double(hashEngine.digest()), 256);
    actualHash = lower(string(reshape(dec2hex(hashBytes, 2).', 1, [])));
    verifyEqual(testCase, actualHash, "dbd475086da1398e4735887b78d3a5466a6c22deb67174fb889540de392c7cac");
end

function testMovingRotatingObstacleFieldRunsHeadlessly(testCase)
    % Verify mixed static/dynamic planning and the requested obstacle motion.
    result = exampleMovingRotatingObstacleField(struct("PlotOutputs", false, "FigureVisible", "off"));
    verifyTrue(testCase, result.Success, result.Message);
    verifyTrue(testCase, result.Validation.Passed, result.Validation.Message);
    verifyTrue(testCase, result.Validation.CollisionFree);
    verifyEqual(testCase, numel(result.Inputs.obstacles), 4);
    movingObstacle = result.Inputs.obstacles(4);
    verifyEqual(testCase, numel(movingObstacle.time_s), 5);
    initialBoundary_units = [movingObstacle.originalX_units{1}, ...
        movingObstacle.originalY_units{1}];
    finalBoundary_units = [movingObstacle.originalX_units{end}, ...
        movingObstacle.originalY_units{end}];
    centerTravel_units     = norm(mean(finalBoundary_units) - mean(initialBoundary_units));
    initialEdge_units      = initialBoundary_units(2, :) - initialBoundary_units(1, :);
    finalEdge_units        = finalBoundary_units(2, :) - finalBoundary_units(1, :);
    rotationMeasure_units2 = abs(det([initialEdge_units; finalEdge_units]));
    verifyGreaterThan(testCase, centerTravel_units, 0);
    verifyGreaterThan(testCase, rotationMeasure_units2, 0);
    verifyGreaterThan(testCase, sum(vecnorm(diff(result.Route_units, 1, 1), 2, 2)), 20);
end
