function [runs, summary, results] = benchmarkRandomCampaign(caseIndices, outputFolder)
%% Section 0: Header & Readme
% SYNTAX
%   [runs, summary, results] = benchmarkRandomCampaign()
%   [runs, summary, results] = benchmarkRandomCampaign(caseIndices)
%   [runs, summary, results] = benchmarkRandomCampaign(caseIndices, outputFolder)
%**************************************************************************
% PURPOSE
%   - Run deterministic random requests that exercise every public planner
%     option across four scene families: random azimuth slews, static-zone
%     interceptions, moving-obstacle interceptions, and synthetic scenes of
%     convex or concave polygons that may translate.
%   - Flip every goal between a moving target and a static point, validate
%     every success independently, and classify every outcome against the
%     planner's documented stable reasons, so a defect (accepted-invalid
%     motion, a validator rejection of the planner's own candidate, an
%     undocumented failure reason, a broken arrival contract, a thrown
%     error, or a request the campaign itself could not build) is never
%     hidden inside a pass count.
%**************************************************************************
% INPUTS
%   - caseIndices (positive integer vector, optional; default 1:200)
%       Case k is fully determined by k: substream k of one fixed seed.
%   - outputFolder (string scalar, optional; default "")
%       Folder for randomCampaign_runs.csv and randomCampaign_summary.csv;
%       empty text disables file output.
%**************************************************************************
% OUTPUTS
%   - runs (table)
%       One row per case: family, target kind, every option value, scene
%       shape, outcome class, and planner wall time. Every column except
%       Wall_s is deterministic in the case index.
%   - summary (table)
%       Case counts per outcome class with the defect flag.
%   - results (N-by-1 cell array)
%       Complete public planner results, or the caught error, for
%       reproduction. They carry measured times and are not byte-identical
%       between runs. Ordinary planning failures remain Success = false.
%**************************************************************************
% UNITS
%   - Coordinate units (degrees for the azimuth family) and seconds.
%**************************************************************************

%% Section 1: Validate Controls And Warm The Planner

if nargin < 1
    caseIndices = 1:200;
end
if nargin < 2
    outputFolder = "";
end
validateattributes(caseIndices, {'numeric'}, ...
    {'vector', 'integer', 'positive', 'finite', 'nonempty'});
outputFolder = string(outputFolder);
assert(isscalar(outputFolder));
repositoryRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repositoryRoot, fullfile(repositoryRoot, 'trajectory'), ...
    fullfile(repositoryRoot, 'examples'));
if strlength(outputFolder) > 0 && ~isfolder(outputFolder)
    mkdir(outputFolder);
end
campaignSeed   = 20260919;
defaultOptions = planner();
% One maintained scenario warms the planner so the first case's wall time is
% not the compiler's. It is not a campaign case: if it throws, the
% installation is broken and the campaign stops here rather than filing that
% as an outcome.
warmScenario   = createRandomAzimuthScenario(1, false);
planner([], warmScenario.InitialState, warmScenario.GoalState, ...
    warmScenario.Limits, warmScenario.Options);
runCount = numel(caseIndices);
results  = cell(runCount, 1);
rows     = cell(runCount, 1);

%% Section 2: Run Every Case And Classify Its Outcome

for runIndex = 1:runCount
    caseIndex = caseIndices(runIndex);
    stream    = RandStream('mrg32k3a', 'Seed', campaignSeed);
    stream.Substream = caseIndex;
    draw   = rand(stream, 1, 64);
    family = familyOf(caseIndex);

    options      = defaultOptions;
    shape        = struct('Family', family, 'Target', "", 'ObstacleCount', 0, ...
        'MovingObstacleCount', 0, 'ConcaveObstacleCount', 0, 'RestStart', true);
    arrival_s    = NaN;
    length_units = NaN;
    reason       = "";
    try
        [request, shape] = createCampaignRequest(caseIndex, family, draw);
        options = request.options;
    catch buildError
        class  = "BUILD_ERROR";
        detail = string(buildError.identifier) + ": " + string(buildError.message);
        results{runIndex} = buildError;
        rows{runIndex} = campaignRow(caseIndex, shape, class, options, NaN, ...
            arrival_s, length_units, reason, detail);
        fprintf('case %3d %-16s %-24s %s\n', caseIndex, family, class, detail);
        continue
    end
    timer = tic;
    try
        result = planner(request.obstacles, request.initialState, request.goalState, ...
            request.limits, options);
        wall_s     = toc(timer);
        validation = obstacleAvoidance.validateTrajectory(result);
        [class, detail] = classifyOutcome(result, validation, request);
        if result.Success
            arrival_s    = result.ArrivalTime_s;
            length_units = result.MotionLength_units;
        end
        reason = string(result.TerminationReason);
        results{runIndex} = result;
    catch caughtError
        wall_s = toc(timer);
        class  = "ERROR";
        detail = string(caughtError.identifier) + ": " + string(caughtError.message);
        reason = "THREW";
        results{runIndex} = caughtError;
    end
    rows{runIndex} = campaignRow(caseIndex, shape, class, options, wall_s, ...
        arrival_s, length_units, reason, detail);
    fprintf('case %3d %-16s %-7s %-15s %-24s %7.2f s\n', caseIndex, family, ...
        shape.Target, string(options.GoalTimeMode), class, wall_s);
end
runs = vertcat(rows{:});

%% Section 3: Summarize And Write

[classes, ~, classIndex] = unique(runs.Class);
counts  = accumarray(classIndex, 1);
defects = false(numel(classes), 1);
for classNumber = 1:numel(classes)
    defects(classNumber) = any(runs.Defect(classIndex == classNumber));
end
summary = table(classes, counts, defects, 'VariableNames', {'Class', 'Count', 'Defect'});
fprintf('\n%d cases, %d defects\n', height(runs), nnz(runs.Defect));
disp(summary);
if any(runs.Defect)
    disp(runs(runs.Defect, {'Case', 'Family', 'Target', 'Class', 'GoalTimeMode', 'Detail'}));
end
if strlength(outputFolder) > 0
    writetable(runs, fullfile(outputFolder, 'randomCampaign_runs.csv'));
    writetable(summary, fullfile(outputFolder, 'randomCampaign_summary.csv'));
end
end

%% Section 4: Local Functions

function family = familyOf(caseIndex)
    % Rotate through the four scene families by case index.
    families = ["synthetic", "interceptStatic", "interceptMoving", "azimuth"];
    family   = families(1 + mod(caseIndex, 4));
end

function [request, shape] = createCampaignRequest(caseIndex, family, draw)
    % Draw one scene from its family, flip the goal between a moving target
    % and a static point, then randomize every public option.
    switch family
        case "azimuth"
            scenario = createRandomAzimuthScenario(caseIndex, draw(1) > 0.5, 831907);
        case "interceptStatic"
            scenario = createRandomInterceptionScenario(caseIndex, 'staticZone', 941207);
        case "interceptMoving"
            scenario = createRandomInterceptionScenario(caseIndex, 'movingObstacles', 941207);
        otherwise
            scenario = createSyntheticScenario(caseIndex, draw);
    end
    obstacles    = scenario.Obstacles;
    initialState = scenario.InitialState;
    goalState    = scenario.GoalState;
    limits       = scenario.Limits;

    % A goal is either a sampled target or a fixed point, never both with
    % explicit derivatives, so derivative matching can never conflict.
    hasTarget = isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion);
    if hasTarget && draw(9) < 0.5
        goalState.position_units = obstacleAvoidance.input.targetPositionAtTime( ...
            goalState.targetMotion, goalState.time_s);
        goalState.targetMotion   = [];
        hasTarget = false;
    elseif ~hasTarget && draw(9) >= 0.5
        span_units  = 0.15 * max(diff(limits.xInterval_units), diff(limits.yInterval_units)) * ...
            (0.2 + 0.8 * draw(8));
        angle_rad   = 2 * pi * draw(7);
        start_units = goalState.position_units - span_units * [cos(angle_rad), sin(angle_rad)];
        goalState.targetMotion = struct( ...
            'time_s',              [initialState.time_s; goalState.time_s], ...
            'position_units',      [start_units; goalState.position_units], ...
            'InterpolationMethod', 'linear');
        goalState = rmfield(goalState, 'position_units');
        hasTarget = true;
    end
    if hasTarget
        goalState = rmfield(goalState, intersect(fieldnames(goalState), ...
            {'velocity_units_s', 'acceleration_units_s2'}));
    end

    options = struct( ...
        'GoalTimeMode',                      pickOne(draw(10), ["fixedArrival", "earliestArrival"]), ...
        'WrapX',                             draw(11) < 0.25, ...
        'WrapY',                             draw(12) < 0.15, ...
        'MatchTargetVelocity',               hasTarget && draw(13) < 0.35, ...
        'MatchTargetAcceleration',           hasTarget && draw(14) < 0.35, ...
        'ConstraintTolerance',               pickOne(draw(16), [1e-9, 1e-8, 1e-8, 1e-6]), ...
        'CollisionClearanceTolerance_units', pickOne(draw(17), [1e-7, 1e-7, 1e-3, 0.05]), ...
        'ArrivalTimeTolerance_s',            pickOne(draw(18), [1e-8, 1e-8, 1e-3]), ...
        'SampleTime_s',                      pickOne(draw(19), [0.02, 0.05, 0.1, 0.25]), ...
        'TemporalResolution_s',              pickOne(draw(20), [0.25, 0.5, 1, 2]), ...
        'SpatialProbeIterationLimit',        pickOne(draw(21), [1, 2, 2, 5, 35]), ...
        'BestSoFarRefinementTrialLimit',     pickOne(draw(22), [0, 0, 1, 3]), ...
        'MaxArrivalTrials',                  pickOne(draw(23), [5, 20, 60, 100]), ...
        'MaxArrivalCandidates',              pickOne(draw(24), [64, 512, 4096]));
    request = struct('obstacles', {obstacles}, 'initialState', initialState, ...
        'goalState', goalState, 'limits', limits, 'options', options);

    movingObstacleCount  = 0;
    concaveObstacleCount = 0;
    if ~isempty(obstacles)
        movingObstacleCount = nnz(arrayfun(@historyMoves, obstacles));
    end
    if isfield(scenario, 'ConcaveObstacleCount')
        concaveObstacleCount = scenario.ConcaveObstacleCount;
    end
    restStart = ~isfield(initialState, 'velocity_units_s') || ...
        (all(initialState.velocity_units_s == 0) && all(initialState.acceleration_units_s2 == 0));
    target = "static";
    if hasTarget
        target = "moving";
    end
    shape = struct('Family', family, 'Target', target, 'ObstacleCount', numel(obstacles), ...
        'MovingObstacleCount', movingObstacleCount, 'ConcaveObstacleCount', concaveObstacleCount, ...
        'RestStart', restStart);
end

function moves = historyMoves(obstacle)
    % A history moves only when some sample differs from the first one.
    moves = false;
    for sampleIndex = 2:numel(obstacle.time_s)
        moves = moves || ~isequaln([obstacle.x_units{sampleIndex}(:), obstacle.y_units{sampleIndex}(:)], ...
            [obstacle.x_units{1}(:), obstacle.y_units{1}(:)]);
    end
end

function scenario = createSyntheticScenario(caseIndex, draw)
    % Zero to five polygons, convex or star-concave, static or translating.
    % Endpoints sit in the two outer bands (|x| >= 13.5) of a canonical frame
    % and every polygon center stays inside the inner square (|x|, |y| <= 9)
    % over its whole history; with a radius of at most 3.5 and a margin of at
    % most 0.3 no footprint reaches 12.8, so no endpoint is ever occupied. A
    % rotation and mirror drawn per case vary the travel direction.
    limits = struct( ...
        'xInterval_units',          [-20, 20], ...
        'yInterval_units',          [-20, 20], ...
        'maxVelocity_units_s',      [1.5, 1.5] + 2 * draw(30), ...
        'maxAcceleration_units_s2', [1, 1] + 1.5 * draw(31), ...
        'maxJerk_units_s3',         [2, 2] + 3 * draw(32));
    innerHalfWidth_units = 9;
    obstacleCount        = floor(6 * draw(33));
    horizon_s            = 10 + 20 * draw(34);
    quarterTurns         = floor(4 * fractionOf(3.3 * draw(42)));
    mirrorSign           = 1 - 2 * (draw(43) < 0.5);
    frame                = [cos(quarterTurns * pi / 2), -sin(quarterTurns * pi / 2); ...
        sin(quarterTurns * pi / 2), cos(quarterTurns * pi / 2)] * [1, 0; 0, mirrorSign];
    obstacleList         = cell(obstacleCount, 1);
    concaveObstacleCount = 0;
    for obstacleIndex = 1:obstacleCount
        drawOffset  = 36 + 4 * obstacleIndex;
        isConcave   = fractionOf(7.9 * draw(drawOffset + 3)) < 0.3;
        vertexCount = 4 + floor(5 * fractionOf(draw(drawOffset) + 0.13 * obstacleIndex));
        if isConcave
            vertexCount          = 2 * (3 + floor(3 * fractionOf(draw(drawOffset) + 0.13 * obstacleIndex)));
            concaveObstacleCount = concaveObstacleCount + 1;
        end
        angles_rad = sort(2 * pi * fractionOf(draw(drawOffset + 1) + (1:vertexCount) / vertexCount + ...
            0.3 * fractionOf(draw(drawOffset + 2) * (1:vertexCount))));
        radius_units = (1 + 2.5 * fractionOf(draw(drawOffset + 3) + 0.37 * obstacleIndex)) * ones(1, vertexCount);
        if isConcave
            radius_units(2:2:end) = radius_units(2:2:end) * (0.4 + 0.3 * fractionOf(3.1 * draw(drawOffset + 1)));
        end
        center_units   = -innerHalfWidth_units + 2 * innerHalfWidth_units * ...
            [fractionOf(7.1 * draw(drawOffset) + obstacleIndex), fractionOf(3.7 * draw(drawOffset + 1) + 2 * obstacleIndex)];
        vertices_units = center_units + radius_units(:) .* [cos(angles_rad(:)), sin(angles_rad(:))];
        margin_units   = 0.05 + 0.25 * fractionOf(5.3 * draw(drawOffset + 2));
        obstacleName   = "Synthetic " + caseIndex + " obstacle " + obstacleIndex;
        if fractionOf(11.7 * draw(drawOffset + 3)) < 0.45
            sampleCount      = 2 + floor(3 * fractionOf(13.1 * draw(drawOffset)));
            sampleTime_s     = linspace(0, horizon_s, sampleCount).';
            velocity_units_s = 1.2 * limits.maxVelocity_units_s(1) * ...
                [fractionOf(17.3 * draw(drawOffset + 1)) - 0.5, fractionOf(19.9 * draw(drawOffset + 2)) - 0.5];
            % Shorten the travel so the final center also stays inside the
            % inner square; the center then never leaves it at any time.
            travel_units     = velocity_units_s * horizon_s;
            allowedTravel_units = innerHalfWidth_units - abs(center_units);
            velocity_units_s = velocity_units_s * min(1, min(allowedTravel_units ./ max(abs(travel_units), realmin)));
            x_units = cell(sampleCount, 1);
            y_units = cell(sampleCount, 1);
            for sampleIndex = 1:sampleCount
                shifted_units        = (vertices_units + sampleTime_s(sampleIndex) * velocity_units_s) * frame.';
                x_units{sampleIndex} = shifted_units(:, 1);
                y_units{sampleIndex} = shifted_units(:, 2);
            end
            obstacleList{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
                obstacleName, sampleTime_s, x_units, y_units, margin_units, ...
                struct('vertexCorrespondence', 'sourceIndex'));
        else
            placed_units = vertices_units * frame.';
            obstacleList{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
                obstacleName, 0, placed_units(:, 1), placed_units(:, 2), margin_units);
        end
    end
    if obstacleCount == 0
        obstacles = [];
    elseif obstacleCount == 1
        obstacles = obstacleList{1};
    else
        obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleList);
    end

    start_units   = [-15 + 1.5 * draw(56), -15 + 30 * draw(57)] * frame.';
    goal_units    = [13.5 + 1.5 * draw(58), -15 + 30 * draw(59)] * frame.';
    isRest        = draw(26) < 0.6;
    velocityScale = 0.5 * limits.maxVelocity_units_s(1);
    initialState  = struct('time_s', 0, 'position_units', start_units, ...
        'velocity_units_s', [0 0], 'acceleration_units_s2', [0 0]);
    goalState     = struct('time_s', horizon_s, 'position_units', goal_units, ...
        'velocity_units_s', [0 0], 'acceleration_units_s2', [0 0], 'targetMotion', []);
    if ~isRest
        initialState.velocity_units_s      = velocityScale * (2 * draw(27:28) - 1);
        initialState.acceleration_units_s2 = 0.3 * limits.maxAcceleration_units_s2(1) * (2 * draw(29:30) - 1);
        if draw(38) < 0.5
            goalState.velocity_units_s = velocityScale * (2 * draw(39:40) - 1);
        end
    end
    minimumDuration_s = obstacleAvoidance.input.minimumTravelTime(initialState, goalState, limits);
    goalState.time_s  = max(horizon_s, (1.3 + 2 * draw(41)) * minimumDuration_s);
    scenario = struct('Obstacles', obstacles, 'InitialState', initialState, ...
        'GoalState', goalState, 'Limits', limits, 'ConcaveObstacleCount', concaveObstacleCount);
end

function [class, detail] = classifyOutcome(result, validation, request)
    % Name the outcome so every defect is visible: a success the independent
    % validator rejects, a validator rejection of the planner's own candidate,
    % a failure reason the planner does not document as stable, an arrival
    % that breaks the requested clock, or an honest failure with its reason.
    honestReasons = ["arrivalSearchExhausted", "continuityProjectionExceedsTolerance", ...
        "departureUncertified", "dynamicEndpointInfeasible", "endpointBlocked", ...
        "endpointOutsideWorkspace", "fixedArrivalInfeasible", "noDepartureWindow", ...
        "noOptimizedFeasibleIterate", "noTimedRoute", "noVisibilityRoute", ...
        "planeCertificateUnavailable", "terminalReachabilityBlocked", ...
        "timeWindowInfeasible", "timedMotionInfeasible", ...
        "unsupportedObstacleInterpolation", "unsupportedTimedRequest"];
    reason = string(result.TerminationReason);
    detail = string(result.Message);
    if result.Success && ~validation.Passed
        class  = "ACCEPTED_INVALID";
        detail = string(validation.Message);
        return
    end
    if ~result.Success && reason == "invalidMotion"
        class = "VALIDATOR_REJECTED";
        return
    end
    if ~result.Success && ~any(reason == honestReasons)
        class = "UNEXPECTED_FAILURE";
        return
    end
    if ~result.Success
        class = "FAIL_" + reason;
        return
    end
    if reason ~= "goalReached"
        class  = "INCONSISTENT";
        detail = "success reported with termination reason " + reason;
        return
    end
    options     = request.options;
    goalTime_s  = request.goalState.time_s;
    tolerance_s = options.ArrivalTimeTolerance_s;
    detail      = "";
    if options.GoalTimeMode == "fixedArrival" && abs(result.ArrivalTime_s - goalTime_s) > tolerance_s
        class  = "INCONSISTENT";
        detail = sprintf("fixed arrival %.9g differs from goal %.9g", result.ArrivalTime_s, goalTime_s);
        return
    end
    if options.GoalTimeMode == "earliestArrival" && result.ArrivalTime_s > goalTime_s + tolerance_s
        class  = "INCONSISTENT";
        detail = sprintf("earliest arrival %.9g is after the horizon %.9g", result.ArrivalTime_s, goalTime_s);
        return
    end
    class = "OK";
end

function row = campaignRow(caseIndex, shape, class, options, wall_s, arrival_s, length_units, reason, detail)
    % One typed row per case; the defect classes are named in one place.
    isDefect = any(class == ["ACCEPTED_INVALID", "VALIDATOR_REJECTED", "UNEXPECTED_FAILURE", ...
        "INCONSISTENT", "ERROR", "BUILD_ERROR"]);
    row = table(caseIndex, shape.Family, shape.Target, class, isDefect, ...
        string(options.GoalTimeMode), options.WrapX, options.WrapY, ...
        options.MatchTargetVelocity, options.MatchTargetAcceleration, ...
        options.ConstraintTolerance, options.CollisionClearanceTolerance_units, ...
        options.ArrivalTimeTolerance_s, options.SampleTime_s, options.TemporalResolution_s, ...
        options.SpatialProbeIterationLimit, options.BestSoFarRefinementTrialLimit, ...
        options.MaxArrivalTrials, options.MaxArrivalCandidates, ...
        shape.ObstacleCount, shape.MovingObstacleCount, shape.ConcaveObstacleCount, ...
        shape.RestStart, wall_s, arrival_s, length_units, reason, detail, ...
        'VariableNames', {'Case', 'Family', 'Target', 'Class', 'Defect', ...
        'GoalTimeMode', 'WrapX', 'WrapY', 'MatchTargetVelocity', 'MatchTargetAcceleration', ...
        'ConstraintTolerance', 'CollisionClearanceTolerance_units', 'ArrivalTimeTolerance_s', ...
        'SampleTime_s', 'TemporalResolution_s', 'SpatialProbeIterationLimit', ...
        'BestSoFarRefinementTrialLimit', 'MaxArrivalTrials', 'MaxArrivalCandidates', ...
        'ObstacleCount', 'MovingObstacleCount', 'ConcaveObstacleCount', 'RestStart', ...
        'Wall_s', 'Arrival_s', 'Length_units', 'Reason', 'Detail'});
end

function choice = pickOne(uniformDraw, choices)
    % Select one entry of choices with equal probability from a draw in [0, 1).
    choice = choices(min(numel(choices), 1 + floor(uniformDraw * numel(choices))));
end

function fraction = fractionOf(value)
    fraction = value - floor(value);
end
