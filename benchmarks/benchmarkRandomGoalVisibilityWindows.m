function [runs, cases, results] = benchmarkRandomGoalVisibilityWindows(requestFilePath, caseCount, outputFolder)
%% Section 0: Header & Readme
% SYNTAX
%   runs = benchmarkRandomGoalVisibilityWindows(requestFilePath)
%   [runs,cases,results] = benchmarkRandomGoalVisibilityWindows( ...
%       requestFilePath,caseCount,outputFolder)
% PURPOSE
%   Generate deterministic randomized variants of an offline sandbox request
%   whose protected goal is visible, then hidden, then visible again. Run the
%   public planner and independent validator on every accepted variant.
% INPUTS
%   requestFilePath: offlineSandboxRequest/v1 JSON template.
%   caseCount: positive integer number of accepted cases (default 50).
%   outputFolder: optional existing or new folder for compact MAT/CSV results.
% OUTPUTS
%   runs: one table row per planner run, including validity, motion quality,
%       runtime, goal-window timing, timed-search stage, and random parameters.
%   cases: caseCount-by-1 cell array of the generated request structs.
%   results: caseCount-by-1 cell array of complete public planner results.
% UNITS
%   Positions and path length are coordinate units. Time is seconds. Angles
%   reported in runs are degrees.

%% Section 1: Validate Controls And Read The Template

if nargin < 1 || strlength(string(requestFilePath)) == 0
    root = fileparts(fileparts(mfilename("fullpath")));
    requestFilePath = fullfile(root,"RogueCasses","x-y-request.json");
end
if nargin < 2 || isempty(caseCount), caseCount = 50; end
if nargin < 3, outputFolder = ""; end
requestFilePath = string(requestFilePath);
outputFolder = string(outputFolder);
assert(isscalar(requestFilePath) && isfile(requestFilePath), ...
    "benchmarkRandomGoalVisibilityWindows:MissingTemplate", ...
    "requestFilePath must name an existing JSON request.");
validateattributes(caseCount,{'numeric'},{'scalar','integer','positive'});
assert(isscalar(outputFolder), ...
    "benchmarkRandomGoalVisibilityWindows:InvalidOutputFolder", ...
    "outputFolder must be scalar text.");
template = jsondecode(fileread(requestFilePath));
assert(isstruct(template) && isscalar(template) && ...
    isfield(template,"schemaVersion") && ...
    string(template.schemaVersion) == "offlineSandboxRequest/v1", ...
    "benchmarkRandomGoalVisibilityWindows:InvalidTemplate", ...
    "The template must be one offlineSandboxRequest/v1 record.");
assert(numel(template.obstacles) >= 2 && ...
    numel(template.obstacles(2).keyframes) >= 3, ...
    "benchmarkRandomGoalVisibilityWindows:InsufficientMotionHistory", ...
    "The template needs a static route obstacle and a moving goal occluder.");
root = fileparts(fileparts(mfilename("fullpath")));
addpath(root,fullfile(root,"trajectory"));
if strlength(outputFolder) > 0 && ~isfolder(outputFolder), mkdir(outputFolder); end

%% Section 2: Generate Only Visible-Hidden-Visible Goal Cases

masterSeed = 20260911;
previousRandomState = rng;
restoreRandomState = onCleanup(@() rng(previousRandomState));
cases = cell(caseCount,1);
caseParameters = cell(caseCount,1);
windowRecords = cell(caseCount,1);
for caseIndex = 1:caseCount
    accepted = false;
    for attemptIndex = 1:100
        caseSeed = masterSeed + 1000*caseIndex + attemptIndex;
        rng(caseSeed,"twister");
        [candidate,parameters] = randomizeRequest(template,caseIndex,caseSeed);
        obstacles = createObstacles(candidate.obstacles);
        [hasRequiredWindows,window] = characterizeGoalWindows( ...
            obstacles,candidate.initialState,candidate.goalState);
        endpointsInsideBounds = all(candidate.initialState.position_units >= ...
            [candidate.limits.xInterval_units(1),candidate.limits.yInterval_units(1)]) && ...
            all(candidate.initialState.position_units <= ...
            [candidate.limits.xInterval_units(2),candidate.limits.yInterval_units(2)]) && ...
            all(candidate.goalState.position_units >= ...
            [candidate.limits.xInterval_units(1),candidate.limits.yInterval_units(1)]) && ...
            all(candidate.goalState.position_units <= ...
            [candidate.limits.xInterval_units(2),candidate.limits.yInterval_units(2)]);
        startOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
            obstacles,candidate.initialState.position_units(1), ...
            candidate.initialState.position_units(2),candidate.initialState.time_s);
        if hasRequiredWindows && endpointsInsideBounds && ~startOccupied
            accepted = true;
            cases{caseIndex} = candidate;
            caseParameters{caseIndex} = parameters;
            windowRecords{caseIndex} = window;
            break;
        end
    end
    assert(accepted, ...
        "benchmarkRandomGoalVisibilityWindows:GenerationFailed", ...
        "Could not generate valid goal windows for case %d.",caseIndex);
end

%% Section 3: Run The Public Planner And Independent Validator

rows = cell(caseCount,1);
results = cell(caseCount,1);
for caseIndex = 1:caseCount
    request = cases{caseIndex};
    parameters = caseParameters{caseIndex};
    window = windowRecords{caseIndex};
    obstacles = createObstacles(request.obstacles);
    initialState = normalizeState(request.initialState);
    goalState = normalizeState(request.goalState);
    necessaryArrivalBound_s = initialState.time_s + ...
        obstacleAvoidance.input.minimumTravelTime( ...
        initialState,goalState,request.limits);
    firstWindowKinematicallyPossible = ...
        necessaryArrivalBound_s < window.HiddenStart_s;
    wallTimer = tic;
    exceptionText = "";
    try
        result = planner(obstacles,initialState,goalState, ...
            request.limits,request.options);
        wallTime_s = toc(wallTimer);
        if result.Success
            validation = obstacleAvoidance.validateTrajectory(result);
        else
            validation = result.Validation;
        end
    catch exception
        wallTime_s = toc(wallTimer);
        result = struct('Success',false,'TerminationReason',"exception", ...
            'ArrivalTime_s',NaN,'TrajectoryDuration_s',NaN, ...
            'MotionLength_units',NaN,'IntegratedSquaredJerk_units2_s5',NaN, ...
            'ElapsedTime_s',NaN);
        validation = struct('Passed',false);
        exceptionText = string(exception.identifier)+": "+string(exception.message);
    end
    results{caseIndex} = result;
    trialStage = "none";
    if isfield(result,"TemporalSearch") && ...
            isfield(result.TemporalSearch,"TrialStage")
        trialStage = string(result.TemporalSearch.TrialStage);
    end
    arrivalTime_s = numericField(result,"ArrivalTime_s");
    trajectoryDuration_s = numericField(result,"TrajectoryDuration_s");
    pathLength_units = numericField(result,"MotionLength_units");
    integratedSquaredJerk_units2_s5 = numericField( ...
        result,"IntegratedSquaredJerk_units2_s5");
    plannerTime_s = numericField(result,"ElapsedTime_s");
    terminationReason = stringField(result,"TerminationReason","unavailable");
    waitDuration_s = routeWaitDuration(result);
    arrivalWindow = classifyArrival(arrivalTime_s,window);
    rows{caseIndex} = table(caseIndex,parameters.Seed, ...
        parameters.SwappedAxes,parameters.ReflectedFirstAxis, ...
        parameters.ReflectedSecondAxis,parameters.SpatialScale, ...
        parameters.TimeScale,parameters.StaticSizeScale, ...
        parameters.OccluderSizeScale,parameters.ReopenChallenge, ...
        parameters.VelocityLimitScale,parameters.SafetyMargin_units, ...
        window.HiddenStart_s,window.Reopen_s,wallTime_s,plannerTime_s, ...
        necessaryArrivalBound_s,firstWindowKinematicallyPossible, ...
        logical(result.Success),logical(validation.Passed), ...
        terminationReason,trialStage,arrivalWindow, ...
        arrivalTime_s,trajectoryDuration_s,pathLength_units, ...
        integratedSquaredJerk_units2_s5, ...
        waitDuration_s,exceptionText, ...
        'VariableNames',{'CaseIndex','Seed','SwappedAxes', ...
        'ReflectedFirstAxis','ReflectedSecondAxis','SpatialScale','TimeScale', ...
        'StaticSizeScale','OccluderSizeScale','ReopenChallenge', ...
        'VelocityLimitScale','SafetyMargin_units','GoalHiddenStart_s', ...
        'GoalReopen_s','WallTime_s','PlannerTime_s', ...
        'NecessaryArrivalBound_s','FirstWindowKinematicallyPossible', ...
        'Success','ValidationPassed','TerminationReason', ...
        'TrialStage','ArrivalWindow','ArrivalTime_s','TrajectoryDuration_s', ...
        'PathLength_units','IntegratedSquaredJerk_units2_s5', ...
        'RouteWaitDuration_s','Exception'});
    fprintf(['Case %02d/%02d: success=%d valid=%d wall=%.3f arrival=%.3f ' ...
        'length=%.3f hidden=[%.2f,%.2f] stage=%s\n'], ...
        caseIndex,caseCount,result.Success,validation.Passed,wallTime_s, ...
        arrivalTime_s,pathLength_units,window.HiddenStart_s, ...
        window.Reopen_s,trialStage);
end
runs = vertcat(rows{:});

%% Section 4: Save Optional Reproducible Evidence

if strlength(outputFolder) > 0
    save(fullfile(outputFolder,"random_goal_visibility_windows.mat"), ...
        "runs","cases","masterSeed");
    writetable(runs,fullfile(outputFolder,"random_goal_visibility_windows.csv"));
    createSummaryPlot(runs,outputFolder);
end
clear restoreRandomState;
end

%% Section 5: Local Functions

function [request,parameters] = randomizeRequest(template,caseIndex,caseSeed)
    request = template;
    request.requestId = sprintf('random-goal-window-%02d-seed-%d',caseIndex,caseSeed);
    spatialScale = 0.85+0.30*rand;
    timeScale = 0.85+0.30*rand;
    swappedAxes = rand >= 0.5;
    axisSign = 2*(rand(1,2) >= 0.5)-1;
    if swappedAxes
        coordinateMap = [0,axisSign(1);axisSign(2),0];
    else
        coordinateMap = diag(axisSign);
    end
    translation_units = [-5+10*rand,-3+6*rand];
    pivot_units = reshape(double(template.goalState.position_units),1,2);
    staticSizeScale = 0.94+0.06*rand;
    movingSizeScale = 0.94+0.06*rand;
    obstacleSizeScale = [staticSizeScale,movingSizeScale, ...
        ones(1,max(0,numel(request.obstacles)-2))];
    marginScale = 0.85+0.15*rand;
    for obstacleIndex = 1:numel(request.obstacles)
        for sampleIndex = 1:numel(request.obstacles(obstacleIndex).keyframes)
            vertices_units = double(request.obstacles(obstacleIndex).keyframes( ...
                sampleIndex).vertices_units);
            center_units = mean(vertices_units,1);
            vertices_units = center_units+obstacleSizeScale(obstacleIndex)* ...
                (vertices_units-center_units);
            vertices_units = transformPoints(vertices_units,pivot_units, ...
                spatialScale,coordinateMap,translation_units);
            request.obstacles(obstacleIndex).keyframes( ...
                sampleIndex).vertices_units = vertices_units;
            sourceTime_s = double(template.obstacles(obstacleIndex).keyframes( ...
                sampleIndex).time_s);
            request.obstacles(obstacleIndex).keyframes(sampleIndex).time_s = ...
                double(template.initialState.time_s)+timeScale* ...
                (sourceTime_s-double(template.initialState.time_s));
        end
        request.obstacles(obstacleIndex).safetyMargin_units = ...
            double(template.obstacles(obstacleIndex).safetyMargin_units)* ...
            spatialScale*marginScale;
    end
    request.initialState.position_units = transformPoints( ...
        reshape(double(template.initialState.position_units),1,2), ...
        pivot_units,spatialScale,coordinateMap,translation_units);
    request.goalState.position_units = transformPoints(pivot_units,pivot_units, ...
        spatialScale,coordinateMap,translation_units);
    request.initialState.time_s = double(template.initialState.time_s);
    request.goalState.time_s = double(template.initialState.time_s)+timeScale* ...
        (double(template.goalState.time_s)-double(template.initialState.time_s));
    reopenChallenge = mod(caseIndex,2) == 0;
    velocityChallengeScale = 1;
    if reopenChallenge
        velocityChallengeScale = 0.66+0.08*rand;
    end
    velocityLimitScale = (spatialScale/timeScale)*velocityChallengeScale;
    request.limits.maxVelocity_units_s = mapDerivativeLimit( ...
        template.limits.maxVelocity_units_s,coordinateMap,velocityLimitScale);
    request.limits.maxAcceleration_units_s2 = mapDerivativeLimit( ...
        template.limits.maxAcceleration_units_s2,coordinateMap, ...
        spatialScale/timeScale^2);
    request.limits.maxJerk_units_s3 = mapDerivativeLimit( ...
        template.limits.maxJerk_units_s3,coordinateMap, ...
        spatialScale/timeScale^3);
    transformedBounds = transformBounds(template.limits,pivot_units, ...
        spatialScale,coordinateMap,translation_units);
    request.limits.xInterval_units = transformedBounds(1,:);
    request.limits.yInterval_units = transformedBounds(2,:);
    request.options.GoalTimeMode = 'earliestArrival';
    parameters = struct('Seed',caseSeed,'SwappedAxes',swappedAxes, ...
        'ReflectedFirstAxis',axisSign(1)<0, ...
        'ReflectedSecondAxis',axisSign(2)<0, ...
        'SpatialScale',spatialScale,'TimeScale',timeScale, ...
        'StaticSizeScale',staticSizeScale,'OccluderSizeScale',movingSizeScale, ...
        'ReopenChallenge',reopenChallenge, ...
        'VelocityLimitScale',velocityLimitScale, ...
        'SafetyMargin_units',request.obstacles(2).safetyMargin_units);
end

function points_units = transformPoints(points_units,pivot_units,scale,coordinateMap,translation_units)
    points_units = ((double(points_units)-pivot_units)*scale)*coordinateMap.'+ ...
        pivot_units+translation_units;
end

function limit = mapDerivativeLimit(sourceLimit,coordinateMap,scale)
    sourceLimit = reshape(double(sourceLimit),1,[]);
    if isscalar(sourceLimit)
        limit = sourceLimit*scale;
        return;
    end
    limit = (abs(coordinateMap)*sourceLimit(:)).'*scale;
end

function bounds = transformBounds(limits,pivot_units,scale,coordinateMap,translation_units)
    corners_units = [limits.xInterval_units(1),limits.yInterval_units(1); ...
        limits.xInterval_units(1),limits.yInterval_units(2); ...
        limits.xInterval_units(2),limits.yInterval_units(1); ...
        limits.xInterval_units(2),limits.yInterval_units(2)];
    corners_units = transformPoints(corners_units,pivot_units,scale, ...
        coordinateMap,translation_units);
    bounds = [min(corners_units(:,1)),max(corners_units(:,1)); ...
        min(corners_units(:,2)),max(corners_units(:,2))];
end

function obstacles = createObstacles(obstacleInput)
    obstacleCells = cell(numel(obstacleInput),1);
    for obstacleIndex = 1:numel(obstacleInput)
        keyframes = obstacleInput(obstacleIndex).keyframes;
        time_s = arrayfun(@(sample) double(sample.time_s),keyframes).';
        xByTime_units = cell(numel(keyframes),1);
        yByTime_units = cell(numel(keyframes),1);
        for sampleIndex = 1:numel(keyframes)
            vertices_units = double(keyframes(sampleIndex).vertices_units);
            xByTime_units{sampleIndex} = vertices_units(:,1);
            yByTime_units{sampleIndex} = vertices_units(:,2);
        end
        obstacleCells{obstacleIndex} = obstacleAvoidance.obstacles.createObstacle( ...
            string(obstacleInput(obstacleIndex).name),time_s,xByTime_units, ...
            yByTime_units,double(obstacleInput(obstacleIndex).safetyMargin_units));
    end
    obstacles = obstacleAvoidance.obstacles.combineObstacles(obstacleCells);
end

function state = normalizeState(input)
    state = input;
    state.time_s = double(input.time_s);
    state.position_units = reshape(double(input.position_units),1,2);
    state.velocity_units_s = reshape(double(input.velocity_units_s),1,2);
    state.acceleration_units_s2 = reshape(double(input.acceleration_units_s2),1,2);
end

function [passed,window] = characterizeGoalWindows(obstacles,initialState,goalState)
    startTime_s = double(initialState.time_s);
    endTime_s = double(goalState.time_s);
    queryTime_s = linspace(startTime_s,endTime_s,3601).';
    goal_units = reshape(double(goalState.position_units),1,2);
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime( ...
        obstacles,repmat(goal_units(1),size(queryTime_s)), ...
        repmat(goal_units(2),size(queryTime_s)),queryTime_s);
    hiddenStartIndex = find(occupied,1,'first');
    reopenIndex = [];
    if ~isempty(hiddenStartIndex)
        reopenOffset = find(~occupied(hiddenStartIndex:end),1,'first');
        if ~isempty(reopenOffset)
            reopenIndex = hiddenStartIndex+reopenOffset-1;
        end
    end
    passed = ~occupied(1) && ~occupied(end) && ...
        ~isempty(hiddenStartIndex) && ~isempty(reopenIndex) && ...
        hiddenStartIndex > 1 && reopenIndex > hiddenStartIndex;
    window = struct('HiddenStart_s',NaN,'Reopen_s',NaN);
    if passed
        window.HiddenStart_s = queryTime_s(hiddenStartIndex);
        window.Reopen_s = queryTime_s(reopenIndex);
    end
end

function waitDuration_s = routeWaitDuration(result)
    waitDuration_s = NaN;
    if ~result.Success || ~isfield(result,'VisibilityGraph') || ...
            ~isfield(result.VisibilityGraph,'RouteTime_s')
        return;
    end
    route_units = result.VisibilityGraph.Route_units;
    routeTime_s = result.VisibilityGraph.RouteTime_s(:);
    if size(route_units,1) ~= numel(routeTime_s) || numel(routeTime_s) < 2
        return;
    end
    stationary = vecnorm(diff(route_units,1,1),2,2) <= 1e-10;
    waitDuration_s = sum(diff(routeTime_s).*stationary);
end

function arrivalWindow = classifyArrival(arrivalTime_s,window)
    arrivalWindow = "unavailable";
    if ~isfinite(arrivalTime_s), return; end
    if arrivalTime_s < window.HiddenStart_s
        arrivalWindow = "firstVisibleWindow";
    elseif arrivalTime_s >= window.Reopen_s
        arrivalWindow = "reopenedWindow";
    else
        arrivalWindow = "hiddenWindow";
    end
end

function value = numericField(container,name)
    value = NaN;
    if isstruct(container) && isfield(container,name) && ...
            isnumeric(container.(name)) && isscalar(container.(name))
        value = double(container.(name));
    end
end

function value = stringField(container,name,defaultValue)
    value = string(defaultValue);
    if isstruct(container) && isfield(container,name)
        candidate = string(container.(name));
        if isscalar(candidate), value = candidate; end
    end
end

function createSummaryPlot(runs,outputFolder)
    caseIndex = runs.CaseIndex;
    control = ~runs.ReopenChallenge;
    challenge = runs.ReopenChallenge;
    valid = runs.Success & runs.ValidationPassed;
    hiddenDuration_s = runs.GoalReopen_s-runs.GoalHiddenStart_s;
    normalizedArrival = (runs.ArrivalTime_s-runs.GoalHiddenStart_s)./hiddenDuration_s;
    figureHandle = figure('Visible','off','Color','white', ...
        'Position',[100,100,1100,850]);
    layout = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    hold on;
    patch([0.5;numel(caseIndex)+0.5;numel(caseIndex)+0.5;0.5], ...
        [0;0;1;1],[1,0.89,0.66],'EdgeColor','none','FaceAlpha',0.65);
    scatter(caseIndex(control & valid),normalizedArrival(control & valid), ...
        34,[0.12,0.47,0.71],'filled');
    scatter(caseIndex(challenge & valid),normalizedArrival(challenge & valid), ...
        42,[0.42,0.24,0.68],'filled');
    scatter(caseIndex(~valid),0.5*ones(sum(~valid),1),54,[0.8,0.1,0.1],'x', ...
        'LineWidth',1.5);
    yline(0,'-','Goal becomes hidden','Color',[0.45,0.3,0.05]);
    yline(1,'-','Goal reopens','Color',[0.45,0.3,0.05]);
    ylabel('Normalized arrival window');
    title('Arrival timing: control versus forced-reopen cases');
    grid on;

    nexttile;
    hold on;
    scatter(caseIndex(control),runs.WallTime_s(control),34,[0.12,0.47,0.71],'filled');
    scatter(caseIndex(challenge & valid),runs.WallTime_s(challenge & valid), ...
        42,[0.42,0.24,0.68],'filled');
    scatter(caseIndex(challenge & ~valid),runs.WallTime_s(challenge & ~valid), ...
        54,[0.8,0.1,0.1],'x','LineWidth',1.5);
    set(gca,'YScale','log');
    ylabel('Wall runtime (s)');
    title('Runtime; red crosses are failed requests');
    grid on;

    nexttile;
    hold on;
    scatter(caseIndex(control & valid),runs.PathLength_units(control & valid), ...
        34,[0.12,0.47,0.71],'filled');
    scatter(caseIndex(challenge & valid),runs.PathLength_units(challenge & valid), ...
        42,[0.42,0.24,0.68],'filled');
    scatter(caseIndex(~valid),zeros(sum(~valid),1),54,[0.8,0.1,0.1],'x', ...
        'LineWidth',1.5);
    xlabel('Deterministic random case');
    ylabel('Motion path length (units)');
    title('Validated path length; failures have no motion');
    grid on;
    legend({'Control success','Forced-reopen success','Failure'}, ...
        'Location','eastoutside');
    title(layout,sprintf(['%d-case visible-hidden-visible benchmark: ' ...
        '%d/%d validated'],height(runs),sum(valid),height(runs)));
    exportgraphics(figureHandle,fullfile(outputFolder, ...
        'random_goal_visibility_summary.png'),'Resolution',180);
    close(figureHandle);
end
