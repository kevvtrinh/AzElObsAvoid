function [seeds, diagnostics, reduction] = buildAzElRlHybridSeeds( ...
        seeds, obstacleField, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics, reduction] = ...
%       azElInternal.buildAzElRlHybridSeeds( ...
%       seeds, obstacleField, limits, options)
%**************************************************************************
% PURPOSE
%   - Use the required RL agent to shape every spatial topology seed.
%   - Preserve a timed SIPP law for HS-3 initialization.
%**************************************************************************
% INPUTS
%   - seeds (N-by-1 structure array)
%       Planner seeds with Route_deg, RouteTime_s, Source, and route cost.
%   - obstacleField (scalar packed-obstacle struct)
%       Protected geometry used to reject unsafe agent corner displacement.
%   - limits (scalar struct)
%       Finite two-axis velocity, acceleration, and jerk limits.
%   - options (scalar struct)
%       RpAgentFile and RpTurnRadius_deg.
%**************************************************************************
% OUTPUTS
%   - seeds (M-by-1 structure array)
%       RL-shaped HS-3 seeds with preserved time-law fields.
%   - diagnostics (N-by-1 structure array)
%       Policy status, corner actions, and before/after route metrics.
%   - reduction (scalar struct)
%       Generated, retained, and cap-dropped RL route counts.
%**************************************************************************
% UNITS
%   - Position and route length use degrees. Time uses seconds.
%**************************************************************************

%% Section 1: Validate Inputs And Load The Required Agent

requiredSeedFields = ["Source" "Route_deg" "RouteTime_s" ...
    "RouteCost_deg"];
if ~isstruct(seeds) || ...
        (~isempty(seeds) && ~all(isfield(seeds, requiredSeedFields)))
    error("buildAzElRlHybridSeeds:InvalidSeeds", ...
        "seeds must be a structure array with the required route fields.");
end
if ~isstruct(obstacleField) || ~isscalar(obstacleField) || ...
        ~isfield(obstacleField, "Obstacles")
    error("buildAzElRlHybridSeeds:InvalidObstacleField", ...
        "obstacleField must be one scalar packed obstacle field.");
end
if ~isstruct(limits) || ~isscalar(limits) || ~all(isfield(limits, ...
        ["maxVelocity_deg_s" "maxAcceleration_deg_s2" ...
        "maxJerk_deg_s3"]))
    error("buildAzElRlHybridSeeds:InvalidLimits", ...
        "limits must contain finite velocity, acceleration, and jerk limits.");
end
if ~isstruct(options) || ~isscalar(options) || ~all(isfield(options, ...
        ["RpAgentFile" "RpTurnRadius_deg" ...
        "MaximumDirectCollocationSeeds"]))
    error("buildAzElRlHybridSeeds:InvalidOptions", ...
        "options must contain the RP seed controls.");
end
agent = loadRequiredAgent(options.RpAgentFile);

%% Section 2: Evaluate The Agent And Add Each Rounded Warm Start

inputSeeds = seeds(diverseSeedOrder(seeds));
if isempty(inputSeeds)
    diagnostics = repmat(seedDiagnosticTemplate(), 0, 1);
    reduction = reductionRecord(0, 0);
    return;
end
seedTemplate = addRpFields(inputSeeds(1));
agentSeeds = repmat(seedTemplate, 0, 1);
agentDiagnostics = repmat(seedDiagnosticTemplate(), 0, 1);
for inputSeedIndex = 1:numel(inputSeeds)
    originalRoute_deg = double(inputSeeds(inputSeedIndex).Route_deg);
    originalRouteTime_s = double( ...
        inputSeeds(inputSeedIndex).RouteTime_s(:));
    [timeFraction, routeProgress, routeDuration_s] = timedSeedLaw( ...
        originalRoute_deg, originalRouteTime_s);
    [rlRoute_deg, calibratedRadiusScale, rawRadiusScale, ...
        cornerStatus] = roundRouteWithAgent( ...
        originalRoute_deg, agent, limits, options);
    [rlRoute_deg, routeScale] = projectRouteToClearSeed( ...
        originalRoute_deg, rlRoute_deg, obstacleField, ...
        inputSeeds(inputSeedIndex).SnapshotTime_s);
    radiusScale = routeScale * calibratedRadiusScale;

    agentSeed = addRpFields(inputSeeds(inputSeedIndex));
    agentSeed.RpSeedVariant = "agentDirect";
    agentSeed.RpSeedRouteTimeFraction = timeFraction;
    agentSeed.RpSeedRouteProgress = routeProgress;
    agentSeed.RpSeedRouteDuration_s = routeDuration_s;

    commonRecord = seedDiagnosticTemplate();
    commonRecord.OriginalSeedIndex = inputSeedIndex;
    commonRecord.Source = string(agentSeed.Source);
    commonRecord.PolicyStatus = "proposalEvaluated";
    if isempty(radiusScale)
        commonRecord.PolicyStatus = "loadedNoCorner";
    elseif routeScale < 1
        commonRecord.PolicyStatus = "proposalClearanceProjected";
        cornerStatus(:) = "agentRadiusReducedForClearance";
    end
    commonRecord.AgentLoaded = true;
    commonRecord.OriginalRoute_deg = originalRoute_deg;
    commonRecord.RlSeedRoute_deg = rlRoute_deg;
    commonRecord.OriginalRouteLength_deg = routeLength(originalRoute_deg);
    commonRecord.RlSeedRouteLength_deg = routeLength(rlRoute_deg);
    commonRecord.CornerCount = numel(radiusScale);
    commonRecord.EvaluatedCornerCount = numel(radiusScale);
    commonRecord.RawRadiusScale = rawRadiusScale;
    commonRecord.CalibratedRadiusScale = calibratedRadiusScale;
    commonRecord.ClearanceProjectionScale = routeScale;
    commonRecord.RadiusScale = radiusScale;
    commonRecord.CornerStatus = cornerStatus;
    commonRecord.TimedSeedLawPreserved = ~isempty(timeFraction);

    routeChanged = ~isequal(size(originalRoute_deg), size(rlRoute_deg)) || ...
        max(abs(originalRoute_deg - rlRoute_deg), [], "all") > 1e-12;
    if routeChanged
        agentSeed.Route_deg = rlRoute_deg;
        agentSeed.RouteCost_deg = routeLength(rlRoute_deg);
        agentSeed.RouteTime_s = zeros(0, 1);
        agentSeed.RpSeedVariant = "agentRounded";
    end
    commonRecord.SeedVariant = agentSeed.RpSeedVariant;
    commonRecord.AgentRouteApplied = true;
    agentSeeds(end + 1, 1) = agentSeed; %#ok<AGROW>
    agentDiagnostics(end + 1, 1) = commonRecord; %#ok<AGROW>
end

generatedSeeds = agentSeeds;
generatedDiagnostics = agentDiagnostics;
maximumSeedCount = min( ...
    numel(generatedSeeds), options.MaximumDirectCollocationSeeds);
seeds = generatedSeeds(1:maximumSeedCount);
diagnostics = generatedDiagnostics(1:maximumSeedCount);
for seedIndex = 1:numel(seeds)
    diagnostics(seedIndex).SeedIndex = seedIndex;
end
reduction = reductionRecord(numel(generatedSeeds), numel(seeds));
end

%% Section 3: Local Functions

function order = diverseSeedOrder(seeds)
% PURPOSE
%   - Place one seed from each route source before same-source repeats.
if isempty(seeds)
    order = zeros(0, 1);
    return;
end
sources = string({seeds.Source}).';
[~, firstIndex] = unique(sources, "stable");
remainingIndex = setdiff((1:numel(seeds)).', firstIndex, "stable");
order = [firstIndex; remainingIndex];
end

function [clearRoute_deg, appliedScale] = projectRouteToClearSeed( ...
        originalRoute_deg, agentRoute_deg, obstacleField, snapshotTime_s)
% PURPOSE
%   - Reduce the RL displacement until the spatial seed is collision-free.
if obstacleField.ObstacleCount == 0
    clearRoute_deg = agentRoute_deg;
    appliedScale = 1;
    return;
end
[originalProgress, originalRoute_deg] = routeProgress(originalRoute_deg);
[agentProgress, agentRoute_deg] = routeProgress(agentRoute_deg);
commonProgress = unique([originalProgress; agentProgress]);
originalCommon_deg = interp1(originalProgress, originalRoute_deg, ...
    commonProgress, "linear");
agentCommon_deg = interp1(agentProgress, agentRoute_deg, ...
    commonProgress, "linear");
scaleCandidates = [1, 0.5 .^ (1:12), 0];
for scaleIndex = 1:numel(scaleCandidates)
    appliedScale = scaleCandidates(scaleIndex);
    trialRoute_deg = originalCommon_deg + appliedScale * ...
        (agentCommon_deg - originalCommon_deg);
    collisionMask = queryAzElTimedPathCollision( ...
        obstacleField, snapshotTime_s, trialRoute_deg, struct( ...
        "BoundaryIsOccupied", false, "StopAtFirstCollision", true));
    if ~any(collisionMask)
        clearRoute_deg = trialRoute_deg;
        return;
    end
end
clearRoute_deg = originalCommon_deg;
appliedScale = 0;
end

function [progress, route_deg] = routeProgress(route_deg)
% PURPOSE
%   - Return unique normalized arc positions for route interpolation.
routeStep_deg = diff(route_deg, 1, 1);
keepRow = [true; hypot(routeStep_deg(:, 1), ...
    routeStep_deg(:, 2)) > 1e-12];
route_deg = route_deg(keepRow, :);
arcLength_deg = [0; cumsum(vecnorm(diff(route_deg, 1, 1), 2, 2))];
progress = arcLength_deg / arcLength_deg(end);
end

function agent = loadRequiredAgent(agentFile)
% PURPOSE
%   - Load and cache one validated deterministic inference policy.
persistent cachedAgent cachedAgentFile
agentFile = string(agentFile);
if ~isfile(agentFile)
    error("buildAzElRlHybridSeeds:AgentNotFound", ...
        "RpAgentFile does not exist: %s", agentFile);
end
if isempty(cachedAgent) || isempty(cachedAgentFile) || ...
        cachedAgentFile ~= agentFile
    agentData = load(agentFile, "agent", "metadata");
    validAgent = isfield(agentData, "agent") && ...
        isfield(agentData, "metadata") && ...
        isfield(agentData.metadata, "Format") && ...
        string(agentData.metadata.Format) == "AzElRpRetimerAgent";
    if ~validAgent
        error("buildAzElRlHybridSeeds:InvalidAgent", ...
            "RpAgentFile must contain an AzElRpRetimerAgent model.");
    end
    cachedAgent = agentData.agent;
    cachedAgent.UseExplorationPolicy = false;
    cachedAgentFile = agentFile;
end
agent = cachedAgent;
end

function [roundedRoute_deg, radiusScale, rawRadiusScale, cornerStatus] = ...
        roundRouteWithAgent(route_deg, agent, limits, options)
% PURPOSE
%   - Convert one polyline to an RL-rounded G3 topology seed.
validateattributes(route_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
routeStep_deg = diff(route_deg, 1, 1);
route_deg = route_deg([true; hypot( ...
    routeStep_deg(:, 1), routeStep_deg(:, 2)) > 1e-10], :);
if size(route_deg, 1) < 2
    error("buildAzElRlHybridSeeds:ZeroLengthRoute", ...
        "Each seed route must contain two distinct positions.");
end

cornerCount = max(0, size(route_deg, 1) - 2);
radiusScale = zeros(cornerCount, 1);
rawRadiusScale = zeros(cornerCount, 1);
cornerStatus = strings(cornerCount, 1);
entryPosition_deg = route_deg(2:end - 1, :);
exitPosition_deg = entryPosition_deg;
controlPointsByCorner_deg = cell(cornerCount, 1);

for cornerIndex = 1:cornerCount
    corner_deg = route_deg(cornerIndex + 1, :);
    incomingVector_deg = corner_deg - route_deg(cornerIndex, :);
    outgoingVector_deg = route_deg(cornerIndex + 2, :) - corner_deg;
    incomingLength_deg = norm(incomingVector_deg);
    outgoingLength_deg = norm(outgoingVector_deg);
    incomingDirection = incomingVector_deg / incomingLength_deg;
    outgoingDirection = outgoingVector_deg / outgoingLength_deg;
    angle_rad = acos(min(1, max(-1, ...
        dot(incomingDirection, outgoingDirection))));
    problem = struct( ...
        "DeflectionAngle_rad", angle_rad, ...
        "IncomingLength_deg", incomingLength_deg, ...
        "OutgoingLength_deg", outgoingLength_deg, ...
        "IncomingDirection", incomingDirection, ...
        "OutgoingDirection", outgoingDirection, ...
        "TurnRadius_deg", options.RpTurnRadius_deg, ...
        "MaxVelocity_deg_s", limits.maxVelocity_deg_s, ...
        "MaxAcceleration_deg_s2", limits.maxAcceleration_deg_s2, ...
        "MaxJerk_deg_s3", limits.maxJerk_deg_s3);
    observation = azElInternal.rpRetimerObservation(problem);
    action = getAction(agent, {observation});
    if iscell(action)
        action = action{1};
    end
    validateattributes(action, {'numeric'}, ...
        {'real', 'finite', 'scalar'});
    normalizedAction = 0.5 * ...
        (min(1, max(-1, double(action))) + 1);
    rawRadiusScale(cornerIndex) = normalizedAction;
    % Obstacle clearance is enforced after inference. A near-zero proposal
    % gives HS-3 a sharp corner and removes the smooth warm-start benefit.
    % Keep the learned ordering inside the smoother half of the safe search
    % range. The later collision projection can still reduce it to zero.
    minimumHs3RadiusFraction = 0.5;
    radiusScale(cornerIndex) = minimumHs3RadiusFraction + ...
        (1 - minimumHs3RadiusFraction) * normalizedAction;

    halfAngleCosine = cos(angle_rad / 2);
    turnCross = incomingDirection(1) * outgoingDirection(2) - ...
        incomingDirection(2) * outgoingDirection(1);
    isReversal = pi - angle_rad <= 1e-6;
    if angle_rad <= 1e-9 || isReversal || abs(turnCross) <= 1e-12
        cornerStatus(cornerIndex) = "agentEvaluatedSharpSeedRetained";
        continue;
    end
    tangentScale = (384 / 125) * sin(angle_rad / 2) / ...
        halfAngleCosine ^ 2;
    geometricMaximumRadius_deg = 0.45 * min( ...
        incomingLength_deg, outgoingLength_deg) / tangentScale;
    radius_deg = radiusScale(cornerIndex) * min( ...
        options.RpTurnRadius_deg, geometricMaximumRadius_deg);
    if radius_deg <= 1e-10
        cornerStatus(cornerIndex) = "agentSelectedSharpSeed";
        continue;
    end
    trim_deg = radius_deg * tangentScale;
    controlPoints_deg = [ ...
        corner_deg - trim_deg * incomingDirection; ...
        corner_deg - 0.5 * trim_deg * incomingDirection; ...
        corner_deg; corner_deg; ...
        corner_deg + 0.5 * trim_deg * outgoingDirection; ...
        corner_deg + trim_deg * outgoingDirection];
    entryPosition_deg(cornerIndex, :) = controlPoints_deg(1, :);
    exitPosition_deg(cornerIndex, :) = controlPoints_deg(end, :);
    controlPointsByCorner_deg{cornerIndex} = controlPoints_deg;
    cornerStatus(cornerIndex) = "agentSelectedG3Seed";
end

routeParts = cell(2 * cornerCount + 1, 1);
routeParts{1} = route_deg(1, :);
for cornerIndex = 1:cornerCount
    routeParts{2 * cornerIndex} = entryPosition_deg(cornerIndex, :);
    controlPoints_deg = controlPointsByCorner_deg{cornerIndex};
    if isempty(controlPoints_deg)
        routeParts{2 * cornerIndex + 1} = ...
            exitPosition_deg(cornerIndex, :);
        continue;
    end
    % Entry, midpoint, and exit preserve the agent's turn decision without
    % displacing the topology knots used by the HS-3 mesh.
    sampleCount = 3;
    parameter = linspace(0, 1, sampleCount).';
    curve_deg = evaluateBernstein(controlPoints_deg, parameter);
    routeParts{2 * cornerIndex + 1} = curve_deg(2:end, :);
end
routeParts{end} = [routeParts{end}; route_deg(end, :)];
roundedRoute_deg = vertcat(routeParts{:});
roundedStep_deg = diff(roundedRoute_deg, 1, 1);
keepRow = [true; hypot(roundedStep_deg(:, 1), ...
    roundedStep_deg(:, 2)) > 1e-10];
roundedRoute_deg = roundedRoute_deg(keepRow, :);
end

function [timeFraction, routeProgress, duration_s] = ...
        timedSeedLaw(route_deg, routeTime_s)
% PURPOSE
%   - Preserve the original SIPP-IP timing law before spatial rounding.
timeFraction = zeros(0, 1);
routeProgress = zeros(0, 1);
duration_s = NaN;
hasTimedRoute = numel(routeTime_s) == size(route_deg, 1) && ...
    numel(routeTime_s) >= 2 && all(isfinite(routeTime_s)) && ...
    all(diff(routeTime_s) > 0);
if ~hasTimedRoute
    return;
end
step_deg = diff(route_deg, 1, 1);
arcLength_deg = [0; cumsum(hypot(step_deg(:, 1), step_deg(:, 2)))];
if arcLength_deg(end) <= eps
    return;
end
duration_s = routeTime_s(end) - routeTime_s(1);
timeFraction = (routeTime_s - routeTime_s(1)) / duration_s;
routeProgress = arcLength_deg / arcLength_deg(end);
end

function value = evaluateBernstein(control, parameter)
% PURPOSE
%   - Evaluate one vector-valued Bernstein polynomial.
degree = size(control, 1) - 1;
basis = zeros(numel(parameter), degree + 1);
oneMinusParameter = 1 - parameter;
for controlIndex = 0:degree
    basis(:, controlIndex + 1) = nchoosek(degree, controlIndex) .* ...
        oneMinusParameter .^ (degree - controlIndex) .* ...
        parameter .^ controlIndex;
end
value = basis * control;
end

function length_deg = routeLength(route_deg)
% PURPOSE
%   - Return the polyline length of one seed.
if size(route_deg, 1) < 2
    length_deg = 0;
    return;
end
step_deg = diff(route_deg, 1, 1);
length_deg = sum(hypot(step_deg(:, 1), step_deg(:, 2)));
end

function seed = addRpFields(seed)
% PURPOSE
%   - Add the stable RP warm-start fields to one topology record.
seed.RpSeedVariant = "notBuilt";
seed.RpSeedRouteTimeFraction = zeros(0, 1);
seed.RpSeedRouteProgress = zeros(0, 1);
seed.RpSeedRouteDuration_s = NaN;
end

function reduction = reductionRecord(generatedCount, retainedCount)
% PURPOSE
%   - Report every warm start removed by the public seed cap.
reduction = struct( ...
    "GeneratedWarmStartCount", generatedCount, ...
    "RetainedWarmStartCount", retainedCount, ...
    "SeedCapDroppedWarmStartCount", generatedCount - retainedCount);
end

function record = seedDiagnosticTemplate()
% PURPOSE
%   - Define the stable RL seed diagnostic schema.
record = struct( ...
    "SeedIndex", 0, ...
    "OriginalSeedIndex", 0, ...
    "Source", "", ...
    "SeedVariant", "notBuilt", ...
    "PolicyStatus", "notRun", ...
    "AgentLoaded", false, ...
    "AgentRouteApplied", false, ...
    "OriginalRoute_deg", zeros(0, 2), ...
    "RlSeedRoute_deg", zeros(0, 2), ...
    "OriginalRouteLength_deg", NaN, ...
    "RlSeedRouteLength_deg", NaN, ...
    "CornerCount", 0, ...
    "EvaluatedCornerCount", 0, ...
    "RawRadiusScale", zeros(0, 1), ...
    "CalibratedRadiusScale", zeros(0, 1), ...
    "ClearanceProjectionScale", NaN, ...
    "RadiusScale", zeros(0, 1), ...
    "CornerStatus", strings(0, 1), ...
    "TimedSeedLawPreserved", false);
end
