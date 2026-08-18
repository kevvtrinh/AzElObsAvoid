function [seeds, diagnostics, reduction] = buildAzElRpHs3Seeds( ...
        seeds, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   [seeds, diagnostics, reduction] = ...
%       azElInternal.buildAzElRpHs3Seeds( ...
%       seeds, limits, options)
%**************************************************************************
% PURPOSE
%   - Use the required RP agent to add a rounded warm start for every
%     spatial topology seed.
%   - Preserve a timed seed law separately for HS-3 initialization.
%**************************************************************************
% INPUTS
%   - seeds (N-by-1 structure array)
%       Planner seeds with Route_deg, RouteTime_s, Source, and route cost.
%   - limits (scalar struct)
%       Finite two-axis velocity, acceleration, and jerk limits.
%   - options (scalar struct)
%       RpAgentFile and RpTurnRadius_deg.
%**************************************************************************
% OUTPUTS
%   - seeds (M-by-1 structure array)
%       Interleaved topology and RL-rounded warm starts. Each record has
%       preserved time-law fields used by the HS-3 optimizer.
%   - diagnostics (N-by-1 structure array)
%       Policy status, corner actions, and before/after route metrics.
%   - reduction (scalar struct)
%       Generated, retained, and cap-dropped HS-3 warm-start counts.
%**************************************************************************
% UNITS
%   - Position and route length use degrees. Time uses seconds.
%**************************************************************************

%% Section 1: Validate Inputs And Load The Required Agent

requiredSeedFields = ["Source" "Route_deg" "RouteTime_s" ...
    "RouteCost_deg"];
if ~isstruct(seeds) || ...
        (~isempty(seeds) && ~all(isfield(seeds, requiredSeedFields)))
    error("buildAzElRpHs3Seeds:InvalidSeeds", ...
        "seeds must be a structure array with the required route fields.");
end
if ~isstruct(limits) || ~isscalar(limits) || ~all(isfield(limits, ...
        ["maxVelocity_deg_s" "maxAcceleration_deg_s2" ...
        "maxJerk_deg_s3"]))
    error("buildAzElRpHs3Seeds:InvalidLimits", ...
        "limits must contain finite velocity, acceleration, and jerk limits.");
end
if ~isstruct(options) || ~isscalar(options) || ~all(isfield(options, ...
        ["RpAgentFile" "RpTurnRadius_deg" ...
        "MaximumDirectCollocationSeeds"]))
    error("buildAzElRpHs3Seeds:InvalidOptions", ...
        "options must contain the RP seed controls.");
end
agent = loadRequiredAgent(options.RpAgentFile);

%% Section 2: Evaluate The Agent And Add Each Rounded Warm Start

inputSeeds = seeds;
if isempty(inputSeeds)
    diagnostics = repmat(seedDiagnosticTemplate(), 0, 1);
    reduction = reductionRecord(0, 0);
    return;
end
seedTemplate = addRpFields(inputSeeds(1));
topologySeeds = repmat(seedTemplate, 0, 1);
topologyDiagnostics = repmat(seedDiagnosticTemplate(), 0, 1);
roundedSeeds = repmat(seedTemplate, 0, 1);
roundedDiagnostics = repmat(seedDiagnosticTemplate(), 0, 1);
for inputSeedIndex = 1:numel(inputSeeds)
    originalRoute_deg = double(inputSeeds(inputSeedIndex).Route_deg);
    originalRouteTime_s = double( ...
        inputSeeds(inputSeedIndex).RouteTime_s(:));
    [timeFraction, routeProgress, routeDuration_s] = timedSeedLaw( ...
        originalRoute_deg, originalRouteTime_s);
    [rlRoute_deg, radiusScale, cornerStatus] = roundRouteWithAgent( ...
        originalRoute_deg, agent, limits, options);

    topologySeed = addRpFields(inputSeeds(inputSeedIndex));
    topologySeed.RpSeedVariant = "topology";
    topologySeed.RpSeedRouteTimeFraction = timeFraction;
    topologySeed.RpSeedRouteProgress = routeProgress;
    topologySeed.RpSeedRouteDuration_s = routeDuration_s;

    commonRecord = seedDiagnosticTemplate();
    commonRecord.OriginalSeedIndex = inputSeedIndex;
    commonRecord.Source = string(topologySeed.Source);
    commonRecord.PolicyStatus = "proposalEvaluated";
    if isempty(radiusScale)
        commonRecord.PolicyStatus = "loadedNoCorner";
    end
    commonRecord.AgentLoaded = true;
    commonRecord.OriginalRoute_deg = originalRoute_deg;
    commonRecord.RlSeedRoute_deg = rlRoute_deg;
    commonRecord.OriginalRouteLength_deg = routeLength(originalRoute_deg);
    commonRecord.RlSeedRouteLength_deg = routeLength(rlRoute_deg);
    commonRecord.CornerCount = numel(radiusScale);
    commonRecord.EvaluatedCornerCount = numel(radiusScale);
    commonRecord.RadiusScale = radiusScale;
    commonRecord.CornerStatus = cornerStatus;
    commonRecord.TimedSeedLawPreserved = ~isempty(timeFraction);

    topologyRecord = commonRecord;
    topologyRecord.SeedVariant = "topology";
    topologySeeds(end + 1, 1) = topologySeed; %#ok<AGROW>
    topologyDiagnostics(end + 1, 1) = topologyRecord; %#ok<AGROW>

    routeChanged = ~isequal(size(originalRoute_deg), size(rlRoute_deg)) || ...
        max(abs(originalRoute_deg - rlRoute_deg), [], "all") > 1e-12;
    if isempty(radiusScale) || ~routeChanged
        continue;
    end
    roundedSeed = topologySeed;
    roundedSeed.Route_deg = rlRoute_deg;
    roundedSeed.RouteCost_deg = routeLength(rlRoute_deg);
    roundedSeed.RouteTime_s = zeros(0, 1);
    roundedSeed.RpSeedVariant = "agentRounded";
    roundedRecord = commonRecord;
    roundedRecord.SeedVariant = "agentRounded";
    roundedRecord.AgentRouteApplied = true;
    roundedSeeds(end + 1, 1) = roundedSeed; %#ok<AGROW>
    roundedDiagnostics(end + 1, 1) = roundedRecord; %#ok<AGROW>
end

% Preserve topology diversity under a small seed cap. Agent variants follow
% their complete topology set and do not displace a distinct route family.
generatedSeeds = [topologySeeds; roundedSeeds];
generatedDiagnostics = [topologyDiagnostics; roundedDiagnostics];
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

function agent = loadRequiredAgent(agentFile)
% PURPOSE
%   - Load and cache one validated deterministic inference policy.
persistent cachedAgent cachedAgentFile
agentFile = string(agentFile);
if ~isfile(agentFile)
    error("buildAzElRpHs3Seeds:AgentNotFound", ...
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
        error("buildAzElRpHs3Seeds:InvalidAgent", ...
            "RpAgentFile must contain an AzElRpRetimerAgent model.");
    end
    cachedAgent = agentData.agent;
    cachedAgent.UseExplorationPolicy = false;
    cachedAgentFile = agentFile;
end
agent = cachedAgent;
end

function [roundedRoute_deg, radiusScale, cornerStatus] = ...
        roundRouteWithAgent(route_deg, agent, limits, options)
% PURPOSE
%   - Convert one polyline to an RL-rounded G3 topology seed.
validateattributes(route_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2, 'nonempty'});
routeStep_deg = diff(route_deg, 1, 1);
route_deg = route_deg([true; hypot( ...
    routeStep_deg(:, 1), routeStep_deg(:, 2)) > 1e-10], :);
if size(route_deg, 1) < 2
    error("buildAzElRpHs3Seeds:ZeroLengthRoute", ...
        "Each seed route must contain two distinct positions.");
end

cornerCount = max(0, size(route_deg, 1) - 2);
radiusScale = zeros(cornerCount, 1);
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
    radiusScale(cornerIndex) = 0.5 * ...
        (min(1, max(-1, double(action))) + 1);

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
    % displacing the important topology knots under the HS-3 mesh cap.
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
    "RadiusScale", zeros(0, 1), ...
    "CornerStatus", strings(0, 1), ...
    "TimedSeedLawPreserved", false);
end
