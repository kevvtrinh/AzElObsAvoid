function result = exampleAlternatingSlalom(exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleAlternatingSlalom()
%   result = exampleAlternatingSlalom(exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Construct 1 to 10 protected alternating baffles and run the automatic
%     planner without waypoints, a preferred side, or a directed route.
%**************************************************************************
% INPUTS
%   - exampleOverrides (scalar struct, optional; default struct())
%       Planner/display overrides plus SlalomCount (default 6) and the
%       finite MaxJerk_deg_s3 limit.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and slalom passage diagnostics.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1 || isempty(exampleOverrides)
    exampleOverrides = struct();
end
if ~isstruct(exampleOverrides) || ~isscalar(exampleOverrides)
    error("exampleAlternatingSlalom:InvalidOverrides", ...
        "exampleOverrides must be a scalar struct.");
end
slalomCount = 6;
if isfield(exampleOverrides, "SlalomCount") && ...
        ~isempty(exampleOverrides.SlalomCount)
    slalomCount = exampleOverrides.SlalomCount;
    exampleOverrides = rmfield(exampleOverrides, "SlalomCount");
end
validateattributes(slalomCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 1, '<=', 10});
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    exampleOverrides, struct( ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", sprintf( ...
    "Alternating slalom with %d baffles", slalomCount)), [2.5 2.5]);

%% Section 2: Create Obstacles

% Use one conservative horizon for all direct-collocation seed solves.
missionEndTime_s = 120 + 60 * slalomCount;
safetyMargin_deg = 0.25;
time_s = [0; missionEndTime_s];
gateSpacing_deg = 5;
gateHalfWidth_deg = 0.6;
corridorHalfHeight_deg = 6;
wallThickness_deg = 2;
gateTipElevation_deg = 1.5;
goalAzimuth_deg = gateSpacing_deg * (slalomCount + 1);
wallAzimuth_deg = [-5; goalAzimuth_deg + 5; ...
    goalAzimuth_deg + 5; -5];
floorElevation_deg = [ ...
    -corridorHalfHeight_deg - wallThickness_deg; ...
    -corridorHalfHeight_deg - wallThickness_deg; ...
    -corridorHalfHeight_deg; -corridorHalfHeight_deg];
ceilingElevation_deg = [ ...
    corridorHalfHeight_deg; corridorHalfHeight_deg; ...
    corridorHalfHeight_deg + wallThickness_deg; ...
    corridorHalfHeight_deg + wallThickness_deg];
obstacles = repmat(makeAzElObstacleData( ...
    "Slalom floor", time_s, wallAzimuth_deg, ...
    floorElevation_deg, safetyMargin_deg), slalomCount + 2, 1);
obstacles(2) = makeAzElObstacleData( ...
    "Slalom ceiling", time_s, wallAzimuth_deg, ceilingElevation_deg, ...
    safetyMargin_deg);
baffleBoundaries_deg = cell(slalomCount, 1);
for baffleIndex = 1:slalomCount
    centerAzimuth_deg = gateSpacing_deg * baffleIndex;
    if mod(baffleIndex, 2) == 1
        elevationBounds_deg = [-corridorHalfHeight_deg, ...
            gateTipElevation_deg];
    else
        elevationBounds_deg = [-gateTipElevation_deg, ...
            corridorHalfHeight_deg];
    end
    boundary_deg = [ ...
        centerAzimuth_deg - gateHalfWidth_deg, elevationBounds_deg(1); ...
        centerAzimuth_deg + gateHalfWidth_deg, elevationBounds_deg(1); ...
        centerAzimuth_deg + gateHalfWidth_deg, elevationBounds_deg(2); ...
        centerAzimuth_deg - gateHalfWidth_deg, elevationBounds_deg(2)];
    baffleBoundaries_deg{baffleIndex} = boundary_deg;
    obstacles(baffleIndex + 2) = makeAzElObstacleData( ...
        sprintf("Slalom baffle %d", baffleIndex), time_s, ...
        boundary_deg(:, 1), boundary_deg(:, 2), safetyMargin_deg);
end

%% Section 3: Create Planner Inputs

initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, ...
    "position_deg", [goalAzimuth_deg, 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75], ...
    "maxJerk_deg_s3", jerkConfiguration.MaxJerk_deg_s3);

%% Section 4: Run Planner

result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);

%% Section 5: Validate Result

exampleValidation = validateAzElExampleResult( ...
    result, "alternating slalom", ...
    struct("RequireDirectBlocked", true));
gateAzimuth_deg = gateSpacing_deg * (1:slalomCount).';
passageElevation_deg = nan(slalomCount, 1);
passageSpeed_deg_s = nan(slalomCount, 1);
passageCrossingFound = false(slalomCount, 1);
turnWindowMinimumSpeed_deg_s = nan(slalomCount, 1);
if exampleValidation.HasTrajectory
    trajectorySpeed_deg_s = vecnorm( ...
        result.timedSlopePath.velocity_deg_s, 2, 2);
    for baffleIndex = 1:slalomCount
        [passageElevation_deg(baffleIndex), ...
            passageSpeed_deg_s(baffleIndex), ...
            passageCrossingFound(baffleIndex)] = interpolateGateCrossing( ...
            result.timedSlopePath, gateAzimuth_deg(baffleIndex));
        turnWindowMask = abs( ...
            result.timedSlopePath.position_deg(:, 1) - ...
            gateAzimuth_deg(baffleIndex)) <= 0.5 * gateSpacing_deg;
        if any(turnWindowMask)
            turnWindowMinimumSpeed_deg_s(baffleIndex) = min( ...
                trajectorySpeed_deg_s(turnWindowMask));
        end
    end
end
expectedPassageSign = 2 * mod((1:slalomCount).', 2) - 1;
requiredProtectedElevation_deg = ...
    gateTipElevation_deg + safetyMargin_deg;
alternatingPassageSatisfied = all( ...
    passageCrossingFound & expectedPassageSign .* passageElevation_deg >= ...
    requiredProtectedElevation_deg - 1e-7);
minimumTurnSpeed_deg_s = 1e-3;
noMandatoryStopSatisfied = all( ...
    passageCrossingFound & passageSpeed_deg_s > minimumTurnSpeed_deg_s & ...
    turnWindowMinimumSpeed_deg_s > minimumTurnSpeed_deg_s);
exampleValidation.AlternatingPassageSatisfied = ...
    alternatingPassageSatisfied;
exampleValidation.PassageCrossingFound = passageCrossingFound;
exampleValidation.PassageSpeed_deg_s = passageSpeed_deg_s;
exampleValidation.TurnWindowMinimumSpeed_deg_s = ...
    turnWindowMinimumSpeed_deg_s;
exampleValidation.MinimumTurnSpeed_deg_s = minimumTurnSpeed_deg_s;
exampleValidation.NoMandatoryStopSatisfied = noMandatoryStopSatisfied;
exampleValidation.ContinuousVelocityThroughTurnsSatisfied = ...
    noMandatoryStopSatisfied;
exampleValidation.Passed = exampleValidation.Passed && ...
    alternatingPassageSatisfied && noMandatoryStopSatisfied;
if ~alternatingPassageSatisfied || ~noMandatoryStopSatisfied
    failedBaffleIndex = find(~passageCrossingFound | ...
        expectedPassageSign .* passageElevation_deg < ...
        requiredProtectedElevation_deg - 1e-7 | ...
        passageSpeed_deg_s <= minimumTurnSpeed_deg_s | ...
        turnWindowMinimumSpeed_deg_s <= minimumTurnSpeed_deg_s);
    slalomIssues = strings(0, 1);
    if ~alternatingPassageSatisfied
        slalomIssues(end + 1, 1) = "alternatingPassage";
    end
    if ~noMandatoryStopSatisfied
        slalomIssues(end + 1, 1) = "mandatoryStop";
    end
    exampleValidation.Issues = unique([ ...
        string(exampleValidation.Issues(:)); slalomIssues], "stable");
    exampleValidation.Message = string(exampleValidation.Message) + ...
        " Slalom gate checks failed at baffles " + ...
        strjoin(string(failedBaffleIndex.'), ", ") + ".";
end

%% Section 6: Plot Diagnostics And Motion

result.PlotHandles = struct();
if jerkConfiguration.PlotOutputs
    result.PlotHandles = plotAzElMotion( ...
        result, jerkConfiguration.PlotOptions);
end

%% Section 7: Return Example Metadata

result.ExampleValidation = exampleValidation;
result.slalomCount = slalomCount;
result.baffleBoundaries_deg = baffleBoundaries_deg;
result.gateSpacing_deg = gateSpacing_deg;
result.passageElevation_deg = passageElevation_deg;
result.passageSpeed_deg_s = passageSpeed_deg_s;
result.turnWindowMinimumSpeed_deg_s = turnWindowMinimumSpeed_deg_s;
result.passageCrossingFound = passageCrossingFound;
result.ExampleConfiguration = jerkConfiguration;
end

function [elevation_deg, speed_deg_s, crossingFound] = ...
        interpolateGateCrossing( ...
        timedSlopePath, gateAzimuth_deg)
% PURPOSE
%   - Interpolate elevation and speed at the first forward gate crossing.
azimuth_deg = timedSlopePath.position_deg(:, 1);
crossingIndex = find( ...
    azimuth_deg(1:end - 1) <= gateAzimuth_deg & ...
    azimuth_deg(2:end) >= gateAzimuth_deg, 1, "first");
elevation_deg = NaN;
speed_deg_s = NaN;
crossingFound = ~isempty(crossingIndex);
if isempty(crossingIndex)
    return;
end
azimuthSpan_deg = azimuth_deg(crossingIndex + 1) - ...
    azimuth_deg(crossingIndex);
fraction = 0;
if azimuthSpan_deg > 0
    fraction = (gateAzimuth_deg - azimuth_deg(crossingIndex)) / ...
        azimuthSpan_deg;
end
position_deg = (1 - fraction) * ...
    timedSlopePath.position_deg(crossingIndex, :) + fraction * ...
    timedSlopePath.position_deg(crossingIndex + 1, :);
velocity_deg_s = (1 - fraction) * ...
    timedSlopePath.velocity_deg_s(crossingIndex, :) + fraction * ...
    timedSlopePath.velocity_deg_s(crossingIndex + 1, :);
elevation_deg = position_deg(2);
speed_deg_s = norm(velocity_deg_s);
end
