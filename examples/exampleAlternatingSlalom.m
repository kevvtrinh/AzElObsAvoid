function result = exampleAlternatingSlalom(slalomCount, options)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleAlternatingSlalom()
%   result = exampleAlternatingSlalom(slalomCount)
%   result = exampleAlternatingSlalom(options)
%   result = exampleAlternatingSlalom(slalomCount, options)
%**************************************************************************
% PURPOSE
%   - Construct 1 to 10 protected alternating baffles and run the automatic
%     planner without waypoints, a preferred side, or a directed route.
%**************************************************************************
% INPUTS
%   - slalomCount (integer scalar from 1 through 10, optional; default 6)
%   - options (scalar struct, optional)
%       Planner option overrides.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and slalom passage diagnostics.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.

if nargin < 1 || isempty(slalomCount)
    slalomCount = 6;
    options = struct();
elseif isstruct(slalomCount)
    options = slalomCount;
    slalomCount = 6;
elseif nargin < 2 || isempty(options)
    options = struct();
end
validateattributes(slalomCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', '>=', 1, '<=', 10});
options = exampleOptions(options, struct( ...
    "MotionType", "velocityCarrying", ...
    "TurnRadius_deg", 0.50, ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", sprintf("Alternating slalom with %d baffles", slalomCount)));

%% Section 1: Construct Canonical Obstacles
missionEndTime_s = 120;
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

%% Section 2: Define The Planning Request
initialState = struct("time_s", 0, "position_deg", [0 0]);
goalState = struct( ...
    "time_s", missionEndTime_s, ...
    "position_deg", [goalAzimuth_deg, 0]);
limits = struct( ...
    "maxVelocity_deg_s", [2 2], ...
    "maxAcceleration_deg_s2", [0.75 0.75]);

%% Section 3: Run The Maintained Planner
result = planAzElMotion( ...
    obstacles, initialState, goalState, limits, options);
if ~result.Success || ~result.Validation.Passed
    error("exampleAlternatingSlalom:PlanningFailed", ...
        "Slalom validation failed. Diagnostic plots remain open. %s", ...
        result.Message);
end
result.slalomCount = slalomCount;
result.baffleBoundaries_deg = baffleBoundaries_deg;
result.gateSpacing_deg = gateSpacing_deg;
gateAzimuth_deg = gateSpacing_deg * (1:slalomCount).';
passageElevation_deg = zeros(slalomCount, 1);
for baffleIndex = 1:slalomCount
    [~, nearestSampleIndex] = min(abs( ...
        result.timedSlopePath.position_deg(:, 1) - ...
        gateAzimuth_deg(baffleIndex)));
    passageElevation_deg(baffleIndex) = ...
        result.timedSlopePath.position_deg(nearestSampleIndex, 2);
end
result.passageElevation_deg = passageElevation_deg;

%% Section 4: Validate The Command
expectedPassageSign = 2 * mod((1:slalomCount).', 2) - 1;
if ~any(result.directBlocked) || any( ...
        expectedPassageSign .* passageElevation_deg <= ...
        gateTipElevation_deg)
    error("exampleAlternatingSlalom:ScenarioValidationFailed", ...
        "The route did not alternate through every gate. " + ...
        "Diagnostic plots remain open.");
end
end

function options = exampleOptions(options, defaults)
%% Section 0: Header & Readme
names = fieldnames(defaults);
for index = 1:numel(names)
    if ~isfield(options, names{index}) || isempty(options.(names{index}))
        options.(names{index}) = defaults.(names{index});
    end
end
end
