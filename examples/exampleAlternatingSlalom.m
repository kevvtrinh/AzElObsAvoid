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
%       Planner option overrides plus EnableJerkConstraint and
%       MaxJerk_deg_s3 example controls.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Validated planner result and slalom passage diagnostics.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls
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
[options, jerkConfiguration] = resolveAzElExampleOptions( ...
    options, struct( ...
    "MotionType", "velocityCarrying", ...
    "TurnRadius_deg", 0.50, ...
    "Verbose", true, ...
    "FigureVisible", "on", ...
    "Title", sprintf( ...
    "Alternating slalom with %d baffles", slalomCount)), [2.5 2.5]);

%% Section 2: Create Obstacles
% Use one conservative horizon for both retimers so jerk is the only paired
% benchmark input that changes.
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
if exampleValidation.HasTrajectory
    for baffleIndex = 1:slalomCount
        [~, nearestSampleIndex] = min(abs( ...
            result.timedSlopePath.position_deg(:, 1) - ...
            gateAzimuth_deg(baffleIndex)));
        passageElevation_deg(baffleIndex) = ...
            result.timedSlopePath.position_deg(nearestSampleIndex, 2);
    end
end
expectedPassageSign = 2 * mod((1:slalomCount).', 2) - 1;
alternatingPassageSatisfied = all( ...
    expectedPassageSign .* passageElevation_deg > gateTipElevation_deg);
exampleValidation.AlternatingPassageSatisfied = ...
    alternatingPassageSatisfied;
exampleValidation.Passed = exampleValidation.Passed && ...
    alternatingPassageSatisfied;

%% Section 6: Plot Diagnostics And Motion
% planAzElMotion created all requested plots from the returned result.

%% Section 7: Return Example Metadata
result.ExampleValidation = exampleValidation;
result.slalomCount = slalomCount;
result.baffleBoundaries_deg = baffleBoundaries_deg;
result.gateSpacing_deg = gateSpacing_deg;
result.passageElevation_deg = passageElevation_deg;
result.ExampleConfiguration = jerkConfiguration;
end
