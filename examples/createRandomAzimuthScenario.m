function scenario = createRandomAzimuthScenario(caseIndex, includeStaticObstacle, randomSeed)
%% Section 0: Header & Readme
% SYNTAX
%   scenario = createRandomAzimuthScenario()
%   scenario = createRandomAzimuthScenario(caseIndex)
%   scenario = createRandomAzimuthScenario(caseIndex, includeStaticObstacle)
%   scenario = createRandomAzimuthScenario( ...
%       caseIndex, includeStaticObstacle, randomSeed)
%**************************************************************************
% PURPOSE
%   - Reproduce an angular slew crossing a translating rectangle, optionally
%     with an additional static rectangle. Every slew spans at least 80 deg
%     in azimuth.
%**************************************************************************
% INPUTS
%   - caseIndex (positive integer scalar, optional; default 1)
%       Selects one independent case block of the reproducible draw.
%   - includeStaticObstacle (logical scalar, optional; default false)
%       Adds the paired static rectangle to the obstacle set.
%   - randomSeed (integer scalar in [0, 2^32-1], optional; default 831907)
%       Seeds the local random stream.
%**************************************************************************
% OUTPUTS
%   - scenario (scalar struct)
%       Planner arguments plus source geometry metadata. Paired cases differ
%       only by the presence of the static obstacle. Randomness uses a local
%       stream and does not change the caller's global random state. Invalid
%       input throws an error.
%**************************************************************************
% UNITS
%   - Coordinates are azimuth/elevation in degrees. Time is seconds and
%     velocity uses deg/s.
%**************************************************************************

%% Section 1: Validate And Draw Reproducible Physical Inputs
if nargin < 1
    caseIndex = 1;
end
if nargin < 2
    includeStaticObstacle = false;
end
if nargin < 3
    randomSeed = 831907;
end
validateattributes(caseIndex, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
validateattributes(includeStaticObstacle, {'logical'}, {'scalar'});
validateattributes(randomSeed, {'numeric'}, {'scalar', 'integer', '>=', 0, '<=', 2^32 - 1});
stream = RandStream('mt19937ar', 'Seed', randomSeed);

% Generate by column-independent case blocks so requesting a larger index does
% not change the preceding cases or the paired obstacle toggle.
draws = reshape(rand(stream, 16 * caseIndex, 1), 16, caseIndex).';
randomDraw = draws(end, :);

azimuthSpan_deg        = 80 + 60 * randomDraw(1);
azimuthDirection       = 2 * (randomDraw(2) >= 0.5) - 1;
center_deg             = [-25 + 50 * randomDraw(3), -12 + 24 * randomDraw(4)];
displacement_deg       = [azimuthDirection * azimuthSpan_deg, -65 + 130 * randomDraw(5)];
start_deg              = center_deg - displacement_deg / 2;
goal_deg               = center_deg + displacement_deg / 2;
duration_s             = 180;
alongDirection         = displacement_deg / norm(displacement_deg);
perpendicularDirection = [-alongDirection(2), alongDirection(1)];

%% Section 2: Build The Translating And Static Rectangles
movingFraction     = 0.38 + 0.24 * randomDraw(6);
movingCenter_deg   = start_deg + movingFraction * displacement_deg;
halfSize_deg       = [4 + 9 * randomDraw(7), 7 + 16 * randomDraw(8)];
angle_rad          = pi * randomDraw(9);
rotationMatrix     = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
movingVertices_deg = ([-1, -1; 1, -1; 1, 1; -1, 1] .* halfSize_deg) * rotationMatrix.';
velocity_deg_s     = (2 * (randomDraw(10) >= 0.5) - 1) * ...
    (0.035 + 0.14 * randomDraw(11)) * perpendicularDirection + ...
    (-0.04 + 0.08 * randomDraw(12)) * alongDirection;

% Constant translation is represented exactly by corresponding vertices.
time_s = [0; duration_s];
x_deg  = cell(2, 1);
y_deg  = cell(2, 1);
for sampleIndex = 1:2
    boundary_deg = movingVertices_deg + movingCenter_deg + time_s(sampleIndex) * velocity_deg_s;
    x_deg{sampleIndex} = boundary_deg(:, 1);
    y_deg{sampleIndex} = boundary_deg(:, 2);
end
movingObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "Translating rectangle", time_s, x_deg, y_deg, 0.2);

staticFraction = 0.18 + 0.10 * randomDraw(13);
if randomDraw(14) > 0.5
    staticFraction = 1 - staticFraction;
end
staticCenter_deg = start_deg + staticFraction * displacement_deg + ...
    (-8 + 16 * randomDraw(15)) * perpendicularDirection;
staticVertices_deg = staticCenter_deg + ...
    [-1, -1; 1, -1; 1, 1; -1, 1] .* ...
    [3 + 4 * randomDraw(16), 6 + 4 * randomDraw(13)];
staticObstacle     = obstacleAvoidance.obstacles.createObstacle( ...
    "Static rectangle", time_s, staticVertices_deg(:, 1), staticVertices_deg(:, 2), 0.2);

obstacles = movingObstacle;
if includeStaticObstacle
    obstacles = obstacleAvoidance.obstacles.combineObstacles({movingObstacle; staticObstacle});
end

%% Section 3: Return Public Planner Inputs And Reproduction Metadata
limits = struct( ...
    'xInterval_units',          [-180, 180], ...
    'yInterval_units',          [-90, 90], ...
    'maxVelocity_units_s',      [2, 2], ...
    'maxAcceleration_units_s2', [0.75, 0.75], ...
    'maxJerk_units_s3',         [2, 2]);
scenario = struct( ...
    'Obstacles',               obstacles, ...
    'InitialState',            struct('time_s', 0, 'position_units', start_deg), ...
    'GoalState',               struct('time_s', duration_s, 'position_units', goal_deg), ...
    'Limits',                  limits, ...
    'Options',                 struct('GoalTimeMode', 'fixedArrival', 'SampleTime_s', 0.1), ...
    'CaseIndex',               caseIndex, ...
    'IncludeStaticObstacle',   includeStaticObstacle, ...
    'RandomSeed',              randomSeed, ...
    'AzimuthSpan_deg',         azimuthSpan_deg, ...
    'EndpointDistance_deg',    norm(displacement_deg), ...
    'MovingCenterAtStart_deg', movingCenter_deg, ...
    'MovingVelocity_deg_s',    velocity_deg_s, ...
    'StaticVertices_deg',      staticVertices_deg);
end
