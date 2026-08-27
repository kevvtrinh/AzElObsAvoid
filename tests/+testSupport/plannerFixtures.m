function fixtures = plannerFixtures()
%% Section 0: Header & Readme
% SYNTAX
%   fixtures = testSupport.plannerFixtures()
%**************************************************************************
% PURPOSE
%   - Provide one shared set of deterministic planner-test fixture builders.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - fixtures (scalar struct of function handles)
%       Builders for states, limits, obstacles, and trajectory records.
%**************************************************************************
% UNITS
%   - Fixture fields retain their documented degree and second suffixes.
%**************************************************************************

%% Section 1: Build Fixture Interface

% Expose small deterministic constructors to all planner tests. Keep units in
% field names. A fixture change affects many tests, so change it only when the
% shared meaning of the test input changes.

fixtures = struct( ...
    "State", @state, ...
    "PhysicalLimits", @physicalLimits, ...
    "RectangleObstacle", @rectangleObstacle, ...
    "ConstantJerkTrajectory", @constantJerkTrajectory, ...
    "LinearTrajectory", @linearTrajectory, ...
    "InteriorVelocityPeakTrajectory", @interiorVelocityPeakTrajectory, ...
    "TrajectoryRecord", @trajectoryRecord);
end

%% Section 2: Local Functions

function value = state(time_s, position_deg, velocity_deg_s, acceleration_deg_s2)
% Construct one complete endpoint state.
value = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, "acceleration_deg_s2", acceleration_deg_s2);
end

function limits = physicalLimits(velocity_deg_s, acceleration_deg_s2, jerk_deg_s3)
% Construct one complete physical-limit record.
limits = struct( ...
    "maxVelocity_deg_s", velocity_deg_s, ...
    "maxAcceleration_deg_s2", acceleration_deg_s2, ...
    "maxJerk_deg_s3", jerk_deg_s3, ...
    "azimuthInterval_deg", [-180 180], ...
    "elevationInterval_deg", [-90 90]);
end

function obstacle = rectangleObstacle(time_s, bounds_deg, margin_deg)
% Construct a static rectangle from [minAz maxAz minEl maxEl].
azimuth_deg = bounds_deg([1 2 2 1]).';
elevation_deg = bounds_deg([3 3 4 4]).';
obstacle = obstacleAvoidance.obstacles.createObstacle( "rectangle", time_s(:), azimuth_deg, elevation_deg, margin_deg);
end

function trajectory = constantJerkTrajectory(duration_s)
% Build analytic q=t^3/6, v=t^2/2, a=t, and j=1 on the first axis.
positionPower_deg = zeros(1, 2, 6);
velocityPower_deg_s = zeros(1, 2, 5);
accelerationPower_deg_s2 = zeros(1, 2, 4);
jerkPower_deg_s3 = zeros(1, 2, 3);
positionPower_deg(1, 1, 4) = duration_s^3 / 6;
velocityPower_deg_s(1, 1, 3) = duration_s^2 / 2;
accelerationPower_deg_s2(1, 1, 2) = duration_s;
jerkPower_deg_s3(1, 1, 1) = 1;
time_s = [0; duration_s];
position_deg = [0 0; duration_s^3 / 6 0];
velocity_deg_s = [0 0; duration_s^2 / 2 0];
acceleration_deg_s2 = [0 0; duration_s 0];
jerk_deg_s3 = [1 0; 1 0];
trajectory = trajectoryRecord( ...
    time_s, position_deg, velocity_deg_s, acceleration_deg_s2, ...
    jerk_deg_s3, positionPower_deg, velocityPower_deg_s, ...
    accelerationPower_deg_s2, jerkPower_deg_s3, duration_s);
end

function trajectory = linearTrajectory(initialState, goalState)
% Build one exact constant-velocity segment sampled only at its endpoints.
duration_s = goalState.time_s - initialState.time_s;
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, :, 1) = initialState.position_deg;
positionPower_deg(1, :, 2) = goalState.position_deg - initialState.position_deg;
velocityPower_deg_s = zeros(1, 2, 5);
velocityPower_deg_s(1, :, 1) = initialState.velocity_deg_s;
accelerationPower_deg_s2 = zeros(1, 2, 4);
jerkPower_deg_s3 = zeros(1, 2, 3);
trajectory = trajectoryRecord( ...
    [initialState.time_s; goalState.time_s], ...
    [initialState.position_deg; goalState.position_deg], ...
    [initialState.velocity_deg_s; goalState.velocity_deg_s], ...
    zeros(2, 2), zeros(2, 2), positionPower_deg, ...
    velocityPower_deg_s, accelerationPower_deg_s2, jerkPower_deg_s3, duration_s);
end

function trajectory = interiorVelocityPeakTrajectory()
% Build v=4s(1-s), which peaks between clear endpoint samples.
duration_s = 1;
positionPower_deg = zeros(1, 2, 6);
positionPower_deg(1, 1, 3) = 2;
positionPower_deg(1, 1, 4) = -4 / 3;
velocityPower_deg_s = zeros(1, 2, 5);
velocityPower_deg_s(1, 1, 2) = 4;
velocityPower_deg_s(1, 1, 3) = -4;
accelerationPower_deg_s2 = zeros(1, 2, 4);
accelerationPower_deg_s2(1, 1, 1) = 4;
accelerationPower_deg_s2(1, 1, 2) = -8;
jerkPower_deg_s3 = zeros(1, 2, 3);
jerkPower_deg_s3(1, 1, 1) = -8;
trajectory = trajectoryRecord( ...
    [0; 1], [0 0; 2 / 3 0], zeros(2, 2), ...
    [4 0; -4 0], [-8 0; -8 0], positionPower_deg, ...
    velocityPower_deg_s, accelerationPower_deg_s2, jerkPower_deg_s3, duration_s);
end

function trajectory = trajectoryRecord(time_s, position_deg, ...
        velocity_deg_s, acceleration_deg_s2, jerk_deg_s3, ...
        positionPower_deg, velocityPower_deg_s, ...
        accelerationPower_deg_s2, jerkPower_deg_s3, duration_s)
% Assemble one manual trajectory with the required polynomial fields.
polynomial = struct( ...
    "SegmentCount", 1, "SegmentStartTime_s", time_s(1), ...
    "SegmentDuration_s", duration_s, "FinalTime_s", time_s(end), ...
    "positionPower_deg", positionPower_deg, ...
    "velocityPower_deg_s", velocityPower_deg_s, ...
    "accelerationPower_deg_s2", accelerationPower_deg_s2, ...
    "jerkPower_deg_s3", jerkPower_deg_s3, "TerminalState", struct());
trajectory = struct( ...
    "time_s", time_s, "position_deg", position_deg, ...
    "velocity_deg_s", velocity_deg_s, ...
    "acceleration_deg_s2", acceleration_deg_s2, ...
    "jerk_deg_s3", jerk_deg_s3, ...
    "Polynomial", polynomial);
end
