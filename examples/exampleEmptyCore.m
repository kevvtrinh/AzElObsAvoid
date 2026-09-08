function result = exampleEmptyCore(showPlot)
%% Section 0: Header & Readme
% SYNTAX
%   result = exampleEmptyCore()
%   result = exampleEmptyCore(showPlot)
%
% PURPOSE
%   - Run the complete empty-core flow: obstacle preparation, exhaustive
%     visibility graph, BMTP motion construction, and independent validation.
%
% INPUTS
%   - showPlot (logical scalar, optional): show the core geometry and motion.
%
% OUTPUTS
%   - result: unmodified obstacleAvoidance.planTrajectory result.
%
% UNITS
%   - Position is coordinate units and time is seconds.

%% Section 1: Add The BMTP Kernel

repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot, fullfile(repositoryRoot, "trajectory"));
if nargin < 1 || isempty(showPlot)
    showPlot = true;
end
validateattributes(showPlot, {'logical'}, {'scalar'});

%% Section 2: Create One Static Convex Obstacle

obstacles = struct("Name", "center block", ...
    "Vertices_units", [-1 -1; 1 -1; 1 1; -1 1], ...
    "SafetyMargin_units", 0.25);

%% Section 3: Create The Fixed-Arrival Request

initialState = struct("time_s", 0, "position_units", [-4 0], ...
    "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
goalState = struct("time_s", 12, "position_units", [4 0], ...
    "velocity_units_s", [0 0], "acceleration_units_s2", [0 0]);
limits = struct("xInterval_units", [-6 6], "yInterval_units", [-4 4], ...
    "maxVelocity_units_s", [2 2], ...
    "maxAcceleration_units_s2", [2 2], "maxJerk_units_s3", [4 4]);
options = struct("GoalTimeMode", "fixedArrival", ...
    "SampleTime_s", 0.05);

%% Section 4: Plan And Independently Validate

result = obstacleAvoidance.planTrajectory(obstacles, initialState, goalState, limits, options);
if ~result.Success || ~result.Validation.Passed
    warning("exampleEmptyCore:PlanningFailed", "%s", result.Message);
end
if showPlot
    obstacleAvoidance.plotting.plotTrajectory(result);
end
end
