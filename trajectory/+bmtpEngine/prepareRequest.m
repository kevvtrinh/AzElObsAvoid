function solverRequest = prepareRequest(startingPath, planningEnvironment, motionRequest)
%% Section 0: Header & Readme
% SYNTAX
%   solverRequest = bmtpEngine.prepareRequest(startingPath, planningEnvironment, motionRequest)
%**************************************************************************
% PURPOSE
%   - Collect the starting route, obstacle regions, and requested motion.
%   - Choose the curve degree and collect inputs needed by the BMTP solver.
%**************************************************************************
% INPUTS
%   - startingPath (scalar struct)
%       Starting route positions, progress from 0 to 1, and timing flags.
%       BMTP adjusts this starting path to produce a complete motion.
%   - planningEnvironment (scalar struct)
%       regions_units (column cell array of convex exclusion regions) and
%       coverage (region count and any active time intervals).
%       The planner builds these inputs valid; the engine trusts them.
%   - motionRequest (scalar struct)
%       initialState and goalState (time, position, velocity, and
%       acceleration), limits (workspace, speed, acceleration, and jerk),
%       and options (resolved timing, sampling, and clearance controls).
%**************************************************************************
% OUTPUTS
%   - solverRequest (scalar struct)
%       Planner inputs, prepared obstacle geometry, and solver settings.
%       The planner validates inputs before calling this internal stage.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Check The Engine Inputs
validateattributes(planningEnvironment, {'struct'}, {'scalar'});
validateattributes(motionRequest, {'struct'}, {'scalar'});
assert(all(isfield(planningEnvironment, ["regions_units", "coverage"])), 'bmtpEngine:InvalidScene', ...
    'A scene declares regions_units and coverage.');
assert(all(isfield(motionRequest, ["initialState", "goalState", "limits", "options"])), ...
    'bmtpEngine:InvalidMotionRequest', ...
    'A motion request declares initialState, goalState, limits, and options.');
regions_units = planningEnvironment.regions_units;
coverage      = planningEnvironment.coverage;
initialState  = motionRequest.initialState;
goalState     = motionRequest.goalState;
limits        = motionRequest.limits;
options       = motionRequest.options;

% Both endpoint states already include velocity and acceleration. Check those
% values here because the solver uses them as required boundary conditions.
for derivativeFieldName = ["velocity_units_s", "acceleration_units_s2"]
    validateattributes(initialState.(derivativeFieldName), {'numeric'}, {'real', 'finite', 'size', [1, 2]});
    validateattributes(goalState.(derivativeFieldName), {'numeric'}, {'real', 'finite', 'size', [1, 2]});
end

%% Section 2: Choose The Curve Degree And Timing
% BMTP represents motion with joined polynomial curves. Use degree 8 for
% earliest arrival without timed regions; all other cases use degree 5.
% SplitCount controls how finely the starting route is divided into curves.

[curveDegree, subdivisionCount] = deal(5, 3);
if options.GoalTimeMode == "earliestArrival" && ~isfield(coverage, 'ActiveTimeInterval_s')
    curveDegree = 8;
end

% A variable clock lets the solver change individual segment durations.
% Those requests always need the solver that handles obstacle timing.
segmentTimesCanChange = isfield(startingPath, 'UsesVariableClock') && startingPath.UsesVariableClock;
useTimedSolver        = segmentTimesCanChange || ...
    (isfield(startingPath, 'UsesTimeScopedSolver') && startingPath.UsesTimeScopedSolver);

availableTime_s = goalState.time_s - initialState.time_s;
if availableTime_s <= 0
    error("bmtpEngine:InvalidGoalTime", "goalState.time_s must be greater than initialState.time_s.");
end

%% Section 3: Prepare Obstacle Geometry For Repeated Checks
% Save each region's x/y bounds and the directions perpendicular to its edges.
% BMTP uses these to check whether a line can separate a curve from a region.

regionMinimum_units    = zeros(numel(regions_units), 2);
regionMaximum_units    = zeros(numel(regions_units), 2);
separatingLineGeometry = cell(numel(regions_units), 1);
for regionIndex = 1:numel(regions_units)
    regionMinimum_units(regionIndex, :) = min(regions_units{regionIndex}, [], 1);
    regionMaximum_units(regionIndex, :) = max(regions_units{regionIndex}, [], 1);

    % Static regions keep the same polygon. Timed regions also supply the
    % polygon at the end of their active interval.
    endRegion_units = regions_units{regionIndex};
    if isfield(coverage, 'EndRegions_units')
        endRegion_units = coverage.EndRegions_units{regionIndex};
    end
    separatingLineGeometry{regionIndex} = createSeparatingLineGeometry( ...
        regions_units{regionIndex}, endRegion_units);
end

%% Section 4: Set Solver Limits And Duration Inputs
maximumTrajectoryIterations = 300;
trajectoryOptions           = optimoptions("coneprog", ...
    "Display",             "none", ...
    "MaxIterations",       maximumTrajectoryIterations, ...
    "ConstraintTolerance", 1e-10, ...
    "OptimalityTolerance", 1e-9);

% Keep the timed solver's tested default tolerances. Tighter values did
% not consistently improve results and need a separate tuning comparison.
timedTrajectoryOptions = optimoptions("coneprog", ...
    "Display",       "none", ...
    "MaxIterations", maximumTrajectoryIterations);

% Each alternating iteration updates the separating lines and the motion.
maximumAlternatingIterations = 35;
if isfield(startingPath, 'MaximumAlternatingIterations')
    validateattributes(startingPath.MaximumAlternatingIterations, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'integer', 'positive', '<=', 35});
    maximumAlternatingIterations = double(startingPath.MaximumAlternatingIterations);
end

% The minimum duration is a lower limit; SeedMotionDuration_s is a starting
% estimate. A zero default means no positive value was supplied here.
minimumMotionDuration_s = 0;
if isfield(coverage, 'MinimumMotionDuration_s')
    minimumMotionDuration_s = double(coverage.MinimumMotionDuration_s);
    validateattributes(minimumMotionDuration_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative', '<=', availableTime_s});
end
seedMotionDuration_s = 0;
if isfield(coverage, 'SeedMotionDuration_s')
    seedMotionDuration_s = double(coverage.SeedMotionDuration_s);
    validateattributes(seedMotionDuration_s, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonnegative', '<=', availableTime_s});
end

%% Section 5: Collect Inputs For The Remaining BMTP Stages
% At rest means zero velocity and acceleration at both ends of the motion.
endpointsAreAtRest = all([initialState.velocity_units_s, initialState.acceleration_units_s2, ...
    goalState.velocity_units_s, goalState.acceleration_units_s2] == 0);
solverRequest = struct( ...
    "Seed",                         startingPath, ...
    "Regions_units",                {regions_units}, ...
    "Coverage",                     coverage, ...
    "InitialState",                 initialState, ...
    "GoalState",                    goalState, ...
    "Limits",                       limits, ...
    "Options",                      options, ...
    "IsRest",                       endpointsAreAtRest, ...
    "Degree",                       curveDegree, ...
    "SplitCount",                   subdivisionCount, ...
    "MotionHorizon_s",              availableTime_s, ...
    "MinimumMotionDuration_s",      minimumMotionDuration_s, ...
    "SeedMotionDuration_s",         seedMotionDuration_s, ...
    "UsesVariableClock",            segmentTimesCanChange, ...
    "UsesTimeScopedSolver",         useTimedSolver, ...
    "RegionMinimum_units",          regionMinimum_units, ...
    "RegionMaximum_units",          regionMaximum_units, ...
    "SeparatingLineGeometry",       {separatingLineGeometry}, ...
    "MaximumAlternatingIterations", maximumAlternatingIterations, ...
    "TrajectoryOptions",            trajectoryOptions, ...
    "TimedTrajectoryOptions",       timedTrajectoryOptions);
end

%% Section 6: Local Functions
function separatingLineGeometry = createSeparatingLineGeometry(startRegion_units, endRegion_units)
    % A separating line places the curve and obstacle on opposite sides.
    % Save directions from the obstacle edges once; the solver also checks
    % directions from the curve as the curve changes.
    edgeVectors_units = diff([startRegion_units; startRegion_units(1, :)], 1, 1);
    if ~isequal(startRegion_units, endRegion_units)
        edgeVectors_units = [edgeVectors_units; diff([endRegion_units; endRegion_units(1, :)], 1, 1)];
    end

    % Ignore repeated vertices, which have zero edge length. Rotate each
    % remaining edge by 90 degrees and divide by its length to get a unit normal.
    edgeLengths_units = vecnorm(edgeVectors_units, 2, 2);
    edgeVectors_units = edgeVectors_units(edgeLengths_units > 0, :);
    edgeLengths_units = edgeLengths_units(edgeLengths_units > 0);
    edgeUnitNormals   = [-edgeVectors_units(:, 2), edgeVectors_units(:, 1)] ./ edgeLengths_units;

    % Project vertices onto each normal. The smallest and largest values
    % locate the two parallel lines that touch opposite sides of the polygon.
    startVertexProjections_units = startRegion_units * edgeUnitNormals.';
    endVertexProjections_units   = endRegion_units * edgeUnitNormals.';
    separatingLineGeometry      = struct( ...
        'PositiveNormals',            edgeUnitNormals, ...
        'FirstPositiveSupport_units', min(startVertexProjections_units, [], 1), ...
        'LastPositiveSupport_units',  min(endVertexProjections_units, [], 1), ...
        'FirstNegativeSupport_units', -max(startVertexProjections_units, [], 1), ...
        'LastNegativeSupport_units',  -max(endVertexProjections_units, [], 1));
end
