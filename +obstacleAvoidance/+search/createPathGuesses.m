function seedSet = createPathGuesses(initialState, goalState, limits, routeSet, obstacleEnvelope_units)
%% Section 0: Header & Readme
% SYNTAX
%   seedSet = obstacleAvoidance.search.createPathGuesses( ...
%       initialState, goalState, limits, routeSet, obstacleEnvelope_units)
%
% PURPOSE
%   - Create initial path guesses for the motion solver.
%   - Preserve spatial routes while keeping duration estimates advisory.
%
% INPUTS
%   - initialState, goalState, limits: normalized planning inputs.
%   - routeSet (scalar struct or empty)
%       Timed and spatial route suggestions returned by searchRoutes.
%   - obstacleEnvelope_units: spatial proposal boundary, or empty without routes.
%
% OUTPUTS
%   - seedSet (struct array)
%       Direct seed first, followed by a timed seed and distinct spatial
%       seeds in search order. Estimates never reject a route.
%
% UNITS
%   - Positions, boundaries, and lengths are coordinate units; duration is seconds.
%

%% Section 1: Create The Required Direct Seed

% Always propose the direct route first. Its velocity-based duration
% is a lower bound, not a deadline for solving the seed.

start_units = initialState.position_units;
goal_units  = obstacleAvoidance.input.goalPositionAtTime(goalState, goalState.time_s);
% The public request already selects each periodic goal coordinate.
available_s      = goalState.time_s - initialState.time_s;
directRoute_units  = [start_units; goal_units];
directLength_units = norm(goal_units - start_units);
directDuration_s = min(available_s, max(1e-3, max(abs(goal_units - start_units) ./ limits.maxVelocity_units_s)));
template         = obstacleAvoidance.search.createEmptyPathGuess();
seedSet          = template;
seedSet.Index               = 1;
seedSet.Source              = "directPathGuess";
seedSet.position_units        = directRoute_units;
seedSet.tau                 = [0; 1];
seedSet.EstimatedDuration_s = directDuration_s;
seedSet.Length_units          = directLength_units;
if isempty(routeSet) || isempty(fieldnames(routeSet))
    return;
end

routeGuesses = obstacleAvoidance.search.createRoutePathGuesses(routeSet, obstacleEnvelope_units, directDuration_s, directLength_units);
% Process each item needed to build path guesses.
for index = 1:numel(routeGuesses)
    routeGuesses(index).Index = index + 1;
end
seedSet = [seedSet; routeGuesses];
end
