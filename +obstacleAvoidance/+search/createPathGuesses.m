function seedSet = createPathGuesses(initialState, goalState, limits, options, routeSet, obstacleEnvelope_deg)
%% Section 0: Header & Readme
% SYNTAX
%   seedSet = obstacleAvoidance.search.createPathGuesses( ...
%       initialState, goalState, limits, options, routeSet, obstacleEnvelope_deg)
%
% PURPOSE
%   - Create initial path guesses for the motion solver.
%   - Preserve spatial routes while keeping duration estimates advisory.
%
% INPUTS
%   - initialState, goalState, limits, options: normalized planning inputs.
%   - routeSet (scalar struct or empty)
%       Timed and spatial route suggestions returned by searchRoutes.
%   - obstacleEnvelope_deg: spatial proposal boundary, or empty without routes.
%
% OUTPUTS
%   - seedSet (struct array)
%       Direct seed first, followed by a timed seed and distinct spatial
%       seeds in search order. Estimates never reject a route.
%
% UNITS
%   - Positions, boundaries, and lengths are degrees; duration is seconds.
%

%% Section 1: Create The Required Direct Seed

% Always propose the direct route first. Its velocity-based duration
% is a lower bound, not a deadline for solving the seed.

start_deg = initialState.position_deg;
goal_deg  = obstacleAvoidance.input.goalPositionAtTime(goalState, goalState.time_s);
% Build wrapped route alternatives when enabled; otherwise search only in the supplied azimuth interval.
if options.AllowAzimuthWrapping
    goal_deg(1) = goal_deg(1) + 360 * round((start_deg(1) - goal_deg(1)) / 360);
end
available_s      = goalState.time_s - initialState.time_s;
directRoute_deg  = [start_deg; goal_deg];
directLength_deg = norm(goal_deg - start_deg);
directDuration_s = min(available_s, max(1e-3, max(abs(goal_deg - start_deg) ./ limits.maxVelocity_deg_s)));
template         = obstacleAvoidance.search.createEmptyPathGuess();
seedSet          = template;
seedSet.Index               = 1;
seedSet.Source              = "directPathGuess";
seedSet.position_deg        = directRoute_deg;
seedSet.tau                 = [0; 1];
seedSet.EstimatedDuration_s = directDuration_s;
seedSet.Length_deg          = directLength_deg;
if isempty(routeSet) || isempty(fieldnames(routeSet))
    return;
end

routeGuesses = obstacleAvoidance.search.createRoutePathGuesses(routeSet, obstacleEnvelope_deg, directDuration_s, directLength_deg);
% Process each item needed to build path guesses.
for index = 1:numel(routeGuesses)
    routeGuesses(index).Index = index + 1;
end
seedSet = [seedSet; routeGuesses];
end
