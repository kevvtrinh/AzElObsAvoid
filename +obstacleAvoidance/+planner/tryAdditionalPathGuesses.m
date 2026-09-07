function [candidateSet, routeSet, generatedSeeds] = tryAdditionalPathGuesses(initialState, goalState, limits, options, candidateSet, routeSet, generatedSeeds, recoveryContext)
%% Section 0: Header & Readme
% SYNTAX
%   [candidateSet, routeSet, generatedSeeds] = tryAdditionalPathGuesses( ...
%       initialState, goalState, limits, options, ...
%       candidateSet, routeSet, generatedSeeds, recoveryContext)
% PURPOSE
%   Try later guesses only after the initial guesses and exact motion fail.
%   Stop at the first validated recovery or MaximumSeedCount.
% INPUTS
%   Normalized states, limits, options, prior candidates, and searched routes.
%   generatedSeeds contains existing guesses. recoveryContext holds the scene,
%   proposal, visibility graph, solver context, exact-motion status, and timer.
% OUTPUTS
%   Updated candidates, route evidence, and guesses, including deferred searches.
% UNITS
%   Position is coordinate units; physical and measured times are seconds.

%% Section 1: Decide Whether Recovery Is Needed

initialSeedCount       = numel(candidateSet.Seeds);
initialCandidatePassed = any([candidateSet.Summaries.ValidationPassed]);
% Stop generating extra seeds once either the initial motion or an exact recovery motion has already validated.
if initialCandidatePassed || recoveryContext.HasValidatedExactMotion
    return;
end
% Skip additional route generation when the configured seed budget is already exhausted.
if initialSeedCount >= options.MaximumSeedCount
    return;
end

%% Section 2: Try Already Generated Later Seeds

lastOrdinarySeedIndex = min(numel(generatedSeeds), options.MaximumSeedCount);
% Evaluate each seed before retaining the best admissible candidate.
for seedIndex = initialSeedCount + 1:lastOrdinarySeedIndex
    [candidateSet, passed, recoveryContext] = solveAndAppend(initialState, goalState, limits, options, candidateSet, generatedSeeds(seedIndex), recoveryContext);
    % Stop ordinary seed evaluation at the first validated motion; later seeds cannot improve the configured first-success policy.
    if passed
        return;
    end
end

%% Section 3: Consume Deferred Route Work Within The Same Seed Limit

remainingSeedCount = options.MaximumSeedCount - numel(candidateSet.Seeds);
% Stop before recovery seeding when ordinary candidates consume the remaining seed budget.
if remainingSeedCount <= 0
    return;
end
if isempty(fieldnames(routeSet))
    return;
end

needsDeferredTimedRecovery   = routeSet.TimedSearchDeferred;
needsDeferredSpatialRecovery = ~isempty(routeSet.DeferredSpatialRoutes_units);
% Run spatial recovery only when timed recovery is not the required next stage.
if ~needsDeferredTimedRecovery && ~needsDeferredSpatialRecovery
    return;
end

% Resume the deferred timed search before generating unrelated spatial recovery seeds.
if needsDeferredTimedRecovery
    recoverySearchTimer         = tic;
    routeSet                    = obstacleAvoidance.search.searchRoutes(initialState, goalState, limits, options, recoveryContext.Scene, recoveryContext.Proposal, recoveryContext.VisibilityGraph, routeSet);
    recoverySearchElapsedTime_s = toc(recoverySearchTimer);
    candidateSet.StageTiming.RouteSearchElapsedTime_s = candidateSet.StageTiming.RouteSearchElapsedTime_s + recoverySearchElapsedTime_s;
end

recoveredOnlyRouteSet = routeSet;
% Run spatial recovery only when timed recovery is not the required next stage.
if ~needsDeferredTimedRecovery
    recoveredOnlyRouteSet.TimedRoute_units   = zeros(0, 2);
    recoveredOnlyRouteSet.TimedRouteTime_s = zeros(0, 1);
end
% Generate spatial recovery seeds only after the primary search explicitly deferred that work.
if needsDeferredSpatialRecovery
    recoveredOnlyRouteSet.SpatialRoutes_units = routeSet.DeferredSpatialRoutes_units;
else
    recoveredOnlyRouteSet.SpatialRoutes_units = cell(0, 1);
end
recoveredSeeds = obstacleAvoidance.search.createRoutePathGuesses(recoveredOnlyRouteSet, recoveryContext.Proposal.shape.Vertices, generatedSeeds(1).EstimatedDuration_s, generatedSeeds(1).Length_units);

% Evaluate each recovery before retaining the best admissible candidate.
for recoveryIndex = 1:min(remainingSeedCount, numel(recoveredSeeds))
    recoveredSeed = recoveredSeeds(recoveryIndex);
    recoveredSeed.Index = numel(generatedSeeds) + 1;
    % At most three recovery seeds are appended.
    generatedSeeds(end + 1, 1) = recoveredSeed; %#ok<AGROW>
    % Preserve visibility-graph provenance for recovered seeds; other recoveries receive the recovery source label.
    if string(recoveredSeed.Source) == "visibilityGraph"
        routeSet.DeferredSpatialSolveAttempted = true;
    end
    [candidateSet, passed, recoveryContext] = solveAndAppend(initialState, goalState, limits, options, candidateSet, recoveredSeed, recoveryContext);
    % Stop ordinary seed evaluation at the first validated motion; later seeds cannot improve the configured first-success policy.
    if passed
        return;
    end
end
end

%% Section 4: Local Functions

function [candidateSet, passed, recoveryContext] = solveAndAppend(initialState, goalState, limits, options, candidateSet, seed, recoveryContext)
    % Solve an additional seed and append its diagnostics.
    seed.Index = numel(candidateSet.Seeds) + 1;
    [candidate, summary, stageTiming, recoveryContext.SeedSolveContext] = obstacleAvoidance.planner.solvePathGuess(recoveryContext.Scene.preparedObstacles, initialState, goalState, limits, options, seed, recoveryContext.SeedSolveContext, candidateSet.StageTiming);
    candidateSet.Seeds(end + 1, 1) = seed;
    candidateSet.Candidates{end + 1, 1} = candidate;
    candidateSet.Summaries(end + 1, 1) = summary;
    candidateSet.StageTiming = stageTiming;
    passed = summary.ValidationPassed;
    % Record the first validation time once; later successful candidates must not overwrite that milestone.
    if passed && isnan(candidateSet.FirstValidatedMotionTime_s)
        candidateSet.FirstValidatedMotionTime_s = toc(recoveryContext.PlanningTimer);
    end
end
