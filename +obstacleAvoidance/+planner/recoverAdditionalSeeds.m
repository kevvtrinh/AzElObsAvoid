function [candidateSet, routeSet, generatedSeeds] = ...
        recoverAdditionalSeeds( ...
        candidateSet, routeSet, generatedSeeds, recoveryContext)
%% Section 0: Header & Readme
% SYNTAX
%   [candidateSet, routeSet, generatedSeeds] = ...
%       obstacleAvoidance.planner.recoverAdditionalSeeds( ...
%       candidateSet, routeSet, generatedSeeds, recoveryContext)
%**************************************************************************
% PURPOSE
%   - Attempt seeds beyond the first two only after both initial seeds and
%     any separately validated exact motion have failed.
%   - Keep deferred timed and multi-winding recovery in the same removable
%     implementation unit as all other later-seed work.
%**************************************************************************
% INPUTS
%   - candidateSet (scalar candidate-set struct)
%       Results from solving at most the first two generated seeds.
%   - routeSet (scalar route-set struct)
%       Initial timed, spatial, and deferred route-search results.
%   - generatedSeeds (route-seed struct array)
%       All ordinary seeds generated within MaximumSeedCount.
%   - recoveryContext (scalar struct)
%       Scene, Request, Proposal, VisibilityGraph, SeedSolveContext,
%       HasValidatedExactMotion, and PlanningTimer used by recovery.
%**************************************************************************
% OUTPUTS
%   - candidateSet (scalar candidate-set struct)
%       Initial attempts plus failure-only recovery attempts, stopping after
%       the first later seed passes full validation.
%   - routeSet (scalar route-set struct)
%       Route diagnostics updated if deferred search was consumed.
%   - generatedSeeds (route-seed struct array)
%       Ordinary and subsequently generated deferred seeds.
%**************************************************************************
% UNITS
%   - Position is degrees; physical and measured times are seconds.
%**************************************************************************

%% Section 1: Decide Whether Recovery Is Needed

options = recoveryContext.Request.options;
initialSeedCount = numel(candidateSet.Seeds);
initialCandidatePassed = any([candidateSet.CheckResults.Passed]);
if initialCandidatePassed || recoveryContext.HasValidatedExactMotion
    return;
end
if initialSeedCount >= options.MaximumSeedCount
    return;
end

%% Section 2: Try Already Generated Later Seeds

lastOrdinarySeedIndex = min(numel(generatedSeeds), ...
    options.MaximumSeedCount);
for seedIndex = initialSeedCount + 1:lastOrdinarySeedIndex
    [candidateSet, passed] = solveAndAppend( ...
        candidateSet, generatedSeeds(seedIndex), recoveryContext);
    if passed
        return;
    end
end

%% Section 3: Consume Deferred Route Work Within The Same Seed Limit

remainingSeedCount = options.MaximumSeedCount - ...
    numel(candidateSet.Seeds);
if remainingSeedCount <= 0
    return;
end
if isempty(fieldnames(routeSet))
    return;
end

needsDeferredTimedRecovery = routeSet.TimedSearchDeferred;
needsDeferredSpatialRecovery = ...
    ~isempty(routeSet.DeferredSpatialRoutes_deg);
if ~needsDeferredTimedRecovery && ~needsDeferredSpatialRecovery
    return;
end

if needsDeferredTimedRecovery
    recoverySearchTimer = tic;
    routeSet = obstacleAvoidance.search.searchRoutes( ...
        recoveryContext.Scene, recoveryContext.Request, ...
        recoveryContext.Proposal, recoveryContext.VisibilityGraph, routeSet);
    recoverySearchElapsedTime_s = toc(recoverySearchTimer);
    candidateSet.StageTiming.TopologyElapsedTime_s = ...
        candidateSet.StageTiming.TopologyElapsedTime_s + ...
        recoverySearchElapsedTime_s;
end

recoveredOnlyRouteSet = routeSet;
if ~needsDeferredTimedRecovery
    recoveredOnlyRouteSet.TimedRoute_deg = zeros(0, 2);
    recoveredOnlyRouteSet.TimedRouteTime_s = zeros(0, 1);
end
if needsDeferredSpatialRecovery
    recoveredOnlyRouteSet.SpatialRoutes_deg = ...
        routeSet.DeferredSpatialRoutes_deg;
else
    recoveredOnlyRouteSet.SpatialRoutes_deg = cell(0, 1);
end
recoveredSeeds = obstacleAvoidance.search.createSeeds( ...
    recoveredOnlyRouteSet, recoveryContext.Proposal, ...
    recoveryContext.Request);
recoveredSeeds = recoveredSeeds(2:end);

for recoveryIndex = 1:min(remainingSeedCount, numel(recoveredSeeds))
    recoveredSeed = recoveredSeeds(recoveryIndex);
    recoveredSeed.Index = numel(generatedSeeds) + 1;
    % Recovery adds at most three records, so bounded growth is clearer than
    % manufacturing a second seed template solely for preallocation.
    generatedSeeds(end + 1, 1) = recoveredSeed; %#ok<AGROW>
    if string(recoveredSeed.Source) == "visibilityGraph"
        routeSet.DeferredSpatialSolveAttempted = true;
    end
    [candidateSet, passed] = solveAndAppend( ...
        candidateSet, recoveredSeed, recoveryContext);
    if passed
        return;
    end
end
end

%% Section 4: Local Functions

function [candidateSet, passed] = solveAndAppend( ...
        candidateSet, seed, recoveryContext)
% Solve one later seed and append all motion and validation evidence.
seed.Index = numel(candidateSet.Seeds) + 1;
[candidate, summary, checkResult, stageTiming] = ...
    obstacleAvoidance.planner.solveOneSeed( ...
    seed, recoveryContext.SeedSolveContext, candidateSet.StageTiming);
candidateSet.Seeds(end + 1, 1) = seed;
candidateSet.Candidates{end + 1, 1} = candidate;
candidateSet.Summaries(end + 1, 1) = summary;
candidateSet.CheckResults(end + 1, 1) = checkResult;
candidateSet.StageTiming = stageTiming;
passed = checkResult.Passed;
if passed && isnan(candidateSet.FirstValidatedMotionTime_s)
    candidateSet.FirstValidatedMotionTime_s = ...
        toc(recoveryContext.PlanningTimer);
end
end
