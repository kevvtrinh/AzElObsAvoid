function candidateSet = solveSeeds( ...
        seeds, context, stageTiming, physicalArrivalLowerBound_s, ...
        planningTimer)
%% Section 0: Header & Readme
% SYNTAX
%   candidateSet = obstacleAvoidance.planner.solveSeeds( ...
%       seeds, context, stageTiming, physicalArrivalLowerBound_s, ...
%       planningTimer)
%**************************************************************************
% PURPOSE
%   - Solve and fully check deterministic seeds in their established order.
%   - Stop only when a passing motion reaches a proven arrival-time floor.
%**************************************************************************
% INPUTS
%   - seeds (nonempty route-seed struct array)
%       Deterministically ordered route and timing suggestions.
%   - context (scalar struct)
%       Motion methods, prepared request, options, and summary template.
%   - stageTiming (scalar struct)
%       Accumulated planner timing before seed solving.
%   - physicalArrivalLowerBound_s (scalar numeric)
%       Proven fixed-goal arrival floor or NaN when no floor is available.
%   - planningTimer (MATLAB tic handle)
%       Planner clock used only for first-passing runtime evidence.
%**************************************************************************
% OUTPUTS
%   - candidateSet (scalar struct)
%       Processed seeds, candidate motions, summaries, check results, early
%       exit details, first-passing time, and updated stage timing.
%**************************************************************************
% UNITS
%   - Physical and measured times are seconds; positions are degrees.
%**************************************************************************

%% Section 1: Create Stable Attempt Storage

% Seed order is part of planner behavior. Preallocate one record per offered
% seed, then trim only when the declared earliest-arrival lower bound proves
% that later seeds cannot improve the accepted objective.

seedCount = numel(seeds);
summaries = repmat(context.SummaryTemplate, seedCount, 1);
candidates = cell(seedCount, 1);
checkTemplate = obstacleAvoidance.validateTrajectory();
checkResults = repmat(checkTemplate, seedCount, 1);
firstValidatedMotionTime_s = NaN;
earlyExit = struct( ...
    "Applied", false, ...
    "Reason", "lowerBoundNotReached", ...
    "PhysicalArrivalLowerBound_s", physicalArrivalLowerBound_s, ...
    "ReachedBySeedIndex", 0);
if context.Options.GoalTimeMode ~= "earliestArrival"
    earlyExit.Reason = "policyRequiresFullComparison";
elseif ~isfinite(physicalArrivalLowerBound_s)
    earlyExit.Reason = "unprovenRequestLowerBound";
end

%% Section 2: Solve And Check Every Required Seed

% Each route suggestion must become a timed motion and pass the same public
% trajectory check before selection can see it. A solver success flag alone
% never makes a seed eligible.

processedSeedCount = 0;
for seedIndex = 1:seedCount
    [candidate, summary, checkResult, stageTiming] = ...
        obstacleAvoidance.planner.solveOneSeed( ...
        seeds(seedIndex), context, stageTiming);
    candidates{seedIndex} = candidate;
    summaries(seedIndex) = summary;
    checkResults(seedIndex) = checkResult;
    processedSeedCount = seedIndex;
    if checkResult.Passed && isnan(firstValidatedMotionTime_s)
        firstValidatedMotionTime_s = toc(planningTimer);
    end
    if reachesPhysicalArrivalLowerBound( ...
            summary, checkResult, physicalArrivalLowerBound_s, ...
            context.Options)
        earlyExit.Applied = true;
        earlyExit.Reason = "validatedPhysicalArrivalLowerBound";
        earlyExit.ReachedBySeedIndex = seedIndex;
        break;
    end
end
if processedSeedCount < seedCount
    seeds = seeds(1:processedSeedCount);
    candidates = candidates(1:processedSeedCount);
    summaries = summaries(1:processedSeedCount);
    checkResults = checkResults(1:processedSeedCount);
end

%% Section 3: Assemble Candidate-Set Evidence

candidateSet = struct( ...
    "Seeds", seeds, ...
    "Candidates", {candidates}, ...
    "Summaries", summaries, ...
    "CheckResults", checkResults, ...
    "FirstValidatedMotionTime_s", firstValidatedMotionTime_s, ...
    "SeedEarlyExit", earlyExit, ...
    "StageTiming", stageTiming);
end

%% Section 4: Local Functions

function reached = reachesPhysicalArrivalLowerBound( ...
        summary, checkResult, lowerBound_s, options)
% Stop earliest-arrival search only at a proven fixed-goal physical floor.
reached = options.GoalTimeMode == "earliestArrival" && ...
    checkResult.Passed && isfinite(lowerBound_s) && ...
    isfinite(summary.ArrivalTime_s) && ...
    abs(summary.ArrivalTime_s - lowerBound_s) <= ...
    options.ArrivalTimeTolerance_s;
end
