function selection = selectValidatedCandidate(summaries, options)
%% Section 0: Header & Readme
% SYNTAX
%   selection = obstacleAvoidance.planner.selectValidatedCandidate( ...
%       summaries, options)
%**************************************************************************
% PURPOSE
%   - Restrict selection to motions that passed the full trajectory check.
%   - Return either the declared best passing index or useful failure evidence.
%**************************************************************************
% INPUTS
%   - summaries (candidate-summary struct array)
%       Stable solve and authoritative-check evidence for attempted seeds.
%   - options (resolved scalar struct)
%       Goal-time mode and declared candidate-ranking policy.
%**************************************************************************
% OUTPUTS
%   - selection (scalar struct)
%       Passing indices, ranking, selected index, best partial index, status,
%       message, and termination reason.
%**************************************************************************
% UNITS
%   - Units are carried by the candidate ranking and summary fields.
%**************************************************************************

%% Section 1: Identify Fully Checked Candidates

% ValidationPassed originates only from obstacleAvoidance.validateTrajectory.
% No kernel feasibility, proposal visibility, or backup status is sufficient
% to enter the ranking set.

validatedIndices = find([summaries.ValidationPassed]).';
emptyRanking = struct( ...
    "GoalTimeMode", options.GoalTimeMode, ...
    "ColumnNames", strings(1, 0), ...
    "Values", zeros(0), ...
    "CandidateIndices", zeros(0, 1), ...
    "OrderedCandidateIndices", zeros(0, 1));
selection = struct( ...
    "Success", false, ...
    "Message", "No compact motion passed independent validation.", ...
    "TerminationReason", "noValidatedSeed", ...
    "ValidatedIndices", validatedIndices, ...
    "ValidatedCandidateCount", numel(validatedIndices), ...
    "SelectedCandidateIndex", 0, ...
    "BestPartialSeedIndex", 0, ...
    "Ranking", emptyRanking);

%% Section 2: Retain Useful Failure Evidence

if isempty(validatedIndices)
    bestIndex = bestPartialSeed(summaries);
    selection.BestPartialSeedIndex = bestIndex;
    if bestIndex > 0
        bestReason = string(summaries(bestIndex).TerminationReason);
        if bestReason == "unsupportedTimedMultiWaypointRoute"
            selection.TerminationReason = bestReason;
            selection.Message = ...
                "A geometric route was found, but the smooth " + ...
                "timed-motion kernel does not yet support its multi-waypoint " + ...
                "topology. The stop-at-waypoint fallback was disabled by policy.";
        elseif strlength(summaries(bestIndex).Message) > 0
            selection.Message = selection.Message + " Best attempt: " + ...
                summaries(bestIndex).Message;
        end
    end
    return;
end

%% Section 3: Rank Only Passing Motions

selection.Ranking = obstacleAvoidance.planner.createCandidateRanking( ...
    summaries, validatedIndices, options);
selection.SelectedCandidateIndex = ...
    selection.Ranking.OrderedCandidateIndices(1);
selection.BestPartialSeedIndex = selection.SelectedCandidateIndex;
selection.Success = true;
selection.Message = "A validated motion was found.";
selection.TerminationReason = "goalReached";
end

%% Section 4: Local Functions

function index = bestPartialSeed(summaries)
% Prefer resolved collision evidence, small residual, and large clearance.
if isempty(summaries)
    index = 0;
    return;
end
violation = [summaries.MaximumConstraintViolation].';
violation(~isfinite(violation)) = Inf;
clearance_deg = [summaries.MinimumClearance_deg].';
clearance_deg(~isfinite(clearance_deg)) = -Inf;
collisionRank = 2 * ~[summaries.CollisionResolved].' + ...
    ~[summaries.CollisionFree].';
[~, order] = sortrows([collisionRank, violation, -clearance_deg, ...
    (1:numel(summaries)).']);
index = order(1);
end
