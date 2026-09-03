function ranking = createCandidateRanking(summaries, indices, options)
%% Section 0: Header & Readme
% SYNTAX
%   ranking = obstacleAvoidance.planner.createCandidateRanking( ...
%       summaries, indices, options)
%**************************************************************************
% PURPOSE
%   - Create the declared deterministic ranking for passing candidates.
%   - Expose objective columns and stable final order for inspection.
%**************************************************************************
% INPUTS
%   - summaries (candidate-summary struct array)
%       Stable solve and authoritative-check evidence.
%   - indices (nonempty integer vector)
%       Candidate indices already proven to have ValidationPassed=true.
%   - options (resolved scalar struct)
%       Goal-time mode and declared tradeoff policy.
%**************************************************************************
% OUTPUTS
%   - ranking (scalar struct)
%       Goal-time mode, column names, numeric values, source indices, and
%       ordered candidate indices.
%**************************************************************************
% UNITS
%   - Arrival time is seconds, travel measures are degrees, and utilization
%     and indices are dimensionless.
%**************************************************************************

%% Section 1: Create Policy-Specific Ranking Columns

% Ranking is allowed to compare only motions that passed the full trajectory
% check. Compare only quantities named by the selected objective, then use
% candidate order as the deterministic final tie break.

indices = indices(:);
length_deg = [summaries(indices).MotionLength_deg].';
if options.GoalTimeMode == "fixedArrival"
    columnNames = ["MotionLength_deg", "CandidateIndex"];
    values = [length_deg, indices];
elseif options.GoalTimeMode == "earliestArrival"
    columnNames = ["ArrivalTime_s", "MotionLength_deg", ...
        "CandidateIndex"];
    values = [[summaries(indices).ArrivalTime_s].', ...
        length_deg, indices];
else
    columnNames = ["TravelTimeTradeoffCost_deg", "ArrivalTime_s", ...
        "MotionLength_deg", "CandidateIndex"];
    values = [ ...
        [summaries(indices).TravelTimeTradeoffCost_deg].', ...
        [summaries(indices).ArrivalTime_s].', ...
        length_deg, indices];
end

%% Section 2: Sort Deterministically

[~, order] = sortrows(values, 1:size(values, 2));
ranking = struct( ...
    "GoalTimeMode", options.GoalTimeMode, ...
    "ColumnNames", columnNames, ...
    "Values", values, ...
    "CandidateIndices", indices, ...
    "OrderedCandidateIndices", indices(order));
end
