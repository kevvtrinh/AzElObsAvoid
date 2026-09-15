function result = planPeriodicRequest(obstacles, initialState, goalState, bandLimits, options, request)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.input.planPeriodicRequest( ...
%       obstacles, initialState, goalState, bandLimits, options, request)
%**************************************************************************
% PURPOSE
%   - Plan a request with wrapped axes as plain requests in the unwrapped
%     frame. Obstacles are copied to every period offset that meets the
%     reach band, and every goal image inside the band is a candidate.
%   - Candidates are planned nearest first and each is accepted against the
%     periodic request in the planner's one acceptance gate. The result is
%     the earliest valid candidate (earliestArrival) or the shortest
%     (fixedArrival); a fixed-goal candidate whose chord lower bound cannot
%     beat the incumbent is not planned.
%**************************************************************************
% INPUTS
%   - obstacles (any public obstacle input)
%       Obstacles in the periodic frame.
%   - initialState, goalState (scalar structs)
%       Normalized states; the goal already holds its nearest image or the
%       lifted target.
%   - bandLimits (scalar struct)
%       Normalized limits whose wrapped intervals are the reach band.
%   - options (scalar struct)
%       Resolved planner options with WrapX and WrapY.
%   - request (scalar struct)
%       SuppliedLimits, SuppliedGoalState, RequestedLimits, and
%       RequestedGoalState of the periodic request.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner record accepted against the periodic request, with a
%       PeriodicImages field recording the candidate offsets and their
%       outcomes. Failure of every candidate returns Success = false.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Represent The Periodic Scene In The Unwrapped Frame

timer           = tic;
wrapAxes        = [options.WrapX, options.WrapY];
intervals_units = [request.RequestedLimits.xInterval_units; request.RequestedLimits.yInterval_units];
band_units      = [bandLimits.xInterval_units; bandLimits.yInterval_units];
period_units    = diff(intervals_units, 1, 2).';
images          = obstacleAvoidance.input.replicatePeriodicObstacles(obstacles, intervals_units, wrapAxes, band_units);

planarOptions       = options;
planarOptions.WrapX = false;
planarOptions.WrapY = false;
outerRequest = struct( ...
    'SuppliedLimits',     request.SuppliedLimits, ...
    'SuppliedGoalState',  request.SuppliedGoalState, ...
    'RequestedLimits',    request.RequestedLimits, ...
    'RequestedGoalState', request.RequestedGoalState, ...
    'Obstacles',          {obstacles}, ...
    'WrapX',              options.WrapX, ...
    'WrapY',              options.WrapY, ...
    'GoalTime_s',         goalState.time_s, ...
    'GoalTimeMode',       options.GoalTimeMode);

%% Section 2: Enumerate Goal Images Inside The Band, Nearest First

offsetsByAxis = {0, 0};
for axisIndex = find(wrapAxes)
    lowestImage  = ceil((band_units(axisIndex, 1) - goalState.position_units(axisIndex)) / period_units(axisIndex));
    highestImage = floor((band_units(axisIndex, 2) - goalState.position_units(axisIndex)) / period_units(axisIndex));
    offsetsByAxis{axisIndex} = (lowestImage:highestImage) * period_units(axisIndex);
end
[dx_units, dy_units] = ndgrid(offsetsByAxis{1}, offsetsByAxis{2});
offsets_units = [dx_units(:), dy_units(:)];
if isempty(offsets_units)
    % No image is reachable inside the band; plan the nearest image so the
    % ordinary request reports the honest failure.
    offsets_units = [0, 0];
end
chord_units = vecnorm(goalState.position_units + offsets_units - initialState.position_units, 2, 2);
[~, order]  = sortrows([chord_units, offsets_units]);
offsets_units = offsets_units(order, :);
chord_units   = chord_units(order);

%% Section 3: Plan Each Candidate As A Plain Request And Keep The Best

candidateCount   = size(offsets_units, 1);
reasons          = strings(candidateCount, 1);
wasPlanned       = false(candidateCount, 1);
result           = [];
bestKey          = [];
bestOffset_units = [NaN, NaN];
isEarliest       = options.GoalTimeMode == "earliestArrival";
% The chord to a fixed goal bounds motion length and travel time from below;
% a moving target can be met earlier and nearer, so it is never pruned.
canPrune = isempty(goalState.targetMotion);
for candidateIndex = 1:candidateCount
    candidateGoalState = shiftGoal(goalState, offsets_units(candidateIndex, :));
    if ~isempty(bestKey) && canPrune
        bound = chord_units(candidateIndex);
        slack = 0;
        if isEarliest
            bound = initialState.time_s + ...
                obstacleAvoidance.input.minimumTravelTime(initialState, candidateGoalState, bandLimits);
            slack = options.ArrivalTimeTolerance_s;
        end
        if bestKey(1) <= bound + slack
            reasons(candidateIndex) = "chordBoundNotBetter";
            continue
        end
    end
    wasPlanned(candidateIndex) = true;
    candidate = planner(images, initialState, candidateGoalState, bandLimits, planarOptions, outerRequest);
    reasons(candidateIndex) = candidate.TerminationReason;
    if candidate.Success
        if isEarliest
            key = [candidate.ArrivalTime_s, candidate.MotionLength_units];
        else
            key = [candidate.MotionLength_units, candidate.ArrivalTime_s];
        end
        isBetter = isempty(bestKey) || key(1) < bestKey(1) || (key(1) == bestKey(1) && key(2) < bestKey(2));
        if isBetter
            result           = candidate;
            bestKey          = key;
            bestOffset_units = offsets_units(candidateIndex, :);
        end
    elseif isempty(result)
        result = candidate;
    end
end

%% Section 4: Record The Candidates On The Returned Result

if isempty(bestKey) && isfield(result, 'OuterRequest')
    % Every candidate failed: report the nearest image's failure against
    % the periodic request.
    result = obstacleAvoidance.input.applyOuterRequest(result, outerRequest);
end
result.PeriodicImages = struct( ...
    'GoalOffset_units',           bestOffset_units, ...
    'CandidateOffsets_units',     offsets_units, ...
    'CandidatePlanned',           wasPlanned, ...
    'CandidateTerminationReason', reasons, ...
    'ObstacleImageCount',         numel(images));
result.ElapsedTime_s = toc(timer);
end

%% Section 5: Local Functions

function goalState = shiftGoal(goalState, offset_units)
    % Move the goal, and a lifted target with it, to another image.
    goalState.position_units = goalState.position_units + offset_units;
    if isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion)
        goalState.targetMotion.position_units = goalState.targetMotion.position_units + offset_units;
    end
end
