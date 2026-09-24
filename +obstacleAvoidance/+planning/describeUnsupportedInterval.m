function message = describeUnsupportedInterval(preparedObstacles, requestedInterval_s)
%% Section 0: Header & Readme
% SYNTAX
%   message = obstacleAvoidance.planning.describeUnsupportedInterval( ...
%       preparedObstacles, requestedInterval_s)
%**************************************************************************
% PURPOSE
%   - Find the first prepared obstacle interval, inside the request's times,
%     whose geometry has no proven exact continuous interpolation, so that
%     planning stops before it checks collisions against geometry it cannot
%     use. This does not prove there is no route; the geometry could not be
%     checked.
%   - Return an empty message when every overlapping interval is usable.
%**************************************************************************
% INPUTS
%   - preparedObstacles (struct array)
%       Obstacles from prepareObstacles, each with InternalPreparation.
%   - requestedInterval_s (1-by-2 numeric)
%       [start time, arrival time] of the request. Only obstacle intervals
%       that overlap it are checked.
%**************************************************************************
% OUTPUTS
%   - message (char or string)
%       Empty when no interval is unsupported. Otherwise it names the
%       obstacle and interval, for example: Obstacle 2 ("wall"), interval
%       [1, 2] s, has no proven exact continuous interpolation.
%**************************************************************************
% UNITS
%   - Seconds.
%**************************************************************************

%% Section 1: Find The First Unsupported Interval Inside The Request

message = '';
for obstacleIndex = 1:numel(preparedObstacles)
    obstaclePreparation = preparedObstacles(obstacleIndex).InternalPreparation;
    obstacleTime_s      = preparedObstacles(obstacleIndex).time_s;

    intervalIsUnsupported   = obstaclePreparation.IntervalPrepared & obstaclePreparation.IntervalIsUnsupported;
    intervalOverlapsRequest = obstacleTime_s(1:end - 1) < requestedInterval_s(2) & ...
        obstacleTime_s(2:end) > requestedInterval_s(1);
    unsupportedIntervalIndex = find(intervalIsUnsupported & intervalOverlapsRequest, 1);
    if isempty(unsupportedIntervalIndex)
        continue
    end

    % Tell the caller which obstacle and time interval prevented planning.
    intervalTime_s = obstacleTime_s(unsupportedIntervalIndex:unsupportedIntervalIndex + 1);
    obstacleName   = string(preparedObstacles(obstacleIndex).targetName);
    message = sprintf(['Obstacle %d ("%s"), interval [%g, %g] s, has no ' ...
        'proven exact continuous interpolation.'], ...
        obstacleIndex, obstacleName, intervalTime_s(1), intervalTime_s(2));

    % Include a more specific explanation if preparation recorded one.
    hasProofReason = isfield(obstaclePreparation, 'IntervalProofReason');
    if hasProofReason
        proofReason = obstaclePreparation.IntervalProofReason(unsupportedIntervalIndex);
        if proofReason == "movingCellsExcludeProtectedSample"
            % The calculated region misses part of a supplied obstacle shape
            % with its safety margin, so it cannot safely represent that shape.
            message = message + ...
                " The given moving-cell margin-square enclosure excludes " + ...
                "supplied protected sample area.";
        end
    end
    return
end
end
