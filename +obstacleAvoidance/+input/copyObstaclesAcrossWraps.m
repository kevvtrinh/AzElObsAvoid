function obstacleCopies = copyObstaclesAcrossWraps(obstacles, intervals_units, wrapAxes, reachableRange_units)
%% Section 0: Header & Readme
% SYNTAX
%   copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
%       obstacles, intervals_units, wrapAxes, reachableRange_units)
%**************************************************************************
% PURPOSE
%   - A wrapped axis (azimuth 359 meets 0) is planned in plain unwrapped
%     coordinates, so an obstacle must also appear one full turn up and one
%     full turn down. This copies every obstacle to each whole-turn shift
%     whose footprint reaches into the range the vehicle can reach in time.
%   - Each copy is the same history shifted by whole turns, protected and
%     original geometry alike, and is prepared on its own later. Copies that
%     never reach the reachable range are left out: no motion can touch them.
%**************************************************************************
% INPUTS
%   - obstacles (any public obstacle input)
%       Obstacles in the wrapped frame.
%   - intervals_units (2-by-2 numeric array)
%       Wrapped workspace intervals [xmin xmax; ymin ymax].
%   - wrapAxes (1-by-2 logical)
%       Wrapped axes.
%   - reachableRange_units (2-by-2 numeric array)
%       Unwrapped range the vehicle can reach in the time given,
%       [xmin xmax; ymin ymax].
%**************************************************************************
% OUTPUTS
%   - copies (column struct array)
%       Canonical copy records ordered by obstacle, then x offset, then y
%       offset; empty input is returned unchanged.
%**************************************************************************
% UNITS
%   - Coordinate units.
%**************************************************************************

%% Section 1: Copy Every Obstacle To Each Whole-Turn Shift That Reaches The Range

canonicalObstacles = obstacleAvoidance.obstacles.canonicalizeObstacles(obstacles);
obstacleCopies = canonicalObstacles;
if isempty(canonicalObstacles) || ~any(wrapAxes)
    return
end

period_units = diff(intervals_units, 1, 2).';
obstacleCopyList    = cell(0, 1);
for obstacleIndex = 1:numel(canonicalObstacles)
    obstacle = canonicalObstacles(obstacleIndex);
    x_units  = vertcat(obstacle.x_units{:});
    y_units  = vertcat(obstacle.y_units{:});
    isFinitePoint = isfinite(x_units) & isfinite(y_units);
    if ~any(isFinitePoint)
        continue
    end
    extent_units = [min(x_units(isFinitePoint)), max(x_units(isFinitePoint)); ...
        min(y_units(isFinitePoint)), max(y_units(isFinitePoint))];

    offsets_units = {0, 0};
    for axisIndex = find(wrapAxes)
        lowestCopy  = ceil((reachableRange_units(axisIndex, 1) - extent_units(axisIndex, 2)) / period_units(axisIndex));
        highestCopy = floor((reachableRange_units(axisIndex, 2) - extent_units(axisIndex, 1)) / period_units(axisIndex));
        offsets_units{axisIndex} = (lowestCopy:highestCopy) * period_units(axisIndex);
    end

    for dx_units = offsets_units{1}
        for dy_units = offsets_units{2}
            obstacleCopy = obstacle;
            if dx_units ~= 0
                obstacleCopy.x_units         = cellfun(@(v) v + dx_units, obstacle.x_units, 'UniformOutput', false);
                obstacleCopy.originalX_units = cellfun(@(v) v + dx_units, obstacle.originalX_units, 'UniformOutput', false);
            end
            if dy_units ~= 0
                obstacleCopy.y_units         = cellfun(@(v) v + dy_units, obstacle.y_units, 'UniformOutput', false);
                obstacleCopy.originalY_units = cellfun(@(v) v + dy_units, obstacle.originalY_units, 'UniformOutput', false);
            end
            if dx_units ~= 0 || dy_units ~= 0
                obstacleCopy.targetName = string(obstacle.targetName) + ...
                    sprintf(" (wrap copy %+g, %+g)", dx_units, dy_units);
            end
            obstacleCopyList{end + 1, 1} = obstacleCopy; %#ok<AGROW>
        end
    end
end

if isempty(obstacleCopyList)
    obstacleCopies = canonicalObstacles(zeros(0, 1));
else
    obstacleCopies = vertcat(obstacleCopyList{:});
end
end
