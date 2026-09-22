function obstacleCopies = copyObstaclesAcrossWraps(obstacles, intervals_units, wrapAxes, reachableRange_units)
%% Section 0: Header & Readme
% SYNTAX
%   copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
%       obstacles, intervals_units, wrapAxes, reachableRange_units)
%**************************************************************************
% PURPOSE
%   - Copy obstacles by whole turns so planning can continue across a wrapped
%     interval's ends. For example, an obstacle at x = 10 also appears at
%     x = 370 on a 360-unit axis.
%   - Keep every copy whose coordinate bounds overlap the possible travel
%     range. Move its original and protected boundaries by the same amount;
%     obstacle preparation handles each copy later.
%**************************************************************************
% INPUTS
%   - obstacles (any public obstacle input)
%       Obstacles in the wrapped frame.
%   - intervals_units (2-by-2 numeric array)
%       Wrapped workspace intervals [xmin xmax; ymin ymax].
%   - wrapAxes (1-by-2 logical)
%       [wrapX wrapY]; true enables wrapping on that axis.
%   - reachableRange_units (2-by-2 numeric array)
%       Possible travel range [xmin xmax; ymin ymax], calculated from maximum
%       speed x available time. Acceleration and obstacles may reduce travel.
%**************************************************************************
% OUTPUTS
%   - copies (column struct array)
%       Obstacle records ordered by obstacle, then x offset, then y offset.
%       Empty input returns an empty array with the standard obstacle fields.
%**************************************************************************
% UNITS
%   - Coordinate units.
%**************************************************************************

%% Section 1: Prepare The Obstacle Inputs

canonicalObstacles = obstacleAvoidance.obstacles.canonicalizeObstacles(obstacles);
obstacleCopies     = canonicalObstacles;
if isempty(canonicalObstacles) || ~any(wrapAxes)
    return
end

%% Section 2: Copy Obstacles Within The Possible Travel Range

wrapLength_units = diff(intervals_units, 1, 2).';
obstacleCopyList = cell(0, 1);
for obstacleIndex = 1:numel(canonicalObstacles)
    obstacle = canonicalObstacles(obstacleIndex);

    % Use boundary points from every supplied time so the copies cover the
    % obstacle's full recorded movement, not just its starting position.
    boundaryX_units = vertcat(obstacle.x_units{:});
    boundaryY_units = vertcat(obstacle.y_units{:});
    pointIsFinite   = isfinite(boundaryX_units) & isfinite(boundaryY_units);
    if ~any(pointIsFinite)
        continue
    end
    obstacleBounds_units = [min(boundaryX_units(pointIsFinite)), max(boundaryX_units(pointIsFinite)); ...
        min(boundaryY_units(pointIsFinite)), max(boundaryY_units(pointIsFinite))];

    % Shifted obstacle bounds must overlap the travel range on each wrapped
    % axis. Ceil and floor select the first and last whole turns that fit.
    wrapOffsets_units = {0, 0};
    for axisIndex = find(wrapAxes)
        lowestWrapCount  = ceil((reachableRange_units(axisIndex, 1) - ...
            obstacleBounds_units(axisIndex, 2)) / wrapLength_units(axisIndex));
        highestWrapCount = floor((reachableRange_units(axisIndex, 2) - ...
            obstacleBounds_units(axisIndex, 1)) / wrapLength_units(axisIndex));
        wrapOffsets_units{axisIndex} = (lowestWrapCount:highestWrapCount) * wrapLength_units(axisIndex);
    end

    % Shift both boundaries together. The protected boundary already includes
    % its margin; copying it must not add that margin again.
    for xOffset_units = wrapOffsets_units{1}
        for yOffset_units = wrapOffsets_units{2}
            obstacleCopy = obstacle;
            if xOffset_units ~= 0
                obstacleCopy.x_units = cellfun(@(coordinates_units) coordinates_units + xOffset_units, ...
                    obstacle.x_units, 'UniformOutput', false);
                obstacleCopy.originalX_units = cellfun(@(coordinates_units) coordinates_units + xOffset_units, ...
                    obstacle.originalX_units, 'UniformOutput', false);
            end
            if yOffset_units ~= 0
                obstacleCopy.y_units = cellfun(@(coordinates_units) coordinates_units + yOffset_units, ...
                    obstacle.y_units, 'UniformOutput', false);
                obstacleCopy.originalY_units = cellfun(@(coordinates_units) coordinates_units + yOffset_units, ...
                    obstacle.originalY_units, 'UniformOutput', false);
            end
            if xOffset_units ~= 0 || yOffset_units ~= 0
                obstacleCopy.targetName = string(obstacle.targetName) + ...
                    sprintf(" (wrap copy %+g, %+g)", xOffset_units, yOffset_units);
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
