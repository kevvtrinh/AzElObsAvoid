function images = replicatePeriodicObstacles(obstacles, intervals_units, wrapAxes, band_units)
%% Section 0: Header & Readme
% SYNTAX
%   images = obstacleAvoidance.input.replicatePeriodicObstacles( ...
%       obstacles, intervals_units, wrapAxes, band_units)
%**************************************************************************
% PURPOSE
%   - Represent the obstacles of a periodic workspace in the unwrapped
%     planning frame. Every obstacle is copied to each period offset along
%     the wrapped axes at which its protected extent meets the reach band.
%   - Each image is an exact translated copy of the supplied history,
%     protected and original geometry alike, and is prepared on its own
%     later. Copies that never meet the band are omitted because no motion
%     inside the band can touch them.
%**************************************************************************
% INPUTS
%   - obstacles (any public obstacle input)
%       Obstacles in the periodic frame.
%   - intervals_units (2-by-2 numeric array)
%       Periodic workspace intervals [xmin xmax; ymin ymax].
%   - wrapAxes (1-by-2 logical)
%       Wrapped axes.
%   - band_units (2-by-2 numeric array)
%       Planning band of the request.
%**************************************************************************
% OUTPUTS
%   - images (column struct array)
%       Canonical image records ordered by obstacle, then x offset, then y
%       offset; empty input is returned unchanged.
%**************************************************************************
% UNITS
%   - Coordinate units.
%**************************************************************************

%% Section 1: Copy Every Obstacle To Each Offset That Meets The Band

canonicalObstacles = obstacleAvoidance.obstacles.canonicalizeObstacles(obstacles);
images = canonicalObstacles;
if isempty(canonicalObstacles) || ~any(wrapAxes)
    return
end

period_units = diff(intervals_units, 1, 2).';
imageList    = cell(0, 1);
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
        lowestImage  = ceil((band_units(axisIndex, 1) - extent_units(axisIndex, 2)) / period_units(axisIndex));
        highestImage = floor((band_units(axisIndex, 2) - extent_units(axisIndex, 1)) / period_units(axisIndex));
        offsets_units{axisIndex} = (lowestImage:highestImage) * period_units(axisIndex);
    end

    for dx_units = offsets_units{1}
        for dy_units = offsets_units{2}
            image = obstacle;
            if dx_units ~= 0
                image.x_units         = cellfun(@(v) v + dx_units, obstacle.x_units, 'UniformOutput', false);
                image.originalX_units = cellfun(@(v) v + dx_units, obstacle.originalX_units, 'UniformOutput', false);
            end
            if dy_units ~= 0
                image.y_units         = cellfun(@(v) v + dy_units, obstacle.y_units, 'UniformOutput', false);
                image.originalY_units = cellfun(@(v) v + dy_units, obstacle.originalY_units, 'UniformOutput', false);
            end
            if dx_units ~= 0 || dy_units ~= 0
                image.targetName = string(obstacle.targetName) + ...
                    sprintf(" (periodic image %+g, %+g)", dx_units, dy_units);
            end
            imageList{end + 1, 1} = image; %#ok<AGROW>
        end
    end
end

if isempty(imageList)
    images = canonicalObstacles(zeros(0, 1));
else
    images = vertcat(imageList{:});
end
end
