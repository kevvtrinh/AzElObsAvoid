function obstacleCopies = copyObstaclesAcrossWraps(obstacles, intervals_units, wrapModes, reachableRange_units, timeRange_s)
%% Section 0: Header & Readme
% SYNTAX
%   copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
%       obstacles, intervals_units, wrapModes, reachableRange_units)
%   copies = obstacleAvoidance.input.copyObstaclesAcrossWraps( ...
%       obstacles, intervals_units, wrapModes, reachableRange_units, timeRange_s)
%**************************************************************************
% PURPOSE
%   - Copy obstacles across wrapped interval ends so planning can continue
%     past them. An x copy is shifted by whole turns: an obstacle at x = 10
%     also appears at x = 370 on a 360-unit axis. A y copy over an end is a
%     pole copy, as for elevation on a sphere: y is mirrored about that end
%     and x turns by half a turn. On x [0 360], y [-90 90], an obstacle at
%     (190, 89) also appears at (10, 91).
%   - Keep every copy whose bounds on copied axes overlap the possible
%     travel range. Move its original and protected boundaries by the same
%     transform; obstacle preparation handles each copy later.
%**************************************************************************
% INPUTS
%   - obstacles (any public obstacle input)
%       Obstacles in the wrapped frame.
%   - intervals_units (2-by-2 numeric array)
%       Wrapped workspace intervals [xmin xmax; ymin ymax].
%   - wrapModes (1-by-2 string array)
%       [WrapX WrapY], each "false", "both", "forward", or "backward".
%   - reachableRange_units (2-by-2 numeric array)
%       Possible travel range [xmin xmax; ymin ymax], calculated from maximum
%       speed x available time. Acceleration and obstacles may reduce travel.
%   - timeRange_s (1-by-2 numeric, optional; default all times)
%       [start end] of the request. Only the samples the request uses set
%       the copy bounds, and only obstacle intervals that overlap it are
%       checked before making a pole copy.
%**************************************************************************
% OUTPUTS
%   - copies (column struct array)
%       Obstacle records ordered by obstacle, then x offset, then y copy.
%       WrapTransform = [x offset, y scale, y offset] records the transform
%       that made each copy. The identity copy has [0, 1, 0].
%       Empty input returns an empty array with the standard obstacle fields.
%**************************************************************************
% UNITS
%   - Coordinate units.
%**************************************************************************

%% Section 1: Prepare The Obstacle Inputs

if nargin < 5
    timeRange_s = [-Inf, Inf];
end
canonicalObstacles = obstacleAvoidance.obstacles.canonicalizeObstacles(obstacles);
obstacleCopies     = canonicalObstacles;
if isempty(canonicalObstacles) || all(wrapModes == "false")
    return
end

%% Section 2: Copy Obstacles Within The Possible Travel Range

obstacleCopyList = cell(0, 1);
for obstacleIndex = 1:numel(canonicalObstacles)
    % Standard records may omit original boundaries when their margin is
    % zero. Normalize once before reading either boundary or making copies;
    % this also checks the stored margin without adding it again.
    obstacle = obstacleAvoidance.obstacles.createObstacle(canonicalObstacles(obstacleIndex));

    % Use boundary points from every sample the request can use, as
    % obstacle preparation does: samples inside the request's times plus
    % both ends of every interval that overlaps them. A sample far outside
    % those times cannot list a copy the request never needs, and an
    % obstacle recorded only outside them is inactive, so it gets no copy.
    sampleTime_s   = obstacle.time_s(:);
    neededSamples  = sampleTime_s >= timeRange_s(1) & sampleTime_s <= timeRange_s(2);
    neededIntervals = find(sampleTime_s(1:end - 1) < timeRange_s(2) & sampleTime_s(2:end) > timeRange_s(1));
    neededSamples([neededIntervals; neededIntervals + 1]) = true;
    if numel(sampleTime_s) == 1
        neededSamples(1) = true;
    end
    boundaryX_units = vertcat(obstacle.x_units{neededSamples});
    boundaryY_units = vertcat(obstacle.y_units{neededSamples});
    pointIsFinite   = isfinite(boundaryX_units) & isfinite(boundaryY_units);
    if ~any(pointIsFinite)
        continue
    end
    obstacleBounds_units = [min(boundaryX_units(pointIsFinite)), max(boundaryX_units(pointIsFinite)); ...
        min(boundaryY_units(pointIsFinite)), max(boundaryY_units(pointIsFinite))];
    images = obstacleAvoidance.input.listWrapImages( ...
        obstacleBounds_units, intervals_units, wrapModes, reachableRange_units);
    if any(images.YScale == -1)
        requireMirroredMatching(obstacle, timeRange_s);
    end

    % Move both boundaries together. The protected boundary already includes
    % its margin; a shift or mirror keeps distances, so no margin is added.
    for imageIndex = 1:numel(images.XOffset_units)
        xOffset_units = images.XOffset_units(imageIndex);
        yScale        = images.YScale(imageIndex);
        yOffset_units = images.YOffset_units(imageIndex);
        obstacleCopy  = obstacle;
        obstacleCopy.WrapTransform = [xOffset_units, yScale, yOffset_units];
        if xOffset_units ~= 0
            obstacleCopy.x_units = cellfun(@(coordinates_units) coordinates_units + xOffset_units, ...
                obstacle.x_units, 'UniformOutput', false);
            obstacleCopy.originalX_units = cellfun(@(coordinates_units) coordinates_units + xOffset_units, ...
                obstacle.originalX_units, 'UniformOutput', false);
        end
        if yScale ~= 1 || yOffset_units ~= 0
            obstacleCopy.y_units = cellfun(@(coordinates_units) yScale * coordinates_units + yOffset_units, ...
                obstacle.y_units, 'UniformOutput', false);
            obstacleCopy.originalY_units = cellfun(@(coordinates_units) yScale * coordinates_units + yOffset_units, ...
                obstacle.originalY_units, 'UniformOutput', false);
        end
        if yScale == -1
            % y -> offset - y mirrors about the pole line y = offset / 2.
            obstacleCopy.targetName = string(obstacle.targetName) + ...
                sprintf(" (pole copy %+g, over y = %g)", xOffset_units, yOffset_units / 2);
        elseif xOffset_units ~= 0 || yOffset_units ~= 0
            obstacleCopy.targetName = string(obstacle.targetName) + ...
                sprintf(" (wrap copy %+g, %+g)", xOffset_units, yOffset_units);
        end
        obstacleCopyList{end + 1, 1} = obstacleCopy; %#ok<AGROW>
    end
end

if isempty(obstacleCopyList)
    obstacleCopies = canonicalObstacles(zeros(0, 1));
else
    obstacleCopies = vertcat(obstacleCopyList{:});
end
end

%% Section 3: Local Functions

function requireMirroredMatching(obstacle, timeRange_s)
    % A moving obstacle's shape between samples comes from matching each
    % sample's vertices to the next by best fit. An exact tie between two
    % fits is broken by vertex position (leftmost, then lowest), which a
    % mirror does not keep: a square turning into a diamond can then turn
    % the other way in its pole copy. The copy would no longer move as the
    % mirror of the obstacle, so stop with a clear error. Declared vertex
    % matching (vertexCorrespondence = "sourceIndex") is kept by the mirror.
    if numel(obstacle.time_s) < 2
        return
    end
    % The source was normalized before copying, so repeated closing
    % vertices are already removed before checking the same rings.
    % The declared rule is authoritative; the derived flag may be absent.
    declaresVertexOrder = isfield(obstacle, 'vertexCorrespondence') && ...
        ~isempty(obstacle.vertexCorrespondence) && string(obstacle.vertexCorrespondence) == "sourceIndex";
    if ~isfield(obstacle, 'vertexCorrespondence') || isempty(obstacle.vertexCorrespondence)
        declaresVertexOrder = isfield(obstacle, 'UsesSourceIndex') && obstacle.UsesSourceIndex;
    end
    if declaresVertexOrder
        return
    end
    ringSets = {obstacle.x_units, obstacle.y_units};
    if all(isfield(obstacle, {'originalX_units', 'originalY_units'}))
        ringSets(2, :) = {obstacle.originalX_units, obstacle.originalY_units};
    end
    for ringSetIndex = 1:size(ringSets, 1)
        xHistory_units = ringSets{ringSetIndex, 1};
        yHistory_units = ringSets{ringSetIndex, 2};
        % Preparation uses only intervals that overlap the request's times
        % (prepareOneObstacle); one that only touches the start or the
        % deadline is not used.
        intervalOverlapsRequest = obstacle.time_s(1:end - 1) < timeRange_s(2) & ...
            obstacle.time_s(2:end) > timeRange_s(1);
        for sampleIndex = reshape(find(intervalOverlapsRequest), 1, [])
            current_units = [xHistory_units{sampleIndex}(:), yHistory_units{sampleIndex}(:)];
            next_units    = [xHistory_units{sampleIndex + 1}(:), yHistory_units{sampleIndex + 1}(:)];
            % Only one ring per sample with the same vertex count is matched
            % this way; other layouts use models that do not reorder vertices.
            ringsAreMatched = size(current_units, 1) >= 3 && isequal(size(current_units), size(next_units)) && ...
                all(isfinite([current_units; next_units]), 'all');
            if ~ringsAreMatched
                continue
            end
            mirror      = [1, -1];
            matched     = obstacleAvoidance.obstacles.alignCorrespondingRing(current_units, next_units);
            mirrorMatch = obstacleAvoidance.obstacles.alignCorrespondingRing( ...
                current_units .* mirror, next_units .* mirror);
            if ~isequal(mirrorMatch, matched .* mirror)
                error("planner:AmbiguousPoleCopyMotion", ...
                    "Obstacle ""%s"" matches its vertices between samples %d and %d by a tie " + ...
                    "that a pole copy cannot mirror. Declare its vertex matching with " + ...
                    "vertexCorrespondence = ""sourceIndex"".", ...
                    string(obstacle.targetName), sampleIndex, sampleIndex + 1);
            end
        end
    end
end
