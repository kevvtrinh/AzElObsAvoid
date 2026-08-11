function [regionRecords, motionSpeedBound_deg_s] = ...
        azElObstacleRegionsAtTime(obstacles, queryTime_s, temporalPadding_s)
%% Section 0: Header & Readme
% SYNTAX
%   [regionRecords, motionSpeedBound_deg_s] = ...
%       azElObstacleRegionsAtTime(obstacles, queryTime_s, temporalPadding_s)
%**************************************************************************
% PURPOSE
%   - Evaluate canonical time-varying polygons using linear vertex motion
%     when topology matches and a conservative adjacent-slice union when it
%     does not.
%**************************************************************************
% INPUTS
%   - obstacles (cell column)
%       Normalized scalar canonical obstacle records.
%   - queryTime_s (finite scalar)
%       Absolute query time.
%   - temporalPadding_s (nonnegative scalar)
%       Geometry at queryTime_s plus and minus this uncertainty is unioned.
%**************************************************************************
% OUTPUTS
%   - regionRecords (struct column)
%       vertices_deg, obstacleIndex, regionIndex, and targetName fields.
%   - motionSpeedBound_deg_s (nonnegative scalar)
%       Local Euclidean vertex-speed bound for clearance certification.
%**************************************************************************
% UNITS
%   - Coordinates are degrees, time is seconds, and speed is deg/s.

validateattributes(queryTime_s, "numeric", ...
    ["scalar", "real", "finite"], mfilename, "queryTime_s");
validateattributes(temporalPadding_s, "numeric", ...
    ["scalar", "real", "finite", "nonnegative"], mfilename, ...
    "temporalPadding_s");

if temporalPadding_s == 0
    paddedTimes_s = queryTime_s;
else
    paddedTimes_s = [queryTime_s - temporalPadding_s, queryTime_s, ...
        queryTime_s + temporalPadding_s];
end

regionRecords = emptyRegionRecords();
motionSpeedBound_deg_s = 0;
for paddedTimeIndex = 1:numel(paddedTimes_s)
    for obstacleIndex = 1:numel(obstacles)
        [obstacleRegions, obstacleSpeedBound_deg_s] = ...
            interpolateOneObstacle(obstacles{obstacleIndex}, ...
                paddedTimes_s(paddedTimeIndex), obstacleIndex);
        regionRecords = [regionRecords; obstacleRegions]; %#ok<AGROW>
        motionSpeedBound_deg_s = max(motionSpeedBound_deg_s, ...
            obstacleSpeedBound_deg_s);
    end
end
end

function [records, speedBound_deg_s] = interpolateOneObstacle( ...
        obstacle, queryTime_s, obstacleIndex)
%% Section 0: Header & Readme
% SYNTAX
%   [records, speedBound_deg_s] = interpolateOneObstacle( ...
%       obstacle, queryTime_s, obstacleIndex)
%**************************************************************************
% PURPOSE
%   - Evaluate one obstacle at one time with explicit extrapolation rules.
%**************************************************************************
% INPUTS
%   - obstacle (scalar canonical struct)
%   - queryTime_s (finite scalar)
%   - obstacleIndex (positive integer)
%**************************************************************************
% OUTPUTS
%   - records (struct column)
%   - speedBound_deg_s (nonnegative scalar)
%**************************************************************************
% UNITS
%   - Coordinates are degrees and speed is deg/s.

sampleTime_s = obstacle.time_s;
sampleCount = numel(sampleTime_s);
timeTolerance_s = 64 .* eps(max(1, max(abs(sampleTime_s))));

if sampleCount == 1 || all(queryTime_s <= ...
        sampleTime_s(1) + timeTolerance_s)
    records = recordsFromSample(obstacle, 1, obstacleIndex);
    speedBound_deg_s = 0;
    return;
end
if all(queryTime_s >= sampleTime_s(end) - timeTolerance_s)
    records = recordsFromSample(obstacle, sampleCount, obstacleIndex);
    speedBound_deg_s = 0;
    return;
end

lowerIndex = find(sampleTime_s <= queryTime_s, 1, "last");
upperIndex = lowerIndex + 1;
lowerRegions_deg = splitAzElRegions( ...
    obstacle.az_deg{lowerIndex}, obstacle.el_deg{lowerIndex});
upperRegions_deg = splitAzElRegions( ...
    obstacle.az_deg{upperIndex}, obstacle.el_deg{upperIndex});

topologyMatches = numel(lowerRegions_deg) == numel(upperRegions_deg);
if topologyMatches
    for regionIndex = 1:numel(lowerRegions_deg)
        if ~isequal(size(lowerRegions_deg{regionIndex}), ...
                size(upperRegions_deg{regionIndex}))
            topologyMatches = false;
            break;
        end
    end
end

if ~topologyMatches
    % The union is held over the entire bracket, so no vertex correspondence
    % is invented and the fallback cannot understate occupied space.
    records = recordsFromRegionCells([lowerRegions_deg; upperRegions_deg], ...
        obstacle.targetName, obstacleIndex);
    speedBound_deg_s = 0;
    return;
end

intervalDuration_s = sampleTime_s(upperIndex) - sampleTime_s(lowerIndex);
interpolationFraction = (queryTime_s - sampleTime_s(lowerIndex)) ./ ...
    intervalDuration_s;
interpolatedRegions_deg = cell(size(lowerRegions_deg));
speedBound_deg_s = 0;
for regionIndex = 1:numel(lowerRegions_deg)
    vertexDelta_deg = upperRegions_deg{regionIndex} - ...
        lowerRegions_deg{regionIndex};
    interpolatedRegions_deg{regionIndex} = ...
        lowerRegions_deg{regionIndex} + interpolationFraction .* ...
        vertexDelta_deg;
    if ~isempty(vertexDelta_deg)
        speedBound_deg_s = max(speedBound_deg_s, ...
            max(vecnorm(vertexDelta_deg, 2, 2)) ./ intervalDuration_s);
    end
end
records = recordsFromRegionCells(interpolatedRegions_deg, ...
    obstacle.targetName, obstacleIndex);
end

function records = recordsFromSample(obstacle, sampleIndex, obstacleIndex)
%% Section 0: Header & Readme
% SYNTAX
%   records = recordsFromSample(obstacle, sampleIndex, obstacleIndex)
%**************************************************************************
% PURPOSE
%   - Convert one canonical sample into region records.
%**************************************************************************
% INPUTS
%   - obstacle (scalar canonical struct)
%   - sampleIndex (positive integer)
%   - obstacleIndex (positive integer)
%**************************************************************************
% OUTPUTS
%   - records (struct column)
%**************************************************************************
% UNITS
%   - Coordinates are degrees.

regions_deg = splitAzElRegions(obstacle.az_deg{sampleIndex}, ...
    obstacle.el_deg{sampleIndex});
records = recordsFromRegionCells(regions_deg, obstacle.targetName, ...
    obstacleIndex);
end

function records = recordsFromRegionCells(regions_deg, targetName, ...
        obstacleIndex)
%% Section 0: Header & Readme
% SYNTAX
%   records = recordsFromRegionCells(regions_deg, targetName, obstacleIndex)
%**************************************************************************
% PURPOSE
%   - Attach stable source diagnostics to polygon vertices.
%**************************************************************************
% INPUTS
%   - regions_deg (cell column)
%   - targetName (scalar string)
%   - obstacleIndex (positive integer)
%**************************************************************************
% OUTPUTS
%   - records (struct column)
%**************************************************************************
% UNITS
%   - Coordinates are degrees.

template = struct( ...
    "vertices_deg", zeros(0, 2), ...
    "obstacleIndex", 0, ...
    "regionIndex", 0, ...
    "targetName", "");
records = repmat(template, numel(regions_deg), 1);
for regionIndex = 1:numel(regions_deg)
    records(regionIndex).vertices_deg = regions_deg{regionIndex};
    records(regionIndex).obstacleIndex = obstacleIndex;
    records(regionIndex).regionIndex = regionIndex;
    records(regionIndex).targetName = string(targetName);
end
end

function records = emptyRegionRecords()
%% Section 0: Header & Readme
% SYNTAX
%   records = emptyRegionRecords()
%**************************************************************************
% PURPOSE
%   - Return a typed empty region-record array.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - records (empty struct column)
%**************************************************************************
% UNITS
%   - Coordinates are degrees.

records = repmat(struct( ...
    "vertices_deg", zeros(0, 2), ...
    "obstacleIndex", 0, ...
    "regionIndex", 0, ...
    "targetName", ""), 0, 1);
end
