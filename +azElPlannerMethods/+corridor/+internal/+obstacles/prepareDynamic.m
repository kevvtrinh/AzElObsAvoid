function obstacles = prepareDynamic(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = azElPlannerMethods.corridor.internal.obstacles.prepareDynamic(obstacles)
%**************************************************************************
% PURPOSE
%   - Precompute immutable shapes and interval metadata once per planner call.
%     Matching vertex topology is interpolated directly; topology changes use
%     a conservative interval union so no fabricated vertex correspondence can
%     create an unsafe intermediate polygon.
%**************************************************************************
% INPUTS
%   - obstacles (normalized canonical obstacle struct array)
%       Complete protected histories remain unchanged and authoritative.
%**************************************************************************
% OUTPUTS
%   - obstacles (internal prepared obstacle struct array)
%       Each record adds reusable shapes, bounds, interpolation data, and
%       one interval-local union when adjacent topology differs.
%**************************************************************************
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%**************************************************************************
if isempty(obstacles) || isfield(obstacles, "InternalPreparation")
    % Prepared records are immutable caches. Reusing them keeps planning,
    % solver, plotting, and validation queries geometrically identical.
    return;
end

%% Section 1: Prepare Each Complete Dynamic History

% Build one immutable cache record for each independently timed obstacle.
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    sampleCount = numel(obstacle.time_s);
    intervalCount = max(0, sampleCount - 1);
    sampleShapes = cell(sampleCount, 1);
    intervalUnionShapes = cell(intervalCount, 1);
    intervalUnionAzimuth_deg = cell(intervalCount, 1);
    intervalUnionElevation_deg = cell(intervalCount, 1);
    deltaAzimuth_deg = cell(intervalCount, 1);
    deltaElevation_deg = cell(intervalCount, 1);
    matchingTopology = false(intervalCount, 1);
    intervalSpeedBound_deg_s = Inf(intervalCount, 1);
    historyBounds_deg = [Inf -Inf Inf -Inf];

    % Convert every source slice once and accumulate a cheap whole-history bound.
    for sampleIndex = 1:sampleCount
        % Cache exact source-time shapes and one broad history bound used by
        % inexpensive distance rejection before detailed polygon work.
        azimuth_deg = obstacle.az_deg{sampleIndex}(:);
        elevation_deg = obstacle.el_deg{sampleIndex}(:);
        finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
        if any(finiteVertex)
            historyBounds_deg = [ ...
                min(historyBounds_deg(1), min(azimuth_deg(finiteVertex))), ...
                max(historyBounds_deg(2), max(azimuth_deg(finiteVertex))), ...
                min(historyBounds_deg(3), min(elevation_deg(finiteVertex))), ...
                max(historyBounds_deg(4), max(elevation_deg(finiteVertex)))];
        end
        sampleShapes{sampleIndex} = azElPlannerMethods.corridor.internal.geometry.boundaryShape( azimuth_deg, elevation_deg);
    end
    intervalDuration_s = diff(obstacle.time_s(:));

    % Classify each adjacent pair as safely interpolable or conservatively unioned.
    for intervalIndex = 1:intervalCount
        lowerAzimuth_deg = obstacle.az_deg{intervalIndex}(:);
        lowerElevation_deg = obstacle.el_deg{intervalIndex}(:);
        upperAzimuth_deg = obstacle.az_deg{intervalIndex + 1}(:);
        upperElevation_deg = obstacle.el_deg{intervalIndex + 1}(:);
        matchingTopology(intervalIndex) = numel(lowerAzimuth_deg) == numel(upperAzimuth_deg) && ...
            isequal(isfinite(lowerAzimuth_deg), isfinite(upperAzimuth_deg)) && ...
            isequal(isfinite(lowerElevation_deg), isfinite(upperElevation_deg));
        if matchingTopology(intervalIndex)
            % Paired vertices describe the same rings, so linear coordinate
            % interpolation has a defined physical meaning within the interval.
            deltaAzimuth_deg{intervalIndex} = upperAzimuth_deg - lowerAzimuth_deg;
            deltaElevation_deg{intervalIndex} = upperElevation_deg - lowerElevation_deg;
            finiteVertex = isfinite(lowerAzimuth_deg) & isfinite(lowerElevation_deg);
            speed_deg_s = hypot( ...
                deltaAzimuth_deg{intervalIndex}(finiteVertex), ...
                deltaElevation_deg{intervalIndex}(finiteVertex)) / intervalDuration_s(intervalIndex);
            intervalSpeedBound_deg_s(intervalIndex) = max([0; speed_deg_s]);
        else
            % Different separators or vertex counts make correspondence
            % ambiguous. The endpoint union is conservative and reports an
            % infinite source speed at adjacent samples to force safe splitting.
            intervalUnionShapes{intervalIndex} = union( sampleShapes{intervalIndex}, sampleShapes{intervalIndex + 1});
            [intervalUnionAzimuth_deg{intervalIndex}, ...
                intervalUnionElevation_deg{intervalIndex}] = boundary( intervalUnionShapes{intervalIndex});
        end
    end
    sampleSpeedBound_deg_s = zeros(sampleCount, 1);

    % A sample inherits the worst adjacent interval speed. Continuous
    % collision validation uses this bound when deciding whether an interval
    % can be certified without further subdivision.
    % Visit every source sample because the first and last have only one adjacent interval.
    for sampleIndex = 1:sampleCount
        adjacentInterval = [sampleIndex - 1, sampleIndex];
        adjacentInterval = adjacentInterval( adjacentInterval >= 1 & adjacentInterval <= intervalCount);
        if any(~matchingTopology(adjacentInterval))
            sampleSpeedBound_deg_s(sampleIndex) = Inf;
        elseif ~isempty(adjacentInterval)
            sampleSpeedBound_deg_s(sampleIndex) = max( intervalSpeedBound_deg_s(adjacentInterval));
        end
    end
    obstacles(obstacleIndex).InternalPreparation = struct( ...
        "HistoryBounds_deg", historyBounds_deg, ...
        "SampleShapes", {sampleShapes}, ...
        "IntervalUnionShapes", {intervalUnionShapes}, ...
        "IntervalUnionAzimuth_deg", {intervalUnionAzimuth_deg}, ...
        "IntervalUnionElevation_deg", {intervalUnionElevation_deg}, ...
        "DeltaAzimuth_deg", {deltaAzimuth_deg}, ...
        "DeltaElevation_deg", {deltaElevation_deg}, ...
        "MatchingTopology", matchingTopology, ...
        "IntervalSpeedBound_deg_s", intervalSpeedBound_deg_s, "SampleSpeedBound_deg_s", sampleSpeedBound_deg_s);
end
end
