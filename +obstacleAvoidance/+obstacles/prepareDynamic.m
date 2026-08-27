function obstacles = prepareDynamic(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles)
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

% Prepare data that many time queries reuse. For each adjacent time pair, note
% whether vertex topology matches. Matching topology means the same ordered
% vertices can move continuously. Otherwise, create a conservative interval
% shape that covers the topology change.

% Build one immutable cache record for each independently timed obstacle.
for obstacleIndex = 1:numel(obstacles)
    % A history with N samples has N-1 time intervals. Separate cell arrays
    % retain either interpolation deltas or conservative unions for each one.
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
    % Bounds use [minimum azimuth, maximum azimuth, minimum elevation,
    % maximum elevation]. Starting from infinities makes the first finite
    % vertex establish each bound naturally.

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
        sampleShapes{sampleIndex} = obstacleAvoidance.geometry.boundaryToShape( azimuth_deg, elevation_deg);
    end
    intervalDuration_s = diff(obstacle.time_s(:));
    % Input normalization guarantees strictly increasing times, so every
    % duration is positive before it is used to convert displacement to speed.

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
            % hypot computes each corresponding vertex's Euclidean angular
            % travel. The largest speed is a safe bound for how quickly any
            % part of this linearly interpolated boundary can move.
            speed_deg_s = hypot( ...
                deltaAzimuth_deg{intervalIndex}(finiteVertex), ...
                deltaElevation_deg{intervalIndex}(finiteVertex)) / intervalDuration_s(intervalIndex);
            intervalSpeedBound_deg_s(intervalIndex) = max([0; speed_deg_s]);
        else
            % Different separators or vertex counts make correspondence
            % ambiguous. The endpoint union is conservative and reports an
            % infinite source speed at adjacent samples to force safe splitting.
            intervalUnionShapes{intervalIndex} = union( sampleShapes{intervalIndex}, sampleShapes{intervalIndex + 1});
            % The union occupies every point covered at either endpoint. It may
            % be conservative between samples, but it never invents a vertex
            % pairing that could make occupied space disappear.
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
        % A sample can border the interval before it, after it, or both. Filter
        % the candidate indices so endpoint samples remain valid.
        adjacentInterval = [sampleIndex - 1, sampleIndex];
        adjacentInterval = adjacentInterval( adjacentInterval >= 1 & adjacentInterval <= intervalCount);
        if any(~matchingTopology(adjacentInterval))
            % Infinite speed signals that ordinary motion-based certification
            % cannot bridge a topology change and must split at the source time.
            sampleSpeedBound_deg_s(sampleIndex) = Inf;
        elseif ~isempty(adjacentInterval)
            sampleSpeedBound_deg_s(sampleIndex) = max( intervalSpeedBound_deg_s(adjacentInterval));
        end
    end
    % This cached information is derived only from protected geometry. All
    % later query paths reuse it so collision checking and plotting observe
    % the same interpolation and topology decisions.
    preparation = struct( ...
        "HistoryBounds_deg", historyBounds_deg, ...
        "SampleShapes", {sampleShapes}, ...
        "IntervalUnionShapes", {intervalUnionShapes}, ...
        "IntervalUnionAzimuth_deg", {intervalUnionAzimuth_deg}, ...
        "IntervalUnionElevation_deg", {intervalUnionElevation_deg}, ...
        "DeltaAzimuth_deg", {deltaAzimuth_deg}, ...
        "DeltaElevation_deg", {deltaElevation_deg}, ...
        "MatchingTopology", matchingTopology, ...
        "IntervalSpeedBound_deg_s", intervalSpeedBound_deg_s, ...
        "SampleSpeedBound_deg_s", sampleSpeedBound_deg_s, ...
        "IsTimeInvariant", false, ...
        "StaticShape", polyshape(), ...
        "StaticGeometry", struct());
    obstacles(obstacleIndex).InternalPreparation = preparation;

    % Identical corresponding vertices prove that every in-range query has
    % the same geometry. Build its complete record once through the ordinary
    % query path, then enable the shortcut only after both outputs exist.
    isTimeInvariant = sampleCount > 0 && (sampleCount == 1 || ...
        (all(matchingTopology) && all(intervalSpeedBound_deg_s == 0)));
    if isTimeInvariant
        [staticShape, staticGeometry] = ...
            obstacleAvoidance.obstacles.shapeAtTime( ...
            obstacles(obstacleIndex), obstacle.time_s(1));
        preparation.StaticShape = staticShape;
        preparation.StaticGeometry = staticGeometry;
        preparation.IsTimeInvariant = true;
        obstacles(obstacleIndex).InternalPreparation = preparation;
    end
end
end
