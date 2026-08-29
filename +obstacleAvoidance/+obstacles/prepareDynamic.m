function obstacles = prepareDynamic(obstacles)
%% Section 0: Header & Readme
% SYNTAX
%   obstacles = obstacleAvoidance.obstacles.prepareDynamic(obstacles)
%**************************************************************************
% PURPOSE
%   - Cache exact sample shapes and dynamic interpolation metadata.
%   - Use a conservative endpoint union when adjacent topology differs.
%**************************************************************************
% INPUTS
%   - obstacles (canonical obstacle struct array)
%       Protected histories remain unchanged and authoritative.
%**************************************************************************
% OUTPUTS
%   - obstacles (prepared obstacle struct array)
%       Each record adds reusable shapes, bounds, deltas, and speed bounds.
%**************************************************************************
% UNITS
%   - Geometry is degrees, time is seconds, and speed is degrees per second.
%**************************************************************************

%% Section 1: Prepare Each Complete History

if isempty(obstacles) || isfield(obstacles, "InternalPreparation")
    return;
end
for obstacleIndex = 1:numel(obstacles)
    obstacle = obstacles(obstacleIndex);
    sampleCount = numel(obstacle.time_s);
    intervalCount = max(0, sampleCount - 1);
    sampleShapes = cell(sampleCount, 1);
    unionShapes = cell(intervalCount, 1);
    deltaAzimuth_deg = cell(intervalCount, 1);
    deltaElevation_deg = cell(intervalCount, 1);
    matchingTopology = false(intervalCount, 1);
    intervalSpeed_deg_s = Inf(intervalCount, 1);
    historyBounds_deg = [Inf -Inf Inf -Inf];
    for sampleIndex = 1:sampleCount
        azimuth_deg = double(obstacle.az_deg{sampleIndex}(:));
        elevation_deg = double(obstacle.el_deg{sampleIndex}(:));
        finiteVertex = isfinite(azimuth_deg) & isfinite(elevation_deg);
        if any(finiteVertex)
            historyBounds_deg = [min(historyBounds_deg(1), min(azimuth_deg(finiteVertex))), ...
                max(historyBounds_deg(2), max(azimuth_deg(finiteVertex))), ...
                min(historyBounds_deg(3), min(elevation_deg(finiteVertex))), ...
                max(historyBounds_deg(4), max(elevation_deg(finiteVertex)))];
        end
        sampleShapes{sampleIndex} = ...
            obstacleAvoidance.geometry.boundaryToShape(azimuth_deg, elevation_deg);
    end
    intervalDuration_s = diff(double(obstacle.time_s(:)));
    for intervalIndex = 1:intervalCount
        lowerAzimuth_deg = double(obstacle.az_deg{intervalIndex}(:));
        lowerElevation_deg = double(obstacle.el_deg{intervalIndex}(:));
        upperAzimuth_deg = double(obstacle.az_deg{intervalIndex + 1}(:));
        upperElevation_deg = double(obstacle.el_deg{intervalIndex + 1}(:));
        lowerFinite = isfinite([lowerAzimuth_deg, lowerElevation_deg]);
        upperFinite = isfinite([upperAzimuth_deg, upperElevation_deg]);
        matchingTopology(intervalIndex) = isequal(size(lowerFinite), size(upperFinite)) && ...
            isequal(lowerFinite, upperFinite);
        if matchingTopology(intervalIndex)
            deltaAzimuth_deg{intervalIndex} = upperAzimuth_deg - lowerAzimuth_deg;
            deltaElevation_deg{intervalIndex} = upperElevation_deg - lowerElevation_deg;
            finiteVertex = all(lowerFinite, 2);
            speed_deg_s = hypot(deltaAzimuth_deg{intervalIndex}(finiteVertex), ...
                deltaElevation_deg{intervalIndex}(finiteVertex)) / ...
                intervalDuration_s(intervalIndex);
            intervalSpeed_deg_s(intervalIndex) = max([0; speed_deg_s]);
        else
            unionShapes{intervalIndex} = union( ...
                sampleShapes{intervalIndex}, sampleShapes{intervalIndex + 1});
        end
    end
    sampleSpeed_deg_s = zeros(sampleCount, 1);
    for intervalIndex = 1:intervalCount
        sampleSpeed_deg_s(intervalIndex) = max( ...
            sampleSpeed_deg_s(intervalIndex), intervalSpeed_deg_s(intervalIndex));
        sampleSpeed_deg_s(intervalIndex + 1) = max( ...
            sampleSpeed_deg_s(intervalIndex + 1), intervalSpeed_deg_s(intervalIndex));
    end
    isTimeInvariant = sampleCount > 0 && (sampleCount == 1 || all(intervalSpeed_deg_s == 0));
    staticShape = polyshape();
    if isTimeInvariant
        staticShape = sampleShapes{1};
    end
    preparation = struct("HistoryBounds_deg", historyBounds_deg, "SampleShapes", {sampleShapes}, ...
        "IntervalUnionShapes", {unionShapes}, "DeltaAzimuth_deg", {deltaAzimuth_deg}, ...
        "DeltaElevation_deg", {deltaElevation_deg}, "MatchingTopology", matchingTopology, ...
        "IntervalSpeedBound_deg_s", intervalSpeed_deg_s, "SelectedEdgeQueryIsExact", false, ...
        "SampleSpeedBound_deg_s", sampleSpeed_deg_s, ...
        "SampleBoundaryRunBounds", {cell(sampleCount, 1)}, ...
        "IntervalUnionBoundaryRunBounds", {cell(intervalCount, 1)}, ...
        "IsTimeInvariant", isTimeInvariant, "StaticShape", staticShape);
    obstacles(obstacleIndex).InternalPreparation = preparation;
end
end
