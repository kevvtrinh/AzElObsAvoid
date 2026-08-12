function inflatedAzElData = inflateAzElObstacleData( ...
        azElData, safetyMargin_deg)
%% Section 0: Header & Readme
% SYNTAX
%   inflatedAzElData = inflateAzElObstacleData( ...
%       azElData, safetyMargin_deg)
%**************************************************************************
% PURPOSE
%   - Apply a Euclidean safety margin to every az/el polygon slice before
%     buildAzElTimeObstacleField packs the geometry.
%   - Return ordinary canonical azElData so planning, visualization, and
%     collision checks all use the same prebuilt polygon boundary.
%**************************************************************************
% INPUTS
%   - azElData (canonical obstacle struct, array, or nested cell array)
%       Original obstacle polygon histories.
%   - safetyMargin_deg (nonnegative scalar)
%       Clearance added to every polygon boundary in az/el degrees.
%**************************************************************************
% OUTPUTS
%   - inflatedAzElData (canonical obstacle struct array)
%       Safety-inflated polygon histories ready for field construction.
%**************************************************************************
% UNITS
%   - Polygon coordinates and safetyMargin_deg are degrees.

validateattributes(safetyMargin_deg, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'});
inflatedAzElData = combineAzElObstacles(azElData);
if safetyMargin_deg <= 0
    return;
end

%% Section 1: Inflate Every Finite Ring In Every Time Slice
for obstacleIndex = 1:numel(inflatedAzElData)
    obstacle = inflatedAzElData(obstacleIndex);
    obstacle.targetName = string(obstacle.targetName) + ...
        sprintf(" + %.3f deg safety margin", safetyMargin_deg);
    obstacle.safetyMargin_deg = ...
        double(obstacle.safetyMargin_deg) + safetyMargin_deg;
    for sampleIndex = 1:numel(obstacle.time_s)
        rawAzimuth_deg = double(obstacle.az_deg{sampleIndex}(:));
        rawElevation_deg = double(obstacle.el_deg{sampleIndex}(:));
        finiteVertex = isfinite(rawAzimuth_deg) & ...
            isfinite(rawElevation_deg);
        ringChanges = diff([false; finiteVertex; false]);
        ringStart = find(ringChanges == 1);
        ringStop = find(ringChanges == -1) - 1;
        inflatedAzimuth_deg = zeros(0, 1);
        inflatedElevation_deg = zeros(0, 1);
        outputRingCount = 0;
        for ringIndex = 1:numel(ringStart)
            rows = ringStart(ringIndex):ringStop(ringIndex);
            rawRegion_deg = [ ...
                rawAzimuth_deg(rows), rawElevation_deg(rows)];
            bufferedRegions_deg = inflateAzElPolygonRegion( ...
                rawRegion_deg, safetyMargin_deg);
            for bufferedRegionIndex = 1:numel(bufferedRegions_deg)
                outputRingCount = outputRingCount + 1;
                bufferedRegion_deg = ...
                    bufferedRegions_deg{bufferedRegionIndex};
                if outputRingCount > 1
                    inflatedAzimuth_deg(end + 1, 1) = NaN; %#ok<AGROW>
                    inflatedElevation_deg(end + 1, 1) = NaN; %#ok<AGROW>
                end
                inflatedAzimuth_deg = [inflatedAzimuth_deg; ...
                    bufferedRegion_deg(:, 1)]; %#ok<AGROW>
                inflatedElevation_deg = [inflatedElevation_deg; ...
                    bufferedRegion_deg(:, 2)]; %#ok<AGROW>
            end
        end
        if outputRingCount == 0
            obstacle.az_deg{sampleIndex} = zeros(0, 1);
            obstacle.el_deg{sampleIndex} = zeros(0, 1);
            continue;
        end
        obstacle.az_deg{sampleIndex} = inflatedAzimuth_deg;
        obstacle.el_deg{sampleIndex} = inflatedElevation_deg;
    end
    inflatedAzElData(obstacleIndex) = ...
        normalizeAzElTimeObstacleData(obstacle);
end
end
