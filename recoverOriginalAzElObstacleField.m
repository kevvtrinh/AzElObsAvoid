function [originalObstacleField, safetyMargin_deg, hasStoredOriginal] = ...
        recoverOriginalAzElObstacleField(obstacleField)
%% Section 0: Header & Readme
% PURPOSE
%   - Recover the original uninflated field stored inside packed source data.
%   - Let plotting show original and final polygons without caller options.
%**************************************************************************
originalObstacleField = obstacleField;
safetyMargin_deg = 0;
hasStoredOriginal = false;
if ~isstruct(obstacleField) || ~isscalar(obstacleField) || ...
        ~isfield(obstacleField, "SourceAzElData") || ...
        isempty(obstacleField.SourceAzElData)
    return;
end
sourceAzElData = obstacleField.SourceAzElData;
if ~all(isfield(sourceAzElData, { ...
        'originalAz_deg', 'originalEl_deg', 'safetyMargin_deg'}))
    return;
end
safetyMargin_deg = max(reshape( ...
    [sourceAzElData.safetyMargin_deg], [], 1));
if safetyMargin_deg <= 0
    return;
end
originalAzElData = sourceAzElData;
for obstacleIndex = 1:numel(originalAzElData)
    originalAzElData(obstacleIndex).az_deg = ...
        originalAzElData(obstacleIndex).originalAz_deg;
    originalAzElData(obstacleIndex).el_deg = ...
        originalAzElData(obstacleIndex).originalEl_deg;
    originalAzElData(obstacleIndex).safetyMargin_deg = 0;
    originalAzElData(obstacleIndex) = ...
        normalizeAzElTimeObstacleData(originalAzElData(obstacleIndex));
end
buildOptions = obstacleField.Options;
originalObstacleField = buildAzElTimeObstacleField( ...
    originalAzElData, buildOptions);
hasStoredOriginal = true;
end
