function [originalObstacleField, safetyMargins_deg, hasStoredOriginal] = ...
        recoverOriginalAzElObstacleField(obstacleField)
%% Section 0: Header & Readme
% SYNTAX
%   [originalField, margins_deg, hasOriginal] = ...
%       recoverOriginalAzElObstacleField(obstacleField)
%**************************************************************************
% PURPOSE
%   - Recover the original uninflated field stored inside packed source data.
%   - Let plotting show original and final polygons without caller options.
%**************************************************************************
% INPUTS
%   - obstacleField (scalar struct)
%       Packed protected field created by buildAzElTimeObstacleField.
%**************************************************************************
% OUTPUTS
%   - originalObstacleField (scalar struct)
%       Packed original geometry when provenance is available; otherwise
%       the supplied field.
%   - safetyMargins_deg (N-by-1 double)
%       Construction-time margin retained for each obstacle.
%   - hasStoredOriginal (logical scalar)
%       True when a distinct original field was reconstructed.
%**************************************************************************
% UNITS
%   - Angles are degrees and time is seconds.
%**************************************************************************

%% Section 1: Validate Stored Geometry Provenance

originalObstacleField = obstacleField;
obstacleCount = 1;
if isstruct(obstacleField) && isscalar(obstacleField) && ...
        isfield(obstacleField, "Obstacles")
    obstacleCount = numel(obstacleField.Obstacles);
end
safetyMargins_deg = zeros(obstacleCount, 1);
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
safetyMargins_deg = reshape( ...
    [sourceAzElData.safetyMargin_deg], [], 1);
if isempty(safetyMargins_deg) || ~any(safetyMargins_deg > 0)
    return;
end

%% Section 2: Rebuild The Original Packed Field

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
buildOptions = struct();
if isfield(obstacleField, "ReferenceTime") && ...
        isdatetime(obstacleField.ReferenceTime)
    buildOptions.ReferenceTime = obstacleField.ReferenceTime;
end
originalObstacleField = buildAzElTimeObstacleField( ...
    originalAzElData, buildOptions);
hasStoredOriginal = true;
end
