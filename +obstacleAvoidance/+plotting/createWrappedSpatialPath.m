function [displayPosition_deg, sourceIndex] = createWrappedSpatialPath( ...
        position_deg, azimuthInterval_deg, allowAzimuthWrapping)
%% Section 0: Header & Readme
% SYNTAX
%   [displayPosition_deg, sourceIndex] = ...
%       obstacleAvoidance.plotting.createWrappedSpatialPath( ...
%       position_deg, azimuthInterval_deg, allowAzimuthWrapping)
%**************************************************************************
% PURPOSE
%   - Wrap continuous azimuth paths without drawing across a periodic seam.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 finite numeric array)
%       Continuous [azimuth elevation] path; empty is accepted.
%   - azimuthInterval_deg (two-element increasing numeric vector)
%       Boundaries of the displayed azimuth period in degrees.
%   - allowAzimuthWrapping (scalar logical or binary numeric)
%       False preserves the input path.
%**************************************************************************
% OUTPUTS
%   - displayPosition_deg (M-by-2 numeric array)
%       Wrapped path with seam endpoints separated by NaN rows.
%   - sourceIndex (M-by-1 integer vector)
%       Source row associated with each output row.
%**************************************************************************
% UNITS
%   - Positions and the azimuth interval are degrees.
%**************************************************************************

%% Section 1: Validate Inputs

if isempty(position_deg)
    displayPosition_deg = zeros(0, 2);
    sourceIndex = zeros(0, 1);
    return;
end
validateattributes(position_deg, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2});
validateattributes(azimuthInterval_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2, 'increasing'});
allowAzimuthWrapping = obstacleAvoidance.input.normalizeLogicalScalar( ...
    allowAzimuthWrapping, "allowAzimuthWrapping", ...
    "createWrappedSpatialPath:InvalidLogicalOption");
position_deg = double(position_deg);
sampleCount = size(position_deg, 1);
if ~allowAzimuthWrapping
    displayPosition_deg = position_deg;
    sourceIndex = (1:sampleCount).';
    return;
end

%% Section 2: Split Seam Crossings

azimuthInterval_deg = reshape(double(azimuthInterval_deg), 1, 2);
lowerAzimuth_deg = azimuthInterval_deg(1);
period_deg = diff(azimuthInterval_deg);
wrappedAzimuth_deg = lowerAzimuth_deg + mod(position_deg(:, 1) - lowerAzimuth_deg, period_deg);
displayPosition_deg = nan(4 * sampleCount - 3, 2);
sourceIndex = zeros(4 * sampleCount - 3, 1);
displayCount = 1;
displayPosition_deg(1, :) = [wrappedAzimuth_deg(1), position_deg(1, 2)];
sourceIndex(1) = 1;
for sampleIndex = 2:sampleCount
    previousIndex = sampleIndex - 1;
    wrappedStep_deg = wrappedAzimuth_deg(sampleIndex) - wrappedAzimuth_deg(previousIndex);
    if abs(wrappedStep_deg) > period_deg / 2
        startAzimuth_deg = position_deg(previousIndex, 1);
        stepAzimuth_deg = position_deg(sampleIndex, 1) - startAzimuth_deg;
        isIncreasing = stepAzimuth_deg > 0;
        seamAzimuth_deg = lowerAzimuth_deg + period_deg * floor( ...
            (startAzimuth_deg - lowerAzimuth_deg) / period_deg);
        seamAzimuth_deg = seamAzimuth_deg + period_deg * isIncreasing;
        fraction = (seamAzimuth_deg - startAzimuth_deg) / stepAzimuth_deg;
        seamElevation_deg = position_deg(previousIndex, 2) + fraction * ...
            diff(position_deg(previousIndex:sampleIndex, 2));
        boundaries_deg = lowerAzimuth_deg + period_deg * [isIncreasing, ~isIncreasing];
        displayPosition_deg(displayCount + (1:3), :) = [ ...
            boundaries_deg(1), seamElevation_deg; NaN, NaN; ...
            boundaries_deg(2), seamElevation_deg];
        sourceIndex(displayCount + (1:3)) = sampleIndex;
        displayCount = displayCount + 3;
    end
    displayCount = displayCount + 1;
    displayPosition_deg(displayCount, :) = ...
        [wrappedAzimuth_deg(sampleIndex), position_deg(sampleIndex, 2)];
    sourceIndex(displayCount) = sampleIndex;
end
displayPosition_deg = displayPosition_deg(1:displayCount, :);
sourceIndex = sourceIndex(1:displayCount);
end
