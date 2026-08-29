function [displayPosition_deg, sourceIndex] = createWrappedSpatialPath( ...
        position_deg, azimuthInterval_deg, allowAzimuthWrapping)
%% Section 0: Header & Readme
% SYNTAX
%   [displayPosition_deg, sourceIndex] = ...
%       obstacleAvoidance.plotting.createWrappedSpatialPath( ...
%       position_deg, azimuthInterval_deg, allowAzimuthWrapping)
%**************************************************************************
% PURPOSE
%   - Map a continuous spatial path into one periodic azimuth display.
%   - Split lines at wrap seams so plots do not draw across the workspace.
%**************************************************************************
% INPUTS
%   - position_deg (N-by-2 finite numeric array)
%       Continuous unwrapped [azimuth elevation] path. Empty is accepted.
%   - azimuthInterval_deg (1-by-2 increasing finite numeric vector)
%       Lower and upper boundaries of one displayed azimuth period.
%   - allowAzimuthWrapping (scalar logical or binary numeric)
%       False returns the input path unchanged.
%**************************************************************************
% OUTPUTS
%   - displayPosition_deg (M-by-2 numeric array)
%       Wrapped path with seam endpoints and NaN separator rows.
%   - sourceIndex (M-by-1 positive integer vector)
%       Source sample associated with each row. Inserted seam rows use the
%       sample that first reaches the opposite side of the seam.
%**************************************************************************
% UNITS
%   - Position and interval values are degrees.
%**************************************************************************

%% Section 1: Validate Display Inputs

if isempty(position_deg)
    displayPosition_deg = zeros(0, 2);
    sourceIndex = zeros(0, 1);
    return;
end
validateattributes(position_deg, {'numeric'}, ...
    {'real', 'finite', '2d', 'ncols', 2});
validateattributes(azimuthInterval_deg, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2, 'increasing'});
allowAzimuthWrapping = ...
    obstacleAvoidance.input.normalizeLogicalScalar( ...
    allowAzimuthWrapping, "allowAzimuthWrapping", ...
    "createWrappedSpatialPath:InvalidLogicalOption");
position_deg = double(position_deg);
azimuthInterval_deg = reshape(double(azimuthInterval_deg), 1, 2);
sampleCount = size(position_deg, 1);
if ~allowAzimuthWrapping
    displayPosition_deg = position_deg;
    sourceIndex = (1:sampleCount).';
    return;
end

%% Section 2: Wrap And Split Seam Crossings

lowerAzimuth_deg = azimuthInterval_deg(1);
upperAzimuth_deg = azimuthInterval_deg(2);
period_deg = upperAzimuth_deg - lowerAzimuth_deg;
wrappedAzimuth_deg = lowerAzimuth_deg + ...
    mod(position_deg(:, 1) - lowerAzimuth_deg, period_deg);

% One edge can add two seam endpoints and one NaN separator. Allocate the
% exact worst case once, then trim after the final source sample.
maximumDisplayCount = 4 * sampleCount - 3;
displayPosition_deg = nan(maximumDisplayCount, 2);
sourceIndex = zeros(maximumDisplayCount, 1);
displayCount = 1;
displayPosition_deg(1, :) = [wrappedAzimuth_deg(1), position_deg(1, 2)];
sourceIndex(1) = 1;
for sampleIndex = 2:sampleCount
    previousIndex = sampleIndex - 1;
    wrappedStep_deg = wrappedAzimuth_deg(sampleIndex) - ...
        wrappedAzimuth_deg(previousIndex);
    crossesSeam = abs(wrappedStep_deg) > period_deg / 2;
    if crossesSeam
        unwrappedStart_deg = position_deg(previousIndex, 1);
        unwrappedStep_deg = position_deg(sampleIndex, 1) - ...
            unwrappedStart_deg;
        if unwrappedStep_deg > 0
            seamAzimuth_deg = upperAzimuth_deg + period_deg * floor( ...
                (unwrappedStart_deg - lowerAzimuth_deg) / period_deg);
            departingBoundary_deg = upperAzimuth_deg;
            arrivingBoundary_deg = lowerAzimuth_deg;
        else
            seamAzimuth_deg = lowerAzimuth_deg + period_deg * floor( ...
                (unwrappedStart_deg - lowerAzimuth_deg) / period_deg);
            departingBoundary_deg = lowerAzimuth_deg;
            arrivingBoundary_deg = upperAzimuth_deg;
        end
        seamFraction = (seamAzimuth_deg - unwrappedStart_deg) / ...
            unwrappedStep_deg;
        seamElevation_deg = position_deg(previousIndex, 2) + ...
            seamFraction * (position_deg(sampleIndex, 2) - ...
            position_deg(previousIndex, 2));
        displayPosition_deg(displayCount + (1:3), :) = [ ...
            departingBoundary_deg, seamElevation_deg; ...
            NaN, NaN; ...
            arrivingBoundary_deg, seamElevation_deg];
        sourceIndex(displayCount + (1:3)) = sampleIndex;
        displayCount = displayCount + 3;
    end
    displayCount = displayCount + 1;
    displayPosition_deg(displayCount, :) = [ ...
        wrappedAzimuth_deg(sampleIndex), position_deg(sampleIndex, 2)];
    sourceIndex(displayCount) = sampleIndex;
end
displayPosition_deg = displayPosition_deg(1:displayCount, :);
sourceIndex = sourceIndex(1:displayCount);
end
