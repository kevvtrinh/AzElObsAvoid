function [displayPosition_units, sourceIndex] = createWrappedSpatialPath(position_units, intervals_units, wrapAxes)
%% Section 0: Header & Readme
% SYNTAX
%   [displayPosition_units, sourceIndex] = ...
%       obstacleAvoidance.plotting.createWrappedSpatialPath( ...
%       position_units, intervals_units, wrapAxes)
% PURPOSE
%   Display a continuous x/y path without drawing across periodic seams.
% INPUTS
%   position_units: N-by-2 finite continuous [x y] positions, or empty.
%   intervals_units: 2-by-2 [xLower xUpper; yLower yUpper] finite bounds.
%   wrapAxes: two logical or binary numeric values [WrapX WrapY].
% OUTPUTS
%   displayPosition_units: wrapped path with NaN rows separating seams.
%   sourceIndex: source row associated with each output row. Interior seam
%   points use the destination row of their original path segment.
% UNITS
%   Caller-consistent coordinate units; periods are interval widths.

%% Section 1: Validate Display Inputs
validateattributes(intervals_units, {'numeric'}, {'real', 'finite', 'size', [2 2]});
validateattributes(wrapAxes, {'numeric', 'logical'}, {'real', 'finite', 'vector', 'numel', 2, 'binary'});
periods_units = diff(double(intervals_units), 1, 2).';
if any(periods_units <= 0 | ~isfinite(periods_units))
    error("createWrappedSpatialPath:InvalidIntervals", "Each display interval must have a positive finite width.");
end
wrapAxes = logical(wrapAxes(:).');
if isempty(position_units)
    displayPosition_units = zeros(0, 2);
    sourceIndex = zeros(0, 1);
    return;
end
validateattributes(position_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2});
position_units = double(position_units);
if ~any(wrapAxes)
    displayPosition_units = position_units;
    sourceIndex = (1:size(position_units, 1)).';
    return;
end

%% Section 2: Split Every Crossed Seam On Either Axis
lower_units = double(intervals_units(:, 1)).';
first_units = position_units(1, :);
first_units(wrapAxes) = lower_units(wrapAxes) + mod(first_units(wrapAxes) - lower_units(wrapAxes), periods_units(wrapAxes));
displayPosition_units = first_units;
sourceIndex = 1;
for sampleIndex = 2:size(position_units, 1)
    start_units = position_units(sampleIndex - 1, :);
    step_units = position_units(sampleIndex, :) - start_units;
    fractions = [0; 1];
    for axisIndex = find(wrapAxes & step_units ~= 0)
        endpoints = (position_units(sampleIndex - 1:sampleIndex, axisIndex) - lower_units(axisIndex)) / periods_units(axisIndex);
        seamIndices = (ceil(min(endpoints)):floor(max(endpoints))).';
        crossings = (lower_units(axisIndex) + periods_units(axisIndex) * seamIndices - start_units(axisIndex)) / step_units(axisIndex);
        fractions = [fractions; crossings(crossings > 0 & crossings < 1)]; %#ok<AGROW>
    end
    fractions = unique(fractions);
    for pieceIndex = 1:numel(fractions) - 1
        piece = start_units + fractions(pieceIndex:pieceIndex + 1) * step_units;
        % The midpoint identifies this piece's periodic cell. Its endpoints
        % then lie on the correct sides of any crossed seams.
        midpoint_units = start_units + mean(fractions(pieceIndex:pieceIndex + 1)) * step_units;
        cellIndex = floor((midpoint_units(wrapAxes) - lower_units(wrapAxes)) ./ periods_units(wrapAxes));
        piece(:, wrapAxes) = piece(:, wrapAxes) - cellIndex .* periods_units(wrapAxes);
        if any(displayPosition_units(end, :) ~= piece(1, :))
            displayPosition_units(end + (1:2), :) = [NaN NaN; piece(1, :)];
            sourceIndex(end + (1:2), 1) = sampleIndex; %#ok<AGROW>
        end
        displayPosition_units(end + 1, :) = piece(2, :); %#ok<AGROW>
        sourceIndex(end + 1, 1) = sampleIndex; %#ok<AGROW>
    end
end
end
