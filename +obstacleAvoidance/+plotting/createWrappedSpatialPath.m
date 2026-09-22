function [displayPosition_units, sourceSampleIndices] = createWrappedSpatialPath( ...
    position_units, intervals_units, wrapAxes)
%% Section 0: Header & Readme
% SYNTAX
%   [displayPosition_units, sourceSampleIndices] = ...
%       obstacleAvoidance.plotting.createWrappedSpatialPath(position_units, intervals_units, wrapAxes)
%**************************************************************************
% PURPOSE
%   - Prepare a continuous x/y path for plotting without lines across
%     wrapped boundaries.
%**************************************************************************
% INPUTS
%   - position_units (N-by-2 real numeric, or empty)
%       Finite continuous [x y] positions sampled along the path.
%   - intervals_units (2-by-2 real numeric)
%       Finite [xLower xUpper; yLower yUpper] display bounds.
%   - wrapAxes (1-by-2 logical or binary numeric)
%       [WrapX WrapY] selection of the wrapped axes.
%**************************************************************************
% OUTPUTS
%   - displayPosition_units (M-by-2 real numeric)
%       Wrapped path with NaN rows separating seam crossings.
%   - sourceSampleIndices (M-by-1 real numeric)
%       Original sample index for each output row. New seam points and NaN
%       rows use the segment's destination sample index, allowing plots to
%       attach a time or value to each row. Invalid input throws an error.
%**************************************************************************
% UNITS
%   - Geometry uses the caller's coordinate units. Each interval width is
%     the distance for one complete wrap.
%**************************************************************************

%% Section 1: Validate Display Inputs

validateattributes(intervals_units, {'numeric'}, {'real', 'finite', 'size', [2 2]});
validateattributes(wrapAxes, {'numeric', 'logical'}, {'real', 'finite', 'vector', 'numel', 2, 'binary'});
wrapLengths_units = diff(double(intervals_units), 1, 2).';
if any(wrapLengths_units <= 0 | ~isfinite(wrapLengths_units))
    error("createWrappedSpatialPath:InvalidIntervals", "Each display interval must have a positive finite width.");
end
wrapAxes = logical(wrapAxes(:).');
if isempty(position_units)
    displayPosition_units = zeros(0, 2);
    sourceSampleIndices   = zeros(0, 1);
    return
end
validateattributes(position_units, {'numeric'}, {'real', 'finite', '2d', 'ncols', 2});
position_units = double(position_units);
if ~any(wrapAxes)
    displayPosition_units = position_units;
    sourceSampleIndices   = (1:size(position_units, 1)).';
    return
end

%% Section 2: Split The Path At Each Wrapped Boundary

% On a [0 360] axis, draw 350 -> 370 as 350 -> 360, then 0 -> 10.
% A NaN row stops the plot from joining 360 back to 0 across the figure.
displayMinimum_units       = double(intervals_units(:, 1)).';
firstDisplayPosition_units = position_units(1, :);
firstDisplayPosition_units(wrapAxes) = displayMinimum_units(wrapAxes) + ...
    mod(firstDisplayPosition_units(wrapAxes) - displayMinimum_units(wrapAxes), wrapLengths_units(wrapAxes));
displayPosition_units = firstDisplayPosition_units;
sourceSampleIndices   = 1;
for sampleIndex = 2:size(position_units, 1)
    segmentStart_units        = position_units(sampleIndex - 1, :);
    segmentDisplacement_units = position_units(sampleIndex, :) - segmentStart_units;
    splitFractions            = [0; 1];

    % Locate every crossed boundary, including multiple turns in one step.
    % Each crossing becomes a fraction along this original path segment.
    for axisIndex = find(wrapAxes & (segmentDisplacement_units ~= 0))
        wrapLength_units   = wrapLengths_units(axisIndex);
        endpointWrapCounts = (position_units(sampleIndex - 1:sampleIndex, axisIndex) - ...
            displayMinimum_units(axisIndex)) / wrapLength_units;
        seamIndices          = (ceil(min(endpointWrapCounts)):floor(max(endpointWrapCounts))).';
        seamCrossingFractions = (displayMinimum_units(axisIndex) + wrapLength_units * seamIndices - ...
            segmentStart_units(axisIndex)) / segmentDisplacement_units(axisIndex);
        splitFractions = [splitFractions; ...
            seamCrossingFractions(seamCrossingFractions > 0 & seamCrossingFractions < 1)]; %#ok<AGROW>
    end

    % Sort crossings from both axes and keep a simultaneous x/y crossing once.
    splitFractions            = unique(splitFractions);

    for pieceIndex = 1:numel(splitFractions) - 1
        pieceFractions      = splitFractions(pieceIndex:pieceIndex + 1);
        pieceVertices_units = segmentStart_units + pieceFractions * segmentDisplacement_units;

        % The midpoint determines how many complete wraps to remove.
        % For [360 370], midpoint = 365 gives one wrap: display [0 10].
        % Using the midpoint keeps boundary points on the correct side.
        midpoint_units = segmentStart_units + mean(pieceFractions) * segmentDisplacement_units;
        wrapCounts     = floor((midpoint_units(wrapAxes) - displayMinimum_units(wrapAxes)) ./ ...
            wrapLengths_units(wrapAxes));
        pieceVertices_units(:, wrapAxes) = pieceVertices_units(:, wrapAxes) - ...
            wrapCounts .* wrapLengths_units(wrapAxes);

        % Break the plotted line only when the next piece starts elsewhere
        % in the display, such as a jump from the right boundary to the left.
        pieceStartsNewRun = any(displayPosition_units(end, :) ~= pieceVertices_units(1, :));
        if pieceStartsNewRun
            displayPosition_units(end + (1:2), :) = [NaN NaN; pieceVertices_units(1, :)];
            sourceSampleIndices(end + (1:2), 1)   = sampleIndex; %#ok<AGROW>
        end
        displayPosition_units(end + 1, :) = pieceVertices_units(2, :); %#ok<AGROW>
        sourceSampleIndices(end + 1, 1)   = sampleIndex; %#ok<AGROW>
    end
end
end
