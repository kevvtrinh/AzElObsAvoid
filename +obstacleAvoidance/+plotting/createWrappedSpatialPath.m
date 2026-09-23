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
%   - x beyond an end is moved back by whole turns. y beyond an end (a
%     pole) is mirrored back about that end and x turns by half a turn,
%     matching the planner's pole copies: on x [0 360], y [-90 90], the
%     point (10, 91) is shown at (190, 89).
%**************************************************************************
% INPUTS
%   - position_units (N-by-2 real numeric, or empty)
%       Finite continuous [x y] positions sampled along the path.
%   - intervals_units (2-by-2 real numeric)
%       Finite [xLower xUpper; yLower yUpper] display bounds.
%   - wrapAxes (1-by-2 logical, binary numeric, or string)
%       [WrapX WrapY] selection of the wrapped axes. Planner wrap modes
%       ("false", "both", "forward", "backward") are also accepted; any mode
%       other than "false" wraps that axis.
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
%   - Geometry uses the caller's coordinate units. The x interval width is
%     one azimuth turn. The y interval height is one pole-to-pole band;
%     ordinary y copies repeat after two heights.
%**************************************************************************

%% Section 1: Validate Display Inputs

validateattributes(intervals_units, {'numeric'}, {'real', 'finite', 'size', [2 2]});
if isstring(wrapAxes) || iscellstr(wrapAxes)
    wrapAxes = lower(string(wrapAxes));
    validateattributes(wrapAxes, {'string'}, {'numel', 2});
    if ~all(ismember(wrapAxes, ["false", "both", "forward", "backward"]))
        error("createWrappedSpatialPath:InvalidWrapMode", ...
            "Wrap modes must be false, both, forward, or backward.");
    end
    wrapAxes = wrapAxes ~= "false";
end
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
displayMinimum_units  = double(intervals_units(:, 1)).';
displayPosition_units = zeros(0, 2);
sourceSampleIndices   = zeros(0, 1);
if size(position_units, 1) == 1
    displayPosition_units = foldIntoDisplay(position_units, position_units, ...
        displayMinimum_units, wrapLengths_units, wrapAxes);
    sourceSampleIndices = 1;
    return
end
% A pole copy turns x by half a turn, so with y wrapping the x seam can also
% sit half a turn away: check x boundaries every half turn.
splitLengths_units = wrapLengths_units;
if wrapAxes(2)
    splitLengths_units(1) = wrapLengths_units(1) / 2;
end
pieceStarts_units    = zeros(0, 2);
pieceEnds_units      = zeros(0, 2);
pieceMidpoints_units = zeros(0, 2);
pieceSampleIndices   = zeros(0, 1);
for sampleIndex = 2:size(position_units, 1)
    segmentStart_units        = position_units(sampleIndex - 1, :);
    segmentDisplacement_units = position_units(sampleIndex, :) - segmentStart_units;
    splitFractions            = [0; 1];

    % Locate every crossed boundary, including multiple turns in one step.
    % Each crossing becomes a fraction along this original path segment.
    for axisIndex = find((wrapAxes | [wrapAxes(2), false]) & (segmentDisplacement_units ~= 0))
        wrapLength_units   = splitLengths_units(axisIndex);
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

    % Keep each piece's unwrapped ends, midpoint and sample. The pieces are
    % folded below, once every piece knows which copy it lies in.
    for pieceIndex = 1:numel(splitFractions) - 1
        pieceFractions                   = splitFractions(pieceIndex:pieceIndex + 1);
        pieceStarts_units(end + 1, :)    = segmentStart_units + pieceFractions(1) * segmentDisplacement_units; %#ok<AGROW>
        pieceEnds_units(end + 1, :)      = segmentStart_units + pieceFractions(2) * segmentDisplacement_units; %#ok<AGROW>
        pieceMidpoints_units(end + 1, :) = segmentStart_units + mean(pieceFractions) * segmentDisplacement_units; %#ok<AGROW>
        pieceSampleIndices(end + 1, 1)   = sampleIndex; %#ok<AGROW>
    end
end

% The midpoint decides which copy a piece lies in; it keeps boundary points
% on the correct side. A midpoint exactly on a boundary line cannot decide
% that coordinate on its own (a wait at the pole, a run along it, or a wait
% on the x seam), so that coordinate is taken from the nearest piece that
% can decide it: the previous one, or the next one at the start of the
% path. Only the undecided coordinate is borrowed, so a wait on the x seam
% keeps its own y band. Examples: (10, 89) -> (10, 90) -> (10, 90) ->
% (10, 89) stays one run at x = 10 instead of jumping to 190;
% (359, 89) -> (360, 89) -> (360, 91) still crosses the pole once.
referencePoints_units  = pieceMidpoints_units;
boundaryOffsets        = (pieceMidpoints_units - displayMinimum_units) ./ splitLengths_units;
coordinateIsUndecided  = (wrapAxes | [wrapAxes(2), false]) & boundaryOffsets == round(boundaryOffsets);
pieceCount = size(pieceMidpoints_units, 1);
for axisIndex = 1:2
    undecided = coordinateIsUndecided(:, axisIndex);
    % Two running passes find, for every piece, the last deciding piece
    % before it and the first one after it, so a long wait costs one sweep.
    decidedIndex    = (1:pieceCount).' .* ~undecided;
    previousDecided = [0; cummax(decidedIndex(1:end - 1))];
    laterIndex      = (1:pieceCount).';
    laterIndex(undecided) = Inf;
    nextDecided     = [flip(cummin(flip(laterIndex(2:end)))); Inf];
    neighborPiece   = previousDecided;
    neighborPiece(neighborPiece == 0) = nextDecided(neighborPiece == 0);
    borrows = undecided & isfinite(neighborPiece);
    referencePoints_units(borrows, axisIndex) = pieceMidpoints_units(neighborPiece(borrows), axisIndex);
end

for pieceIndex = 1:numel(pieceSampleIndices)
    pieceVertices_units = foldIntoDisplay( ...
        [pieceStarts_units(pieceIndex, :); pieceEnds_units(pieceIndex, :)], ...
        referencePoints_units(pieceIndex, :), displayMinimum_units, wrapLengths_units, wrapAxes);
    sampleIndex         = pieceSampleIndices(pieceIndex);

    % The first piece decides how a starting boundary sample is shown.
    % From y = 90 toward y = 89, show (10, 90), even if a later piece
    % crosses a pole or the first segment spans several pole bands.
    if isempty(displayPosition_units)
        displayPosition_units = pieceVertices_units(1, :);
        sourceSampleIndices   = 1;
    end

    % Break the plotted line only when the next piece starts elsewhere
    % in the display, such as a jump from the right boundary to the left.
    pieceStartsNewRun = any(displayPosition_units(end, :) ~= pieceVertices_units(1, :));
    if pieceStartsNewRun
        displayPosition_units(end + (1:2), :) = [NaN NaN; pieceVertices_units(1, :)];
        sourceSampleIndices(end + (1:2), 1)   = sampleIndex;
    end
    displayPosition_units(end + 1, :) = pieceVertices_units(2, :); %#ok<AGROW>
    sourceSampleIndices(end + 1, 1)   = sampleIndex; %#ok<AGROW>
end
end

%% Section 3: Local Functions

function vertices_units = foldIntoDisplay(vertices_units, reference_units, ...
        displayMinimum_units, wrapLengths_units, wrapAxes)
    % Move points into the display intervals, using the copy that holds the
    % reference point. y moves back by an even number of heights, or for an
    % odd number (a pole copy) is mirrored back, y -> 2 x ymin +
    % (count + 1) x height - y, while x turns by half a turn. x then moves
    % back by whole turns: [360 370] shows at [0 10]. On x [0 360],
    % y [-90 90], (10, 91) has y count 1 and is shown at (190, 89). When y
    % wraps, x repeats every turn even if x does not wrap.
    yWrapCount       = floor((reference_units(2) - displayMinimum_units(2)) / wrapLengths_units(2));
    isPoleCopy       = wrapAxes(2) && mod(yWrapCount, 2) == 1;
    xReference_units = reference_units(1);
    if wrapAxes(2)
        if isPoleCopy
            vertices_units(:, 2) = 2 * displayMinimum_units(2) + ...
                (yWrapCount + 1) * wrapLengths_units(2) - vertices_units(:, 2);
            vertices_units(:, 1) = vertices_units(:, 1) - wrapLengths_units(1) / 2;
            xReference_units     = xReference_units - wrapLengths_units(1) / 2;
        else
            vertices_units(:, 2) = vertices_units(:, 2) - yWrapCount * wrapLengths_units(2);
        end
    end
    if any(wrapAxes)
        xWrapCount = floor((xReference_units - displayMinimum_units(1)) / wrapLengths_units(1));
        vertices_units(:, 1) = vertices_units(:, 1) - xWrapCount * wrapLengths_units(1);
    end
end
