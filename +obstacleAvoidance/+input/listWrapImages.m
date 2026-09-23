function images = listWrapImages(bounds_units, intervals_units, wrapModes, planningRange_units)
%% Section 0: Header & Readme
% SYNTAX
%   images = obstacleAvoidance.input.listWrapImages( ...
%       bounds_units, intervals_units, wrapModes, planningRange_units)
%**************************************************************************
% PURPOSE
%   - List the copies of a point or shape across wrapped interval ends.
%     Bounds on copied axes must overlap the planning range; an axis that
%     does not wrap keeps its source coordinate without a range check.
%   - x is azimuth: copies are shifted by whole turns, where one turn is the
%     x interval width. Example: on [0 360], x = 10 also appears at 370.
%   - y is elevation on a sphere. Going over a y end (a pole) mirrors y about
%     that end and turns x by half a turn:
%       ordinary copy: (x + 360k,       y + 360n)
%       pole copy:     (x + 180 + 360k, 180 - y + 360n)
%     on x [0 360], y [-90 90]. Example: the pole copy of (190, 89) is
%     (10, 91), so crossing the top pole from (10, 89) to (10, 91) reaches
%     (190, 89).
%**************************************************************************
% INPUTS
%   - bounds_units (2-by-2 numeric array)
%       [xmin xmax; ymin ymax] of the point or shape in the wrapped frame.
%   - intervals_units (2-by-2 numeric array)
%       Wrapped workspace intervals [xmin xmax; ymin ymax].
%   - wrapModes (1-by-2 string array)
%       [WrapX WrapY], each "false", "both", "forward", or "backward".
%   - planningRange_units (2-by-2 numeric array)
%       Unwrapped planning range [xmin xmax; ymin ymax].
%**************************************************************************
% OUTPUTS
%   - images (scalar struct)
%       Column vectors XOffset_units, YScale, and YOffset_units, one row per
%       copy, ordered by x offset and then by y copy. A copy maps [x y] to
%       [x + XOffset_units, YScale x y + YOffset_units]; YScale = -1 marks a
%       pole copy. An axis that does not wrap keeps its original position
%       and is not checked against the planning range. When y wraps, x is
%       azimuth on a sphere and repeats every turn even if WrapX is "false",
%       so every copy is placed by whole turns to overlap the x range; WrapX
%       then only decides how far that range reaches.
%**************************************************************************
% UNITS
%   - Coordinate units; YScale is unitless.
%**************************************************************************

%% Section 1: List y Copies Over Each Pole

% Copy number n fills [ymin + n x H, ymin + (n + 1) x H], H = interval height.
% Even n shifts y by n x H. Odd n mirrors y: y -> 2 x ymin + (n + 1) x H - y.
% Example on [-90 90]: n = 1 mirrors about 90, so y = 89 becomes 91.
copyNumbers = 0;
if wrapModes(2) ~= "false"
    intervalMinimum_units = intervals_units(2, 1);
    height_units          = diff(intervals_units(2, :));
    rangeMinimum_units    = planningRange_units(2, 1);
    rangeMaximum_units    = planningRange_units(2, 2);
    boundsMinimum_units   = bounds_units(2, 1);
    boundsMaximum_units   = bounds_units(2, 2);

    % Ordinary copies n = 2k move the bounds by 2k x H. They overlap the
    % range when bounds maximum + 2k x H >= range minimum and
    % bounds minimum + 2k x H <= range maximum.
    ordinaryCopyNumbers = 2 * (ceil((rangeMinimum_units - boundsMaximum_units) / (2 * height_units)): ...
        floor((rangeMaximum_units - boundsMinimum_units) / (2 * height_units)));

    % Pole copies n = 2k + 1 map y to c - y with c = 2 x ymin + (2k + 2) x H.
    % They overlap the range when c - bounds minimum >= range minimum and
    % c - bounds maximum <= range maximum. Solving for k keeps the count
    % exact even for a shape far outside the interval.
    mirrorBase_units = 2 * intervalMinimum_units + 2 * height_units;
    poleCopyNumbers  = 2 * (ceil((rangeMinimum_units + boundsMinimum_units - mirrorBase_units) / (2 * height_units)): ...
        floor((rangeMaximum_units + boundsMaximum_units - mirrorBase_units) / (2 * height_units))) + 1;
    copyNumbers = sort([ordinaryCopyNumbers(:); poleCopyNumbers(:)]);
end
isPoleCopy = mod(copyNumbers, 2) == 1;

%% Section 2: List x Copies For Each y Copy

% A pole copy turns x by half a turn first. Then whole turns place copies
% across the x range when x or y wraps: on a sphere, x = 361 is x = 1 even
% when the motion may not cross x = 360. With neither, x keeps its value.
turnLength_units = diff(intervals_units(1, :));
imageRows        = zeros(0, 3);
for copyIndex = 1:numel(copyNumbers)
    halfTurn_units = isPoleCopy(copyIndex) * turnLength_units / 2;
    if any(wrapModes ~= "false")
        % Copy bounds must overlap the x range. Ceil and floor select the
        % first and last whole turns that fit.
        lowestTurnCount  = ceil((planningRange_units(1, 1) - bounds_units(1, 2) - halfTurn_units) / turnLength_units);
        highestTurnCount = floor((planningRange_units(1, 2) - bounds_units(1, 1) - halfTurn_units) / turnLength_units);
        xOffsets_units   = halfTurn_units + (lowestTurnCount:highestTurnCount).' * turnLength_units;
    else
        xOffsets_units = 0;
    end
    copyCount = numel(xOffsets_units);
    imageRows = [imageRows; xOffsets_units, repmat(copyNumbers(copyIndex), copyCount, 1), ...
        repmat(isPoleCopy(copyIndex), copyCount, 1)]; %#ok<AGROW>
end
imageRows = sortrows(imageRows, [1, 2]);

%% Section 3: Return Each Copy As x Offset, y Scale, And y Offset

rowIsPoleCopy  = imageRows(:, 3) == 1;
yOffsets_units = zeros(size(imageRows, 1), 1);
if wrapModes(2) ~= "false"
    yOffsets_units = imageRows(:, 2) * height_units;
    yOffsets_units(rowIsPoleCopy) = 2 * intervalMinimum_units + (imageRows(rowIsPoleCopy, 2) + 1) * height_units;
end
images = struct( ...
    'XOffset_units', imageRows(:, 1), ...
    'YScale',        1 - 2 * rowIsPoleCopy, ...
    'YOffset_units', yOffsets_units);
end
