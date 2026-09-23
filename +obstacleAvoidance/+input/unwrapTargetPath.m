function targetMotion = unwrapTargetPath(targetMotion, startPosition_units, intervals_units, wrapModes)
%% Section 0: Header & Readme
% SYNTAX
%   targetMotion = obstacleAvoidance.input.unwrapTargetPath( ...
%       targetMotion, startPosition_units, intervals_units, wrapModes)
%**************************************************************************
% PURPOSE
%   - Keep the target path continuous when samples cross a wrapped interval's
%     ends. For example, 359 to 0 becomes 359 to 360 on a 360-unit x axis.
%     On a wrapped y axis, a sample over a pole is its pole copy: on x
%     [0 360], y [-90 90], a track from (10, 89) to (190, 89) over the top
%     pole becomes (10, 89) to (10, 91).
%   - Choose the first sample's copy nearest the vehicle's start position.
%     Choose each later copy nearest the previous unwrapped sample. This
%     interprets each step as the shortest move between those samples.
%**************************************************************************
% INPUTS
%   - targetMotion (scalar struct)
%       Sampled target with time_s and N-by-2 position_units.
%   - startPosition_units (1-by-2 numeric row)
%       Vehicle position used to choose the first target sample's copy.
%   - intervals_units (2-by-2 numeric array)
%       Wrapped workspace intervals [xmin xmax; ymin ymax].
%   - wrapModes (1-by-2 string array)
%       [WrapX WrapY], each "false", "both", "forward", or "backward". The
%       direction limits apply to the vehicle, not the target path. When y
%       wraps, azimuth also repeats each turn even if WrapX is "false".
%**************************************************************************
% OUTPUTS
%   - targetMotion (scalar struct)
%       The same target with unwrapped position_units; sample times and the
%       interpolation method are unchanged.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Choose The Nearest Copy Of Each Target Sample

targetPositions_units = double(targetMotion.position_units);
if wrapModes(2) == "false"
    % Only x wraps. Adding a whole wrap length gives the same wrapped
    % location. If two copies are equally close, choose the higher coordinate.
    if wrapModes(1) ~= "false"
        wrapLength_units        = diff(intervals_units(1, :));
        referencePosition_units = startPosition_units(1);
        for sampleIndex = 1:size(targetPositions_units, 1)
            samplePosition_units = targetPositions_units(sampleIndex, 1);
            targetPositions_units(sampleIndex, 1) = samplePosition_units + ...
                wrapLength_units * floor((referencePosition_units - samplePosition_units) / ...
                wrapLength_units + 0.5);
            referencePosition_units = targetPositions_units(sampleIndex, 1);
        end
    end
else
    % y wraps, so a pole copy changes x and y together. Compare copies by
    % their distance in both coordinates. Every copy band holds one copy
    % within a turn in x and a height in y, so search that window around the
    % reference. If two copies are equally close, choose the higher x, then
    % the higher y.
    % On a sphere x repeats every turn even when WrapX is "false", so the
    % path may pass an x end: from start (359, 89), (179, 89) -> (181, 89)
    % -> (2, 89) becomes (359, 91) -> (361, 91) -> (362, 89). The goal copies
    % are placed back into the vehicle's x range by whole turns later.
    xyWrapModes = ["both", "both"];
    windowSize_units        = [diff(intervals_units(1, :)); diff(intervals_units(2, :))];
    referencePosition_units = startPosition_units(:);
    for sampleIndex = 1:size(targetPositions_units, 1)
        samplePosition_units = targetPositions_units(sampleIndex, :);
        searchWindow_units   = [referencePosition_units - windowSize_units, referencePosition_units + windowSize_units];
        images = obstacleAvoidance.input.listWrapImages( ...
            [samplePosition_units(:), samplePosition_units(:)], intervals_units, xyWrapModes, searchWindow_units);
        copyPositions_units = [samplePosition_units(1) + images.XOffset_units, ...
            images.YScale * samplePosition_units(2) + images.YOffset_units];
        copyDistance_units  = vecnorm(copyPositions_units - referencePosition_units.', 2, 2);
        [~, copyOrder]      = sortrows([copyDistance_units, -copyPositions_units]);
        targetPositions_units(sampleIndex, :) = copyPositions_units(copyOrder(1), :);
        referencePosition_units = targetPositions_units(sampleIndex, :).';
    end
end
targetMotion.position_units = targetPositions_units;
end
