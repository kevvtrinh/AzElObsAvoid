function targetMotion = unwrapTargetPath(targetMotion, startPosition_units, intervals_units, wrapAxes)
%% Section 0: Header & Readme
% SYNTAX
%   targetMotion = obstacleAvoidance.input.unwrapTargetPath( ...
%       targetMotion, startPosition_units, intervals_units, wrapAxes)
%**************************************************************************
% PURPOSE
%   - Keep the target path continuous when samples cross a wrapped interval's
%     ends. For example, 359 to 0 becomes 359 to 360 on a 360-unit axis.
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
%   - wrapAxes (1-by-2 logical)
%       [wrapX wrapY]; true enables wrapping on that axis.
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
for axisIndex = find(wrapAxes)
    wrapLength_units        = diff(intervals_units(axisIndex, :));
    referencePosition_units = startPosition_units(axisIndex);
    for sampleIndex = 1:size(targetPositions_units, 1)
        samplePosition_units = targetPositions_units(sampleIndex, axisIndex);
        % Adding a whole wrap length gives the same wrapped location. If
        % two copies are equally close, choose the higher coordinate.
        targetPositions_units(sampleIndex, axisIndex) = samplePosition_units + ...
            wrapLength_units * floor((referencePosition_units - samplePosition_units) / ...
            wrapLength_units + 0.5);
        referencePosition_units = targetPositions_units(sampleIndex, axisIndex);
    end
end
targetMotion.position_units = targetPositions_units;
end
