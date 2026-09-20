function targetMotion = unwrapTargetPath(targetMotion, startPosition_units, intervals_units, wrapAxes)
%% Section 0: Header & Readme
% SYNTAX
%   targetMotion = obstacleAvoidance.input.unwrapTargetPath( ...
%       targetMotion, startPosition_units, intervals_units, wrapAxes)
%**************************************************************************
% PURPOSE
%   - A target sampled in a wrapped workspace can jump from 359 to 0 between
%     samples. This turns the samples into one continuous path in unwrapped
%     coordinates: the first sample takes the copy nearest the start
%     position, and each later sample takes the copy nearest the previous
%     unwrapped sample, so the path never jumps at the seam.
%**************************************************************************
% INPUTS
%   - targetMotion (scalar struct)
%       Sampled target with time_s and N-by-2 position_units.
%   - startPosition_units (1-by-2 numeric row)
%       Usually the initial position.
%   - intervals_units (2-by-2 numeric array)
%       Wrapped workspace intervals [xmin xmax; ymin ymax].
%   - wrapAxes (1-by-2 logical)
%       Wrapped axes.
%**************************************************************************
% OUTPUTS
%   - targetMotion (scalar struct)
%       The same target with unwrapped position_units; sample times and the
%       interpolation method are unchanged.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Unwrap Each Wrapped Axis By Continuity

position_units = double(targetMotion.position_units);
for axisIndex = find(wrapAxes)
    period_units    = diff(intervals_units(axisIndex, :));
    reference_units = startPosition_units(axisIndex);
    for sampleIndex = 1:size(position_units, 1)
        sample_units = position_units(sampleIndex, axisIndex);
        position_units(sampleIndex, axisIndex) = sample_units + ...
            period_units * floor((reference_units - sample_units) / period_units + 0.5);
        reference_units = position_units(sampleIndex, axisIndex);
    end
end
targetMotion.position_units = position_units;
end
