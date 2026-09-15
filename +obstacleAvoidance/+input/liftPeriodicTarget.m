function targetMotion = liftPeriodicTarget(targetMotion, anchor_units, intervals_units, wrapAxes)
%% Section 0: Header & Readme
% SYNTAX
%   targetMotion = obstacleAvoidance.input.liftPeriodicTarget( ...
%       targetMotion, anchor_units, intervals_units, wrapAxes)
%**************************************************************************
% PURPOSE
%   - Lift a sampled target of a periodic workspace to one continuous path
%     in the unwrapped frame. The first sample takes its image nearest the
%     anchor and every later sample takes its image nearest the lifted
%     sample before it, so the declared interpolant never crosses a seam.
%**************************************************************************
% INPUTS
%   - targetMotion (scalar struct)
%       Sampled target with time_s and N-by-2 position_units.
%   - anchor_units (1-by-2 numeric row)
%       Usually the initial position.
%   - intervals_units (2-by-2 numeric array)
%       Periodic workspace intervals [xmin xmax; ymin ymax].
%   - wrapAxes (1-by-2 logical)
%       Wrapped axes.
%**************************************************************************
% OUTPUTS
%   - targetMotion (scalar struct)
%       The same target with lifted position_units; sample times and the
%       interpolation method are unchanged.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Lift Each Wrapped Axis By Continuity

position_units = double(targetMotion.position_units);
for axisIndex = find(wrapAxes)
    period_units    = diff(intervals_units(axisIndex, :));
    reference_units = anchor_units(axisIndex);
    for sampleIndex = 1:size(position_units, 1)
        sample_units = position_units(sampleIndex, axisIndex);
        position_units(sampleIndex, axisIndex) = sample_units + ...
            period_units * floor((reference_units - sample_units) / period_units + 0.5);
        reference_units = position_units(sampleIndex, axisIndex);
    end
end
targetMotion.position_units = position_units;
end
