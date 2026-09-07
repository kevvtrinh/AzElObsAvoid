function position_units = resolveWrappedGoal(initialPosition_units, position_units, limits, options)
%% Section 0: Header & Readme
% SYNTAX
%   position_units = obstacleAvoidance.input.resolveWrappedGoal( ...
%       initialPosition_units, position_units, limits, options)
% PURPOSE
%   Select the nearest equivalent goal on each independently periodic axis.
% INPUTS
%   Initial and goal positions: normalized 1-by-2 [x y] rows.
%   limits: resolved xInterval_units and yInterval_units workspace intervals.
%   options: resolved WrapX and WrapY logical flags.
% OUTPUTS
%   position_units: continuous goal coordinates; nonperiodic axes are unchanged.
% UNITS
%   Caller-consistent coordinate units; periods are workspace interval widths.

%% Section 1: Resolve Each Enabled Axis
wrapAxes = [options.WrapX, options.WrapY];
intervals_units = [limits.xInterval_units; limits.yInterval_units];
for axisIndex = find(wrapAxes)
    period_units = diff(intervals_units(axisIndex, :));
    if ~isfinite(period_units) || period_units <= 0
        error("planTrajectory:InvalidWrapPeriod", "A wrapped axis requires a positive finite workspace interval width.");
    end
    % Resolve half-period ties toward positive displacement. This makes a
    % second normalization preserve the selected endpoint exactly.
    turns = floor((initialPosition_units(axisIndex) - position_units(axisIndex)) / period_units + 0.5);
    position_units(axisIndex) = position_units(axisIndex) + period_units * turns;
end
end
