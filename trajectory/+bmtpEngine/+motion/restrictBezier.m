function restrictedControlPoint_units = restrictBezier(controlPoint_units, segmentFractionInterval)
%% Section 0: Header & Readme
% SYNTAX
%   restrictedControlPoint_units = bmtpEngine.motion.restrictBezier( ...
%       controlPoint_units, segmentFractionInterval)
%**************************************************************************
% PURPOSE
%   - Cut one Bezier curve to the requested fraction interval. Return new
%     controls that trace exactly that portion as the new fraction runs from
%     0 to 1; no sample-based approximation is used.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-M numeric array)
%       N controls for one curve, with one row per control and M coordinate
%       axes in the columns.
%   - segmentFractionInterval (1-by-2 numeric row)
%       [start end] with 0 <= start <= end <= 1. A fraction measures progress
%       in the curve parameter, not distance traveled; 0.25 need not be a
%       quarter of the curve's length.
%**************************************************************************
% OUTPUTS
%   - restrictedControlPoint_units (N-by-M numeric array)
%       Controls for the selected portion, with its new fraction from 0 to 1.
%       If start equals end, every control marks that one point.
%**************************************************************************
% UNITS
%   - Coordinate units; interval endpoints are dimensionless.
%**************************************************************************

%% Section 1: Keep The Curve From Its Start Through The Requested End

startFraction = segmentFractionInterval(1);
endFraction   = segmentFractionInterval(2);
restrictedControlPoint_units = controlPoint_units;
controlPointCount = size(controlPoint_units, 1);

% At endFraction, repeatedly blend neighboring controls. This is the
% de Casteljau split: the first point at each level becomes a control for
% the left piece [0, endFraction]. If endFraction is 1, keep the full curve.
if endFraction < 1
    interpolatedControls_units = restrictedControlPoint_units;
    for splitLevel = 1:controlPointCount - 1
        interpolatedControls_units = (1 - endFraction) * interpolatedControls_units(1:end - 1, :) + ...
            endFraction * interpolatedControls_units(2:end, :);
        restrictedControlPoint_units(splitLevel + 1, :) = interpolatedControls_units(1, :);
    end
end

%% Section 2: Remove The Portion Before The Requested Start

% The retained curve covers [0, endFraction] on a new 0-to-1 scale. The
% requested start is therefore startFraction / endFraction on this piece.
% For [0.25, 0.5], cut the retained half at 0.25 / 0.5 = 0.5. The last
% point at each level becomes a control for the right piece.
if startFraction > 0
    remainingSplitFraction     = startFraction / endFraction;
    interpolatedControls_units = restrictedControlPoint_units;
    for splitLevel = 1:controlPointCount - 1
        interpolatedControls_units = (1 - remainingSplitFraction) * interpolatedControls_units(1:end - 1, :) + ...
            remainingSplitFraction * interpolatedControls_units(2:end, :);
        restrictedControlPoint_units(controlPointCount - splitLevel, :) = interpolatedControls_units(end, :);
    end
end
end
