function restrictedControlPoint_units = restrictBezier(controlPoint_units, segmentFractionInterval)
%% Section 0: Header & Readme
% SYNTAX
%   restrictedControlPoint_units = bmtpEngine.motion.restrictBezier( ...
%       controlPoint_units, segmentFractionInterval)
%**************************************************************************
% PURPOSE
%   - Return controls for just the requested portion of a Bezier curve.
%     Preserve its shape and degree without approximating it with samples.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-M numeric array)
%       Control points of one Bezier curve, one row per control point.
%   - segmentFractionInterval (1-by-2 numeric row)
%       [start end] fractions with 0 <= start <= end <= 1. Fractions refer
%       to the curve parameter, not the distance traveled along the curve.
%**************************************************************************
% OUTPUTS
%   - restrictedControlPoint_units (N-by-M numeric array)
%       Controls for the selected portion, now parameterized from 0 to 1.
%**************************************************************************
% UNITS
%   - Coordinate units; interval endpoints are dimensionless.
%**************************************************************************

%% Section 1: Keep The Curve From Its Start Through The Requested End

startFraction = segmentFractionInterval(1);
endFraction   = segmentFractionInterval(2);
restrictedControlPoint_units = controlPoint_units;
controlPointCount = size(controlPoint_units, 1);

% Repeatedly blend adjacent controls (the de Casteljau split). The first
% point at each level gives a control for the portion before the split.
if endFraction < 1
    interpolatedControls_units = restrictedControlPoint_units;
    for splitLevel = 1:controlPointCount - 1
        interpolatedControls_units = (1 - endFraction) * interpolatedControls_units(1:end - 1, :) + ...
            endFraction * interpolatedControls_units(2:end, :);
        restrictedControlPoint_units(splitLevel + 1, :) = interpolatedControls_units(1, :);
    end
end

%% Section 2: Remove The Portion Before The Requested Start

% The retained curve now covers [0 end], so its local split is start / end.
% For [0.25 0.5], split the retained half again at 0.25 / 0.5 = 0.5.
% Keeping the last point at each level selects the portion after this split.
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
