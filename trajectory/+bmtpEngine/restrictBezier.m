function restricted_units = restrictBezier(controlPoint_units, interval)
%% Section 0: Header & Readme
% SYNTAX
%   restricted_units = bmtpEngine.restrictBezier(controlPoint_units, interval)
%**************************************************************************
% PURPOSE
%   - Express the same Bezier curve on a closed normalized subinterval.
%**************************************************************************
% INPUTS
%   - controlPoint_units (N-by-M numeric array)
%       Control points of one Bezier curve, one row per control point.
%   - interval (1-by-2 numeric row)
%       Normalized endpoints 0 <= a <= b <= 1 from physical interval overlap.
%**************************************************************************
% OUTPUTS
%   - restricted_units (N-by-M numeric array)
%       Same-degree controls, preserving the curve without sampling.
%**************************************************************************
% UNITS
%   - Coordinate units; interval endpoints are dimensionless.
%**************************************************************************

%% Section 1: Restrict With Two De Casteljau Splits

intervalStart    = interval(1);
intervalEnd      = interval(2);
restricted_units = controlPoint_units;
controlCount     = size(controlPoint_units, 1);

if intervalEnd < 1
    work_units = restricted_units;
    for levelIndex = 1:controlCount - 1
        work_units = (1 - intervalEnd) * work_units(1:end - 1, :) + intervalEnd * work_units(2:end, :);
        restricted_units(levelIndex + 1, :) = work_units(1, :);
    end
end

if intervalStart > 0
    fraction   = intervalStart / intervalEnd;
    work_units = restricted_units;
    for levelIndex = 1:controlCount - 1
        work_units = (1 - fraction) * work_units(1:end - 1, :) + fraction * work_units(2:end, :);
        restricted_units(controlCount - levelIndex, :) = work_units(end, :);
    end
end
end
