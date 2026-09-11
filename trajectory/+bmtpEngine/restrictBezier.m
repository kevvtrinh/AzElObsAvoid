function restricted_units = restrictBezier(controlPoint_units, interval)
%% Section 0: Header & Readme
% SYNTAX: restricted = bmtpEngine.restrictBezier(controls,[a b])
% PURPOSE: Express the same Bezier curve on a closed normalized subinterval.
% INPUTS: N-by-M controls and 0 <= a <= b <= 1 from physical interval overlap.
% OUTPUTS: Same-degree controls, preserving the curve without sampling.
% UNITS: Coordinate units; interval endpoints are dimensionless.

%% Section 1: Restrict With Two De Casteljau Splits
a = interval(1); b = interval(2);
restricted_units = controlPoint_units;
count = size(controlPoint_units,1);
if b < 1
    work_units = restricted_units;
    for level = 1:count-1
        work_units = (1-b)*work_units(1:end-1,:)+b*work_units(2:end,:);
        restricted_units(level+1,:) = work_units(1,:);
    end
end
if a > 0
    fraction = a/b;
    work_units = restricted_units;
    for level = 1:count-1
        work_units = (1-fraction)*work_units(1:end-1,:)+fraction*work_units(2:end,:);
        restricted_units(count-level,:) = work_units(end,:);
    end
end
end
