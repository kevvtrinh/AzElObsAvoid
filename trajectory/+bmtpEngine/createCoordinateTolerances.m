function [coordinateScale_units, roundoffReserve_units] = createCoordinateTolerances(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   coordinateScale_units = bmtpEngine.createCoordinateTolerances(values_units)
%   [coordinateScale_units, roundoffReserve_units] = ...
%       bmtpEngine.createCoordinateTolerances(values_units, ...)
%**************************************************************************
% PURPOSE
%   - Derive the coordinate scale and shared geometric roundoff reserve.
%**************************************************************************
% INPUTS
%   - values_units (numeric arrays or cells of numeric arrays)
%       Coordinate collections; nonfinite entries do not affect the scale.
%**************************************************************************
% OUTPUTS
%   - coordinateScale_units (finite numeric scalar)
%       Largest finite coordinate magnitude, never below one.
%   - roundoffReserve_units (finite numeric scalar)
%       Conservative geometric roundoff reserve at that scale.
%   - A non-numeric coordinate collection throws.
%**************************************************************************
% UNITS
%   - Inputs, scale, and reserve are coordinate units.
%**************************************************************************

%% Section 1: Accumulate The Finite Coordinate Scale
coordinateScale_units = 1;
for inputIndex = 1:nargin
    values_units = varargin{inputIndex};
    if iscell(values_units)
        for cellIndex = 1:numel(values_units)
            coordinateScale_units = updateScale(coordinateScale_units, values_units{cellIndex});
        end
    else
        coordinateScale_units = updateScale(coordinateScale_units, values_units);
    end
end

%% Section 2: Derive The Shared Reserve
roundoffReserve_units = 2 ^ 20 * eps * coordinateScale_units;
end

%% Section 3: Local Functions
function coordinateScale_units = updateScale(coordinateScale_units, values_units)
    % Ignore nonfinite ring separators when measuring coordinate scale.
    if ~isnumeric(values_units)
        error("createCoordinateTolerances:InvalidCoordinates", ...
            "Each coordinate collection must be numeric or a cell of numeric arrays.");
    end
    finiteValues_units = abs(double(values_units(isfinite(values_units))));
    if ~isempty(finiteValues_units)
        coordinateScale_units = max(coordinateScale_units, max(finiteValues_units));
    end
end
