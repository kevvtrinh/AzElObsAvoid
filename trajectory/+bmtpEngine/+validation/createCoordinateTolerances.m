function [coordinateScale_units, roundoffReserve_units] = createCoordinateTolerances(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   coordinateScale_units = bmtpEngine.validation.createCoordinateTolerances(coordinateValues_units)
%   [coordinateScale_units, roundoffReserve_units] = ...
%       bmtpEngine.validation.createCoordinateTolerances(coordinateValues_units, ...)
%**************************************************************************
% PURPOSE
%   - Use the largest coordinate magnitude to size the numerical gap
%     reserved for rounding error in geometry calculations.
%**************************************************************************
% INPUTS
%   - coordinateValues_units (numeric arrays or cells of numeric arrays)
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

%% Section 1: Find The Coordinate Scale And Calculate Its Rounding Reserve

% Larger coordinates can accumulate larger absolute rounding errors. Use at
% least a scale of 1 so the reserve does not collapse for values near zero.
coordinateScale_units = 1;
for inputIndex = 1:nargin
    coordinateValues_units = varargin{inputIndex};
    if iscell(coordinateValues_units)
        for cellIndex = 1:numel(coordinateValues_units)
            coordinateScale_units = includeCoordinateMagnitudes( ...
                coordinateScale_units, coordinateValues_units{cellIndex});
        end
    else
        coordinateScale_units = includeCoordinateMagnitudes(coordinateScale_units, coordinateValues_units);
    end
end

% Multiply the coordinate scale by the shared rounding factor. All geometry
% checks using this function receive the same allowance for the same inputs.
roundoffReserve_units = 2 ^ 20 * eps * coordinateScale_units;
end

%% Section 2: Local Functions
function coordinateScale_units = includeCoordinateMagnitudes(coordinateScale_units, coordinateValues_units)
    % NaN can separate polygon rings. Ignore it and other nonfinite values
    % when measuring scale; they do not represent finite vertex coordinates.
    if ~isnumeric(coordinateValues_units)
        error("createCoordinateTolerances:InvalidCoordinates", ...
            "Each coordinate collection must be numeric or a cell of numeric arrays.");
    end
    finiteMagnitudes_units = abs(double(coordinateValues_units(isfinite(coordinateValues_units))));
    if ~isempty(finiteMagnitudes_units)
        coordinateScale_units = max(coordinateScale_units, max(finiteMagnitudes_units));
    end
end
