function [coordinateScale_units, geometryTolerance_units, roundoffReserve_units] = createCoordinateTolerances(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   coordinateScale_units = bmtpEngine.createCoordinateTolerances(values_units)
%   [coordinateScale_units, geometryTolerance_units, roundoffReserve_units] = ...
%       bmtpEngine.createCoordinateTolerances(values_units, ...)
%
% PURPOSE
%   - Derive one coordinate scale and the shared geometric tolerances used by
%     motion construction and authoritative trajectory verification.
%
% INPUTS
%   - values_units (numeric arrays or cells of numeric arrays)
%       Any number of coordinate collections. Nonfinite entries are ignored,
%       and empty collections contribute no scale.
%
% OUTPUTS
%   - coordinateScale_units (finite numeric scalar)
%       Maximum absolute finite coordinate, with a lower bound of one degree.
%   - geometryTolerance_units (finite numeric scalar)
%       Dyadic geometry tolerance, 2^16 times eps(coordinateScale_units).
%   - roundoffReserve_units (finite numeric scalar)
%       Conservative reserve, 2^20 times eps times coordinateScale_units. This
%       matches the authoritative verifier and is never smaller than the
%       alternative 2^20 times eps(coordinateScale_units) for scale at least one.
%
% UNITS
%   - Inputs, scale, tolerances, and reserve are coordinate units.
%

%% Section 1: Accumulate The Finite Coordinate Scale

coordinateScale_units = 1;
% Process each input needed to build coordinate tolerances.
for inputIndex = 1:nargin
    values_units = varargin{inputIndex};
    if iscell(values_units)
        % Process each geometric cell while constructing or checking the region topology.
        for cellIndex = 1:numel(values_units)
            coordinateScale_units = updateScale(coordinateScale_units, values_units{cellIndex});
        end
    else
        coordinateScale_units = updateScale(coordinateScale_units, values_units);
    end
end

%% Section 2: Derive The Shared Tolerances

geometryTolerance_units = 2 ^ 16 * eps(coordinateScale_units);
roundoffReserve_units   = 2 ^ 20 * eps * coordinateScale_units;
end

%% Section 3: Local Functions

function coordinateScale_units = updateScale(coordinateScale_units, values_units)
    % Ignore nonfinite ring separators when measuring coordinate scale.
    if ~isnumeric(values_units)
        error("createCoordinateTolerances:InvalidCoordinates", "Each coordinate collection must be numeric or a cell of numeric arrays.");
    end
    finiteValues_units = abs(double(values_units(isfinite(values_units))));
    if ~isempty(finiteValues_units)
        coordinateScale_units = max(coordinateScale_units, max(finiteValues_units));
    end
end
