function [coordinateScale_deg, geometryTolerance_deg, ...
        roundoffReserve_deg] = createCoordinateTolerances(varargin)
%% Section 0: Header & Readme
% SYNTAX
%   coordinateScale_deg = bmtpEngine.createCoordinateTolerances(values_deg)
%   [coordinateScale_deg, geometryTolerance_deg, roundoffReserve_deg] = ...
%       bmtpEngine.createCoordinateTolerances(values_deg, ...)
%**************************************************************************
% PURPOSE
%   - Derive one coordinate scale and the shared geometric tolerances used by
%     motion construction and authoritative trajectory verification.
%**************************************************************************
% INPUTS
%   - values_deg (numeric arrays or cells of numeric arrays)
%       Any number of coordinate collections. Nonfinite entries are ignored,
%       and empty collections contribute no scale.
%**************************************************************************
% OUTPUTS
%   - coordinateScale_deg (finite numeric scalar)
%       Maximum absolute finite coordinate, with a lower bound of one degree.
%   - geometryTolerance_deg (finite numeric scalar)
%       Dyadic geometry tolerance, 2^16 times eps(coordinateScale_deg).
%   - roundoffReserve_deg (finite numeric scalar)
%       Conservative reserve, 2^20 times eps times coordinateScale_deg. This
%       matches the authoritative verifier and is never smaller than the
%       alternative 2^20 times eps(coordinateScale_deg) for scale at least one.
%**************************************************************************
% UNITS
%   - Inputs, scale, tolerances, and reserve are degrees.
%**************************************************************************

%% Section 1: Accumulate The Finite Coordinate Scale

coordinateScale_deg = 1;
for inputIndex = 1:nargin
    values_deg = varargin{inputIndex};
    if iscell(values_deg)
        for cellIndex = 1:numel(values_deg)
            coordinateScale_deg = updateScale( ...
                coordinateScale_deg, values_deg{cellIndex});
        end
    else
        coordinateScale_deg = updateScale(coordinateScale_deg, values_deg);
    end
end

%% Section 2: Derive The Shared Tolerances

geometryTolerance_deg = 2 ^ 16 * eps(coordinateScale_deg);
roundoffReserve_deg = 2 ^ 20 * eps * coordinateScale_deg;
end

%% Section 3: Local Functions

function coordinateScale_deg = updateScale(coordinateScale_deg, values_deg)
% Include finite numeric coordinates without allowing separators to dominate.
if ~isnumeric(values_deg)
    error("createCoordinateTolerances:InvalidCoordinates", ...
        "Each coordinate collection must be numeric or a cell of numeric arrays.");
end
finiteValues_deg = abs(double(values_deg(isfinite(values_deg))));
if ~isempty(finiteValues_deg)
    coordinateScale_deg = max(coordinateScale_deg, max(finiteValues_deg));
end
end
