function controlValues_units = powerToBernstein(powerCoefficients_units, degree)
%% Section 0: Header & Readme
% SYNTAX
%   controlValues_units = bmtpEngine.motion.powerToBernstein(powerCoefficients_units)
%   controlValues_units = bmtpEngine.motion.powerToBernstein(powerCoefficients_units, degree)
%**************************************************************************
% PURPOSE
%   - Express p(u) = c0 + c1 x u + c2 x u^2 + ... as Bezier controls for
%     0 <= u <= 1. Conversion, including a higher requested degree, keeps
%     the same curve.
%**************************************************************************
% INPUTS
%   - powerCoefficients_units (P-by-M matrix or S-by-2-by-P array)
%       Matrix columns contain coefficients c0 through c(P-1). For a
%       three-dimensional array, the dimensions are segment, x/y, and power.
%   - degree (nonnegative integer scalar, optional)
%       Requested Bezier degree, at least P - 1. Defaults to P - 1.
%**************************************************************************
% OUTPUTS
%   - controlValues_units ((degree+1)-by-M matrix or S-by-(degree+1)-by-2 array)
%       Control points of the identical curve.
%**************************************************************************
% UNITS
%   - Coefficients and controls keep the units of the supplied quantity;
%     segment fraction u is dimensionless.
%**************************************************************************

%% Section 1: Read The Coefficient Layout And Get Conversion Weights

hasSegmentLayout = ndims(powerCoefficients_units) == 3;
if hasSegmentLayout
    coefficientCount = size(powerCoefficients_units, 3);
else
    coefficientCount = size(powerCoefficients_units, 1);
end
if nargin < 2
    degree = coefficientCount - 1;
end
powerToControlMap = getPowerToControlMap(degree, coefficientCount);

%% Section 2: Convert Every Coefficient Set Without Changing Its Curve

if ~hasSegmentLayout
    controlValues_units = powerToControlMap * powerCoefficients_units;
    return
end

% Put power first for multiplication, then return segment/control/x-y order.
coefficientsByPower_units = permute(powerCoefficients_units, [3 1 2]);
controlValues_units       = permute(pagemtimes(powerToControlMap, coefficientsByPower_units), [2 1 3]);
end

%% Section 3: Local Functions

function powerToControlMap = getPowerToControlMap(degree, coefficientCount)
    % Conversion weights depend only on degree and coefficient count.
    % Reuse a saved matrix when another curve has the same layout.
    persistent savedConversionMaps
    if isempty(savedConversionMaps)
        savedConversionMaps = cell(0, 0);
    end
    conversionMapWasSaved = size(savedConversionMaps, 1) >= degree + 1 && ...
        size(savedConversionMaps, 2) >= coefficientCount && ...
        ~isempty(savedConversionMaps{degree + 1, coefficientCount});
    if conversionMapWasSaved
        powerToControlMap = savedConversionMaps{degree + 1, coefficientCount};
        return
    end

    % Control b uses weight choose(b,p) / choose(degree,p) for power p.
    % At degree 1, the line c0 + c1 x u has controls c0 and c0 + c1.
    % b and p start at 0, so add 1 when indexing MATLAB arrays.
    powerToControlMap = zeros(degree + 1, coefficientCount);
    for controlPointIndex = 0:degree
        for powerDegree = 0:min(controlPointIndex, coefficientCount - 1)
            powerToControlMap(controlPointIndex + 1, powerDegree + 1) = ...
                nchoosek(controlPointIndex, powerDegree) / nchoosek(degree, powerDegree);
        end
    end
    savedConversionMaps{degree + 1, coefficientCount} = powerToControlMap;
end
