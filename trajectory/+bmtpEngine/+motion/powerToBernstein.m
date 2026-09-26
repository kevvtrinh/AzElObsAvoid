function controlValues_units = powerToBernstein(powerCoefficients_units, degree)
%% Section 0: Header & Readme
% SYNTAX
%   controlValues_units = bmtpEngine.motion.powerToBernstein(powerCoefficients_units)
%   controlValues_units = bmtpEngine.motion.powerToBernstein(powerCoefficients_units, degree)
%**************************************************************************
% PURPOSE
%   - Turn coefficients of p(u) = c0 + c1 x u + c2 x u^2 + ... into Bezier
%     control points for segment fraction u from 0 to 1.
%   - A higher requested degree adds controls but does not change the curve.
%**************************************************************************
% INPUTS
%   - powerCoefficients_units (P-by-M matrix or S-by-2-by-P array)
%       P is the number of coefficients; M is the number of independent
%       curves in a matrix. Each matrix column lists constant, u, u^2, and
%       later coefficients. In the array, S is the segment count and the
%       dimensions are segment, position axis, then coefficient.
%   - degree (nonnegative integer scalar, optional)
%       Degree of the returned Bezier curve; defaults to P - 1. It must be
%       at least P - 1 to preserve the curve. Smaller values are not rejected.
%**************************************************************************
% OUTPUTS
%   - controlValues_units ((degree+1)-by-M matrix or S-by-(degree+1)-by-2 array)
%       Bezier controls describing the same curve in the matching layout.
%**************************************************************************
% UNITS
%   - Coefficients and controls keep the units of the supplied quantity;
%     segment fraction u is dimensionless.
%**************************************************************************

%% Section 1: Read The Coefficient Layout And Get Conversion Weights

% Matrix rows hold powers; a segmented array holds powers last. Read that
% dimension so the default degree matches the supplied polynomial.
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

% Put coefficients first so the conversion map can multiply each segment
% and axis. Restore segment/control/axis order for the output.
coefficientsByPower_units = permute(powerCoefficients_units, [3 1 2]);
controlValues_units       = permute(pagemtimes(powerToControlMap, coefficientsByPower_units), [2 1 3]);
end

%% Section 3: Local Functions

function powerToControlMap = getPowerToControlMap(degree, coefficientCount)
    % The weights depend on degree and coefficient count, not on coefficient
    % values. Reuse them when another curve has the same shape.
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

    % Control b combines coefficient p with weight choose(b,p) / choose(degree,p).
    % For p(u) = 2 + 3 x u, degree 1 gives controls [2; 5]; degree 2 gives
    % [2; 3.5; 5]. Both describe the same line. b and p start at 0, so add
    % 1 when indexing MATLAB arrays.
    powerToControlMap = zeros(degree + 1, coefficientCount);
    for controlPointIndex = 0:degree
        for powerDegree = 0:min(controlPointIndex, coefficientCount - 1)
            powerToControlMap(controlPointIndex + 1, powerDegree + 1) = ...
                nchoosek(controlPointIndex, powerDegree) / nchoosek(degree, powerDegree);
        end
    end
    savedConversionMaps{degree + 1, coefficientCount} = powerToControlMap;
end
