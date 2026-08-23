function coefficient = powerToBernstein(powerCoefficient)
%% Section 0: Header & Readme
% SYNTAX
%   coefficient = azElPlannerMethods.hs3.internal.powerToBernstein(powerCoefficient)
%**************************************************************************
% PURPOSE
%   - Apply the Farouki Bernstein-basis conversion listed in citation.md.
%**************************************************************************
% INPUTS
%   - powerCoefficient (finite numeric vector or N-by-M matrix)
%       Each column contains ascending coefficients for one polynomial.
%**************************************************************************
% OUTPUTS
%   - coefficient (N-by-M numeric array)
%       Same-degree Bernstein coefficients for every input column.
%**************************************************************************
% UNITS
%   - Coefficients retain the physical units of the input polynomial.
%**************************************************************************
%% Section 1: Apply The Exact Basis Conversion
% Treat a vector as one polynomial column so the matrix conversion is uniform.
powerCoefficient = double(powerCoefficient);
if isvector(powerCoefficient)
    powerCoefficient = powerCoefficient(:);
end

degree = size(powerCoefficient, 1) - 1;
persistent conversionMatrixByDegree

% Reuse a previously constructed degree-specific conversion matrix when available.
if numel(conversionMatrixByDegree) > degree && ...
        ~isempty(conversionMatrixByDegree{degree + 1})
    coefficient = conversionMatrixByDegree{degree + 1} * powerCoefficient;
    return;
end

% Build the exact power-to-Bernstein matrix once for this polynomial degree.
conversionMatrix = pascal(degree + 1, 1);
conversionMatrix = conversionMatrix ./ conversionMatrix(end, :);
conversionMatrixByDegree{degree + 1} = conversionMatrix;

% Each input column is converted independently by the same cached matrix.
coefficient = conversionMatrix * powerCoefficient;
end
