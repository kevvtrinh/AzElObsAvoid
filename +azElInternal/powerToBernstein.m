function coefficient = powerToBernstein(powerCoefficient)
%% Section 0: Header & Readme
% SYNTAX
%   coefficient = azElInternal.powerToBernstein(powerCoefficient)
%**************************************************************************
% PURPOSE
%   - Apply the Farouki Bernstein-basis conversion listed in citation.md.
%**************************************************************************
% INPUTS
%   - powerCoefficient (finite numeric vector)
%       Ascending coefficients from degree zero through the highest degree.
%**************************************************************************
% OUTPUTS
%   - coefficient (N-by-1 numeric column)
%       Same-degree Bernstein coefficients in ascending basis-index order.
%**************************************************************************
% UNITS
%   - Coefficients retain the physical units of the input polynomial.
%**************************************************************************
%% Section 1: Apply The Exact Basis Conversion
powerCoefficient = double(powerCoefficient(:));
degree = numel(powerCoefficient) - 1;
persistent conversionMatrixByDegree
if numel(conversionMatrixByDegree) > degree && ...
        ~isempty(conversionMatrixByDegree{degree + 1})
    coefficient = conversionMatrixByDegree{degree + 1} * powerCoefficient;
    return;
end
conversionMatrix = pascal(degree + 1, 1);
conversionMatrix = conversionMatrix ./ conversionMatrix(end, :);
conversionMatrixByDegree{degree + 1} = conversionMatrix;
coefficient = conversionMatrix * powerCoefficient;
end
