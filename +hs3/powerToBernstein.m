function coefficient = powerToBernstein(powerCoefficient)
%% Section 0: Header & Readme
% SYNTAX
%   coefficient = hs3.powerToBernstein(powerCoefficient)
%**************************************************************************
% PURPOSE
%   - Convert ascending power coefficients to Bernstein coefficients on
%     the normalized interval [0, 1].
%**************************************************************************
% INPUTS
%   - powerCoefficient (finite numeric vector or N-by-M matrix)
%       Each column contains one polynomial's ascending coefficients.
%**************************************************************************
% OUTPUTS
%   - coefficient (N-by-M numeric array)
%       Same-degree Bernstein coefficients for every input column.
%**************************************************************************
% UNITS
%   - Coefficients retain the physical units of the input polynomial.
%**************************************************************************

%% Section 1: Apply The Exact Basis Conversion

powerCoefficient = double(powerCoefficient);
if isvector(powerCoefficient)
    powerCoefficient = powerCoefficient(:);
end
degree = size(powerCoefficient, 1) - 1;
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
