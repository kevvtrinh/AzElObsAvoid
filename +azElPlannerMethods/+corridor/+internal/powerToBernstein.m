function coefficient = powerToBernstein(powerCoefficient)
%% Section 0: Header & Readme
% SYNTAX
%   coefficient = azElPlannerMethods.corridor.internal.powerToBernstein(powerCoefficient)
%**************************************************************************
% PURPOSE
%   - Convert ascending power coefficients to Bernstein coefficients on [0,1].
%     The convex-hull property of the Bernstein basis turns continuous span
%     bounds into finite coefficient inequalities used by corridor validation.
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

% The conversion depends only on polynomial degree, so cache its small Pascal
% matrix while applying it to any number of coefficient columns.
powerCoefficient = double(powerCoefficient);
if isvector(powerCoefficient)
    powerCoefficient = powerCoefficient(:);
end
degree = size(powerCoefficient, 1) - 1;
persistent conversionMatrixByDegree
if numel(conversionMatrixByDegree) > degree && ~isempty(conversionMatrixByDegree{degree + 1})
    coefficient = conversionMatrixByDegree{degree + 1} * powerCoefficient;
    % Returning the cached conversion avoids rebuilding identical matrices in
    % inner solver and validation loops.
    return;
end
conversionMatrix = pascal(degree + 1, 1);
conversionMatrix = conversionMatrix ./ conversionMatrix(end, :);
conversionMatrixByDegree{degree + 1} = conversionMatrix;
coefficient = conversionMatrix * powerCoefficient;
end
