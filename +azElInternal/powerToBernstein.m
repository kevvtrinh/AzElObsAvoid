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
coefficient = zeros(degree + 1, 1);
for bernsteinIndex = 0:degree
    for powerIndex = 0:bernsteinIndex
        coefficient(bernsteinIndex + 1) = ...
            coefficient(bernsteinIndex + 1) + ...
            nchoosek(bernsteinIndex, powerIndex) / ...
            nchoosek(degree, powerIndex) * ...
            powerCoefficient(powerIndex + 1);
    end
end
end
