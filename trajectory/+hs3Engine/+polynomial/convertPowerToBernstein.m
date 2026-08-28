function coefficient = convertPowerToBernstein(powerCoefficient)
%% Section 0: Header & Readme
% SYNTAX
%   coefficient = hs3Engine.polynomial.convertPowerToBernstein(powerCoefficient)
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

% For p(tau)=sum(a_k*tau^k) on 0<=tau<=1, the returned values are the
% same-degree Bernstein control values. A polynomial remains inside the convex
% hull of those values, which turns continuous scalar bounds into finitely many
% linear inequalities. This is a sufficient certificate over the interval.
% It does not use samples of the curve.
% "Convex hull" means the range between combinations of the control values.
% If every control value is below an upper limit, the full curve is also below
% that limit. A failed Bernstein bound can be conservative even when samples
% appear valid, so inspect the control values instead of adding more samples.

powerCoefficient = double(powerCoefficient);
if isvector(powerCoefficient)
    powerCoefficient = powerCoefficient(:);
end
degree = size(powerCoefficient, 1) - 1;
persistent conversionMatrixByDegree
% Polynomial degrees repeat throughout optimization. Cache the small exact
% basis matrix by degree so constraint evaluation performs only multiplication.
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
