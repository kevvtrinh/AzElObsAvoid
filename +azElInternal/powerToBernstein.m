function bernsteinCoefficient = powerToBernstein(powerCoefficient)
%% Section 0: Header & Readme
% SYNTAX
%   bernsteinCoefficient = ...
%       azElInternal.powerToBernstein(powerCoefficient)
%**************************************************************************
% PURPOSE
%   - Convert ascending power coefficients to an equivalent Bernstein
%     control hull on the normalized interval from zero to one.
%**************************************************************************
% INPUTS
%   - powerCoefficient (nonempty finite numeric matrix)
%       Each column is one polynomial. Rows are ascending powers.
%**************************************************************************
% OUTPUTS
%   - bernsteinCoefficient (numeric matrix)
%       Bernstein coefficients with the same size and column ordering.
%**************************************************************************
% UNITS
%   - Coefficients retain the units of their input polynomial columns.
%**************************************************************************

%% Section 1: Build Or Reuse The Degree Transform

degree = size(powerCoefficient, 1) - 1;
persistent transformByDegree
if isempty(transformByDegree)
    transformByDegree = cell(0, 1);
end
cacheIndex = degree + 1;
if numel(transformByDegree) < cacheIndex || ...
        isempty(transformByDegree{cacheIndex})
    transform = zeros(cacheIndex, cacheIndex);
    for bernsteinIndex = 0:degree
        for powerIndex = 0:bernsteinIndex
            transform(bernsteinIndex + 1, powerIndex + 1) = ...
                nchoosek(bernsteinIndex, powerIndex) / ...
                nchoosek(degree, powerIndex);
        end
    end
    transformByDegree{cacheIndex} = transform;
end

%% Section 2: Convert The Coefficients

bernsteinCoefficient = ...
    transformByDegree{cacheIndex} * powerCoefficient;
end
