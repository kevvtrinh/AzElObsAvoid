function coefficient = powerToBernstein(powerCoefficient)
%% Section 0: Header & Readme
% SYNTAX
%   coefficient = azElInternal.powerToBernstein(powerCoefficient)
%**************************************************************************
% PURPOSE
%   - Provide a deprecated compatibility alias to the neutral HS3 basis
%     conversion while existing callers migrate to hs3.powerToBernstein.
%**************************************************************************
% INPUTS
%   - powerCoefficient (N-by-M numeric), ascending-power columns.
%**************************************************************************
% OUTPUTS
%   - coefficient (N-by-M numeric), Bernstein-basis columns.
%**************************************************************************
% UNITS
%   - Coefficients retain the input's physical units.
%**************************************************************************
coefficient = hs3.powerToBernstein(powerCoefficient);
end
