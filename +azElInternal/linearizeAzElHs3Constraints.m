function [inequalityMatrix, inequalityBound, ...
        equalityMatrix, equalityBound] = ...
        linearizeAzElHs3Constraints(constraintFunction, decisionCount)
%% Section 0: Header & Readme
% SYNTAX
%   [inequalityMatrix, inequalityBound, equalityMatrix, equalityBound] = ...
%       azElInternal.linearizeAzElHs3Constraints( ...
%       constraintFunction, decisionCount)
%**************************************************************************
% PURPOSE
%   - Convert affine HS3 constraint values to solver matrix form.
%**************************************************************************
% INPUTS
%   - constraintFunction (function handle)
%       Returns inequality and equality columns for one jerk decision.
%   - decisionCount (positive integer scalar)
%       Number of fixed-time jerk decision values.
%**************************************************************************
% OUTPUTS
%   - inequalityMatrix, inequalityBound (numeric matrix and column)
%       Define inequalityMatrix * decision <= inequalityBound.
%   - equalityMatrix, equalityBound (numeric matrix and column)
%       Define equalityMatrix * decision == equalityBound.
%**************************************************************************
% UNITS
%   - Rows retain the heterogeneous units of their source constraints.
%**************************************************************************

%% Section 1: Evaluate The Affine Basis

zeroDecision = zeros(decisionCount, 1);
[inequalityOffset, equalityOffset] = constraintFunction(zeroDecision);
inequalityMatrix = zeros(numel(inequalityOffset), decisionCount);
equalityMatrix = zeros(numel(equalityOffset), decisionCount);
for decisionIndex = 1:decisionCount
    basisDecision = zeroDecision;
    basisDecision(decisionIndex) = 1;
    [basisInequality, basisEquality] = constraintFunction(basisDecision);
    inequalityMatrix(:, decisionIndex) = ...
        basisInequality - inequalityOffset;
    equalityMatrix(:, decisionIndex) = basisEquality - equalityOffset;
end
inequalityBound = -inequalityOffset;
equalityBound = -equalityOffset;
end
