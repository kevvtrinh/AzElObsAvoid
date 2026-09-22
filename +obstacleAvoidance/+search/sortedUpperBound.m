function lastMatchingIndex = sortedUpperBound(sortedValues, maximumValue)
%% Section 0: Header & Readme
% SYNTAX
%   lastMatchingIndex = obstacleAvoidance.search.sortedUpperBound(sortedValues, maximumValue)
%**************************************************************************
% PURPOSE
%   - Find the last value <= the supplied limit in an ascending list.
%     For [1 3 3 7] and a limit of 3, return index 3.
%**************************************************************************
% INPUTS
%   - sortedValues (numeric vector)
%       Values in ascending order.
%   - maximumValue (numeric scalar)
%       Largest allowed value, including equality.
%**************************************************************************
% OUTPUTS
%   - lastMatchingIndex (numeric scalar)
%       Last matching index, or 0 when no value matches or the list is empty.
%**************************************************************************
% UNITS
%   - sortedValues and maximumValue must use the same units.
%**************************************************************************

%% Section 1: Narrow The Sorted List To Its Last Match

% Each comparison discards half the remaining indices. Start at 0 so
% an empty list or a limit below the first value returns no match.
firstCandidateIndex = 1;
lastCandidateIndex  = numel(sortedValues);
lastMatchingIndex   = 0;
while firstCandidateIndex <= lastCandidateIndex
    middleIndex = floor((firstCandidateIndex + lastCandidateIndex) / 2);
    if sortedValues(middleIndex) <= maximumValue
        % Keep this match, then look right for a later one, including ties.
        lastMatchingIndex   = middleIndex;
        firstCandidateIndex = middleIndex + 1;
    else
        % This value and every value to its right exceed the limit.
        lastCandidateIndex = middleIndex - 1;
    end
end
end
