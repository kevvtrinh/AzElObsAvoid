function finalOffset = sortedUpperBound(sortedValues, queryValue)
%% Section 0: Header & Readme
% SYNTAX
%   finalOffset = obstacleAvoidance.search.sortedUpperBound(sortedValues, queryValue)
%**************************************************************************
% PURPOSE
%   - Last index whose sorted value is not greater than the scalar query,
%     zero when every value is greater.
%**************************************************************************
% INPUTS
%   - sortedValues (numeric vector)
%       Values in ascending order.
%   - queryValue (numeric scalar)
%       The query.
%**************************************************************************
% OUTPUTS
%   - finalOffset (numeric scalar)
%       Index into sortedValues, or zero.
%**************************************************************************
% UNITS
%   - Whatever the caller sorts by.
%**************************************************************************

%% Section 1: Bisect

% Last index whose sorted value is not greater than the scalar query.
lowOffset  = 1;
highOffset = numel(sortedValues);
finalOffset = 0;
while lowOffset <= highOffset
    middleOffset = floor((lowOffset + highOffset) / 2);
    if sortedValues(middleOffset) <= queryValue
        finalOffset = middleOffset;
        lowOffset   = middleOffset + 1;
    else
        highOffset = middleOffset - 1;
    end
end
end
