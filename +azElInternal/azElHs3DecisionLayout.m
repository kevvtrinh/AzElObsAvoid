function layout = azElHs3DecisionLayout( ...
        segmentCount, hasFinalTimeVariable)
%% Section 0: Header & Readme
% SYNTAX
%   layout = azElInternal.azElHs3DecisionLayout( ...
%       segmentCount, hasFinalTimeVariable)
%**************************************************************************
% PURPOSE
%   - Define the reduced HS-3 jerk and optional final-time decision layout.
%**************************************************************************
% INPUTS
%   - segmentCount (positive integer scalar)
%       Number of HS-3 mesh segments.
%   - hasFinalTimeVariable (logical scalar)
%       True when final time is part of the decision vector.
%**************************************************************************
% OUTPUTS
%   - layout (scalar struct)
%       Stable decision indices and total count.
%**************************************************************************
% UNITS
%   - Indices and counts are dimensionless.
%**************************************************************************

%% Section 1: Assign Decision Indices

nextIndex = 1;
finalTimeIndex = zeros(0, 1);
if hasFinalTimeVariable
    finalTimeIndex = nextIndex;
    nextIndex = nextIndex + 1;
end
knotControlIndex = nextIndex:nextIndex + 2 * (segmentCount + 1) - 1;
nextIndex = knotControlIndex(end) + 1;
midpointControlIndex = nextIndex:nextIndex + 2 * segmentCount - 1;
nextIndex = midpointControlIndex(end) + 1;
layout = struct( ...
    "SegmentCount", segmentCount, ...
    "HasFinalTimeVariable", logical(hasFinalTimeVariable), ...
    "FinalTimeIndex", finalTimeIndex, ...
    "KnotControlIndex", knotControlIndex, ...
    "MidpointControlIndex", midpointControlIndex, ...
    "DecisionCount", nextIndex - 1);
end
