function layerTimes_s = boundedTimeLayers( ...
        sampleTimes_s, startTime_s, endTime_s, maximumLayerCount)
%% Section 0: Header & Readme
% SYNTAX
%   layerTimes_s = obstacleAvoidance.search.boundedTimeLayers( ...
%       sampleTimes_s, startTime_s, endTime_s, maximumLayerCount)
%**************************************************************************
% PURPOSE
%   - Retain source-aware and uniformly spaced time layers within one
%     deterministic work bound.
%**************************************************************************
% INPUTS
%   - sampleTimes_s (numeric vector)
%       Candidate source and event-aware times.
%   - startTime_s, endTime_s (finite scalars)
%       Inclusive request interval.
%   - maximumLayerCount (positive integer scalar)
%       Maximum retained layer count.
%**************************************************************************
% OUTPUTS
%   - layerTimes_s (numeric column vector)
%       Sorted retained times including both interval endpoints.
%**************************************************************************
% UNITS
%   - All times are seconds.
%**************************************************************************

%% Section 1: Select Bounded Layers

% Time-expanded search grows in proportion to retained layers. Keep the start
% and end exactly, then choose a deterministic spread of interior obstacle
% event times so changing geometry remains represented within a fixed budget.

uniformTime_s = linspace(startTime_s, endTime_s, 9).';
candidateTime_s = unique( ...
    [startTime_s; sampleTimes_s(:); uniformTime_s; endTime_s]);
candidateTime_s = candidateTime_s( ...
    candidateTime_s >= startTime_s & candidateTime_s <= endTime_s);
if numel(candidateTime_s) <= maximumLayerCount
    layerTimes_s = candidateTime_s;
    return;
end

targetTime_s = linspace(startTime_s, endTime_s, maximumLayerCount).';
selectedIndex = zeros(maximumLayerCount, 1);
for targetIndex = 1:maximumLayerCount
    [~, selectedIndex(targetIndex)] = min(abs( ...
        candidateTime_s - targetTime_s(targetIndex)));
end
selectedIndex = unique([1; selectedIndex; numel(candidateTime_s)]);
layerTimes_s = candidateTime_s(selectedIndex);
end
