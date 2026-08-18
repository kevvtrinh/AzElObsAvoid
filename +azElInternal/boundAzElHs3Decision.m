function [lowerBound, upperBound] = boundAzElHs3Decision( ...
        layout, initialState, goalState, limits, ...
        durationLowerBound_s, finalTimeUpperOverride_s)
%% Section 0: Header & Readme
% SYNTAX
%   [lowerBound, upperBound] = azElInternal.boundAzElHs3Decision( ...
%       layout, initialState, goalState, limits, ...
%       durationLowerBound_s, finalTimeUpperOverride_s)
%**************************************************************************
% PURPOSE
%   - Bound HS-3 jerk ordinates and the optional final time.
%**************************************************************************
% INPUTS
%   - layout (scalar struct)
%       Reduced HS-3 decision indices.
%   - initialState, goalState (scalar structs)
%       Initial and maximum goal times.
%   - limits (scalar struct)
%       Two-axis maximum jerk.
%   - durationLowerBound_s (nonnegative numeric scalar)
%       Necessary motion duration bound.
%   - finalTimeUpperOverride_s (numeric scalar)
%       Additional absolute final-time upper bound, or Inf.
%**************************************************************************
% OUTPUTS
%   - lowerBound, upperBound (N-by-1 numeric vectors)
%       Bounds in the supplied decision layout.
%**************************************************************************
% UNITS
%   - Time is seconds. Jerk is degrees per second cubed.
%**************************************************************************

%% Section 1: Bound Final Time And Jerk

lowerBound = -Inf(layout.DecisionCount, 1);
upperBound = Inf(layout.DecisionCount, 1);
if layout.HasFinalTimeVariable
    lowerBound(layout.FinalTimeIndex) = ...
        initialState.time_s + durationLowerBound_s;
    finalTimeUpper_s = goalState.time_s;
    if isfinite(finalTimeUpperOverride_s)
        finalTimeUpper_s = min(finalTimeUpper_s, ...
            finalTimeUpperOverride_s);
    end
    upperBound(layout.FinalTimeIndex) = finalTimeUpper_s;
end
segmentCount = layout.SegmentCount;
controlLower = repmat(-limits.maxJerk_deg_s3, segmentCount + 1, 1);
controlUpper = repmat(limits.maxJerk_deg_s3, segmentCount + 1, 1);
midpointControlLower = repmat( ...
    -limits.maxJerk_deg_s3, segmentCount, 1);
midpointControlUpper = repmat( ...
    limits.maxJerk_deg_s3, segmentCount, 1);
lowerBound(layout.KnotControlIndex) = controlLower(:);
upperBound(layout.KnotControlIndex) = controlUpper(:);
lowerBound(layout.MidpointControlIndex) = midpointControlLower(:);
upperBound(layout.MidpointControlIndex) = midpointControlUpper(:);
end
