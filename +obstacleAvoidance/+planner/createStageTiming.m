function timing = createStageTiming()
%% Section 0: Header & Readme
% SYNTAX
%   timing = createStageTiming()
% PURPOSE
%   Initialize the measured planner stages.
% INPUTS
%   None.
% OUTPUTS
%   timing: zeroed exclusive stages and derived totals.
% UNITS
%   Seconds.

%% Section 1: Initialize Timing
timing = struct( ...
        "RouteSearchElapsedTime_s", 0, ...
        "MotionSolvingElapsedTime_s", 0, ...
        "CollisionCheckingElapsedTime_s", 0, ...
        "FinalValidationElapsedTime_s", 0, ...
        "UnattributedElapsedTime_s", 0, ...
        "TotalElapsedTime_s", 0);
end
