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
timing = struct();
timing.RouteSearchElapsedTime_s       = 0;
timing.MotionSolvingElapsedTime_s     = 0;
timing.CollisionCheckingElapsedTime_s = 0;
timing.FinalValidationElapsedTime_s   = 0;
timing.UnattributedElapsedTime_s      = 0;
timing.TotalElapsedTime_s             = 0;
end
