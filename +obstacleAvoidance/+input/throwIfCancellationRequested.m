function throwIfCancellationRequested(options)
%% Section 0: Header & Readme
% SYNTAX
%   obstacleAvoidance.input.throwIfCancellationRequested(options)
%**************************************************************************
% PURPOSE
%   - Process pending UI callbacks and stop planning when the caller requests
%     cooperative cancellation.
%**************************************************************************
% INPUTS
%   - options (resolved scalar planner-options struct)
%       CancellationCheckFcn is empty for uninterrupted planning or a scalar
%       function handle that returns one scalar logical stop request.
%**************************************************************************
% OUTPUTS
%   - None.
%       A requested stop throws planTrajectory:UserCancelled at a planner
%       checkpoint so the owning application can restore a recoverable state.
%**************************************************************************
% UNITS
%   - All values are dimensionless.
%**************************************************************************

%% Section 1: Poll The Caller

if ~isfield(options, "CancellationCheckFcn") || ...
        isempty(options.CancellationCheckFcn)
    return;
end

% A synchronous MATLAB callback cannot run until the planner yields to the
% event queue. Limiting redraw rate keeps polling inexpensive while preserving
% button and timer callbacks.
drawnow limitrate;
stopRequested = options.CancellationCheckFcn();
if ~(islogical(stopRequested) || ...
        (isnumeric(stopRequested) && isreal(stopRequested))) || ...
        ~isscalar(stopRequested) || ~isfinite(double(stopRequested))
    error("planTrajectory:InvalidCancellationResponse", ...
        "CancellationCheckFcn must return one finite logical or numeric scalar.");
end
if logical(stopRequested)
    error("planTrajectory:UserCancelled", ...
        "Planning was stopped by the caller at a safe checkpoint.");
end
end
