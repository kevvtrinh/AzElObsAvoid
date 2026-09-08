function position_units = targetPositionAtTime(targetMotion, time_s)
%% Section 0: Header & Readme
% SYNTAX: position = obstacleAvoidance.input.targetPositionAtTime(target,time)
% PURPOSE: Validate and evaluate an explicitly sampled target without extrapolation.
% INPUTS: Target time_s, N-by-2 position_units, optional linear/pchip interpolation.
% OUTPUTS: Interpolated position rows at the requested physical times.
% UNITS: Coordinate units and seconds.

%% Section 1: Validate The Target And Query Domain
if ~isstruct(targetMotion) || ~isscalar(targetMotion) || ~all(isfield(targetMotion,{'time_s','position_units'}))
    error('planner:InvalidTarget','targetMotion requires time_s and position_units.');
end
validateattributes(targetMotion.time_s,{'numeric'},{'real','finite','vector','nonempty'});
sampleTime_s = double(targetMotion.time_s(:));
validateattributes(targetMotion.position_units,{'numeric'},{'real','finite','size',[numel(sampleTime_s),2]});
if numel(sampleTime_s)<2 || any(diff(sampleTime_s)<=0)
    error('planner:InvalidTargetTime','Target sample times must strictly increase.');
end
validateattributes(time_s,{'numeric'},{'real','finite','vector'});
if any(time_s < sampleTime_s(1) | time_s > sampleTime_s(end))
    error('planner:TargetTimeOutsideHistory','Target evaluation cannot extrapolate beyond the supplied history.');
end
method = 'linear';
if isfield(targetMotion,'InterpolationMethod'), method = string(targetMotion.InterpolationMethod); end
if ~isscalar(string(method)) || ~any(string(method)==["linear","pchip"])
    error('planner:InvalidTargetInterpolation','Target interpolation must be linear or pchip.');
end

%% Section 2: Evaluate The Declared Interpolant
position_units = interp1(sampleTime_s,double(targetMotion.position_units),time_s(:),method);
end
