function [position_units, velocity_units_s, acceleration_units_s2] = targetPositionAtTime(targetMotion, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   position_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, time_s)
%   [position_units, velocity_units_s, acceleration_units_s2] = ...
%       obstacleAvoidance.input.targetPositionAtTime(targetMotion, time_s)
%**************************************************************************
% PURPOSE
%   - Validate and evaluate an explicitly sampled target without extrapolation.
%**************************************************************************
% INPUTS
%   - targetMotion (scalar struct)
%       Sample times, positions, and an optional interpolation method.
%   - time_s (numeric vector)
%       Physical query times within the supplied history.
%**************************************************************************
% OUTPUTS
%   - position_units (N-by-2 numeric array)
%       Interpolated target position at each query time.
%   - velocity_units_s (N-by-2 numeric array)
%       Interpolated target velocity at each query time.
%   - acceleration_units_s2 (N-by-2 numeric array)
%       Interpolated target acceleration at each query time. An invalid
%       target, an out-of-history query, or an undefined derivative throws.
%**************************************************************************
% UNITS
%   - Position is coordinate units and time is seconds.
%**************************************************************************

%% Section 1: Validate The Target And Query Domain

requiredFields = {'time_s', 'position_units'};
if ~isstruct(targetMotion) || ~isscalar(targetMotion) || ~all(isfield(targetMotion, requiredFields))
    error('planner:InvalidTarget', 'targetMotion requires time_s and position_units.');
end
validateattributes(targetMotion.time_s, {'numeric'}, {'real', 'finite', 'vector', 'nonempty'});
sampleTime_s = double(targetMotion.time_s(:));
validateattributes(targetMotion.position_units, {'numeric'}, ...
    {'real', 'finite', 'size', [numel(sampleTime_s), 2]});
if numel(sampleTime_s) < 2 || any(diff(sampleTime_s) <= 0)
    error('planner:InvalidTargetTime', 'Target sample times must strictly increase.');
end

validateattributes(time_s, {'numeric'}, {'real', 'finite', 'vector'});
if any(time_s < sampleTime_s(1) | time_s > sampleTime_s(end))
    error('planner:TargetTimeOutsideHistory', 'Target evaluation cannot extrapolate beyond the supplied history.');
end

methodName = "linear";
if isfield(targetMotion, 'InterpolationMethod')
    methodName = string(targetMotion.InterpolationMethod);
end
if ~isscalar(methodName) || ~any(methodName == ["linear", "pchip"])
    error('planner:InvalidTargetInterpolation', 'Target interpolation must be linear or pchip.');
end
usesPchip = methodName == "pchip";

%% Section 2: Evaluate The Declared Interpolant

if usesPchip
    position_units = interp1(sampleTime_s, double(targetMotion.position_units), time_s(:), 'pchip');
else
    position_units = interp1(sampleTime_s, double(targetMotion.position_units), time_s(:), 'linear');
end
if nargout < 2
    return
end

%% Section 3: Differentiate The Declared Piecewise Polynomial

% Interior knots use the right-hand piece; the final knot uses the left-hand
% piece. A linear corner has no velocity or acceleration and therefore cannot
% supply a matched state.
velocity_units_s     = zeros(numel(time_s), 2);
acceleration_units_s2 = velocity_units_s;
for axisIndex = 1:2
    axisPosition_units = double(targetMotion.position_units(:, axisIndex));
    if usesPchip
        positionPP = pchip(sampleTime_s, axisPosition_units);
    else
        slope_units_s = diff(axisPosition_units) ./ diff(sampleTime_s);
        for sampleIndex = 2:numel(sampleTime_s) - 1
            queryHitsKnot = any(time_s == sampleTime_s(sampleIndex));
            if queryHitsKnot
                slopeChangesAtKnot = slope_units_s(sampleIndex) ~= slope_units_s(sampleIndex - 1);
                if slopeChangesAtKnot
                    error('planner:UndefinedTargetDerivative', ...
                        'A linear target corner has no defined matched derivative.');
                end
            end
        end
        positionPP = mkpp(sampleTime_s, [slope_units_s, axisPosition_units(1:end - 1)]);
    end
    polynomialOrder = positionPP.order;
    if polynomialOrder > 1
        velocityPP = mkpp(positionPP.breaks, positionPP.coefs(:, 1:end - 1) .* (polynomialOrder - 1:-1:1));
        velocity_units_s(:, axisIndex) = ppval(velocityPP, time_s(:));
        if polynomialOrder > 2
            accelerationPP = mkpp(positionPP.breaks, velocityPP.coefs(:, 1:end - 1) .* (polynomialOrder - 2:-1:1));
            acceleration_units_s2(:, axisIndex) = ppval(accelerationPP, time_s(:));
        end
    end
end
end
