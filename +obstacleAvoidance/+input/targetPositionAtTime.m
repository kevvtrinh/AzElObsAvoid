function [position_units, velocity_units_s, acceleration_units_s2] = targetPositionAtTime(targetMotion, time_s)
%% Section 0: Header & Readme
% SYNTAX
%   position_units = obstacleAvoidance.input.targetPositionAtTime(targetMotion, time_s)
%   [position_units, velocity_units_s, acceleration_units_s2] = ...
%       obstacleAvoidance.input.targetPositionAtTime(targetMotion, time_s)
%**************************************************************************
% PURPOSE
%   - Calculate target position at the requested times using the supplied
%     samples. Optionally calculate velocity and acceleration from that path.
%     Requested times must stay between the first and last sample times.
%**************************************************************************
% INPUTS
%   - targetMotion (scalar struct)
%       time_s contains at least two increasing sample times.
%       position_units contains one [x y] row per sample.
%       InterpolationMethod is "linear" (default) or "pchip".
%   - time_s (numeric vector)
%       One or more requested times between the first and last sample times.
%**************************************************************************
% OUTPUTS
%   - position_units (N-by-2 numeric array)
%       Interpolated target position at each query time.
%   - velocity_units_s (N-by-2 numeric array)
%       Target velocity calculated from the path at each requested time.
%   - acceleration_units_s2 (N-by-2 numeric array)
%       Target acceleration calculated from the path at each requested time.
%       Invalid inputs or times outside the sample range cause an error.
%       Requesting velocity at a sharp linear corner also causes an error.
%**************************************************************************
% UNITS
%   - Position: coordinate units; time: seconds; velocity: units/s;
%     acceleration: units/s^2. Output rows are ordered [x y].
%**************************************************************************

%% Section 1: Check Target Samples And Requested Times

requiredFieldNames = {'time_s', 'position_units'};
if ~isstruct(targetMotion) || ~isscalar(targetMotion) || ~all(isfield(targetMotion, requiredFieldNames))
    error('planner:InvalidTarget', 'targetMotion requires time_s and position_units.');
end

% Each position row must have a matching sample time.
validateattributes(targetMotion.time_s, {'numeric'}, {'real', 'finite', 'vector', 'nonempty'});
sampleTime_s = double(targetMotion.time_s(:));
validateattributes(targetMotion.position_units, {'numeric'}, ...
    {'real', 'finite', 'size', [numel(sampleTime_s), 2]});

% At least two samples are needed to describe motion between positions.
% Times must increase: repeated times would give a zero time interval.
if numel(sampleTime_s) < 2 || any(diff(sampleTime_s) <= 0)
    error('planner:InvalidTargetTime', 'Target sample times must strictly increase.');
end

validateattributes(time_s, {'numeric'}, {'real', 'finite', 'vector'});

% Do not estimate target motion before the first sample or after the last.
if any(time_s < sampleTime_s(1) | time_s > sampleTime_s(end))
    error('planner:TargetTimeOutsideHistory', 'Target evaluation cannot extrapolate beyond the supplied history.');
end

% Linear joins samples with straight lines. PCHIP fits cubic curves between
% samples while preserving their shape and avoiding overshoot on each axis.
interpolationMethod = "linear";
if isfield(targetMotion, 'InterpolationMethod')
    interpolationMethod = string(targetMotion.InterpolationMethod);
end

if ~isscalar(interpolationMethod) || ~any(interpolationMethod == ["linear", "pchip"])
    error('planner:InvalidTargetInterpolation', 'Target interpolation must be linear or pchip.');
end
useCubicInterpolation = interpolationMethod == "pchip";

%% Section 2: Calculate Target Position

% Return one [x y] row per requested time, whether times came in as a row or column.
if useCubicInterpolation
    position_units = interp1(sampleTime_s, double(targetMotion.position_units), time_s(:), 'pchip');
else
    position_units = interp1(sampleTime_s, double(targetMotion.position_units), time_s(:), 'linear');
end

% Skip velocity and acceleration calculations when only position is requested.
if nargout < 2
    return
end

%% Section 3: Calculate Target Velocity And Acceleration

% At an interior sample time, use the segment that starts there. At the last
% sample time, use the segment that ends there. For PCHIP, acceleration can
% differ on either side of a sample; this rule chooses which value to return.
velocity_units_s     = zeros(numel(time_s), 2);
acceleration_units_s2 = velocity_units_s;

for axisIndex = 1:2
    axisPosition_units = double(targetMotion.position_units(:, axisIndex));

    if useCubicInterpolation
        positionPolynomial = pchip(sampleTime_s, axisPosition_units);
    else
        % Segment velocity = change in position / change in time.
        segmentVelocity_units_s = diff(axisPosition_units) ./ diff(sampleTime_s);

        % At a linear corner, velocity changes instantly. There is no single
        % velocity at that exact time, so report an error instead of choosing one.
        for sampleIndex = 2:numel(sampleTime_s) - 1
            requestedTimeMatchesSample = any(time_s == sampleTime_s(sampleIndex));
            if requestedTimeMatchesSample
                velocityChangesAtSample = segmentVelocity_units_s(sampleIndex) ~= segmentVelocity_units_s(sampleIndex - 1);
                if velocityChangesAtSample
                    error('planner:UndefinedTargetDerivative', ...
                        'A linear target corner has no defined matched derivative.');
                end
            end
        end

        % Each straight segment is position = starting position + velocity x
        % elapsed time. MATLAB stores its coefficients as [velocity position].
        positionPolynomial = mkpp(sampleTime_s, [segmentVelocity_units_s, axisPosition_units(1:end - 1)]);
    end

    % Differentiate each position segment to get velocity, then acceleration.
    % Example: position = a x t^2 + b x t + c gives velocity = 2 x a x t + b.
    % MATLAB stores coefficients from the highest power of time to the lowest.
    polynomialOrder = positionPolynomial.order;
    if polynomialOrder > 1
        velocityPolynomial = mkpp(positionPolynomial.breaks, ...
            positionPolynomial.coefs(:, 1:end - 1) .* (polynomialOrder - 1:-1:1));
        velocity_units_s(:, axisIndex) = ppval(velocityPolynomial, time_s(:));

        % Linear segments have constant velocity, so their acceleration stays 0.
        if polynomialOrder > 2
            accelerationPolynomial = mkpp(positionPolynomial.breaks, ...
                velocityPolynomial.coefs(:, 1:end - 1) .* (polynomialOrder - 2:-1:1));
            acceleration_units_s2(:, axisIndex) = ppval(accelerationPolynomial, time_s(:));
        end
    end
end
end
