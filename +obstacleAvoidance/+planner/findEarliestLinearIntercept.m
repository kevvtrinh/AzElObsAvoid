function [interceptTime_s, diagnostics] = findEarliestLinearIntercept(initialState, targetTime_s, targetPosition_units, limits, horizonTime_s)
%% Section 0: Header & Readme
% SYNTAX
%   [interceptTime_s, diagnostics] = ...
%       obstacleAvoidance.planner.findEarliestLinearIntercept( ...
%       initialState, targetTime_s, targetPosition_units, limits, horizonTime_s)
%
% PURPOSE
%   - Find the globally earliest obstacle-free position-only interception
%     of a piecewise-linear target by a rest-to-rest triple integrator.
%
% INPUTS
%   - initialState (scalar struct)
%       Requires scalar time_s and one-by-D position_units. Velocity and
%       acceleration must be omitted, empty, or zero.
%   - targetTime_s (N-by-1 numeric vector)
%       Strictly increasing absolute target sample times.
%   - targetPosition_units (N-by-D numeric array)
%       Target positions joined by linear interpolation.
%   - limits (scalar struct)
%       Positive one-by-D maxVelocity_units_s, maxAcceleration_units_s2, and
%       maxJerk_units_s3 limits.
%   - horizonTime_s (finite scalar)
%       Latest allowed absolute intercept time.
%
% OUTPUTS
%   - interceptTime_s (scalar)
%       Earliest complete polynomial-inequality solution, or NaN.
%   - diagnostics (scalar struct)
%       Search coverage, algebraic residuals, and termination reason.
%
% UNITS
%   - Position is coordinate units; time is seconds; derivatives use units/s, units/s^2,
%     and units/s^3. Histories are N-by-D.
%

%% Section 1: Normalize The Algebraic Request

if ~isstruct(initialState) || ~isscalar(initialState) || ~all(isfield(initialState, {'time_s', 'position_units'}))
    error("findEarliestLinearIntercept:InvalidInitialState", "initialState requires scalar time_s and one-by-D position_units.");
end
initialTime_s       = double(initialState.time_s);
initialPosition_units = double(initialState.position_units(:).');
validateattributes(initialTime_s, {'numeric'}, {'real', 'finite', 'scalar'});
validateattributes(initialPosition_units, {'numeric'}, {'real', 'finite', 'vector', 'nonempty'});
dimensionCount = numel(initialPosition_units);
if any([stateDerivative(initialState, "velocity_units_s", dimensionCount), stateDerivative(initialState, "acceleration_units_s2", dimensionCount)] ~= 0)
    error("findEarliestLinearIntercept:NonrestInitialState", "The exact linear-target kernel requires zero initial velocity and acceleration.");
end
targetTime_s = double(targetTime_s(:));
validateattributes(targetTime_s, {'numeric'}, {'real', 'finite', 'vector', 'increasing', 'nonempty'});
if numel(targetTime_s) < 2
    error("findEarliestLinearIntercept:ShortTargetHistory", "targetTime_s requires at least two samples.");
end
validateattributes(targetPosition_units, {'numeric'}, {'real', 'finite', '2d', 'nrows', numel(targetTime_s), 'ncols', dimensionCount});
targetPosition_units = double(targetPosition_units);
validateattributes(horizonTime_s, {'numeric'}, {'real', 'finite', 'scalar', '>', initialTime_s});
horizonTime_s              = min(double(horizonTime_s), targetTime_s(end));
maximumVelocity_units_s      = limitRow(limits, "maxVelocity_units_s", dimensionCount);
maximumAcceleration_units_s2 = limitRow(limits, "maxAcceleration_units_s2", dimensionCount);
maximumJerk_units_s3         = limitRow(limits, "maxJerk_units_s3", dimensionCount);
switchTime_s               = zeros(2, dimensionCount);
% Evaluate each coordinate axis and combine its limiting result.
for axisIndex = 1:dimensionCount
    switchTime_s(:, axisIndex) = reachableSwitches(maximumVelocity_units_s(axisIndex), maximumAcceleration_units_s2(axisIndex), maximumJerk_units_s3(axisIndex));
end

%% Section 2: Enumerate Every Target And Switching Regime

interceptTime_s         = NaN;
testedCount             = 0;
rootCount               = 0;
maximumRootResidual_units = 0;
selectedSlack_units       = NaN(1, dimensionCount);
selectedSegmentIndex    = 0;
selectedElapsedTime_s   = NaN;
% Process each segment while assembling the complete motion or interval result.
for segmentIndex = 1:numel(targetTime_s) - 1
    segmentStart_s = max(initialTime_s, targetTime_s(segmentIndex));
    segmentEnd_s   = min(horizonTime_s, targetTime_s(segmentIndex + 1));
    if segmentEnd_s < segmentStart_s
        continue;
    end
    segmentDuration_s = targetTime_s(segmentIndex + 1) - targetTime_s(segmentIndex);
    targetSlope_units_s = (targetPosition_units(segmentIndex + 1, :) - targetPosition_units(segmentIndex, :)) / segmentDuration_s;
    targetOffset_units  = targetPosition_units(segmentIndex, :) + targetSlope_units_s * (initialTime_s - targetTime_s(segmentIndex)) - initialPosition_units;
    elapsedStart_s    = segmentStart_s - initialTime_s;
    elapsedEnd_s      = segmentEnd_s - initialTime_s;
    movingAxis        = targetSlope_units_s ~= 0;
    signChange_s      = -targetOffset_units(movingAxis) ./ targetSlope_units_s(movingAxis);
    eventTime_s       = [elapsedStart_s; elapsedEnd_s; ...
        switchTime_s(isfinite(switchTime_s)); signChange_s(:)];
    eventTime_s      = unique(eventTime_s(eventTime_s >= elapsedStart_s & eventTime_s <= elapsedEnd_s));
    transitionTime_s = eventTime_s;

    % Process each slab in temporal order and accumulate its result.
    for slabIndex = 1:numel(eventTime_s) - 1
        slabStart_s    = eventTime_s(slabIndex);
        slabEnd_s      = eventTime_s(slabIndex + 1);
        slabMidpoint_s = 0.5 * (slabStart_s + slabEnd_s);
        % Evaluate each coordinate axis and combine its limiting result.
        for axisIndex = 1:dimensionCount
            differencePower = reachablePower(slabMidpoint_s, maximumVelocity_units_s(axisIndex), maximumAcceleration_units_s2(axisIndex), maximumJerk_units_s3(axisIndex));
            targetSign      = sign(targetOffset_units(axisIndex) + targetSlope_units_s(axisIndex) * slabMidpoint_s);
            if targetSign == 0
                targetSign = 1;
            end
            differencePower(1:2) = differencePower(1:2) - targetSign * [targetOffset_units(axisIndex), targetSlope_units_s(axisIndex)];
            slabRoot_s = realRoots(differencePower, slabStart_s, slabEnd_s);
            if ~isempty(slabRoot_s)
                maximumRootResidual_units = max(maximumRootResidual_units, max(abs(polyval(flip(differencePower), slabRoot_s))));
                rootCount               = rootCount + numel(slabRoot_s);
                transitionTime_s        = [transitionTime_s; slabRoot_s]; %#ok<AGROW>
            end
        end
    end
    transitionTime_s = unique(sort(transitionTime_s));

    % Select the earliest interval where all axes can reach the target.

    for probeIndex = 1:(2 * numel(transitionTime_s) - 1)
        boundaryIndex          = ceil(probeIndex / 2);
        candidateElapsedTime_s = transitionTime_s(boundaryIndex);
        elapsedTime_s          = candidateElapsedTime_s;
        if mod(probeIndex, 2) == 0
            elapsedTime_s = 0.5 * (transitionTime_s(boundaryIndex) + transitionTime_s(boundaryIndex + 1));
        end
        testedCount = testedCount + 1;
        % Discard infeasible intercept times and continue searching later event intervals.
        if ~isFeasible(elapsedTime_s, targetOffset_units, targetSlope_units_s, maximumVelocity_units_s, maximumAcceleration_units_s2, maximumJerk_units_s3)
            continue;
        end
        [boundaryFeasible, boundarySlack_units] = isFeasible(candidateElapsedTime_s, targetOffset_units, targetSlope_units_s, maximumVelocity_units_s, maximumAcceleration_units_s2, maximumJerk_units_s3);
        % Skip infeasible boundary times; feasible boundaries remain candidates for the earliest intercept.
        if ~boundaryFeasible
            candidateElapsedTime_s = candidateElapsedTime_s + 64 * eps(max(1, candidateElapsedTime_s));
            [boundaryFeasible, boundarySlack_units] = isFeasible(candidateElapsedTime_s, targetOffset_units, targetSlope_units_s, maximumVelocity_units_s, maximumAcceleration_units_s2, maximumJerk_units_s3);
        end
        % Accept the first feasible boundary because times are examined in increasing order.
        if boundaryFeasible
            interceptTime_s       = initialTime_s + candidateElapsedTime_s;
            selectedElapsedTime_s = candidateElapsedTime_s;
            selectedSlack_units     = boundarySlack_units;
            selectedSegmentIndex  = segmentIndex;
            break;
        end
    end
    if isfinite(interceptTime_s)
        break;
    end
end

%% Section 3: Assemble Complete Search Diagnostics

if isfinite(interceptTime_s)
    terminationReason = "directInterceptRootFound";
    message           = "The earliest piecewise-linear direct intercept was found.";
else
    terminationReason = "directInterceptWindowExhausted";
    message           = "No direct rest-to-rest intercept exists in the supplied window.";
end
diagnostics = struct("Success", isfinite(interceptTime_s), ...
    "Message", message, "TerminationReason", terminationReason, ...
    "Policy", "completePiecewisePolynomialEvents", ...
    "TargetSegmentCount", numel(targetTime_s) - 1, ...
    "SelectedTargetSegmentIndex", selectedSegmentIndex, ...
    "TestedCellOrBoundaryCount", testedCount, ...
    "AlgebraicRootCount", rootCount, ...
    "MaximumRootResidual_units", maximumRootResidual_units, ...
    "SelectedElapsedTime_s", selectedElapsedTime_s, ...
    "SelectedAxisSlack_units", selectedSlack_units, ...
    "SearchStartTime_s", max(initialTime_s, targetTime_s(1)), ...
    "SearchEndTime_s", horizonTime_s, "InterceptTime_s", interceptTime_s);
end

%% Section 4: Local Functions

function value = stateDerivative(state, fieldName, dimensionCount)
    % Default missing derivatives and check their dimensions.
    value = zeros(1, dimensionCount);
    if isfield(state, fieldName) && ~isempty(state.(fieldName))
        value = double(state.(fieldName)(:).');
        validateattributes(value, {'numeric'}, {'real', 'finite', 'vector', 'numel', dimensionCount});
    end
end

function value = limitRow(limits, fieldName, dimensionCount)
    % Read a positive limit for each axis.
    if ~isstruct(limits) || ~isscalar(limits) || ~isfield(limits, fieldName)
        error("findEarliestLinearIntercept:MissingLimit", "limits.%s is required.", fieldName);
    end
    value = double(limits.(fieldName)(:).');
    validateattributes(value, {'numeric'}, {'real', 'finite', 'vector', 'numel', dimensionCount, 'positive'});
end

function switchTime_s = reachableSwitches(velocity, acceleration, jerk)
    % Return switches between jerk-, acceleration-, and speed-limited regimes.
    ramp_s = acceleration / jerk;
    if velocity <= acceleration ^ 2 / jerk
        switchTime_s = [4 * sqrt(velocity / jerk); Inf];
    else
        switchTime_s = [4 * ramp_s; 2 * (velocity / acceleration + ramp_s)];
    end
end

function power = reachablePower(time_s, velocity, acceleration, jerk)
    % Return ascending coefficients of maximum rest-to-rest distance D(T).
    ramp_s = acceleration / jerk;
    if velocity <= acceleration ^ 2 / jerk
        velocityRamp_s = sqrt(velocity / jerk);
        if time_s <= 4 * velocityRamp_s
            power = [0, 0, 0, jerk / 32];
        else
            power = [-2 * velocity * velocityRamp_s, velocity, 0, 0];
        end
    elseif time_s <= 4 * ramp_s
        power = [0, 0, 0, jerk / 32];
    elseif time_s <= 2 * (velocity / acceleration + ramp_s)
        power = [0, -acceleration * ramp_s / 2, acceleration / 4, 0];
    else
        power = [-velocity * (velocity / acceleration + ramp_s), velocity, 0, 0];
    end
end

function root_s = realRoots(power, lower_s, upper_s)
    % Find real roots within this closed interval.
    scale     = max(1, max(abs(power)));
    lastIndex = find(abs(power) > 64 * eps(scale), 1, "last");
    if isempty(lastIndex) || lastIndex == 1
        root_s = zeros(0, 1);
        return;
    end
    candidate   = roots(flip(power(1:lastIndex)));
    isReal      = abs(imag(candidate)) <= 128 * eps(max(1, max(abs(real(candidate))))) .* max(1, abs(real(candidate)));
    candidate   = real(candidate(isReal));
    tolerance_s = 128 * eps(max(1, max(abs([lower_s upper_s]))));
    candidate   = candidate(candidate >= lower_s - tolerance_s & candidate <= upper_s + tolerance_s);
    root_s      = unique(min(upper_s, max(lower_s, candidate)));
end

function [feasible, slack_units] = isFeasible(elapsedTime_s, targetOffset_units, targetSlope_units_s, maximumVelocity_units_s, maximumAcceleration_units_s2, maximumJerk_units_s3)
    % Evaluate every componentwise reachable-distance inequality conservatively.
    slack_units = zeros(size(targetOffset_units));
    % Evaluate each coordinate axis and combine its limiting result.
    for axisIndex = 1:numel(targetOffset_units)
        power                = reachablePower(elapsedTime_s, maximumVelocity_units_s(axisIndex), maximumAcceleration_units_s2(axisIndex), maximumJerk_units_s3(axisIndex));
        requiredDistance_units = abs(targetOffset_units(axisIndex) + targetSlope_units_s(axisIndex) * elapsedTime_s);
        slack_units(axisIndex) = polyval(flip(power), elapsedTime_s) - requiredDistance_units;
    end
    reserve_units = 256 * eps(max(1, max(abs([targetOffset_units, targetSlope_units_s * elapsedTime_s]))));
    feasible    = all(slack_units >= -reserve_units);
end
