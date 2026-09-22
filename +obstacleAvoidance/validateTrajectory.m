function validation = validateTrajectory(result)
%% Section 0: Header & Readme
% SYNTAX
%   validation = obstacleAvoidance.validateTrajectory(result)
%**************************************************************************
% PURPOSE
%   - Independently check the returned curve, endpoint states, workspace,
%     speed, acceleration, jerk, and sampled motion values.
%   - Rebuild obstacle geometry and check the saved separating lines.
%**************************************************************************
% INPUTS
%   - result (scalar struct)
%       Public planner result record to check independently of the planner.
%       ArrivalTimeTolerance_s bounds comparisons in seconds;
%       ConstraintTolerance allows small differences in coordinates, motion
%       derivatives, and polynomial calculations.
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%       Pass/fail record with a separate flag for each check. A failed
%       or incomplete result returns Passed = false with an explanatory
%       Message rather than throwing.
%**************************************************************************
% UNITS
%   - Uses the coordinate, time, and derivative units stored in result.
%**************************************************************************

%% Section 1: Create The Validation Record And Check Required Fields

validation                           = struct();
validation.Passed                    = false;
validation.Message                   = "No successful motion is available.";
validation.OutputMetadataConsistent  = false;
validation.PolynomialValid           = false;
validation.SegmentTimingConsistent   = false;
validation.InterSegmentContinuous    = false;
validation.EndpointStatesMatched     = false;
validation.SampledHistoriesMatched   = false;
validation.DynamicsConsistent        = false;
validation.PositionWithinLimits      = false;
validation.VelocityWithinLimits      = false;
validation.AccelerationWithinLimits  = false;
validation.JerkWithinLimits          = false;
validation.SeparationProofValid      = false;
validation.MaximumDynamicsResidual   = Inf;
validation.MaximumContinuityResidual = Inf;
validation.MaximumHistoryResidual    = Inf;
if ~isstruct(result) || ~isscalar(result) || ~isfield(result, "Success") || ~result.Success
    return
end
requiredFieldNames = {'Polynomial', 'time_s', 'position_units', 'velocity_units_s', ...
    'acceleration_units_s2', 'jerk_units_s3', 'SeparationProof', ...
    'PreparedObstacles',     'Inputs', 'Limits', 'Options', 'Route_units', ...
    'ArrivalTime_s',         'TrajectoryDuration_s'};
if ~all(isfield(result, requiredFieldNames))
    validation.Message = "The result does not contain the complete core motion record.";
    return
end

% Check required fields before reading them below so missing data can be
% reported in the validation result.
optionFieldNames = {'ConstraintTolerance', 'ArrivalTimeTolerance_s', 'GoalTimeMode', ...
    'MatchTargetVelocity', 'MatchTargetAcceleration', 'WrapX', 'WrapY', ...
    'CollisionClearanceTolerance_units'};
limitFieldNames = {'xInterval_units', 'yInterval_units', 'maxVelocity_units_s', ...
    'maxAcceleration_units_s2', 'maxJerk_units_s3'};
stateFieldNames  = {'time_s', 'position_units', 'velocity_units_s', 'acceleration_units_s2'};
recordIsComplete = isstruct(result.Options) && isscalar(result.Options) && ...
    all(isfield(result.Options, optionFieldNames)) && ...
    isstruct(result.Limits) && isscalar(result.Limits) && all(isfield(result.Limits, limitFieldNames)) && ...
    isstruct(result.Inputs) && isscalar(result.Inputs) && ...
    all(isfield(result.Inputs, {'obstacles', 'initialState', 'goalState'})) && ...
    isstruct(result.Inputs.initialState) && all(isfield(result.Inputs.initialState, stateFieldNames)) && ...
    isstruct(result.Inputs.goalState) && all(isfield(result.Inputs.goalState, stateFieldNames)) && ...
    (~isfield(result, 'SuppliedLimits') || isfield(result, 'RequestedLimits'));
if recordIsComplete
    targetIsPresent = isfield(result.Inputs.goalState, 'targetMotion') && ...
        ~isempty(result.Inputs.goalState.targetMotion);
    recordIsComplete = ~targetIsPresent || (isfield(result, 'Intercept') && ...
        isstruct(result.Intercept) && all(isfield(result.Intercept, {'Time_s', 'TargetPosition_units'})));
end
if ~recordIsComplete
    validation.Message = "The result record is missing option, limit, input, or intercept fields.";
    return
end

%% Section 2: Check Polynomial Arrays And Segment Times

polynomial           = result.Polynomial;
polynomialFieldNames = {'Degree', 'SegmentCount', 'SegmentStartTime_s', 'SegmentDuration_s', ...
    'FinalTime_s',                'positionPower_units', 'velocityPower_units_s', ...
    'accelerationPower_units_s2', 'jerkPower_units_s3'};
if ~isstruct(polynomial) || ~isscalar(polynomial) || ~all(isfield(polynomial, polynomialFieldNames))
    validation.Message = "The polynomial record is incomplete.";
    return
end
segmentCount       = polynomial.SegmentCount;
segmentDuration_s  = double(polynomial.SegmentDuration_s(:));
segmentStartTime_s = double(polynomial.SegmentStartTime_s(:));
coefficientArrays  = {polynomial.positionPower_units, polynomial.velocityPower_units_s, ...
    polynomial.accelerationPower_units_s2, polynomial.jerkPower_units_s3};
positionCoefficientCount = size(coefficientArrays{1}, 3);

% Check counts and sizes before comparing values. Each polynomial array
% stores [segment, x/y, coefficient]; differentiation removes one coefficient.
polynomialIsValid = isnumeric(segmentCount) && isscalar(segmentCount) && ...
    segmentCount >= 1 && segmentCount == fix(segmentCount) && ...
    numel(segmentDuration_s) == segmentCount && numel(segmentStartTime_s) == segmentCount && ...
    all(isfinite(segmentDuration_s)) && all(segmentDuration_s > 0) && all(isfinite(segmentStartTime_s)) && ...
    isnumeric(polynomial.FinalTime_s) && isscalar(polynomial.FinalTime_s) && ...
    isfinite(polynomial.FinalTime_s);
for derivativeOrder = 0:3
    motionCoefficients = coefficientArrays{derivativeOrder + 1};
    polynomialIsValid  = polynomialIsValid && isnumeric(motionCoefficients) && ...
        size(motionCoefficients, 1) == segmentCount && size(motionCoefficients, 2) == 2 && ...
        size(motionCoefficients, 3) == positionCoefficientCount - derivativeOrder && ...
        all(isfinite(motionCoefficients), "all");
end
curveDegreeIsValid = isnumeric(polynomial.Degree) && isscalar(polynomial.Degree) && ...
    isfinite(polynomial.Degree) && polynomial.Degree >= 3 && ...
    polynomial.Degree == fix(polynomial.Degree);
polynomialIsValid = polynomialIsValid && curveDegreeIsValid && ...
    positionCoefficientCount == polynomial.Degree + 1;
validation.PolynomialValid = polynomialIsValid;
if ~polynomialIsValid
    validation.Message = "The polynomial arrays or segment times are invalid.";
    return
end

% Each segment must start when the previous one ends. Final time must equal
% the first start time + the sum of all segment durations.
timeTolerance_s            = result.Options.ArrivalTimeTolerance_s;
constraintTolerance        = result.Options.ConstraintTolerance;
expectedSegmentStartTimes_s = segmentStartTime_s(1) + [0; cumsum(segmentDuration_s(1:end - 1))];
maximumTimingError_s        = max(abs([segmentStartTime_s - expectedSegmentStartTimes_s; ...
    polynomial.FinalTime_s - segmentStartTime_s(1) - sum(segmentDuration_s)]));
validation.SegmentTimingConsistent = maximumTimingError_s <= timeTolerance_s;

%% Section 3: Recompute Derivatives And Check Curve Joins
% Segment progress u = (time - start time) / duration. Calculate physical
% velocity using dp/dt = (dp/du) / duration, then repeat for acceleration
% and jerk. Stored derivatives must match these calculations.
derivativeDifferences = zeros(0, 1);
for derivativeOrder = 0:2
    sourceCoefficients               = coefficientArrays{derivativeOrder + 1};
    calculatedDerivativeCoefficients = sourceCoefficients(:, :, 2:end) .* ...
        reshape(1:size(sourceCoefficients, 3) - 1, 1, 1, []) ./ reshape(segmentDuration_s, [], 1, 1);
    derivativeDifferences = [derivativeDifferences; ...
        calculatedDerivativeCoefficients(:) - coefficientArrays{derivativeOrder + 2}(:)]; %#ok<AGROW>
end
validation.MaximumDynamicsResidual = max(abs(derivativeDifferences));
validation.DynamicsConsistent      = validation.MaximumDynamicsResidual <= constraintTolerance;

% Position, velocity, acceleration, and jerk must agree where two segments
% meet. At progress 1 the value is the sum of the coefficients; at progress 0
% it is the constant coefficient. This is the continuity called C3.
joinDifferences = zeros(0, 1);
for derivativeOrder = 0:3
    motionCoefficients = coefficientArrays{derivativeOrder + 1};
    joinDifferences    = [joinDifferences; ...
        reshape(sum(motionCoefficients(1:end - 1, :, :), 3) - motionCoefficients(2:end, :, 1), [], 1)]; %#ok<AGROW>
end
if isempty(joinDifferences)
    joinDifferences = 0;
end
validation.MaximumContinuityResidual = max(abs(joinDifferences));
validation.InterSegmentContinuous    = validation.MaximumContinuityResidual <= constraintTolerance;

%% Section 4: Check Limits Throughout Every Curve Segment
% Check position, velocity, acceleration, and jerk on both axes over each
% complete segment. Values between returned sample times must also pass.

limits      = result.Limits;
lowerBounds = {[limits.xInterval_units(1) limits.yInterval_units(1)], ...
    -limits.maxVelocity_units_s, -limits.maxAcceleration_units_s2, -limits.maxJerk_units_s3};
upperBounds = {[limits.xInterval_units(2) limits.yInterval_units(2)], ...
    limits.maxVelocity_units_s, limits.maxAcceleration_units_s2, limits.maxJerk_units_s3};
quantityIsWithinLimits = true(1, 4);
for segmentIndex = 1:segmentCount
    for axisIndex = 1:2
        for derivativeOrder = 0:3
            quantityIndex    = derivativeOrder + 1;
            axisCoefficients = reshape(coefficientArrays{quantityIndex}(segmentIndex, axisIndex, :), [], 1);
            quantityIsWithinLimits(quantityIndex) = quantityIsWithinLimits(quantityIndex) && ...
                bmtpEngine.validation.provePolynomialRange(axisCoefficients, ...
                lowerBounds{quantityIndex}(axisIndex), upperBounds{quantityIndex}(axisIndex), constraintTolerance);
        end
    end
end
validation.PositionWithinLimits     = quantityIsWithinLimits(1);
validation.VelocityWithinLimits     = quantityIsWithinLimits(2);
validation.AccelerationWithinLimits = quantityIsWithinLimits(3);
validation.JerkWithinLimits         = quantityIsWithinLimits(4);

%% Section 5: Check The Start And Goal States
initialState = result.Inputs.initialState;
goalState    = result.Inputs.goalState;

% For an intercept, evaluate the target at the actual arrival time. Use its
% velocity or acceleration as a goal requirement only when matching was requested.
if isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion)
    goalState.position_units = obstacleAvoidance.input.targetPositionAtTime( ...
        goalState.targetMotion, polynomial.FinalTime_s);
    if result.Options.MatchTargetVelocity || result.Options.MatchTargetAcceleration
        [~, targetVelocity_units_s, targetAcceleration_units_s2] = ...
            obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion, polynomial.FinalTime_s);
        if result.Options.MatchTargetVelocity
            goalState.velocity_units_s = targetVelocity_units_s;
        end
        if result.Options.MatchTargetAcceleration
            goalState.acceleration_units_s2 = targetAcceleration_units_s2;
        end
    end
end

% Evaluate the curve endpoints as [x y vx vy ax ay]. Compare these with the
% requested endpoint values after accounting for any moving target.
curveInitialStateValues = [reshape(coefficientArrays{1}(1, :, 1), 1, 2), ...
    reshape(coefficientArrays{2}(1, :, 1), 1, 2), reshape(coefficientArrays{3}(1, :, 1), 1, 2)];
curveGoalStateValues = [sum(reshape(coefficientArrays{1}(end, :, :), 2, []), 2).', ...
    sum(reshape(coefficientArrays{2}(end, :, :), 2, []), 2).', ...
    sum(reshape(coefficientArrays{3}(end, :, :), 2, []), 2).'];
requestedEndpointValues = [initialState.position_units initialState.velocity_units_s ...
    initialState.acceleration_units_s2 goalState.position_units ...
    goalState.velocity_units_s goalState.acceleration_units_s2];
endpointTimesMatch = abs(polynomial.SegmentStartTime_s(1) - initialState.time_s) <= timeTolerance_s;
if result.Options.GoalTimeMode == "fixedArrival"
    endpointTimesMatch = endpointTimesMatch && ...
        abs(polynomial.FinalTime_s - goalState.time_s) <= timeTolerance_s;
else
    endpointTimesMatch = endpointTimesMatch && polynomial.FinalTime_s <= goalState.time_s + timeTolerance_s;
end

%% Section 6: Check Reported Values Against The Motion And Request
% Arrival time, duration, and the reported final state must describe the
% polynomial that was just checked.
reportedFieldNames  = {'TerminalState'};
reportedValuesMatch = all(isfield(polynomial, reportedFieldNames)) && ...
    all(isfield(polynomial.TerminalState, {'position_units', 'velocity_units_s', 'acceleration_units_s2'}));
if reportedValuesMatch
    reportedGoalState       = polynomial.TerminalState;
    reportedGoalStateValues = [reportedGoalState.position_units reportedGoalState.velocity_units_s ...
        reportedGoalState.acceleration_units_s2];
    reportedValuesMatch = max(abs(reportedGoalStateValues - curveGoalStateValues)) <= constraintTolerance;
end
reportedValuesMatch = reportedValuesMatch && ...
    abs(result.ArrivalTime_s - polynomial.FinalTime_s) <= timeTolerance_s && ...
    abs(result.TrajectoryDuration_s - sum(segmentDuration_s)) <= timeTolerance_s;

% Check that the planner retained the requested motion limits. A scalar
% limit is shared between axes as x limit = y limit = scalar limit / sqrt(2).
if isfield(result, 'SuppliedLimits')
    for limitFieldName = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"]
        expectedAxisLimits = result.RequestedLimits.(limitFieldName);
        if isfield(result.SuppliedLimits, limitFieldName) && ~isempty(result.SuppliedLimits.(limitFieldName))
            expectedAxisLimits = result.SuppliedLimits.(limitFieldName);
            if isscalar(expectedAxisLimits)
                expectedAxisLimits = [expectedAxisLimits expectedAxisLimits] / sqrt(2);
            end
        end
        reportedValuesMatch = reportedValuesMatch && ...
            isequal(reshape(expectedAxisLimits, 1, []), result.Limits.(limitFieldName));
    end
end

% An earliest-arrival search may accept a trial with a fixed arrival time.
% Its motion must end at that trial time and still meet the original deadline
% checked above. For example, a 12-second trial can meet a 20-second deadline.
if isfield(result, 'FixedArrivalTrialTime_s')
    reportedValuesMatch = reportedValuesMatch && ...
        abs(result.FixedArrivalTrialTime_s - polynomial.FinalTime_s) <= timeTolerance_s;
end

% For a wrapped axis, check the planning interval against:
% maximum travel distance = maximum speed x available time.
% This range contains possible positions; it does not guarantee a usable path.
wrapAxes = [result.Options.WrapX, result.Options.WrapY];
if isfield(result, 'RequestedLimits')
    for intervalFieldName = ["xInterval_units", "yInterval_units"]
        axisIndex              = 1 + (intervalFieldName == "yInterval_units");
        expectedInterval_units = result.RequestedLimits.(intervalFieldName);
        if wrapAxes(axisIndex)
            maximumTravelDistance_units = result.Limits.maxVelocity_units_s(axisIndex) * ...
                (goalState.time_s - initialState.time_s);
            expectedInterval_units = initialState.position_units(axisIndex) + ...
                [-maximumTravelDistance_units maximumTravelDistance_units];
        end
        reportedValuesMatch = reportedValuesMatch && ...
            isequal(expectedInterval_units, result.Limits.(intervalFieldName));
    end
end

% Equivalent wrapped coordinates differ by whole turns: on a 360-unit axis,
% goal = 10 and goal = 370 describe the same location. A moving target must
% follow the original continuous path with one whole-turn shift applied to
% every sample. Both fixed and moving goals must lie inside the planning range.
if isfield(result, 'RequestedGoalState') && any(wrapAxes)
    requestedIntervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    planningIntervals_units  = [result.Limits.xInterval_units; result.Limits.yInterval_units];
    requestedGoalState       = result.RequestedGoalState;
    if isfield(requestedGoalState, 'targetMotion') && ~isempty(requestedGoalState.targetMotion)
        continuousTargetMotion = obstacleAvoidance.input.unwrapTargetPath( ...
            requestedGoalState.targetMotion, initialState.position_units, requestedIntervals_units, wrapAxes);
        plannedTargetMotion         = result.Inputs.goalState.targetMotion;
        plannedTargetPosition_units = double(plannedTargetMotion.position_units);
        targetWrapOffset_units      = zeros(1, 2);
        for axisIndex = find(wrapAxes)
            wrapLength_units = diff(requestedIntervals_units(axisIndex, :));
            targetWrapOffset_units(axisIndex) = wrapLength_units * round( ...
                (plannedTargetPosition_units(1, axisIndex) - ...
                continuousTargetMotion.position_units(1, axisIndex)) / wrapLength_units);
        end
        reportedValuesMatch = reportedValuesMatch && ...
            isequal(size(plannedTargetPosition_units), size(continuousTargetMotion.position_units)) && ...
            isequal(double(plannedTargetMotion.time_s(:)), double(requestedGoalState.targetMotion.time_s(:))) && ...
            max(abs(plannedTargetPosition_units - ...
            (continuousTargetMotion.position_units + targetWrapOffset_units)), [], 'all') <= constraintTolerance;
    else
        for axisIndex = find(wrapAxes)
            wrapLength_units = diff(requestedIntervals_units(axisIndex, :));
            goalWrapCount    = round( ...
                (goalState.position_units(axisIndex) - requestedGoalState.position_units(axisIndex)) / wrapLength_units);
            reportedValuesMatch = reportedValuesMatch && ...
                abs(goalState.position_units(axisIndex) - requestedGoalState.position_units(axisIndex) - ...
                goalWrapCount * wrapLength_units) <= constraintTolerance;
        end
    end
    for axisIndex = find(wrapAxes)
        reportedValuesMatch = reportedValuesMatch && ...
            goalState.position_units(axisIndex) >= planningIntervals_units(axisIndex, 1) - constraintTolerance && ...
            goalState.position_units(axisIndex) <= planningIntervals_units(axisIndex, 2) + constraintTolerance;
    end
end

% Intercept details must agree with the target state at the curve's arrival time.
if isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion)
    reportedValuesMatch = reportedValuesMatch && ...
        max(abs(result.Inputs.goalState.velocity_units_s - goalState.velocity_units_s)) <= constraintTolerance && ...
        max(abs(result.Inputs.goalState.acceleration_units_s2 - goalState.acceleration_units_s2)) <= constraintTolerance;
    reportedValuesMatch = reportedValuesMatch && ...
        max(abs(result.Intercept.TargetPosition_units - goalState.position_units)) <= constraintTolerance && ...
        abs(result.Intercept.Time_s - polynomial.FinalTime_s) <= timeTolerance_s;
end
validation.OutputMetadataConsistent = reportedValuesMatch;
validation.EndpointStatesMatched    = endpointTimesMatch && ...
    max(abs([curveInitialStateValues curveGoalStateValues] - requestedEndpointValues)) <= constraintTolerance;

%% Section 7: Check Sampled Values And Obstacle Separation
% Recalculate the motion at each returned sample time. The sampled arrays
% must describe the same polynomial, starting and ending at the same times.

[~, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = ...
    bmtpEngine.motion.evaluatePolynomial(polynomial, result.time_s);
sampleSizesAndTimesMatch = ~isempty(result.time_s) && all(isfinite(result.time_s)) && ...
    all(diff(result.time_s) > 0) && ...
    abs(result.time_s(1) - segmentStartTime_s(1)) <= timeTolerance_s && ...
    abs(result.time_s(end) - polynomial.FinalTime_s) <= timeTolerance_s && ...
    isequal(size(position_units), size(result.position_units)) && ...
    isequal(size(velocity_units_s), size(result.velocity_units_s)) && ...
    isequal(size(acceleration_units_s2), size(result.acceleration_units_s2)) && ...
    isequal(size(jerk_units_s3), size(result.jerk_units_s3));
if sampleSizesAndTimesMatch
    sampleDifferences = [position_units - result.position_units, ...
        velocity_units_s - result.velocity_units_s, ...
        acceleration_units_s2 - result.acceleration_units_s2, ...
        jerk_units_s3 - result.jerk_units_s3];
    validation.MaximumHistoryResidual  = max(abs(sampleDifferences), [], "all");
    validation.SampledHistoriesMatched = validation.MaximumHistoryResidual <= constraintTolerance;
end
validation.SeparationProofValid = verifySeparationProof(result, coefficientArrays{1});

%% Section 8: Require Every Independent Check To Pass

validation.Passed = validation.OutputMetadataConsistent && validation.PolynomialValid && ...
    validation.SegmentTimingConsistent && validation.InterSegmentContinuous && ...
    validation.EndpointStatesMatched && validation.SampledHistoriesMatched && ...
    validation.DynamicsConsistent && all(quantityIsWithinLimits) && validation.SeparationProofValid;
if validation.Passed
    validation.Message = "Independent polynomial, limit, endpoint, history, and collision checks passed.";
else
    validation.Message = "One or more independent core trajectory checks failed.";
end
end

%% Section 9: Local Functions

function separationIsVerified = verifySeparationProof(result, positionPower_units)
    % A separating line keeps a curve segment and an obstacle on opposite
    % sides. Rebuild the curve controls and obstacle regions, then verify
    % every saved line against those independently reconstructed inputs.
    separationProof    = result.SeparationProof;
    requiredFieldNames = {'Passed', 'Regions_units', 'Planes', 'RegionActiveBySegment', ...
        'RequiredGap_units', 'RoundoffReserve_units', 'AllPairCount', ...
        'VerifiedPairCount', 'ExactRegionCount', 'SolverRegionCount'};
    separationIsVerified = isstruct(separationProof) && isscalar(separationProof) && ...
        all(isfield(separationProof, requiredFieldNames));
    if ~separationIsVerified
        return
    end

    % Rebuild preparation from the original obstacle data. A cached
    % preparation inside the returned record cannot establish correctness.
    obstaclesToRebuild = result.Inputs.obstacles;
    if isstruct(obstaclesToRebuild) && isfield(obstaclesToRebuild, 'InternalPreparation')
        obstaclesToRebuild = rmfield(obstaclesToRebuild, 'InternalPreparation');
    end
    if (result.Options.WrapX || result.Options.WrapY) && ~isempty(obstaclesToRebuild)
        % Wrapped obstacles are rebuilt as the same translated copies the
        % planner used, covering the full unwrapped planning interval.
        obstaclesToRebuild = obstacleAvoidance.input.copyObstaclesAcrossWraps(obstaclesToRebuild, ...
            [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units], ...
            [result.Options.WrapX, result.Options.WrapY], ...
            [result.Limits.xInterval_units; result.Limits.yInterval_units]);
    end
    obstacleCoverageEndTime_s = result.Inputs.goalState.time_s;
    if isfield(result, 'TrajectoryCoverageEndTime_s')
        obstacleCoverageEndTime_s = result.TrajectoryCoverageEndTime_s;
    elseif isfield(result, 'FixedArrivalTrialTime_s')
        obstacleCoverageEndTime_s = result.Polynomial.FinalTime_s;
    end
    % The obstacle checks must cover the whole returned motion. They may
    % extend beyond arrival, but cannot start too late or end too early.
    motionStartTime_s      = result.Polynomial.SegmentStartTime_s(1);
    motionEndTime_s        = result.Polynomial.FinalTime_s;
    coverageExcludesMotion = ~(isnumeric(obstacleCoverageEndTime_s) && ...
        isscalar(obstacleCoverageEndTime_s) && isfinite(obstacleCoverageEndTime_s)) || ...
        obstacleCoverageEndTime_s < motionEndTime_s - result.Options.ArrivalTimeTolerance_s || ...
        result.Inputs.initialState.time_s > motionStartTime_s + result.Options.ArrivalTimeTolerance_s;
    if coverageExcludesMotion
        separationIsVerified = false;
        return
    end
    rebuiltObstacles = obstacleAvoidance.obstacles.prepareObstacles(obstaclesToRebuild, ...
        [result.Inputs.initialState.time_s, obstacleCoverageEndTime_s]);

    % Check endpoint occupancy using the rebuilt obstacles, including the
    % target position at the actual arrival time for an intercept.
    endpoints_units = [result.Inputs.initialState.position_units; result.Inputs.goalState.position_units];
    endpointTimes_s = [result.Polynomial.SegmentStartTime_s(1); result.Polynomial.FinalTime_s];
    if isfield(result.Inputs.goalState, 'targetMotion') && ~isempty(result.Inputs.goalState.targetMotion)
        endpoints_units(2, :) = obstacleAvoidance.input.targetPositionAtTime( ...
            result.Inputs.goalState.targetMotion, endpointTimes_s(2));
    end
    endpointsAreOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(rebuiltObstacles, ...
        endpoints_units(:, 1), endpoints_units(:, 2), endpointTimes_s);
    if any(endpointsAreOccupied)
        separationIsVerified = false;
        return
    end

    % Reconstruct the regions and the time intervals where they apply. A row
    % in the pair table is a curve segment; a column is an obstacle region.
    usesTimedRegions = isfield(separationProof, 'Coverage') && ...
        isfield(separationProof.Coverage, 'ActiveTimeInterval_s');
    if usesTimedRegions
        timedRegions = obstacleAvoidance.obstacles.createTimeCells(rebuiltObstacles, ...
            result.Inputs.initialState.time_s, obstacleCoverageEndTime_s);
        regions_units       = timedRegions.Regions_units;
        segmentStartTime_s  = result.Polynomial.SegmentStartTime_s;
        segmentEndTime_s    = segmentStartTime_s + result.Polynomial.SegmentDuration_s;
        expectedActivePairs = segmentStartTime_s < timedRegions.ActiveTimeInterval_s(:, 2).' & ...
            segmentEndTime_s > timedRegions.ActiveTimeInterval_s(:, 1).';
        timedGeometryDiffers = ~isequal(separationProof.Coverage.ActiveTimeInterval_s, timedRegions.ActiveTimeInterval_s) || ...
            ~isfield(separationProof.Coverage, 'EndRegions_units') || ...
            ~isequal(separationProof.Coverage.EndRegions_units, timedRegions.EndRegions_units);
        if timedGeometryDiffers
            separationIsVerified = false;
            return
        end
    else
        obstacleSnapshot = obstacleAvoidance.obstacles.snapshot( ...
            rebuiltObstacles, result.Inputs.initialState.time_s);
        regions_units = cell(0, 1);
        for obstacleIndex = 1:numel(obstacleSnapshot)
            regions_units = [regions_units; obstacleSnapshot(obstacleIndex).Regions_units]; %#ok<AGROW>
        end
        expectedActivePairs = true(size(positionPower_units, 1), numel(regions_units));

        % One snapshot is sufficient only while every obstacle keeps the
        % same shape and position, and stays active throughout the motion.
        for obstacleIndex = 1:numel(rebuiltObstacles)
            obstacle = rebuiltObstacles(obstacleIndex);
            motionExceedsObstacleHistory = numel(obstacle.time_s) > 1 && ...
                (result.time_s(1) < obstacle.time_s(1) || result.time_s(end) > obstacle.time_s(end));
            if ~obstacle.InternalPreparation.IsTimeInvariant || motionExceedsObstacleHistory
                separationIsVerified = false;
                return
            end
        end
    end

    % The saved proof must contain every reconstructed region and every
    % applicable curve/obstacle pair, with the same shapes and time overlaps.
    reportedActivePairs  = separationProof.RegionActiveBySegment;
    expectedPairCount    = nnz(expectedActivePairs);
    separationIsVerified = separationProof.Passed && isequaln(separationProof.Regions_units, regions_units) && ...
        isequal(reportedActivePairs, expectedActivePairs) && ...
        isequal(size(reportedActivePairs), size(separationProof.Planes)) && ...
        separationProof.AllPairCount == expectedPairCount && separationProof.VerifiedPairCount == expectedPairCount && ...
        separationProof.ExactRegionCount == numel(regions_units) && separationProof.SolverRegionCount == numel(regions_units);
    if ~separationIsVerified
        return
    end

    % Derive the numerical reserve again from the coordinates. Reject a
    % proof whose required gap or rounding allowance differs from the request.
    controlPoint_units = bmtpEngine.motion.powerToBernstein(positionPower_units);
    endRegions_units   = cell(0, 1);
    if usesTimedRegions
        endRegions_units = timedRegions.EndRegions_units;
    end
    [~, roundoffReserve_units] = bmtpEngine.validation.createCoordinateTolerances(result.Route_units, ...
        result.Limits.xInterval_units, result.Limits.yInterval_units, regions_units, endRegions_units);
    requiredSeparation_units = (1 + 2 ^ 20 * eps) * ...
        result.Options.CollisionClearanceTolerance_units + roundoffReserve_units;
    separationTolerancesDiffer = ~isequal(separationProof.RoundoffReserve_units, roundoffReserve_units) || ...
        ~isequal(separationProof.RequiredGap_units, requiredSeparation_units + roundoffReserve_units);
    if separationTolerancesDiffer
        separationIsVerified = false;
        return
    end

    for segmentIndex = 1:size(reportedActivePairs, 1)
        if ~usesTimedRegions
            % For static regions, each line must separate the whole curve
            % segment from its obstacle. Check all regions together.
            checkedSeparatingLines = bmtpEngine.separation.verifyStaticSeparatingLines( ...
                separationProof.Planes(segmentIndex, :), squeeze(controlPoint_units(segmentIndex, :, :)), ...
                regions_units, roundoffReserve_units, requiredSeparation_units);
            if ~all([checkedSeparatingLines.Verified])
                separationIsVerified = false;
                return
            end
            continue
        end
        segmentControlPoint_units = squeeze(controlPoint_units(segmentIndex, :, :));
        segmentDuration_s         = segmentEndTime_s(segmentIndex) - segmentStartTime_s(segmentIndex);
        activeRegionIndices       = find(reportedActivePairs(segmentIndex, :));
        if isempty(activeRegionIndices)
            continue
        end
        % Regions with identical active intervals can share the same portion
        % of the curve. Keep one group for each consecutive matching interval.
        activeIntervals_s  = timedRegions.ActiveTimeInterval_s(activeRegionIndices, :);
        startsNewTimeGroup = [true; any(diff(activeIntervals_s, 1, 1) ~= 0, 2)];
        timeGroupIndices   = cumsum(startsNewTimeGroup);
        for timeGroupIndex = 1:timeGroupIndices(end)
            groupRegionIndices = activeRegionIndices(timeGroupIndices == timeGroupIndex);
            activeInterval_s   = timedRegions.ActiveTimeInterval_s(groupRegionIndices(1), :);

            % Check only the times shared by this segment and obstacle group.
            % Express that overlap as 0-to-1 curve progress and as seconds.
            overlapProgressInterval = max(0, min(1, ...
                (activeInterval_s - segmentStartTime_s(segmentIndex)) / segmentDuration_s));
            overlapControlPoint_units = bmtpEngine.motion.restrictBezier( ...
                segmentControlPoint_units, overlapProgressInterval);
            overlapTimeInterval_s     = [max(segmentStartTime_s(segmentIndex), activeInterval_s(1)), ...
                min(segmentEndTime_s(segmentIndex), activeInterval_s(2))];

            % Evaluate each moving region at both ends of the shared interval.
            % The line check covers the motion between those two polygons.
            overlapStartRegions_units = cell(numel(groupRegionIndices), 1);
            overlapEndRegions_units   = cell(numel(groupRegionIndices), 1);
            for localRegionIndex = 1:numel(groupRegionIndices)
                regionIndex                 = groupRegionIndices(localRegionIndex);
                overlapRegionVertices_units = bmtpEngine.separation.regionOnInterval( ...
                    regions_units{regionIndex}, timedRegions, regionIndex, overlapTimeInterval_s);
                overlapStartRegions_units{localRegionIndex} = overlapRegionVertices_units(:, :, 1);
                overlapEndRegions_units{localRegionIndex}   = overlapRegionVertices_units(:, :, end);
            end
            checkedSeparatingLines = bmtpEngine.separation.verifyMovingSeparatingLines( ...
                separationProof.Planes(segmentIndex, groupRegionIndices), ...
                overlapControlPoint_units, overlapStartRegions_units, overlapEndRegions_units, ...
                roundoffReserve_units, requiredSeparation_units);
            if ~all([checkedSeparatingLines.Verified])
                separationIsVerified = false;
                return
            end
        end
    end
end
