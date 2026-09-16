function validation = validateTrajectory(result)
%% Section 0: Header & Readme
% SYNTAX
%   validation = obstacleAvoidance.validateTrajectory(result)
%**************************************************************************
% PURPOSE
%   - Independently check the complete returned polynomial, endpoint states,
%     workspace and derivative extrema, sampled histories, and BMTP planes.
%**************************************************************************
% INPUTS
%   - result (scalar struct)
%       Public planner result record to check independently of the planner.
%       ArrivalTimeTolerance_s bounds comparisons in seconds;
%       ConstraintTolerance bounds coordinates, derivatives, and algebraic residuals.
%**************************************************************************
% OUTPUTS
%   - validation (scalar struct)
%       Stable pass/fail record with individual certificate status. A failed
%       or incomplete result returns Passed = false with an explanatory
%       Message rather than throwing.
%**************************************************************************
% UNITS
%   - Uses the coordinate, time, and derivative units stored in result.
%**************************************************************************

%% Section 1: Initialize The Stable Validation Record

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
validation.PlaneCertificateValid     = false;
validation.MaximumDynamicsResidual   = Inf;
validation.MaximumContinuityResidual = Inf;
validation.MaximumHistoryResidual    = Inf;
if ~isstruct(result) || ~isscalar(result) || ~isfield(result, "Success") || ~result.Success
    return
end
requiredFields = {'Polynomial', 'time_s', 'position_units', 'velocity_units_s', ...
    'acceleration_units_s2', 'jerk_units_s3', 'PlaneCertificate', ...
    'PreparedObstacles', 'Inputs', 'Limits', 'Options', 'Route_units', ...
    'ArrivalTime_s', 'TrajectoryDuration_s'};
if ~all(isfield(result, requiredFields))
    validation.Message = "The result does not contain the complete core motion record.";
    return
end

% Every nested field dereferenced below is checked here so an incomplete
% record is reported, never thrown.
optionFields = {'ConstraintTolerance', 'ArrivalTimeTolerance_s', 'GoalTimeMode', ...
    'MatchTargetVelocity', 'MatchTargetAcceleration', 'WrapX', 'WrapY', ...
    'CollisionClearanceTolerance_units'};
limitFields  = {'xInterval_units', 'yInterval_units', 'maxVelocity_units_s', ...
    'maxAcceleration_units_s2', 'maxJerk_units_s3'};
stateFields  = {'time_s', 'position_units', 'velocity_units_s', 'acceleration_units_s2'};
recordIsComplete = isstruct(result.Options) && isscalar(result.Options) && ...
    all(isfield(result.Options, optionFields)) && ...
    isstruct(result.Limits) && isscalar(result.Limits) && all(isfield(result.Limits, limitFields)) && ...
    isstruct(result.Inputs) && isscalar(result.Inputs) && ...
    all(isfield(result.Inputs, {'obstacles', 'initialState', 'goalState'})) && ...
    isstruct(result.Inputs.initialState) && all(isfield(result.Inputs.initialState, stateFields)) && ...
    isstruct(result.Inputs.goalState) && all(isfield(result.Inputs.goalState, stateFields)) && ...
    (~isfield(result, 'SuppliedLimits') || isfield(result, 'RequestedLimits'));
if recordIsComplete
    targetIsPresent  = isfield(result.Inputs.goalState, 'targetMotion') && ...
        ~isempty(result.Inputs.goalState.targetMotion);
    recordIsComplete = ~targetIsPresent || (isfield(result, 'Intercept') && ...
        isstruct(result.Intercept) && all(isfield(result.Intercept, {'Time_s', 'TargetPosition_units'})));
end
if ~recordIsComplete
    validation.Message = "The result record is missing option, limit, input, or intercept fields.";
    return
end

%% Section 2: Check Polynomial Shape, Time, And Dynamics

polynomial       = result.Polynomial;
polynomialFields = {'Degree', 'SegmentCount', 'SegmentStartTime_s', 'SegmentDuration_s', ...
    'FinalTime_s', 'positionPower_units', 'velocityPower_units_s', ...
    'accelerationPower_units_s2', 'jerkPower_units_s3'};
if ~isstruct(polynomial) || ~isscalar(polynomial) || ~all(isfield(polynomial, polynomialFields))
    validation.Message = "The polynomial record is incomplete.";
    return
end
segmentCount       = polynomial.SegmentCount;
duration_s         = double(polynomial.SegmentDuration_s(:));
segmentStartTime_s = double(polynomial.SegmentStartTime_s(:));
powerArrays        = {polynomial.positionPower_units, polynomial.velocityPower_units_s, ...
    polynomial.accelerationPower_units_s2, polynomial.jerkPower_units_s3};
positionPowerCount = size(powerArrays{1}, 3);

% Every guard below stays inside one short-circuit chain so that a malformed
% record is rejected rather than compared elementwise.
polynomialIsValid = isnumeric(segmentCount) && isscalar(segmentCount) && ...
    segmentCount >= 1 && segmentCount == fix(segmentCount) && ...
    numel(duration_s) == segmentCount && numel(segmentStartTime_s) == segmentCount && ...
    all(isfinite(duration_s)) && all(duration_s > 0) && all(isfinite(segmentStartTime_s)) && ...
    isnumeric(polynomial.FinalTime_s) && isscalar(polynomial.FinalTime_s) && ...
    isfinite(polynomial.FinalTime_s);
for derivativeOrder = 0:3
    powerArray = powerArrays{derivativeOrder + 1};
    polynomialIsValid = polynomialIsValid && isnumeric(powerArray) && ...
        size(powerArray, 1) == segmentCount && size(powerArray, 2) == 2 && ...
        size(powerArray, 3) == positionPowerCount - derivativeOrder && all(isfinite(powerArray), "all");
end
degreeIsValid = isnumeric(polynomial.Degree) && isscalar(polynomial.Degree) && ...
    isfinite(polynomial.Degree) && polynomial.Degree >= 3 && ...
    polynomial.Degree == fix(polynomial.Degree);
polynomialIsValid = polynomialIsValid && degreeIsValid && ...
    positionPowerCount == polynomial.Degree + 1;
validation.PolynomialValid = polynomialIsValid;
if ~polynomialIsValid
    validation.Message = "The polynomial arrays or segment times are invalid.";
    return
end

timeTolerance_s     = result.Options.ArrivalTimeTolerance_s;
tolerance           = result.Options.ConstraintTolerance;
expectedStartTime_s = segmentStartTime_s(1) + [0; cumsum(duration_s(1:end - 1))];
timingResidual_s    = max(abs([segmentStartTime_s - expectedStartTime_s; ...
    polynomial.FinalTime_s - segmentStartTime_s(1) - sum(duration_s)]));
validation.SegmentTimingConsistent = timingResidual_s <= timeTolerance_s;

dynamicsResidual = zeros(0, 1);
for derivativeOrder = 0:2
    sourcePower         = powerArrays{derivativeOrder + 1};
    differentiatedPower = sourcePower(:, :, 2:end) .* ...
        reshape(1:size(sourcePower, 3) - 1, 1, 1, []) ./ reshape(duration_s, [], 1, 1);
    dynamicsResidual = [dynamicsResidual; ...
        differentiatedPower(:) - powerArrays{derivativeOrder + 2}(:)]; %#ok<AGROW>
end
validation.MaximumDynamicsResidual = max(abs(dynamicsResidual));
validation.DynamicsConsistent      = validation.MaximumDynamicsResidual <= tolerance;

% C3 motion requires continuous physical jerk at every interior join.
continuityResidual = zeros(0, 1);
for derivativeOrder = 0:3
    powerArray = powerArrays{derivativeOrder + 1};
    continuityResidual = [continuityResidual; ...
        reshape(sum(powerArray(1:end - 1, :, :), 3) - powerArray(2:end, :, 1), [], 1)]; %#ok<AGROW>
end
if isempty(continuityResidual)
    continuityResidual = 0;
end
validation.MaximumContinuityResidual = max(abs(continuityResidual));
validation.InterSegmentContinuous    = validation.MaximumContinuityResidual <= tolerance;

%% Section 3: Check Continuous Ranges And Endpoints

limits      = result.Limits;
lowerBounds = {[limits.xInterval_units(1) limits.yInterval_units(1)], ...
    -limits.maxVelocity_units_s, -limits.maxAcceleration_units_s2, -limits.maxJerk_units_s3};
upperBounds = {[limits.xInterval_units(2) limits.yInterval_units(2)], ...
    limits.maxVelocity_units_s, limits.maxAcceleration_units_s2, limits.maxJerk_units_s3};
withinLimits = true(1, 4);
for segmentIndex = 1:segmentCount
    for axisIndex = 1:2
        for derivativeOrder = 0:3
            boundIndex   = derivativeOrder + 1;
            coefficients = reshape(powerArrays{boundIndex}(segmentIndex, axisIndex, :), [], 1);
            withinLimits(boundIndex) = withinLimits(boundIndex) && ...
                obstacleAvoidance.validation.certifyPolynomialRange(coefficients, ...
                lowerBounds{boundIndex}(axisIndex), upperBounds{boundIndex}(axisIndex), tolerance);
        end
    end
end
validation.PositionWithinLimits     = withinLimits(1);
validation.VelocityWithinLimits     = withinLimits(2);
validation.AccelerationWithinLimits = withinLimits(3);
validation.JerkWithinLimits         = withinLimits(4);

initialState = result.Inputs.initialState;
goalState    = result.Inputs.goalState;
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

initialPolynomialState = [reshape(powerArrays{1}(1, :, 1), 1, 2), ...
    reshape(powerArrays{2}(1, :, 1), 1, 2), reshape(powerArrays{3}(1, :, 1), 1, 2)];
terminalPolynomialState = [sum(reshape(powerArrays{1}(end, :, :), 2, []), 2).', ...
    sum(reshape(powerArrays{2}(end, :, :), 2, []), 2).', ...
    sum(reshape(powerArrays{3}(end, :, :), 2, []), 2).'];
expectedEndpointState = [initialState.position_units initialState.velocity_units_s ...
    initialState.acceleration_units_s2 goalState.position_units ...
    goalState.velocity_units_s goalState.acceleration_units_s2];
timeMatched = abs(polynomial.SegmentStartTime_s(1) - initialState.time_s) <= timeTolerance_s;
if result.Options.GoalTimeMode == "fixedArrival"
    timeMatched = timeMatched && ...
        abs(polynomial.FinalTime_s - goalState.time_s) <= timeTolerance_s;
else
    timeMatched = timeMatched && polynomial.FinalTime_s <= goalState.time_s + timeTolerance_s;
end

metadataFields       = {'TerminalState'};
metadataIsConsistent = all(isfield(polynomial, metadataFields)) && ...
    all(isfield(polynomial.TerminalState, {'position_units', 'velocity_units_s', 'acceleration_units_s2'}));
if metadataIsConsistent
    terminalStateRecord = polynomial.TerminalState;
    terminalState       = [terminalStateRecord.position_units terminalStateRecord.velocity_units_s ...
        terminalStateRecord.acceleration_units_s2];
    metadataIsConsistent = max(abs(terminalState - terminalPolynomialState)) <= tolerance;
end
metadataIsConsistent = metadataIsConsistent && ...
    abs(result.ArrivalTime_s - polynomial.FinalTime_s) <= timeTolerance_s && ...
    abs(result.TrajectoryDuration_s - sum(duration_s)) <= timeTolerance_s;
if isfield(result, 'SuppliedLimits')
    for limitName = ["maxVelocity_units_s", "maxAcceleration_units_s2", "maxJerk_units_s3"]
        suppliedLimit = result.RequestedLimits.(limitName);
        if isfield(result.SuppliedLimits, limitName) && ~isempty(result.SuppliedLimits.(limitName))
            suppliedLimit = result.SuppliedLimits.(limitName);
            if isscalar(suppliedLimit)
                suppliedLimit = [suppliedLimit suppliedLimit] / sqrt(2);
            end
        end
        metadataIsConsistent = metadataIsConsistent && ...
            isequal(reshape(suppliedLimit, 1, []), result.Limits.(limitName));
    end
end

% A fixed-arrival trial accepted for an earliest-arrival request was
% planned on its declared trial clock: its final time is checked against
% that clock here and against the outer horizon above.
if isfield(result, 'FixedArrivalTrialTime_s')
    metadataIsConsistent = metadataIsConsistent && ...
        abs(result.FixedArrivalTrialTime_s - polynomial.FinalTime_s) <= timeTolerance_s;
end

% A wrapped axis is planned inside the reach band of the whole request.
wrapAxes = [result.Options.WrapX, result.Options.WrapY];
if isfield(result, 'RequestedLimits')
    for intervalName = ["xInterval_units", "yInterval_units"]
        axisIndex              = 1 + (intervalName == "yInterval_units");
        expectedInterval_units = result.RequestedLimits.(intervalName);
        if wrapAxes(axisIndex)
            reach_units            = result.Limits.maxVelocity_units_s(axisIndex) * ...
                (goalState.time_s - initialState.time_s);
            expectedInterval_units = initialState.position_units(axisIndex) + [-reach_units reach_units];
        end
        metadataIsConsistent = metadataIsConsistent && ...
            isequal(expectedInterval_units, result.Limits.(intervalName));
    end
end

% The unwrapped goal must be an image of the requested goal inside the band.
% A lifted target must be the requested target lifted by continuity from the
% initial position and moved by one period offset.
if isfield(result, 'RequestedGoalState') && any(wrapAxes)
    requestedIntervals_units = [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units];
    bands_units              = [result.Limits.xInterval_units; result.Limits.yInterval_units];
    requestedGoal            = result.RequestedGoalState;
    if isfield(requestedGoal, 'targetMotion') && ~isempty(requestedGoal.targetMotion)
        liftedTarget = obstacleAvoidance.input.liftPeriodicTarget( ...
            requestedGoal.targetMotion, initialState.position_units, requestedIntervals_units, wrapAxes);
        actualTarget         = result.Inputs.goalState.targetMotion;
        actualPosition_units = double(actualTarget.position_units);
        offset_units         = zeros(1, 2);
        for axisIndex = find(wrapAxes)
            period_units = diff(requestedIntervals_units(axisIndex, :));
            offset_units(axisIndex) = period_units * round( ...
                (actualPosition_units(1, axisIndex) - liftedTarget.position_units(1, axisIndex)) / period_units);
        end
        metadataIsConsistent = metadataIsConsistent && ...
            isequal(size(actualPosition_units), size(liftedTarget.position_units)) && ...
            isequal(double(actualTarget.time_s(:)), double(requestedGoal.targetMotion.time_s(:))) && ...
            max(abs(actualPosition_units - (liftedTarget.position_units + offset_units)), [], 'all') <= tolerance;
    else
        for axisIndex = find(wrapAxes)
            period_units = diff(requestedIntervals_units(axisIndex, :));
            imageCount   = round( ...
                (goalState.position_units(axisIndex) - requestedGoal.position_units(axisIndex)) / period_units);
            metadataIsConsistent = metadataIsConsistent && ...
                abs(goalState.position_units(axisIndex) - requestedGoal.position_units(axisIndex) - ...
                imageCount * period_units) <= tolerance;
        end
    end
    for axisIndex = find(wrapAxes)
        metadataIsConsistent = metadataIsConsistent && ...
            goalState.position_units(axisIndex) >= bands_units(axisIndex, 1) - tolerance && ...
            goalState.position_units(axisIndex) <= bands_units(axisIndex, 2) + tolerance;
    end
end
if isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion)
    metadataIsConsistent = metadataIsConsistent && ...
        max(abs(result.Inputs.goalState.velocity_units_s - goalState.velocity_units_s)) <= tolerance && ...
        max(abs(result.Inputs.goalState.acceleration_units_s2 - goalState.acceleration_units_s2)) <= tolerance;
    metadataIsConsistent = metadataIsConsistent && ...
        max(abs(result.Intercept.TargetPosition_units - goalState.position_units)) <= tolerance && ...
        abs(result.Intercept.Time_s - polynomial.FinalTime_s) <= timeTolerance_s;
end
validation.OutputMetadataConsistent = metadataIsConsistent;
validation.EndpointStatesMatched    = timeMatched && ...
    max(abs([initialPolynomialState terminalPolynomialState] - expectedEndpointState)) <= tolerance;

%% Section 4: Check Returned Histories And Collision Certificate

[~, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = ...
    bmtpEngine.evaluatePolynomial(polynomial, result.time_s);
historyMatches = ~isempty(result.time_s) && all(isfinite(result.time_s)) && ...
    all(diff(result.time_s) > 0) && ...
    abs(result.time_s(1) - segmentStartTime_s(1)) <= timeTolerance_s && ...
    abs(result.time_s(end) - polynomial.FinalTime_s) <= timeTolerance_s && ...
    isequal(size(position_units), size(result.position_units)) && ...
    isequal(size(velocity_units_s), size(result.velocity_units_s)) && ...
    isequal(size(acceleration_units_s2), size(result.acceleration_units_s2)) && ...
    isequal(size(jerk_units_s3), size(result.jerk_units_s3));
if historyMatches
    historyResidual = [position_units - result.position_units, ...
        velocity_units_s - result.velocity_units_s, ...
        acceleration_units_s2 - result.acceleration_units_s2, ...
        jerk_units_s3 - result.jerk_units_s3];
    validation.MaximumHistoryResidual  = max(abs(historyResidual), [], "all");
    validation.SampledHistoriesMatched = validation.MaximumHistoryResidual <= tolerance;
end
validation.PlaneCertificateValid = verifyPlaneCertificate(result, powerArrays{1});

%% Section 5: Finalize The Independent Decision

validation.Passed = validation.OutputMetadataConsistent && validation.PolynomialValid && ...
    validation.SegmentTimingConsistent && validation.InterSegmentContinuous && ...
    validation.EndpointStatesMatched && validation.SampledHistoriesMatched && ...
    validation.DynamicsConsistent && all(withinLimits) && validation.PlaneCertificateValid;
if validation.Passed
    validation.Message = "Independent polynomial, limit, endpoint, history, and collision checks passed.";
else
    validation.Message = "One or more independent core trajectory checks failed.";
end
end

%% Section 6: Local Functions

function certificateIsValid = verifyPlaneCertificate(result, positionPower_units)
    % Rebuild Bezier controls and directly recheck every active separating plane.
    % Shared source preparation reconstructs merged spans and conservative swept
    % cells; certificate equality and all acceptance tolerances remain unchanged.
    certificate    = result.PlaneCertificate;
    requiredFields = {'Passed', 'Regions_units', 'Planes', 'RegionActiveBySegment', ...
        'RequiredGap_units', 'RoundoffReserve_units', 'AllPairCount', ...
        'VerifiedPairCount', 'ExactRegionCount', 'SolverRegionCount'};
    certificateIsValid = isstruct(certificate) && isscalar(certificate) && ...
        all(isfield(certificate, requiredFields));
    if ~certificateIsValid
        return
    end

    authoritativeInput = result.Inputs.obstacles;
    if isstruct(authoritativeInput) && isfield(authoritativeInput, 'InternalPreparation')
        authoritativeInput = rmfield(authoritativeInput, 'InternalPreparation');
    end
    if (result.Options.WrapX || result.Options.WrapY) && ~isempty(authoritativeInput)
        % Periodic obstacles are rebuilt as the same translated images the
        % planner used, from the supplied obstacles and the record's band.
        authoritativeInput = obstacleAvoidance.input.replicatePeriodicObstacles(authoritativeInput, ...
            [result.RequestedLimits.xInterval_units; result.RequestedLimits.yInterval_units], ...
            [result.Options.WrapX, result.Options.WrapY], ...
            [result.Limits.xInterval_units; result.Limits.yInterval_units]);
    end
    coverageEnd_s = result.Inputs.goalState.time_s;
    if isfield(result, 'TrajectoryCoverageEndTime_s')
        coverageEnd_s = result.TrajectoryCoverageEndTime_s;
    elseif isfield(result, 'FixedArrivalTrialTime_s')
        coverageEnd_s = result.Polynomial.FinalTime_s;
    end
    % Declared coverage may extend past the motion; it may never exclude
    % any part of the returned polynomial's physical interval.
    motionStart_s          = result.Polynomial.SegmentStartTime_s(1);
    motionEnd_s            = result.Polynomial.FinalTime_s;
    coverageExcludesMotion = ~(isnumeric(coverageEnd_s) && isscalar(coverageEnd_s) && isfinite(coverageEnd_s)) || ...
        coverageEnd_s < motionEnd_s - result.Options.ArrivalTimeTolerance_s || ...
        result.Inputs.initialState.time_s > motionStart_s + result.Options.ArrivalTimeTolerance_s;
    if coverageExcludesMotion
        certificateIsValid = false;
        return
    end
    authoritativeObstacles = obstacleAvoidance.obstacles.prepareObstacles(authoritativeInput, ...
        [result.Inputs.initialState.time_s, coverageEnd_s]);

    endpoints_units = [result.Inputs.initialState.position_units; result.Inputs.goalState.position_units];
    endpointTimes_s = [result.Polynomial.SegmentStartTime_s(1); result.Polynomial.FinalTime_s];
    if isfield(result.Inputs.goalState, 'targetMotion') && ~isempty(result.Inputs.goalState.targetMotion)
        endpoints_units(2, :) = obstacleAvoidance.input.targetPositionAtTime( ...
            result.Inputs.goalState.targetMotion, endpointTimes_s(2));
    end
    endpointsAreOccupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(authoritativeObstacles, ...
        endpoints_units(:, 1), endpoints_units(:, 2), endpointTimes_s);
    if any(endpointsAreOccupied)
        certificateIsValid = false;
        return
    end

    usesDynamicCells = isfield(certificate, 'Coverage') && ...
        isfield(certificate.Coverage, 'ActiveTimeInterval_s');
    if usesDynamicCells
        cells = obstacleAvoidance.obstacles.createTimeCells(authoritativeObstacles, ...
            result.Inputs.initialState.time_s, coverageEnd_s);
        regions_units  = cells.Regions_units;
        starts_s       = result.Polynomial.SegmentStartTime_s;
        ends_s         = starts_s + result.Polynomial.SegmentDuration_s;
        expectedActive = starts_s < cells.ActiveTimeInterval_s(:, 2).' & ends_s > cells.ActiveTimeInterval_s(:, 1).';
        coverageDiffers = ~isequal(certificate.Coverage.ActiveTimeInterval_s, cells.ActiveTimeInterval_s) || ...
            ~isfield(certificate.Coverage, 'EndRegions_units') || ...
            ~isequal(certificate.Coverage.EndRegions_units, cells.EndRegions_units);
        if coverageDiffers
            certificateIsValid = false;
            return
        end
    else
        scene         = obstacleAvoidance.obstacles.snapshot( ...
            authoritativeObstacles, result.Inputs.initialState.time_s);
        regions_units = cell(0, 1);
        for obstacleIndex = 1:numel(scene)
            regions_units = [regions_units; scene(obstacleIndex).Regions_units]; %#ok<AGROW>
        end
        expectedActive = true(size(positionPower_units, 1), numel(regions_units));

        % Static certificates cannot certify changing geometry or activity.
        for obstacleIndex = 1:numel(authoritativeObstacles)
            obstacle = authoritativeObstacles(obstacleIndex);
            spanIsUncovered = numel(obstacle.time_s) > 1 && ...
                (result.time_s(1) < obstacle.time_s(1) || result.time_s(end) > obstacle.time_s(end));
            if ~obstacle.InternalPreparation.IsTimeInvariant || spanIsUncovered
                certificateIsValid = false;
                return
            end
        end
    end

    activePairs        = certificate.RegionActiveBySegment;
    expectedPairCount  = nnz(expectedActive);
    certificateIsValid = certificate.Passed && isequaln(certificate.Regions_units, regions_units) && ...
        isequal(activePairs, expectedActive) && isequal(size(activePairs), size(certificate.Planes)) && ...
        certificate.AllPairCount == expectedPairCount && certificate.VerifiedPairCount == expectedPairCount && ...
        certificate.ExactRegionCount == numel(regions_units) && certificate.SolverRegionCount == numel(regions_units);
    if ~certificateIsValid
        return
    end

    controlPoint_units = powerToBernstein(positionPower_units);
    endRegions_units   = cell(0, 1);
    if usesDynamicCells
        endRegions_units = cells.EndRegions_units;
    end
    [~, reserve_units] = bmtpEngine.createCoordinateTolerances(result.Route_units, ...
        result.Limits.xInterval_units, result.Limits.yInterval_units, regions_units, endRegions_units);
    target_units = (1 + 2^20 * eps) * result.Options.CollisionClearanceTolerance_units + reserve_units;
    toleranceDiffers = ~isequal(certificate.RoundoffReserve_units, reserve_units) || ...
        ~isequal(certificate.RequiredGap_units, target_units + reserve_units);
    if toleranceDiffers
        certificateIsValid = false;
        return
    end

    for segmentIndex = 1:size(activePairs, 1)
        if ~usesDynamicCells
            % Recheck every independently rebuilt static cell in one batch.
            % The coefficient products, gap, and roundoff conditions match
            % the scalar verifier used for affine time-dependent geometry.
            verifiedPlanes = bmtpEngine.verifyStaticSeparatingLines(certificate.Planes(segmentIndex, :), ...
                squeeze(controlPoint_units(segmentIndex, :, :)), regions_units, reserve_units, target_units);
            if ~all([verifiedPlanes.Verified])
                certificateIsValid = false;
                return
            end
            continue
        end
        segmentControl_units = squeeze(controlPoint_units(segmentIndex, :, :));
        segmentSpan_s        = ends_s(segmentIndex) - starts_s(segmentIndex);
        activeRegionIndices = find(activePairs(segmentIndex, :));
        if isempty(activeRegionIndices)
            continue
        end
        activeIntervals_s   = cells.ActiveTimeInterval_s(activeRegionIndices, :);
        intervalGroupStarts = [true; any(diff(activeIntervals_s, 1, 1) ~= 0, 2)];
        intervalGroup       = cumsum(intervalGroupStarts);
        for groupIndex = 1:intervalGroup(end)
            groupRegionIndices = activeRegionIndices(intervalGroup == groupIndex);
            activeInterval_s   = cells.ActiveTimeInterval_s(groupRegionIndices(1), :);
            interval = max(0, min(1, ...
                (activeInterval_s - starts_s(segmentIndex)) / segmentSpan_s));
            restricted_units = bmtpEngine.restrictBezier(segmentControl_units, interval);
            interval_s = [max(starts_s(segmentIndex), activeInterval_s(1)), ...
                min(ends_s(segmentIndex), activeInterval_s(2))];
            firstRegions_units = cell(numel(groupRegionIndices), 1);
            lastRegions_units  = cell(numel(groupRegionIndices), 1);
            for localRegionIndex = 1:numel(groupRegionIndices)
                regionIndex = groupRegionIndices(localRegionIndex);
                vertices_units = bmtpEngine.regionOnInterval( ...
                    regions_units{regionIndex}, cells, regionIndex, interval_s);
                firstRegions_units{localRegionIndex} = vertices_units(:, :, 1);
                lastRegions_units{localRegionIndex}  = vertices_units(:, :, end);
            end
            verifiedPlanes = bmtpEngine.verifyMovingSeparatingLines( ...
                certificate.Planes(segmentIndex, groupRegionIndices), ...
                restricted_units, firstRegions_units, lastRegions_units, ...
                reserve_units, target_units);
            if ~all([verifiedPlanes.Verified])
                certificateIsValid = false;
                return
            end
        end
    end
end

function controlPoint_units = powerToBernstein(powerCoefficient_units)
    % Convert each ascending-power segment to same-degree Bezier controls.
    degree    = size(powerCoefficient_units, 3) - 1;
    transform = zeros(degree + 1);
    for bernsteinIndex = 0:degree
        for powerIndex = 0:bernsteinIndex
            transform(bernsteinIndex + 1, powerIndex + 1) = ...
                nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
        end
    end
    powerPages         = permute(powerCoefficient_units, [3 1 2]);
    controlPoint_units = permute(pagemtimes(transform, powerPages), [2 1 3]);
end
