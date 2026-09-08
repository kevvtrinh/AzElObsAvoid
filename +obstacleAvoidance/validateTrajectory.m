function validation = validateTrajectory(result)
%% Section 0: Header & Readme
% SYNTAX
%   validation = obstacleAvoidance.validateTrajectory(result)
%
% PURPOSE
%   - Independently check the complete returned polynomial, endpoint states,
%     workspace and derivative extrema, sampled histories, and BMTP planes.
%
% INPUTS
%   - result: output from planner.
%
% OUTPUTS
%   - validation: stable pass/fail record with individual certificate status.
%
% UNITS
%   - Uses the coordinate, time, and derivative units stored in result.

%% Section 1: Initialize The Stable Validation Record

validation = struct();
validation.Passed                    = false;
validation.Message                   = "No successful motion is available.";
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
validation.CollisionFree             = false;
validation.MaximumDynamicsResidual   = Inf;
validation.MaximumContinuityResidual = Inf;
validation.MaximumHistoryResidual    = Inf;
if ~isstruct(result) || ~isscalar(result) || ~isfield(result, "Success") || ~result.Success
    return;
end
requiredFields = {'Polynomial', 'time_s', 'position_units', 'velocity_units_s', ...
    'acceleration_units_s2', 'jerk_units_s3', 'PlaneCertificate', ...
    'PreparedObstacles', 'Inputs', 'Limits', 'Options'};
if ~all(isfield(result, requiredFields))
    validation.Message = "The result does not contain the complete core motion record.";
    return;
end

%% Section 2: Check Polynomial Shape, Time, And Dynamics

polynomial = result.Polynomial;
polynomialFields = {'SegmentCount', 'SegmentStartTime_s', 'SegmentDuration_s', ...
    'FinalTime_s', 'positionPower_units', 'velocityPower_units_s', ...
    'accelerationPower_units_s2', 'jerkPower_units_s3'};
if ~isstruct(polynomial) || ~isscalar(polynomial) || ~all(isfield(polynomial, polynomialFields))
    validation.Message = "The polynomial record is incomplete.";
    return;
end
segmentCount = polynomial.SegmentCount;
duration_s = double(polynomial.SegmentDuration_s(:));
segmentStartTime_s = double(polynomial.SegmentStartTime_s(:));
powerArrays = {polynomial.positionPower_units, polynomial.velocityPower_units_s, ...
    polynomial.accelerationPower_units_s2, polynomial.jerkPower_units_s3};
positionPowerCount = size(powerArrays{1}, 3);
valid = isnumeric(segmentCount) && isscalar(segmentCount) && segmentCount >= 1 && segmentCount == fix(segmentCount) && numel(duration_s) == segmentCount && numel(segmentStartTime_s) == segmentCount && all(isfinite(duration_s)) && all(duration_s > 0) && all(isfinite(segmentStartTime_s)) && isnumeric(polynomial.FinalTime_s) && isscalar(polynomial.FinalTime_s) && isfinite(polynomial.FinalTime_s);
for derivativeOrder = 0:3
    array = powerArrays{derivativeOrder + 1};
    valid = valid && isnumeric(array) && size(array, 1) == segmentCount && size(array, 2) == 2 && size(array, 3) == positionPowerCount - derivativeOrder && all(isfinite(array), "all");
end
validation.PolynomialValid = valid;
if ~valid
    validation.Message = "The polynomial arrays or segment times are invalid.";
    return;
end
tolerance = result.Options.ConstraintTolerance;
expectedStartTime_s = segmentStartTime_s(1) + [0; cumsum(duration_s(1:end - 1))];
timingResidual_s = max(abs([segmentStartTime_s - expectedStartTime_s; polynomial.FinalTime_s - segmentStartTime_s(1) - sum(duration_s)]));
validation.SegmentTimingConsistent = timingResidual_s <= tolerance;
dynamicsResidual = zeros(0, 1);
for derivativeOrder = 0:2
    source = powerArrays{derivativeOrder + 1};
    derivative = source(:, :, 2:end) .* reshape(1:size(source, 3) - 1, 1, 1, []) ./ reshape(duration_s, [], 1, 1);
    dynamicsResidual = [dynamicsResidual; derivative(:) - powerArrays{derivativeOrder + 2}(:)]; %#ok<AGROW>
end
validation.MaximumDynamicsResidual = max(abs(dynamicsResidual));
validation.DynamicsConsistent = validation.MaximumDynamicsResidual <= tolerance;
continuityResidual = zeros(0, 1);
% C2 motion permits bounded jerk jumps; certify jerk on both closed spans.
for derivativeOrder = 0:2
    array = powerArrays{derivativeOrder + 1};
    continuityResidual = [continuityResidual; reshape(sum(array(1:end - 1, :, :), 3) - array(2:end, :, 1), [], 1)]; %#ok<AGROW>
end
if isempty(continuityResidual)
    continuityResidual = 0;
end
validation.MaximumContinuityResidual = max(abs(continuityResidual));
validation.InterSegmentContinuous = validation.MaximumContinuityResidual <= tolerance;

%% Section 3: Check Continuous Ranges And Endpoints

limits = result.Limits;
lowerBounds = {[limits.xInterval_units(1) limits.yInterval_units(1)], ...
    -limits.maxVelocity_units_s, -limits.maxAcceleration_units_s2, -limits.maxJerk_units_s3};
upperBounds = {[limits.xInterval_units(2) limits.yInterval_units(2)], ...
    limits.maxVelocity_units_s, limits.maxAcceleration_units_s2, limits.maxJerk_units_s3};
within = true(1, 4);
for segmentIndex = 1:segmentCount
    for axisIndex = 1:2
        for derivativeOrder = 0:3
            coefficients = reshape(powerArrays{derivativeOrder + 1}(segmentIndex, axisIndex, :), [], 1);
            within(derivativeOrder + 1) = within(derivativeOrder + 1) && obstacleAvoidance.validation.certifyPolynomialRange(coefficients, lowerBounds{derivativeOrder + 1}(axisIndex), upperBounds{derivativeOrder + 1}(axisIndex), tolerance);
        end
    end
end
validation.PositionWithinLimits     = within(1);
validation.VelocityWithinLimits     = within(2);
validation.AccelerationWithinLimits = within(3);
validation.JerkWithinLimits         = within(4);
initialState = result.Inputs.initialState;
goalState    = result.Inputs.goalState;
if isfield(goalState,'targetMotion') && ~isempty(goalState.targetMotion)
    goalState.position_units = obstacleAvoidance.input.targetPositionAtTime(goalState.targetMotion,polynomial.FinalTime_s);
end
initialPolynomialState = [reshape(powerArrays{1}(1, :, 1), 1, 2), reshape(powerArrays{2}(1, :, 1), 1, 2), reshape(powerArrays{3}(1, :, 1), 1, 2)];
terminalPolynomialState = [sum(reshape(powerArrays{1}(end, :, :), 2, []), 2).', sum(reshape(powerArrays{2}(end, :, :), 2, []), 2).', sum(reshape(powerArrays{3}(end, :, :), 2, []), 2).'];
expectedEndpointState = [initialState.position_units initialState.velocity_units_s initialState.acceleration_units_s2 goalState.position_units goalState.velocity_units_s goalState.acceleration_units_s2];
timeMatched = abs(polynomial.SegmentStartTime_s(1) - initialState.time_s) <= tolerance;
if result.Options.GoalTimeMode == "fixedArrival"
    timeMatched = timeMatched && abs(polynomial.FinalTime_s - goalState.time_s) <= result.Options.ArrivalTimeTolerance_s;
else
    timeMatched = timeMatched && polynomial.FinalTime_s <= goalState.time_s + result.Options.ArrivalTimeTolerance_s;
end
validation.EndpointStatesMatched = timeMatched && max(abs([initialPolynomialState terminalPolynomialState] - expectedEndpointState)) <= tolerance;

%% Section 4: Check Returned Histories And Collision Certificate

[~, position_units, velocity_units_s, acceleration_units_s2, jerk_units_s3] = bmtpEngine.evaluatePolynomial(polynomial, result.time_s);
historyMatches = isequal(size(position_units), size(result.position_units)) && isequal(size(velocity_units_s), size(result.velocity_units_s)) && isequal(size(acceleration_units_s2), size(result.acceleration_units_s2)) && isequal(size(jerk_units_s3), size(result.jerk_units_s3));
if historyMatches
    validation.MaximumHistoryResidual = max(abs([position_units - result.position_units, velocity_units_s - result.velocity_units_s, acceleration_units_s2 - result.acceleration_units_s2, jerk_units_s3 - result.jerk_units_s3]), [], "all");
    validation.SampledHistoriesMatched = validation.MaximumHistoryResidual <= tolerance;
end
validation.PlaneCertificateValid = verifyPlaneCertificate(result, powerArrays{1});
validation.CollisionFree = validation.PlaneCertificateValid;

%% Section 5: Finalize The Independent Decision

validation.Passed = validation.PolynomialValid && validation.SegmentTimingConsistent && validation.InterSegmentContinuous && validation.EndpointStatesMatched && validation.SampledHistoriesMatched && validation.DynamicsConsistent && all(within) && validation.CollisionFree;
if validation.Passed
    validation.Message = "Independent polynomial, limit, endpoint, history, and collision checks passed.";
else
    validation.Message = "One or more independent core trajectory checks failed.";
end
end

%% Section 6: Local Functions

function passed = verifyPlaneCertificate(result, positionPower_units)
    % Rebuild Bezier controls and directly recheck every active separating plane.
    certificate = result.PlaneCertificate;
    requiredFields = {'Passed', 'Regions_units', 'Planes', 'RegionActiveBySegment', ...
        'RequiredGap_units', 'RoundoffReserve_units', 'AllPairCount', ...
        'VerifiedPairCount', 'ExactRegionCount', 'SolverRegionCount'};
    passed = isstruct(certificate) && isscalar(certificate) && all(isfield(certificate, requiredFields));
    if ~passed
        return;
    end
    authoritativeInput = result.Inputs.obstacles;
    if isstruct(authoritativeInput) && isfield(authoritativeInput,'InternalPreparation')
        authoritativeInput = rmfield(authoritativeInput,'InternalPreparation');
    end
    authoritativeObstacles = obstacleAvoidance.obstacles.prepareObstacles(authoritativeInput);
    endpoints_units = [result.Inputs.initialState.position_units;result.Inputs.goalState.position_units];
    endpointTimes_s = [result.Polynomial.SegmentStartTime_s(1);result.Polynomial.FinalTime_s];
    if isfield(result.Inputs.goalState,'targetMotion') && ~isempty(result.Inputs.goalState.targetMotion)
        endpoints_units(2,:) = obstacleAvoidance.input.targetPositionAtTime(result.Inputs.goalState.targetMotion,endpointTimes_s(2));
    end
    occupied = obstacleAvoidance.obstacles.queryObstacleOccupancyAtTime(authoritativeObstacles, ...
        endpoints_units(:,1),endpoints_units(:,2),endpointTimes_s);
    if any(occupied), passed = false; return; end
    scene = obstacleAvoidance.obstacles.snapshot(authoritativeObstacles, result.Inputs.initialState.time_s);
    regions_units = cell(0,1);
    for k = 1:numel(scene), regions_units = [regions_units; scene(k).Regions_units]; end
    expectedActive = true(size(positionPower_units,1),numel(regions_units));
    if isfield(certificate,'Coverage') && isfield(certificate.Coverage,'ActiveTimeInterval_s')
        cells = obstacleAvoidance.obstacles.createTimeCells(authoritativeObstacles, ...
            result.Inputs.initialState.time_s,result.Inputs.goalState.time_s);
        regions_units = cells.Regions_units;
        starts_s = result.Polynomial.SegmentStartTime_s;
        ends_s = starts_s+result.Polynomial.SegmentDuration_s;
        expectedActive = starts_s < cells.ActiveTimeInterval_s(:,2).' & ends_s > cells.ActiveTimeInterval_s(:,1).';
        if ~isequal(certificate.Coverage.ActiveTimeInterval_s,cells.ActiveTimeInterval_s) || ...
                ~isfield(certificate.Coverage,'EndRegions_units') || ...
                ~isequal(certificate.Coverage.EndRegions_units,cells.EndRegions_units)
            passed = false; return;
        end
    else
        % Static certificates cannot certify changing geometry or activity.
        for k = 1:numel(authoritativeObstacles)
            obstacle = authoritativeObstacles(k);
            if ~obstacle.InternalPreparation.IsTimeInvariant || ...
                    (numel(obstacle.time_s) > 1 && (result.time_s(1) < obstacle.time_s(1) || result.time_s(end) > obstacle.time_s(end)))
                passed = false; return;
            end
        end
    end
    activePairs = certificate.RegionActiveBySegment;
    expectedPairCount = nnz(expectedActive);
    passed = certificate.Passed && isequaln(certificate.Regions_units, regions_units) && isequal(activePairs,expectedActive) && isequal(size(activePairs), size(certificate.Planes)) && certificate.AllPairCount == expectedPairCount && certificate.VerifiedPairCount == expectedPairCount && certificate.ExactRegionCount == numel(regions_units) && certificate.SolverRegionCount == numel(regions_units);
    if ~passed
        return;
    end
    controlPoint_units = powerToBernstein(positionPower_units);
    endRegions_units = cell(0,1);
    if exist('cells','var'), endRegions_units = cells.EndRegions_units; end
    [~,~,reserve_units] = bmtpEngine.createCoordinateTolerances(result.Route_units, ...
        result.Limits.xInterval_units,result.Limits.yInterval_units,regions_units,endRegions_units);
    target_units = (1+2^20*eps)*result.Options.CollisionClearanceTolerance_units+reserve_units;
    if ~isequal(certificate.RoundoffReserve_units,reserve_units) || ~isequal(certificate.RequiredGap_units,target_units+reserve_units)
        passed = false; return;
    end
    for segmentIndex = 1:size(activePairs, 1)
        for regionIndex = 1:size(activePairs, 2)
            if ~activePairs(segmentIndex, regionIndex)
                continue;
            end
            restricted_units = squeeze(controlPoint_units(segmentIndex,:,:));
            vertices_units = regions_units{regionIndex};
            if exist('cells','var')
                interval = (cells.ActiveTimeInterval_s(regionIndex,:)-starts_s(segmentIndex))/(ends_s(segmentIndex)-starts_s(segmentIndex));
                restricted_units = bmtpEngine.restrictBezier(restricted_units,max(0,min(1,interval)));
                interval_s = [max(starts_s(segmentIndex),cells.ActiveTimeInterval_s(regionIndex,1)), ...
                    min(ends_s(segmentIndex),cells.ActiveTimeInterval_s(regionIndex,2))];
                vertices_units = bmtpEngine.regionOnInterval(regions_units{regionIndex},cells,regionIndex,interval_s);
            end
            plane = bmtpEngine.verifySeparatingLine(certificate.Planes(segmentIndex, regionIndex), restricted_units, vertices_units, reserve_units, target_units);
            if ~plane.Verified
                passed = false;
                return;
            end
        end
    end
end

function controlPoint_units = powerToBernstein(powerCoefficient_units)
    % Convert each ascending-power segment to same-degree Bezier controls.
    degree = size(powerCoefficient_units, 3) - 1;
    transform = zeros(degree + 1);
    for bernsteinIndex = 0:degree
        for powerIndex = 0:bernsteinIndex
            transform(bernsteinIndex + 1, powerIndex + 1) = nchoosek(bernsteinIndex, powerIndex) / nchoosek(degree, powerIndex);
        end
    end
    powerPages = permute(powerCoefficient_units, [3 1 2]);
    controlPoint_units = permute(pagemtimes(transform, powerPages), [2 1 3]);
end
