function result = planPeriodicRequest(request, requestContext)
%% Section 0: Header & Readme
% SYNTAX
%   result = obstacleAvoidance.planning.planPeriodicRequest( ...
%       request, requestContext)
%**************************************************************************
% PURPOSE
%   - Plan a request with wrapped axes as plain requests in the unwrapped
%     frame. Obstacles are copied to every period offset that meets the
%     reach band, and every goal image inside the band is a candidate.
%   - Candidates are planned nearest first and each is accepted against the
%     periodic request in the planner's one acceptance gate. The result is
%     the earliest valid candidate (earliestArrival) or the shortest
%     (fixedArrival); a fixed-goal candidate whose chord lower bound cannot
%     beat the incumbent is not planned.
%**************************************************************************
% INPUTS
%   - request (scalar struct)
%       Normalized periodic request. Its goal holds the nearest image or the
%       lifted target, and its wrapped intervals are the reach band.
%   - requestContext (scalar struct)
%       Original obstacles and supplied/requested provenance.
%**************************************************************************
% OUTPUTS
%   - result (scalar struct)
%       Planner record accepted against the periodic request, with a
%       PeriodicImages field recording the candidate offsets and their
%       outcomes. Failure of every candidate returns Success = false.
%**************************************************************************
% UNITS
%   - Coordinate units and seconds.
%**************************************************************************

%% Section 1: Represent The Periodic Scene In The Unwrapped Frame

timer           = tic;
wrapAxes        = [request.options.WrapX, request.options.WrapY];
intervals_units = [requestContext.requestedLimits.xInterval_units; ...
    requestContext.requestedLimits.yInterval_units];
band_units      = [request.limits.xInterval_units; request.limits.yInterval_units];
period_units    = diff(intervals_units, 1, 2).';
images          = obstacleAvoidance.input.replicatePeriodicObstacles( ...
    requestContext.obstacles, intervals_units, wrapAxes, band_units);

planarOptions       = request.options;
planarOptions.WrapX = false;
planarOptions.WrapY = false;
outerRequest                 = obstacleAvoidance.planning.createOuterRequest(request, requestContext);
outerRequest.RequestedLimits = requestContext.requestedLimits;
outerRequest.WrapX           = request.options.WrapX;
outerRequest.WrapY           = request.options.WrapY;

%% Section 2: Enumerate Goal Images Inside The Band, Nearest First

offsetsByAxis = {0, 0};
for axisIndex = find(wrapAxes)
    lowestImage = ceil((band_units(axisIndex, 1) - ...
        request.goalState.position_units(axisIndex)) / period_units(axisIndex));
    highestImage = floor((band_units(axisIndex, 2) - ...
        request.goalState.position_units(axisIndex)) / period_units(axisIndex));
    offsetsByAxis{axisIndex} = (lowestImage:highestImage) * period_units(axisIndex);
end
[dx_units, dy_units] = ndgrid(offsetsByAxis{1}, offsetsByAxis{2});
offsets_units = [dx_units(:), dy_units(:)];
if isempty(offsets_units)
    % No image is reachable inside the band; plan the nearest image so the
    % ordinary request reports the honest failure.
    offsets_units = [0, 0];
end
chord_units = vecnorm(request.goalState.position_units + offsets_units - ...
    request.initialState.position_units, 2, 2);
[~, order]  = sortrows([chord_units, offsets_units]);
offsets_units = offsets_units(order, :);
chord_units   = chord_units(order);

%% Section 3: Plan Each Candidate As A Plain Request And Keep The Best

candidateCount   = size(offsets_units, 1);
reasons          = strings(candidateCount, 1);
wasPlanned       = false(candidateCount, 1);
result           = [];
bestKey          = [];
bestOffset_units = [NaN, NaN];
selectedImageRequest        = [];
selectedImageRequestContext = [];
isEarliest       = request.options.GoalTimeMode == "earliestArrival";
% The chord to a fixed goal bounds motion length and travel time from below;
% a moving target can be met earlier and nearer, so it is never pruned.
canPrune = isempty(request.goalState.targetMotion);
for candidateIndex = 1:candidateCount
    imageRequest           = request;
    imageRequest.goalState = shiftGoal( ...
        request.goalState, offsets_units(candidateIndex, :));
    imageRequest.options   = planarOptions;
    if ~isempty(bestKey) && canPrune
        bound = chord_units(candidateIndex);
        slack = 0;
        if isEarliest
            bound = imageRequest.initialState.time_s + ...
                obstacleAvoidance.input.minimumTravelTime( ...
                imageRequest.initialState, imageRequest.goalState, imageRequest.limits);
            slack = imageRequest.options.ArrivalTimeTolerance_s;
        end
        if bestKey(1) <= bound + slack
            reasons(candidateIndex) = "chordBoundNotBetter";
            continue
        end
    end
    imageRequest.goalState = resolveMatchedImageDerivatives( ...
        imageRequest.goalState, request.options);
    goalIsRequiredEndpoint = isempty(imageRequest.goalState.targetMotion) || ...
        imageRequest.options.GoalTimeMode == "fixedArrival";
    endpointsCoincide = norm(imageRequest.goalState.position_units - ...
        imageRequest.initialState.position_units) <= imageRequest.options.ConstraintTolerance;
    if goalIsRequiredEndpoint && endpointsCoincide
        error("planTrajectory:CoincidentEndpoints", ...
            "Initial and goal positions must be distinct.");
    end
    imageRequestContext = struct( ...
        'obstacles',          {images}, ...
        'suppliedLimits',     imageRequest.limits, ...
        'requestedLimits',    imageRequest.limits, ...
        'suppliedGoalState',  imageRequest.goalState, ...
        'requestedGoalState', imageRequest.goalState, ...
        'outerRequest',       outerRequest);
    wasPlanned(candidateIndex) = true;
    candidate = obstacleAvoidance.planning.planNormalizedRequest( ...
        imageRequest, imageRequestContext);
    reasons(candidateIndex) = candidate.TerminationReason;
    if candidate.Success
        if isEarliest
            key = [candidate.ArrivalTime_s, candidate.MotionLength_units];
        else
            key = [candidate.MotionLength_units, candidate.ArrivalTime_s];
        end
        isBetter = isempty(bestKey) || key(1) < bestKey(1) || (key(1) == bestKey(1) && key(2) < bestKey(2));
        if isBetter
            result                      = candidate;
            selectedImageRequest        = imageRequest;
            selectedImageRequestContext = imageRequestContext;
            bestKey                     = key;
            bestOffset_units            = offsets_units(candidateIndex, :);
        end
    elseif isempty(result)
        result                      = candidate;
        selectedImageRequest        = imageRequest;
        selectedImageRequestContext = imageRequestContext;
    end
end

%% Section 4: Record The Candidates On The Returned Result

if isempty(bestKey) && isfield(result, 'OuterRequest')
    % Every candidate failed: report the nearest image's failure against
    % the periodic request.
    result = assembleFailedImageResult( ...
        selectedImageRequest, selectedImageRequestContext, result, outerRequest);
end
result.PeriodicImages = struct( ...
    'GoalOffset_units',           bestOffset_units, ...
    'CandidateOffsets_units',     offsets_units, ...
    'CandidatePlanned',           wasPlanned, ...
    'CandidateTerminationReason', reasons, ...
    'ObstacleImageCount',         numel(images));
result.ElapsedTime_s = toc(timer);
end

%% Section 5: Local Functions

function goalState = shiftGoal(goalState, offset_units)
    % Move the goal, and a lifted target with it, to another image.
    goalState.position_units = goalState.position_units + offset_units;
    if isfield(goalState, 'targetMotion') && ~isempty(goalState.targetMotion)
        goalState.targetMotion.position_units = goalState.targetMotion.position_units + offset_units;
    end
end

function goalState = resolveMatchedImageDerivatives(goalState, options)
    % Recheck derivatives after target lifting creates the image goal.
    derivativeNames     = ["velocity_units_s", "acceleration_units_s2"];
    matches             = [options.MatchTargetVelocity, options.MatchTargetAcceleration];
    if ~any(matches)
        return
    end
    [~, tgtVel, tgtAcc] = obstacleAvoidance.input.targetPositionAtTime( ...
        goalState.targetMotion, goalState.time_s);
    derivatives         = [tgtVel; tgtAcc];
    for derivativeIndex = find(matches)
        derivativeName     = derivativeNames(derivativeIndex);
        targetDerivative   = derivatives(derivativeIndex, :);
        derivativeResidual = abs(goalState.(derivativeName) - targetDerivative);
        if any(derivativeResidual > options.ConstraintTolerance)
            error('planner:ConflictingTargetDerivative', ...
                'Explicit and matched target derivatives conflict.');
        end
        goalState.(derivativeName) = targetDerivative;
    end
end

function result = assembleFailedImageResult( ...
        imageRequest, imageRequestContext, imageResult, outerRequest)
    % Compose the retained image failure with the periodic declaration.
    declarationRequest           = imageRequest;
    declarationRequest.goalState = imageResult.Inputs.goalState;
    declarationRequest.goalState.time_s = outerRequest.GoalTime_s;
    declarationRequest.options.WrapX = outerRequest.WrapX;
    declarationRequest.options.WrapY = outerRequest.WrapY;
    declarationRequest.options.GoalTimeMode = outerRequest.GoalTimeMode;

    declarationContext                    = imageRequestContext;
    declarationContext.obstacles          = outerRequest.Obstacles;
    declarationContext.suppliedLimits     = outerRequest.SuppliedLimits;
    declarationContext.requestedLimits    = outerRequest.RequestedLimits;
    declarationContext.suppliedGoalState  = outerRequest.SuppliedGoalState;
    declarationContext.requestedGoalState = outerRequest.RequestedGoalState;
    declarationContext.outerRequest       = [];

    result = obstacleAvoidance.planning.createEmptyResult( ...
        imageResult.PreparedObstacles, declarationRequest, declarationContext, ...
        imageResult.VisibilityGraph, imageResult.Attempts, imageResult.ElapsedTime_s);
    declarationFieldNames = ["Inputs", "Limits", "Options", ...
        "SuppliedLimits", "RequestedLimits", "RequestedGoalState", ...
        "SuppliedGoalState", "OuterRequest"];
    for fieldName = reshape(string(fieldnames(imageResult)), 1, [])
        if ~any(fieldName == declarationFieldNames)
            result.(fieldName) = imageResult.(fieldName);
        end
    end
end
