function [controlPoint_deg, segmentTime_s, exitFlag, output] = ...
        solveTrajectoryStep( ...
        segmentCount, degree, start_deg, goal_deg, limits, planes, reserve_deg, ...
        maximumMotionDuration_s, goalTimeMode, travelSavingsRate_deg_s, ...
        referenceSegmentTime_s, options)
%% Section 0: Header & Readme
% SYNTAX
%   [controlPoint_deg, segmentTime_s, exitFlag, output] = ...
%       bmtpEngine.solveTrajectoryStep( ...
%       segmentCount, degree, start_deg, goal_deg, limits, planes, ...
%       reserve_deg, maximumMotionDuration_s, goalTimeMode, ...
%       travelSavingsRate_deg_s, referenceSegmentTime_s, options)
%**************************************************************************
% PURPOSE
%   - Solve one convex trajectory step for fixed separating lines, timing
%     policy, and derivative limits.
%**************************************************************************
% INPUTS
%   - segmentCount, degree (positive integer scalars)
%       Composite Bezier representation size.
%   - start_deg, goal_deg (1-by-2 numeric rows)
%       Fixed endpoint positions.
%   - limits (scalar struct)
%       Workspace, velocity, acceleration, and jerk limits.
%   - planes (S-by-R struct array)
%       Fixed active separating-line constraints.
%   - reserve_deg (nonnegative scalar)
%       Numerical separation reserve.
%   - maximumMotionDuration_s (positive scalar)
%       Upper bound or fixed motion duration.
%   - goalTimeMode (scalar text)
%       earliestArrival, balancedArrival, or fixedArrival.
%   - travelSavingsRate_deg_s, referenceSegmentTime_s (numeric scalars)
%       Travel/time objective rate and linearization time.
%   - options (coneprog options)
%       Numerical solver controls.
%**************************************************************************
% OUTPUTS
%   - controlPoint_deg (S-by-(D+1)-by-2 numeric array)
%       Solved control points, or an empty array on expected solve failure.
%   - segmentTime_s (scalar numeric)
%       Common segment time, or NaN on expected solve failure.
%   - exitFlag (numeric scalar), output (solver record)
%       Fastcone method, acceptance checks, and original coneprog recovery.
%**************************************************************************
% UNITS
%   - Position is degrees and time is seconds.
%**************************************************************************

%% Section 1: Create Decision Bounds And Continuity Rows

controlCount = segmentCount * (degree + 1) * 2;
powerIndex = controlCount + (1:4);
travelBoundCount = ...
    (goalTimeMode ~= "earliestArrival") * segmentCount * degree;
travelBoundIndex = controlCount + 4 + (1:travelBoundCount);
variableCount = controlCount + 4 + travelBoundCount;
differenceCoefficients = {1, [-1 1], [1 -2 1], [-1 3 -3 1]};
baseInequalityCount = 4 * segmentCount * (3 * degree - 3);
activePlaneCount = nnz(reshape([planes.Active], size(planes)));
inequalityCount = baseInequalityCount + ...
    activePlaneCount * (degree + 2);
equalityCount = 13 + 8 * (segmentCount - 1);
A = spalloc(inequalityCount, variableCount, 6 * inequalityCount);
Aeq = spalloc(equalityCount, variableCount, 8 * equalityCount);
beq = zeros(equalityCount, 1);
lb = -Inf(variableCount, 1);
ub = Inf(variableCount, 1);
domain_deg = [limits.azimuthInterval_deg; limits.elevationInterval_deg];
lb(1:controlCount) = repmat( ...
    domain_deg(:, 1), segmentCount * (degree + 1), 1);
ub(1:controlCount) = repmat( ...
    domain_deg(:, 2), segmentCount * (degree + 1), 1);
lb(powerIndex) = 0;
lb(powerIndex(2)) = eps;
lb(travelBoundIndex) = 0;
equalityIndex = 0;
for axisIndex = 1:2
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, ...
        controlIndexOf(1, 0, axisIndex, degree)) = 1; %#ok<SPRIX>
    beq(equalityIndex) = start_deg(axisIndex);
    equalityIndex = equalityIndex + 1;
    Aeq(equalityIndex, controlIndexOf( ...
        segmentCount, degree, axisIndex, degree)) = 1; %#ok<SPRIX>
    beq(equalityIndex) = goal_deg(axisIndex);
    for endpointOrder = 1:2
        equalityIndex = equalityIndex + 1;
        indices = controlIndexOf( ...
            1, [endpointOrder 0], axisIndex, degree);
        Aeq(equalityIndex, indices) = [1 -1]; %#ok<SPRIX>
        equalityIndex = equalityIndex + 1;
        indices = controlIndexOf(segmentCount, ...
            [degree - endpointOrder degree], axisIndex, degree);
        Aeq(equalityIndex, indices) = [1 -1]; %#ok<SPRIX>
    end
end
for segmentIndex = 1:segmentCount - 1
    for order = 0:3
        coefficients = differenceCoefficients{order + 1};
        coefficientIndex = 0:order;
        for axisIndex = 1:2
            equalityIndex = equalityIndex + 1;
            left = controlIndexOf(segmentIndex, ...
                degree - order + coefficientIndex, axisIndex, degree);
            right = controlIndexOf( ...
                segmentIndex + 1, coefficientIndex, axisIndex, degree);
            Aeq(equalityIndex, left) = coefficients; %#ok<SPRIX>
            Aeq(equalityIndex, right) = ...
                Aeq(equalityIndex, right) - coefficients; %#ok<SPRIX>
        end
    end
end
equalityIndex = equalityIndex + 1;
Aeq(equalityIndex, powerIndex(1)) = 1;
beq(equalityIndex) = 1;

%% Section 2: Create Derivative And Separating-Line Bounds

limitValues = [limits.maxVelocity_deg_s; ...
    limits.maxAcceleration_deg_s2; limits.maxJerk_deg_s3];
inequalityIndex = 0;
for segmentIndex = 1:segmentCount
    controlColumns = (segmentIndex - 1) * 2 * (degree + 1) + ...
        (1:2 * (degree + 1));
    for order = 1:3
        coefficients = differenceCoefficients{order + 1};
        scale = factorial(degree) / factorial(degree - order);
        derivativeCount = degree - order + 1;
        derivativeRows = spdiags(repmat( ...
            scale * coefficients, derivativeCount, 1), ...
            0:order, derivativeCount, degree + 1);
        signedRows = kron(kron(derivativeRows, speye(2)), [1; -1]);
        targets = inequalityIndex + (1:size(signedRows, 1));
        A(targets, controlColumns) = signedRows; %#ok<SPRIX>
        axisLimits = repmat(limitValues(order, :), derivativeCount, 1);
        A(targets, powerIndex(order + 1)) = ...
            -repelem(reshape(axisLimits.', [], 1), 2); %#ok<SPRIX>
        inequalityIndex = targets(end);
    end
end
b = zeros(inequalityCount, 1);
inequalityIndex = baseInequalityCount;
for segmentIndex = 1:segmentCount
    for regionIndex = 1:size(planes, 2)
        plane = planes(segmentIndex, regionIndex);
        if ~plane.Active
            continue;
        end
        [rows, offset_deg] = fixedPlaneRows( ...
            plane, degree, variableCount, segmentIndex);
        targets = inequalityIndex + (1:size(rows, 1));
        A(targets, :) = rows; %#ok<SPRIX>
        b(targets) = -reserve_deg - offset_deg;
        inequalityIndex = targets(end);
    end
end

%% Section 3: Create The Objective And Solve

cones = [createTimePowerCones(variableCount, powerIndex); ...
    createTravelBoundCones(variableCount, travelBoundIndex, ...
    segmentCount, degree)];
f = zeros(variableCount, 1);
if goalTimeMode == "earliestArrival"
    f(powerIndex(4)) = 1;
else
    f(travelBoundIndex) = 1;
end
maximumSegmentTime_s = maximumMotionDuration_s / segmentCount;
timePowers_s = [1; maximumSegmentTime_s; ...
    maximumSegmentTime_s ^ 2; maximumSegmentTime_s ^ 3];
if goalTimeMode == "fixedArrival"
    lb(powerIndex) = timePowers_s;
    ub(powerIndex) = timePowers_s;
else
    ub(powerIndex) = timePowers_s;
end
if goalTimeMode == "balancedArrival"
    referenceSegmentTime_s = max(referenceSegmentTime_s, eps);
    f(powerIndex(4)) = travelSavingsRate_deg_s * segmentCount / ...
        (3 * referenceSegmentTime_s ^ 2);
end
[x, ~, exitFlag, output] = fastcone.solve( ...
    f, cones, A, b, Aeq, beq, lb, ub, options);
if exitFlag <= 0 || isempty(x) || any(~isfinite(x))
    controlPoint_deg = zeros(0, degree + 1, 2);
    segmentTime_s = NaN;
    return;
end
segmentTime_s = max(x(powerIndex(4)), 0) ^ (1 / 3);
controlPoint_deg = permute(reshape( ...
    x(1:controlCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 4: Local Functions

function soc = createTimePowerCones(variableCount, powerIndex)
% Create p0*p2>=p1^2 and p1*p3>=p2^2 as standard cones.
emptyCone = secondordercone(zeros(2, variableCount), zeros(2, 1), ...
    zeros(variableCount, 1), 0);
soc = repmat(emptyCone, 2, 1);
for coneIndex = 1:2
    coneA = zeros(2, variableCount);
    coneA(1, powerIndex(coneIndex + 1)) = 2;
    coneA(2, powerIndex(coneIndex)) = 1;
    coneA(2, powerIndex(coneIndex + 2)) = -1;
    coneD = zeros(variableCount, 1);
    coneD(powerIndex([coneIndex coneIndex + 2])) = 1;
    soc(coneIndex) = secondordercone( ...
        coneA, zeros(2, 1), coneD, 0);
end
end

function soc = createTravelBoundCones( ...
        variableCount, travelBoundIndex, segmentCount, degree)
% Bound each Bezier control edge with a convex travel surrogate.
if isempty(travelBoundIndex)
    soc = repmat(secondordercone( ...
        zeros(2, variableCount), zeros(2, 1), ...
        zeros(variableCount, 1), 0), 0, 1);
    return;
end
soc = repmat(secondordercone( ...
    zeros(2, variableCount), zeros(2, 1), ...
    zeros(variableCount, 1), 0), numel(travelBoundIndex), 1);
boundIndex = 0;
for segmentIndex = 1:segmentCount
    for controlIndex = 0:degree - 1
        boundIndex = boundIndex + 1;
        coneA = zeros(2, variableCount);
        for axisIndex = 1:2
            firstIndex = controlIndexOf( ...
                segmentIndex, controlIndex, axisIndex, degree);
            secondIndex = controlIndexOf( ...
                segmentIndex, controlIndex + 1, axisIndex, degree);
            coneA(axisIndex, [firstIndex secondIndex]) = [-1 1];
        end
        coneC = zeros(variableCount, 1);
        coneC(travelBoundIndex(boundIndex)) = 1;
        soc(boundIndex) = secondordercone( ...
            coneA, zeros(2, 1), coneC, 0);
    end
end
end

function [rows, offset_deg] = fixedPlaneRows( ...
        plane, degree, variableCount, segmentIndex)
% Expand a fixed line times decision-valued trajectory controls.
[alpha, beta] = productWeights(degree);
rows = spalloc(degree + 2, variableCount, 4 * (degree + 2));
for productIndex = 1:degree + 2
    if alpha(productIndex) > 0
        indices = controlIndexOf( ...
            segmentIndex, productIndex - 1, 1:2, degree);
        rows(productIndex, indices) = ...
            alpha(productIndex) * plane.Normal(1, :); %#ok<SPRIX>
    end
    if beta(productIndex) > 0
        indices = controlIndexOf( ...
            segmentIndex, productIndex - 2, 1:2, degree);
        currentValues = full(rows(productIndex, indices));
        rows(productIndex, indices) = currentValues + ...
            beta(productIndex) * plane.Normal(2, :); %#ok<SPRIX>
    end
end
offset_deg = alpha * plane.Offset_deg(1) + ...
    beta * plane.Offset_deg(2);
end

function [alpha, beta] = productWeights(degree)
% Return exact degree-N by degree-one Bernstein product weights.
beta = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;
end

function index = controlIndexOf( ...
        segmentIndex, controlIndex, axisIndex, degree)
% Map trajectory controls into the conic decision vector.
index = ((segmentIndex - 1) * (degree + 1) + controlIndex) * 2 + ...
    axisIndex;
end
