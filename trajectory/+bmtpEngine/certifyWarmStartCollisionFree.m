function certificate = certifyWarmStartCollisionFree( ...
        warmStart, regions_deg, collisionClearanceTolerance_deg)
%% Section 0: Header & Readme
% SYNTAX
%   certificate = bmtpEngine.certifyWarmStartCollisionFree( ...
%       warmStart, regions_deg, collisionClearanceTolerance_deg)
%**************************************************************************
% PURPOSE
%   - Certify every Bernstein warm-start span against every supplied convex
%     exclusion region before the warm start reaches BMTP.
%   - Use a constant separator when available, then the same degree-one
%     Bernstein plane form and direct inequalities used by BMTP.
%**************************************************************************
% INPUTS
%   - warmStart (scalar struct)
%       ControlPoint_deg is S-by-(D+1)-by-2 and SegmentTime_s is a positive
%       scalar. Geometry is interpreted as Bernstein control points.
%   - regions_deg (cell vector of numeric arrays)
%       Each cell is one finite convex N-by-2 protected exclusion polygon.
%   - collisionClearanceTolerance_deg (nonnegative numeric scalar)
%       Required certified distance after the shared roundoff reserve.
%**************************************************************************
% OUTPUTS
%   - certificate (scalar struct)
%       Stable all-pair result, verified planes, failure reasons, counts, and
%       minimum certified clearance. Failure is a normal Passed=false result.
%**************************************************************************
% UNITS
%   - Control points, region vertices, clearance, and reserves use degrees.
%     SegmentTime_s uses seconds but does not alter geometric separation.
%**************************************************************************

%% Section 1: Validate And Normalize Geometry

[controlPoint_deg, regions_deg, collisionClearanceTolerance_deg] = ...
    validateRequest( ...
    warmStart, regions_deg, collisionClearanceTolerance_deg);
segmentCount = size(controlPoint_deg, 1);
regionCount = numel(regions_deg);
[~, geometryTolerance_deg, roundoffReserve_deg] = ...
    bmtpEngine.createCoordinateTolerances(controlPoint_deg, regions_deg);
normalNormLimit = 1 + 2 ^ 20 * eps;
obstacleTarget_deg = normalNormLimit * ...
    collisionClearanceTolerance_deg + roundoffReserve_deg;
planeOptions = optimoptions("coneprog", "Display", "none", ...
    "ConstraintTolerance", 1e-11, "OptimalityTolerance", 1e-11, ...
    "MaxIterations", 400);

emptyPlane = createEmptyPlane();
planes = repmat(emptyPlane, segmentCount, regionCount);
pairPassed = false(segmentCount, regionCount);
pairReason = strings(segmentCount, regionCount);
minimumCertifiedClearance_deg = Inf;
[constantPairCount, conicPairCount] = deal(0);

%% Section 2: Certify Every Span And Region Pair

for segmentIndex = 1:segmentCount
    spanControl_deg = squeeze(controlPoint_deg(segmentIndex, :, :));
    for regionIndex = 1:regionCount
        [plane, reason, usedConicSolve] = certifyPair( ...
            spanControl_deg, regions_deg{regionIndex}, ...
            collisionClearanceTolerance_deg, roundoffReserve_deg, ...
            geometryTolerance_deg, obstacleTarget_deg, planeOptions);
        planes(segmentIndex, regionIndex) = plane;
        pairPassed(segmentIndex, regionIndex) = plane.Verified;
        pairReason(segmentIndex, regionIndex) = reason;
        conicPairCount = conicPairCount + double(usedConicSolve);
        constantPairCount = constantPairCount + double(~usedConicSolve);
        minimumCertifiedClearance_deg = min( ...
            minimumCertifiedClearance_deg, ...
            plane.CertifiedClearance_deg);
    end
end

%% Section 3: Assemble Stable Certificate Evidence

allPairCount = segmentCount * regionCount;
certifiedPairCount = nnz(pairPassed);
passed = certifiedPairCount == allPairCount;
if passed
    terminationReason = "certified";
    message = "Every Bernstein span is certified against every region.";
else
    terminationReason = "uncertifiedWarmStartCollision";
    message = "At least one Bernstein span lacks required separation.";
end
certificate = struct( ...
    "Kind", "degreeOneBernsteinPlane", ...
    "Passed", passed, ...
    "Message", message, ...
    "TerminationReason", terminationReason, ...
    "SegmentCount", segmentCount, ...
    "RegionCount", regionCount, ...
    "AllPairCount", allPairCount, ...
    "CertifiedPairCount", certifiedPairCount, ...
    "FailedPairCount", allPairCount - certifiedPairCount, ...
    "ConstantPairCount", constantPairCount, ...
    "ConicPairCount", conicPairCount, ...
    "CollisionClearanceTolerance_deg", ...
    collisionClearanceTolerance_deg, ...
    "RoundoffReserve_deg", roundoffReserve_deg, ...
    "MinimumCertifiedClearance_deg", minimumCertifiedClearance_deg, ...
    "PairPassed", pairPassed, ...
    "PairReason", pairReason, ...
    "Planes", planes);
end

%% Section 4: Local Functions

function [controlPoint_deg, regions_deg, clearance_deg] = ...
        validateRequest(warmStart, regions_deg, clearance_deg)
% Reject malformed warm starts and regions before geometric certification.
hasWarmFields = isstruct(warmStart) && isscalar(warmStart) && ...
    all(isfield(warmStart, {'ControlPoint_deg', 'SegmentTime_s'}));
if ~hasWarmFields
    error("bmtpEngine:InvalidWarmStartCertificateInput", ...
        "warmStart must contain ControlPoint_deg and SegmentTime_s.");
end
controlPoint_deg = warmStart.ControlPoint_deg;
controlIsValid = isnumeric(controlPoint_deg) && isreal(controlPoint_deg) && ...
    ndims(controlPoint_deg) == 3 && size(controlPoint_deg, 1) >= 1 && ...
    size(controlPoint_deg, 2) >= 2 && size(controlPoint_deg, 3) == 2 && ...
    all(isfinite(controlPoint_deg), "all");
timeIsValid = isnumeric(warmStart.SegmentTime_s) && ...
    isreal(warmStart.SegmentTime_s) && isscalar(warmStart.SegmentTime_s) && ...
    isfinite(warmStart.SegmentTime_s) && warmStart.SegmentTime_s > 0;
if ~controlIsValid || ~timeIsValid
    error("bmtpEngine:InvalidWarmStartCertificateInput", ...
        "Warm controls must be finite S-by-C-by-2 and time must be positive.");
end
if ~iscell(regions_deg) || ~isvector(regions_deg)
    error("bmtpEngine:InvalidWarmStartCertificateRegions", ...
        "regions_deg must be a cell vector of finite N-by-2 polygons.");
end
regions_deg = regions_deg(:);
for regionIndex = 1:numel(regions_deg)
    vertices_deg = regions_deg{regionIndex};
    regionIsValid = isnumeric(vertices_deg) && isreal(vertices_deg) && ...
        ismatrix(vertices_deg) && size(vertices_deg, 1) >= 3 && ...
        size(vertices_deg, 2) == 2 && all(isfinite(vertices_deg), "all");
    if ~regionIsValid
        error("bmtpEngine:InvalidWarmStartCertificateRegions", ...
            "Every region must be a finite numeric N-by-2 polygon.");
    end
    regions_deg{regionIndex} = double(vertices_deg);
end
validateattributes(clearance_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'nonnegative'});
controlPoint_deg = double(controlPoint_deg);
clearance_deg = double(clearance_deg);
end

function [plane, reason, usedConicSolve] = certifyPair( ...
        control_deg, vertices_deg, requiredClearance_deg, ...
        reserve_deg, geometryTolerance_deg, obstacleTarget_deg, planeOptions)
% Use a cheap constant separator, then the engine's degree-one conic form.
candidateNormal = createCandidateNormals( ...
    control_deg, vertices_deg, geometryTolerance_deg);
controlProjection_deg = control_deg * candidateNormal.';
regionProjection_deg = vertices_deg * candidateNormal.';
forwardGap_deg = min(regionProjection_deg, [], 1) - ...
    max(controlProjection_deg, [], 1);
reverseGap_deg = min(controlProjection_deg, [], 1) - ...
    max(regionProjection_deg, [], 1);
[~, gapIndex] = max([forwardGap_deg, reverseGap_deg]);
normalCount = size(candidateNormal, 1);
if gapIndex > normalCount
    normal = -candidateNormal(gapIndex - normalCount, :);
else
    normal = candidateNormal(gapIndex, :);
end
normal = normal / norm(normal);
controlSide_deg = control_deg * normal.';
obstacleSide_deg = vertices_deg * normal.';
offset_deg = -0.5 * (min(obstacleSide_deg) + max(controlSide_deg));
controlSide_deg = controlSide_deg + offset_deg;
obstacleSide_deg = obstacleSide_deg + offset_deg;
minimumObstacleSide_deg = min(obstacleSide_deg);
maximumTrajectorySide_deg = max(controlSide_deg);
signedGap_deg = minimumObstacleSide_deg - maximumTrajectorySide_deg;
normalNorm = norm(normal);
certifiedClearance_deg = (signedGap_deg - 2 * reserve_deg) / ...
    max(normalNorm, realmin);
normalNormLimit = 1 + 2 ^ 20 * eps;
verified = minimumObstacleSide_deg >= reserve_deg && ...
    maximumTrajectorySide_deg <= -reserve_deg && ...
    certifiedClearance_deg >= requiredClearance_deg && ...
    normalNorm <= normalNormLimit;
if verified
    reason = "certifiedConstantPlane";
end
plane = struct( ...
    "Active", true, ...
    "Normal", repmat(normal, 2, 1), ...
    "Offset_deg", repmat(offset_deg, 1, 2), ...
    "SignedGap_deg", signedGap_deg, ...
    "CertifiedClearance_deg", certifiedClearance_deg, ...
    "Verified", verified, ...
    "ExitFlag", 1);
usedConicSolve = false;
if verified
    return;
end
[plane, exitFlag] = solveMaximumMarginPlane( ...
    control_deg, vertices_deg, obstacleTarget_deg, reserve_deg, ...
    planeOptions);
usedConicSolve = true;
if plane.Verified
    reason = "certifiedDegreeOnePlane";
elseif exitFlag <= 0
    reason = "noDegreeOneSeparatingPlane";
elseif plane.CertifiedClearance_deg < requiredClearance_deg
    reason = "insufficientClearance";
else
    reason = "degreeOnePlaneVerificationFailed";
end
end

function [plane, exitFlag] = solveMaximumMarginPlane( ...
        controlPoint_deg, vertices_deg, target_deg, reserve_deg, options)
% Solve and directly verify the same degree-one plane used by BMTP.
offsetIndex = 5:6;
marginIndex = 7;
variableCount = 7;
[A, b] = maximumMarginRows(controlPoint_deg, vertices_deg, target_deg);
f = zeros(variableCount, 1);
f(marginIndex) = 1;
emptyCone = secondordercone( ...
    zeros(2, variableCount), zeros(2, 1), ...
    zeros(variableCount, 1), -1);
soc = repmat(emptyCone, 2, 1);
for planeIndex = 0:1
    coneA = zeros(2, variableCount);
    coneA(:, planeIndex * 2 + (1:2)) = eye(2);
    soc(planeIndex + 1) = secondordercone( ...
        coneA, zeros(2, 1), zeros(variableCount, 1), -1);
end
[x, ~, exitFlag] = coneprog( ...
    f, soc, A, b, [], [], [], [], options);
plane = createEmptyPlane();
plane.ExitFlag = exitFlag;
if isempty(x) || any(~isfinite(x))
    return;
end
[plane.Active, plane.Normal, plane.Offset_deg] = ...
    deal(true, reshape(x(1:4), 2, []).', x(offsetIndex).');
plane = verifyDegreeOnePlane( ...
    plane, controlPoint_deg, vertices_deg, reserve_deg, target_deg);
end

function [A, b] = maximumMarginRows( ...
        controlPoint_deg, vertices_deg, target_deg)
% Create linear rows for one degree-one maximum-margin plane solve.
degree = size(controlPoint_deg, 1) - 1;
variableCount = 7;
offsetIndex = 5:6;
marginIndex = 7;
A = zeros(2 * size(vertices_deg, 1) + degree + 2, variableCount);
b = zeros(size(A, 1), 1);
rowIndex = 0;
for planeIndex = 0:1
    targets = rowIndex + (1:size(vertices_deg, 1));
    normalIndex = planeIndex * 2 + (1:2);
    A(targets, normalIndex) = -vertices_deg;
    A(targets, offsetIndex(planeIndex + 1)) = -1;
    b(targets) = -target_deg;
    rowIndex = targets(end);
end
objectiveRows = variablePlaneRows(controlPoint_deg, variableCount);
targets = rowIndex + (1:size(objectiveRows, 1));
A(targets, :) = objectiveRows;
A(targets, marginIndex) = -1;
end

function plane = verifyDegreeOnePlane( ...
        plane, control_deg, vertices_deg, reserve_deg, target_deg)
% Replay obstacle, Bernstein-product, clearance, and norm inequalities.
minimumObstacleSide_deg = min( ...
    vertices_deg * plane.Normal.' + plane.Offset_deg, [], "all");
degree = size(control_deg, 1) - 1;
[alpha, beta] = productWeights(degree);
product_deg = ...
    alpha .* [sum(control_deg .* plane.Normal(1, :), 2); 0] + ...
    beta .* [0; sum(control_deg .* plane.Normal(2, :), 2)] + ...
    alpha * plane.Offset_deg(1) + beta * plane.Offset_deg(2);
[maximumTrajectorySide_deg, maximumNormalNorm] = ...
    deal(max(product_deg), max(vecnorm(plane.Normal, 2, 2)));
[minimumCorrection_deg, maximumCorrection_deg] = deal( ...
    target_deg - minimumObstacleSide_deg, ...
    -reserve_deg - maximumTrajectorySide_deg);
if minimumCorrection_deg <= maximumCorrection_deg
    scale_deg = bmtpEngine.createCoordinateTolerances( ...
        plane.Offset_deg, vertices_deg, control_deg);
    roundoff_deg = 16 * eps(scale_deg);
    [robustMinimum_deg, robustMaximum_deg] = deal( ...
        minimumCorrection_deg + roundoff_deg, ...
        maximumCorrection_deg - roundoff_deg);
    if robustMinimum_deg <= robustMaximum_deg
        correction_deg = min(max(0, robustMinimum_deg), robustMaximum_deg);
    else
        correction_deg = ...
            0.5 * (minimumCorrection_deg + maximumCorrection_deg);
    end
    plane.Offset_deg = plane.Offset_deg + correction_deg;
    minimumObstacleSide_deg = minimumObstacleSide_deg + correction_deg;
    maximumTrajectorySide_deg = maximumTrajectorySide_deg + correction_deg;
end
signedGap_deg = minimumObstacleSide_deg - maximumTrajectorySide_deg;
normalNormLimit = 1 + 2 ^ 20 * eps;
clearanceTarget_deg = ...
    (target_deg - reserve_deg) / normalNormLimit;
certifiedClearance_deg = (signedGap_deg - 2 * reserve_deg) / ...
    max(maximumNormalNorm, realmin);
plane.SignedGap_deg = signedGap_deg;
plane.CertifiedClearance_deg = certifiedClearance_deg;
plane.Verified = minimumObstacleSide_deg >= target_deg && ...
    maximumTrajectorySide_deg <= -reserve_deg && ...
    signedGap_deg >= target_deg + reserve_deg && ...
    certifiedClearance_deg >= clearanceTarget_deg && ...
    maximumNormalNorm <= normalNormLimit;
end

function rows = variablePlaneRows(control_deg, variableCount)
% Expand a decision-valued degree-one plane against fixed controls.
degree = size(control_deg, 1) - 1;
[alpha, beta] = productWeights(degree);
rows = zeros(degree + 2, variableCount);
rows(1:end - 1, 1:2) = alpha(1:end - 1) .* control_deg;
rows(2:end, 3:4) = beta(2:end) .* control_deg;
rows(:, 5:6) = [alpha beta];
end

function [alpha, beta] = productWeights(degree)
% Return exact degree-N by degree-one Bernstein product weights.
beta = (0:degree + 1).' / (degree + 1);
alpha = 1 - beta;
end

function normals = createCandidateNormals( ...
        control_deg, vertices_deg, geometryTolerance_deg)
% Every hull edge is a point pair, so all pair normals include every SAT axis.
pointSets = {control_deg, vertices_deg};
maximumNormalCount = 2;
for setIndex = 1:numel(pointSets)
    pointCount = size(pointSets{setIndex}, 1);
    maximumNormalCount = maximumNormalCount + ...
        pointCount * (pointCount - 1) / 2;
end
normals = zeros(maximumNormalCount, 2);
normals(1:2, :) = eye(2);
normalCount = 2;
for setIndex = 1:numel(pointSets)
    points_deg = pointSets{setIndex};
    for firstIndex = 1:size(points_deg, 1) - 1
        difference_deg = points_deg(firstIndex + 1:end, :) - ...
            points_deg(firstIndex, :);
        differenceLength_deg = vecnorm(difference_deg, 2, 2);
        keepDifference = differenceLength_deg > geometryTolerance_deg;
        difference_deg = difference_deg(keepDifference, :);
        differenceLength_deg = differenceLength_deg(keepDifference);
        targetIndex = normalCount + (1:size(difference_deg, 1));
        normals(targetIndex, :) = ...
            [-difference_deg(:, 2), difference_deg(:, 1)] ./ ...
            differenceLength_deg;
        normalCount = normalCount + size(difference_deg, 1);
    end
end
normals = normals(1:normalCount, :);
end

function plane = createEmptyPlane()
% Define the stable per-pair record before certification.
plane = struct( ...
    "Active", false, ...
    "Normal", zeros(2, 2), ...
    "Offset_deg", zeros(1, 2), ...
    "SignedGap_deg", NaN, ...
    "CertifiedClearance_deg", NaN, ...
    "Verified", false, ...
    "ExitFlag", NaN);
end
