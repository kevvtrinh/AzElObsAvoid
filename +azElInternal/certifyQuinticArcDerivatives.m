function certificate = certifyQuinticArcDerivatives( ...
        controlPoints_deg, parameterInterval)
%% Section 0: Header & Readme
% SYNTAX
%   certificate = azElInternal.certifyQuinticArcDerivatives( ...
%       controlPoints_deg, parameterInterval)
%**************************************************************************
% PURPOSE
%   - Certify continuous per-axis bounds on the first three arc-length
%     derivatives of a regular planar quintic Bezier curve.
%   - Use exact Bernstein product identities and convex-hull bounds between
%     samples, with a sound analytic fallback for a regular curve.
%**************************************************************************
% INPUTS
%   - controlPoints_deg (6-by-2 finite numeric matrix)
%       Quintic Bezier controls ordered by increasing curve parameter.
%   - parameterInterval (2-element finite numeric vector)
%       Strictly increasing closed interval [u0 u1] within [0, 1].
%**************************************************************************
% OUTPUTS
%   - certificate (scalar struct)
%       .TangentByAxis
%       .SecondDerivativeByAxis_deg_inv
%       .ThirdDerivativeByAxis_deg_inv2
%       .ParameterSpeedLowerBound_deg
%       .SubdivisionCount
%       .FallbackCount
%       .Method
%       Bounds apply continuously throughout parameterInterval. A
%       degenerate or otherwise uncertifiable input throws an error.
%**************************************************************************
% UNITS
%   - Position and parameter-derivative controls use degrees. Arc-length
%     derivative bounds use 1, deg^-1, and deg^-2, respectively.
%**************************************************************************

%% Section 1: Validate Inputs & Build Analytic Reference Bounds
validateattributes(controlPoints_deg, {'numeric'}, ...
    {'real', 'finite', 'size', [6 2]}, mfilename, ...
    'controlPoints_deg');
validateattributes(parameterInterval, {'numeric'}, ...
    {'real', 'finite', 'vector', 'numel', 2}, mfilename, ...
    'parameterInterval');
parameterInterval = double(parameterInterval(:).');
if parameterInterval(1) < 0 || parameterInterval(2) > 1 || ...
        parameterInterval(2) <= parameterInterval(1)
    error("certifyQuinticArcDerivatives:InvalidParameterInterval", ...
        "parameterInterval must satisfy 0 <= u0 < u1 <= 1.");
end
controlPoints_deg = double(controlPoints_deg);
firstControl_deg = 5 * diff(controlPoints_deg, 1, 1);
secondControl_deg = 20 * diff(controlPoints_deg, 2, 1);
thirdControl_deg = 60 * diff(controlPoints_deg, 3, 1);
[analyticTangentBound, analyticSecondBound_deg_inv, ...
    analyticThirdBound_deg_inv2, analyticSpeedLowerBound_deg] = ...
    wholeQuinticAnalyticBounds( ...
        controlPoints_deg, firstControl_deg, secondControl_deg, ...
        thirdControl_deg);

%% Section 2: Restrict The Polynomial & Certify Every Leaf
restrictedFirstControl_deg = restrictBernsteinControl( ...
    firstControl_deg, parameterInterval(1), parameterInterval(2));
restrictedSecondControl_deg = restrictBernsteinControl( ...
    secondControl_deg, parameterInterval(1), parameterInterval(2));
restrictedThirdControl_deg = restrictBernsteinControl( ...
    thirdControl_deg, parameterInterval(1), parameterInterval(2));
parameterSpan = diff(parameterInterval);
baseSubdivisionCount = max(1, ceil(8 * parameterSpan));
parameterEdges = linspace(0, 1, baseSubdivisionCount + 1);
tangentBound = zeros(1, 2);
secondBound_deg_inv = zeros(1, 2);
thirdBound_deg_inv2 = zeros(1, 2);
speedLowerBound_deg = Inf;
subdivisionCount = 0;
fallbackCount = 0;
for subdivisionIndex = 1:baseSubdivisionCount
    intervalStart = parameterEdges(subdivisionIndex);
    intervalEnd = parameterEdges(subdivisionIndex + 1);
    leafFirstControl_deg = restrictBernsteinControl( ...
        restrictedFirstControl_deg, intervalStart, intervalEnd);
    leafSecondControl_deg = restrictBernsteinControl( ...
        restrictedSecondControl_deg, intervalStart, intervalEnd);
    leafThirdControl_deg = restrictBernsteinControl( ...
        restrictedThirdControl_deg, intervalStart, intervalEnd);
    [leafTangentBound, leafSecondBound_deg_inv, ...
        leafThirdBound_deg_inv2, leafSpeedLowerBound_deg, ...
        leafCount, leafFallbackCount] = certifyBernsteinLeaf( ...
            leafFirstControl_deg, leafSecondControl_deg, ...
            leafThirdControl_deg, analyticTangentBound, ...
            analyticSecondBound_deg_inv, ...
            analyticThirdBound_deg_inv2, ...
            analyticSpeedLowerBound_deg, 0);
    tangentBound = max(tangentBound, leafTangentBound);
    secondBound_deg_inv = max( ...
        secondBound_deg_inv, leafSecondBound_deg_inv);
    thirdBound_deg_inv2 = max( ...
        thirdBound_deg_inv2, leafThirdBound_deg_inv2);
    speedLowerBound_deg = min( ...
        speedLowerBound_deg, leafSpeedLowerBound_deg);
    subdivisionCount = subdivisionCount + leafCount;
    fallbackCount = fallbackCount + leafFallbackCount;
end

%% Section 3: Publish Certificate Diagnostics
method = "continuousBernsteinRationalEnvelope";
if fallbackCount > 0
    method = "continuousBernsteinWithAnalyticFallback";
end
certificate = struct( ...
    "TangentByAxis", tangentBound, ...
    "SecondDerivativeByAxis_deg_inv", secondBound_deg_inv, ...
    "ThirdDerivativeByAxis_deg_inv2", thirdBound_deg_inv2, ...
    "ParameterSpeedLowerBound_deg", speedLowerBound_deg, ...
    "SubdivisionCount", subdivisionCount, ...
    "FallbackCount", fallbackCount, ...
    "Method", method);
end

function [tangentBound, secondBound_deg_inv, ...
        thirdBound_deg_inv2, speedLowerBound_deg] = ...
        wholeQuinticAnalyticBounds(controlPoints_deg, firstControl_deg, ...
        secondControl_deg, thirdControl_deg)
%% Section 0: Header & Readme
% SYNTAX
%   [tangentBound, secondBound_deg_inv, ...
%       thirdBound_deg_inv2, speedLowerBound_deg] = ...
%       wholeQuinticAnalyticBounds(controlPoints_deg, firstControl_deg, ...
%       secondControl_deg, thirdControl_deg)
%**************************************************************************
% PURPOSE
%   - Establish simple whole-curve bounds used both to tighten Bernstein
%     leaves and as their denominator-proof fallback.
%**************************************************************************
% INPUTS
%   - controlPoints_deg (6-by-2 numeric matrix)
%       Quintic Bezier position controls.
%   - firstControl_deg, secondControl_deg, thirdControl_deg
%       Bernstein controls of the first three parameter derivatives.
%**************************************************************************
% OUTPUTS
%   - tangentBound, secondBound_deg_inv, thirdBound_deg_inv2
%       Sound whole-curve absolute arc-length derivative bounds.
%   - speedLowerBound_deg (positive scalar)
%       Sound lower bound on norm(dp/du).
%**************************************************************************
% UNITS
%   - Parameter derivatives use degrees; outputs use 1, deg^-1, deg^-2.
%**************************************************************************
geometryScale_deg = max(1, max(abs(controlPoints_deg(:))));
structureTolerance_deg = 512 * eps(geometryScale_deg);
incomingHalfTrim_deg = norm( ...
    controlPoints_deg(2, :) - controlPoints_deg(1, :));
outgoingHalfTrim_deg = norm( ...
    controlPoints_deg(6, :) - controlPoints_deg(5, :));
hasSymmetricConstruction = incomingHalfTrim_deg > ...
    structureTolerance_deg && outgoingHalfTrim_deg > ...
    structureTolerance_deg && ...
    norm(controlPoints_deg(3, :) - controlPoints_deg(4, :)) <= ...
    structureTolerance_deg && ...
    norm(2 * controlPoints_deg(2, :) - controlPoints_deg(1, :) - ...
    controlPoints_deg(3, :)) <= structureTolerance_deg && ...
    norm(2 * controlPoints_deg(5, :) - controlPoints_deg(4, :) - ...
    controlPoints_deg(6, :)) <= structureTolerance_deg && ...
    abs(incomingHalfTrim_deg - outgoingHalfTrim_deg) <= ...
    structureTolerance_deg;
if hasSymmetricConstruction
    incomingDirection = (controlPoints_deg(2, :) - ...
        controlPoints_deg(1, :)) / incomingHalfTrim_deg;
    outgoingDirection = (controlPoints_deg(6, :) - ...
        controlPoints_deg(5, :)) / outgoingHalfTrim_deg;
    halfAngleCosine = sqrt(max(0, ...
        0.5 * (1 + dot(incomingDirection, outgoingDirection))));
    trimDistance_deg = 2 * incomingHalfTrim_deg;
    speedLowerBound_deg = (25 / 16) * ...
        trimDistance_deg * halfAngleCosine;
else
    chord_deg = controlPoints_deg(end, :) - controlPoints_deg(1, :);
    chordLength_deg = norm(chord_deg);
    if chordLength_deg <= structureTolerance_deg
        error("certifyQuinticArcDerivatives:UncertifiedRegularity", ...
            "The quintic chord is too short to certify parameter speed.");
    end
    chordDirection = chord_deg / chordLength_deg;
    speedLowerBound_deg = min(firstControl_deg * chordDirection.');
end
floatingGuard_deg = 4096 * eps * max( ...
    max(vecnorm(firstControl_deg, 2, 2)), realmin);
speedLowerBound_deg = speedLowerBound_deg - floatingGuard_deg;
if ~isfinite(speedLowerBound_deg) || speedLowerBound_deg <= 0
    error("certifyQuinticArcDerivatives:UncertifiedRegularity", ...
        "Could not prove a positive parameter-speed lower bound.");
end
secondNormBound_deg = max(vecnorm(secondControl_deg, 2, 2));
thirdNormBound_deg = max(vecnorm(thirdControl_deg, 2, 2));
tangentBound = [1 1];
secondBound_deg_inv = repmat(secondNormBound_deg / ...
    speedLowerBound_deg^2, 1, 2);
thirdNormBound_deg_inv2 = 2 * thirdNormBound_deg / ...
    speedLowerBound_deg^3 + 8 * secondNormBound_deg^2 / ...
    speedLowerBound_deg^4;
thirdBound_deg_inv2 = repmat(thirdNormBound_deg_inv2, 1, 2);
end

function [tangentBound, secondBound_deg_inv, thirdBound_deg_inv2, ...
        speedLowerBound_deg, leafCount, fallbackCount] = ...
        certifyBernsteinLeaf(firstControl_deg, secondControl_deg, ...
        thirdControl_deg, analyticTangentBound, ...
        analyticSecondBound_deg_inv, analyticThirdBound_deg_inv2, ...
        analyticSpeedLowerBound_deg, recursionDepth)
%% Section 0: Header & Readme
% SYNTAX
%   [tangentBound, secondBound_deg_inv, thirdBound_deg_inv2, ...
%       speedLowerBound_deg, leafCount, fallbackCount] = ...
%       certifyBernsteinLeaf(firstControl_deg, secondControl_deg, ...
%       thirdControl_deg, analyticTangentBound, ...
%       analyticSecondBound_deg_inv, analyticThirdBound_deg_inv2, ...
%       analyticSpeedLowerBound_deg, recursionDepth)
%**************************************************************************
% PURPOSE
%   - Certify a parameter leaf with Bernstein rational bounds, subdividing
%     until squared speed has a positive convex-hull lower bound.
%**************************************************************************
% INPUTS
%   - firstControl_deg, secondControl_deg, thirdControl_deg
%       Bernstein controls of parameter derivatives on one interval.
%   - analyticTangentBound, analyticSecondBound_deg_inv,
%       analyticThirdBound_deg_inv2, analyticSpeedLowerBound_deg
%       Sound whole-curve fallback bounds.
%   - recursionDepth (nonnegative integer scalar)
%       Current subdivision depth.
%**************************************************************************
% OUTPUTS
%   - tangentBound, secondBound_deg_inv, thirdBound_deg_inv2
%       Certified absolute per-axis arc-length derivative bounds.
%   - speedLowerBound_deg (positive scalar)
%       Certified lower bound on parameter speed for this subtree.
%   - leafCount, fallbackCount (nonnegative integer scalars)
%       Certificate complexity and analytic fallback diagnostics.
%**************************************************************************
% UNITS
%   - Parameter derivatives use degrees; bounds use 1, deg^-1, deg^-2.
%**************************************************************************
firstX_deg = firstControl_deg(:, 1);
firstY_deg = firstControl_deg(:, 2);
secondX_deg = secondControl_deg(:, 1);
secondY_deg = secondControl_deg(:, 2);
thirdX_deg = thirdControl_deg(:, 1);
thirdY_deg = thirdControl_deg(:, 2);
firstSquaredXControl_deg2 = bernsteinProductControl( ...
    firstX_deg, firstX_deg);
firstSquaredYControl_deg2 = bernsteinProductControl( ...
    firstY_deg, firstY_deg);
speedSquaredControl_deg2 = firstSquaredXControl_deg2 + ...
    firstSquaredYControl_deg2;
speedSquaredScale_deg2 = max(abs(firstSquaredXControl_deg2)) + ...
    max(abs(firstSquaredYControl_deg2));
speedSquaredGuard_deg2 = floatingPointGuard(speedSquaredScale_deg2);
speedSquaredLowerBound_deg2 = min(speedSquaredControl_deg2) - ...
    speedSquaredGuard_deg2;
if speedSquaredLowerBound_deg2 <= 0
    maximumRecursionDepth = 14;
    if recursionDepth < maximumRecursionDepth
        [firstLeft_deg, firstRight_deg] = splitBernsteinControl( ...
            firstControl_deg, 0.5);
        [secondLeft_deg, secondRight_deg] = splitBernsteinControl( ...
            secondControl_deg, 0.5);
        [thirdLeft_deg, thirdRight_deg] = splitBernsteinControl( ...
            thirdControl_deg, 0.5);
        [leftTangent, leftSecond, leftThird, leftSpeedLower_deg, ...
            leftCount, leftFallbackCount] = certifyBernsteinLeaf( ...
                firstLeft_deg, secondLeft_deg, thirdLeft_deg, ...
                analyticTangentBound, analyticSecondBound_deg_inv, ...
                analyticThirdBound_deg_inv2, ...
                analyticSpeedLowerBound_deg, recursionDepth + 1);
        [rightTangent, rightSecond, rightThird, rightSpeedLower_deg, ...
            rightCount, rightFallbackCount] = certifyBernsteinLeaf( ...
                firstRight_deg, secondRight_deg, thirdRight_deg, ...
                analyticTangentBound, analyticSecondBound_deg_inv, ...
                analyticThirdBound_deg_inv2, ...
                analyticSpeedLowerBound_deg, recursionDepth + 1);
        tangentBound = max(leftTangent, rightTangent);
        secondBound_deg_inv = max(leftSecond, rightSecond);
        thirdBound_deg_inv2 = max(leftThird, rightThird);
        speedLowerBound_deg = min( ...
            leftSpeedLower_deg, rightSpeedLower_deg);
        leafCount = leftCount + rightCount;
        fallbackCount = leftFallbackCount + rightFallbackCount;
        return;
    end
    tangentBound = analyticTangentBound;
    secondBound_deg_inv = analyticSecondBound_deg_inv;
    thirdBound_deg_inv2 = analyticThirdBound_deg_inv2;
    speedLowerBound_deg = analyticSpeedLowerBound_deg;
    leafCount = 1;
    fallbackCount = 1;
    return;
end

firstSecondXControl_deg2 = bernsteinProductControl( ...
    firstX_deg, secondX_deg);
firstSecondYControl_deg2 = bernsteinProductControl( ...
    firstY_deg, secondY_deg);
firstSecondDotControl_deg2 = firstSecondXControl_deg2 + ...
    firstSecondYControl_deg2;
secondSquaredXControl_deg2 = bernsteinProductControl( ...
    secondX_deg, secondX_deg);
secondSquaredYControl_deg2 = bernsteinProductControl( ...
    secondY_deg, secondY_deg);
firstThirdXControl_deg2 = bernsteinProductControl( ...
    firstX_deg, thirdX_deg);
firstThirdYControl_deg2 = bernsteinProductControl( ...
    firstY_deg, thirdY_deg);
secondNormAndFirstThirdControl_deg2 = ...
    secondSquaredXControl_deg2 + secondSquaredYControl_deg2 + ...
    firstThirdXControl_deg2 + firstThirdYControl_deg2;
speedFourthControl_deg4 = bernsteinProductControl( ...
    speedSquaredControl_deg2, speedSquaredControl_deg2);
firstSecondDotSquaredControl_deg4 = bernsteinProductControl( ...
    firstSecondDotControl_deg2, firstSecondDotControl_deg2);

tangentBound = zeros(1, 2);
secondBound_deg_inv = zeros(1, 2);
thirdBound_deg_inv2 = zeros(1, 2);
for axisIndex = 1:2
    firstAxis_deg = firstControl_deg(:, axisIndex);
    secondAxis_deg = secondControl_deg(:, axisIndex);
    thirdAxis_deg = thirdControl_deg(:, axisIndex);
    secondFirstTerm_deg3 = bernsteinProductControl( ...
        secondAxis_deg, speedSquaredControl_deg2);
    secondSecondTerm_deg3 = bernsteinProductControl( ...
        firstAxis_deg, firstSecondDotControl_deg2);
    secondNumeratorControl_deg3 = secondFirstTerm_deg3 - ...
        secondSecondTerm_deg3;
    secondNumeratorScale_deg3 = max(abs(secondFirstTerm_deg3)) + ...
        max(abs(secondSecondTerm_deg3));

    thirdFirstTerm_deg5 = bernsteinProductControl( ...
        thirdAxis_deg, speedFourthControl_deg4);
    thirdSecondTerm_deg5 = 3 * bernsteinProductControl( ...
        bernsteinProductControl( ...
        secondAxis_deg, firstSecondDotControl_deg2), ...
        speedSquaredControl_deg2);
    thirdThirdTerm_deg5 = bernsteinProductControl( ...
        bernsteinProductControl(firstAxis_deg, ...
        secondNormAndFirstThirdControl_deg2), ...
        speedSquaredControl_deg2);
    thirdFourthTerm_deg5 = 4 * bernsteinProductControl( ...
        firstAxis_deg, firstSecondDotSquaredControl_deg4);
    thirdNumeratorControl_deg5 = thirdFirstTerm_deg5 - ...
        thirdSecondTerm_deg5 - thirdThirdTerm_deg5 + ...
        thirdFourthTerm_deg5;
    thirdNumeratorScale_deg5 = max(abs(thirdFirstTerm_deg5)) + ...
        max(abs(thirdSecondTerm_deg5)) + ...
        max(abs(thirdThirdTerm_deg5)) + ...
        max(abs(thirdFourthTerm_deg5));

    firstAxisUpper_deg = max(abs(firstAxis_deg)) + ...
        floatingPointGuard(max(abs(firstAxis_deg)));
    secondNumeratorUpper_deg3 = max(abs( ...
        secondNumeratorControl_deg3)) + ...
        floatingPointGuard(secondNumeratorScale_deg3);
    thirdNumeratorUpper_deg5 = max(abs( ...
        thirdNumeratorControl_deg5)) + ...
        floatingPointGuard(thirdNumeratorScale_deg5);
    tangentBound(axisIndex) = min(analyticTangentBound(axisIndex), ...
        firstAxisUpper_deg / sqrt(speedSquaredLowerBound_deg2));
    secondBound_deg_inv(axisIndex) = min( ...
        analyticSecondBound_deg_inv(axisIndex), ...
        secondNumeratorUpper_deg3 / speedSquaredLowerBound_deg2^2);
    thirdBound_deg_inv2(axisIndex) = min( ...
        analyticThirdBound_deg_inv2(axisIndex), ...
        thirdNumeratorUpper_deg5 / speedSquaredLowerBound_deg2^3.5);
end
speedLowerBound_deg = max(analyticSpeedLowerBound_deg, ...
    sqrt(speedSquaredLowerBound_deg2));
leafCount = 1;
fallbackCount = 0;
end

function guardedValue = floatingPointGuard(scale)
%% Section 0: Header & Readme
% SYNTAX
%   guardedValue = floatingPointGuard(scale)
%**************************************************************************
% PURPOSE
%   - Add an outward error allowance to small fixed-degree polynomial
%     products and sums performed in binary floating point.
%**************************************************************************
% INPUTS
%   - scale (nonnegative finite scalar)
%       Sum-of-term magnitude scale in the quantity's native units.
%**************************************************************************
% OUTPUTS
%   - guardedValue (nonnegative scalar)
%       Conservative roundoff allowance in the same units as scale.
%**************************************************************************
% UNITS
%   - Input and output have identical caller-defined units.
%**************************************************************************
guardedValue = 8192 * eps * max(double(scale), realmin);
end

function restrictedControl = restrictBernsteinControl( ...
        control, intervalStart, intervalEnd)
%% Section 0: Header & Readme
% SYNTAX
%   restrictedControl = restrictBernsteinControl( ...
%       control, intervalStart, intervalEnd)
%**************************************************************************
% PURPOSE
%   - Reparameterize Bernstein controls to a closed subinterval using
%     de Casteljau subdivision without changing polynomial values.
%**************************************************************************
% INPUTS
%   - control (N-by-D finite numeric matrix)
%       Bernstein controls for a degree N-1 polynomial.
%   - intervalStart, intervalEnd (ordered scalars in [0,1])
%       Requested parameter interval.
%**************************************************************************
% OUTPUTS
%   - restrictedControl (N-by-D numeric matrix)
%       Controls of the same polynomial on a local [0,1] parameter.
%**************************************************************************
% UNITS
%   - Parameters are dimensionless; controls retain their input units.
%**************************************************************************
if intervalStart < 0 || intervalEnd > 1 || ...
        intervalEnd <= intervalStart
    error("certifyQuinticArcDerivatives:InvalidSubinterval", ...
        "A Bernstein interval must satisfy 0 <= start < end <= 1.");
end
if intervalEnd < 1
    [lowerControl, ~] = splitBernsteinControl(control, intervalEnd);
else
    lowerControl = control;
end
if intervalStart > 0
    localStart = intervalStart / intervalEnd;
    [~, restrictedControl] = splitBernsteinControl( ...
        lowerControl, localStart);
else
    restrictedControl = lowerControl;
end
end

function [leftControl, rightControl] = splitBernsteinControl( ...
        control, splitParameter)
%% Section 0: Header & Readme
% SYNTAX
%   [leftControl, rightControl] = splitBernsteinControl( ...
%       control, splitParameter)
%**************************************************************************
% PURPOSE
%   - Split scalar or vector Bernstein controls by de Casteljau evaluation.
%**************************************************************************
% INPUTS
%   - control (N-by-D finite numeric matrix)
%       Degree N-1 Bernstein controls.
%   - splitParameter (scalar in [0,1])
%       Parameter at which to split the polynomial.
%**************************************************************************
% OUTPUTS
%   - leftControl, rightControl (N-by-D numeric matrices)
%       Exact Bernstein controls for the two subintervals.
%**************************************************************************
% UNITS
%   - splitParameter is dimensionless; controls retain their input units.
%**************************************************************************
degree = size(control, 1) - 1;
leftControl = zeros(size(control));
rightControl = zeros(size(control));
levelControl = double(control);
leftControl(1, :) = levelControl(1, :);
rightControl(end, :) = levelControl(end, :);
for levelIndex = 1:degree
    levelControl = (1 - splitParameter) * ...
        levelControl(1:end - 1, :) + splitParameter * ...
        levelControl(2:end, :);
    leftControl(levelIndex + 1, :) = levelControl(1, :);
    rightControl(end - levelIndex, :) = levelControl(end, :);
end
end

function productControl = bernsteinProductControl( ...
        firstControl, secondControl)
%% Section 0: Header & Readme
% SYNTAX
%   productControl = bernsteinProductControl( ...
%       firstControl, secondControl)
%**************************************************************************
% PURPOSE
%   - Form exact Bernstein controls for the product of two scalar
%     Bernstein polynomials.
%**************************************************************************
% INPUTS
%   - firstControl, secondControl (finite numeric columns)
%       Bernstein controls of arbitrary nonnegative degrees.
%**************************************************************************
% OUTPUTS
%   - productControl (numeric column)
%       Degree-sum Bernstein controls of the polynomial product.
%**************************************************************************
% UNITS
%   - Output units are the product of the two input units.
%**************************************************************************
firstControl = double(firstControl(:));
secondControl = double(secondControl(:));
firstDegree = numel(firstControl) - 1;
secondDegree = numel(secondControl) - 1;
productDegree = firstDegree + secondDegree;
firstBinomial = bernsteinBinomialCoefficients(firstDegree);
secondBinomial = bernsteinBinomialCoefficients(secondDegree);
productBinomial = bernsteinBinomialCoefficients(productDegree);
productControl = zeros(productDegree + 1, 1);
for productIndex = 0:productDegree
    firstIndexMinimum = max(0, productIndex - secondDegree);
    firstIndexMaximum = min(firstDegree, productIndex);
    for firstIndex = firstIndexMinimum:firstIndexMaximum
        secondIndex = productIndex - firstIndex;
        productWeight = firstBinomial(firstIndex + 1) * ...
            secondBinomial(secondIndex + 1) / ...
            productBinomial(productIndex + 1);
        productControl(productIndex + 1) = ...
            productControl(productIndex + 1) + productWeight * ...
            firstControl(firstIndex + 1) * ...
            secondControl(secondIndex + 1);
    end
end
end

function coefficients = bernsteinBinomialCoefficients(degree)
%% Section 0: Header & Readme
% SYNTAX
%   coefficients = bernsteinBinomialCoefficients(degree)
%**************************************************************************
% PURPOSE
%   - Return the binomial row needed by Bernstein product conversion.
%**************************************************************************
% INPUTS
%   - degree (nonnegative integer scalar)
%       Polynomial degree.
%**************************************************************************
% OUTPUTS
%   - coefficients (degree+1 numeric column)
%       Binomial coefficients from index zero through degree.
%**************************************************************************
% UNITS
%   - All values are dimensionless.
%**************************************************************************
coefficients = ones(degree + 1, 1);
for coefficientIndex = 1:degree
    coefficients(coefficientIndex + 1) = ...
        coefficients(coefficientIndex) * ...
        (degree - coefficientIndex + 1) / coefficientIndex;
end
end
