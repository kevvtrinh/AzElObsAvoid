function observation = rpRetimerObservation(problem)
%% Section 0: Header & Readme
% SYNTAX
%   observation = azElInternal.rpRetimerObservation(problem)
%**************************************************************************
% PURPOSE
%   - Convert one unconstrained corner-retiming problem into stable
%     dimensionless features shared by RP training and inference.
%**************************************************************************
% INPUTS
%   - problem (scalar struct)
%       DeflectionAngle_rad, IncomingLength_deg, OutgoingLength_deg,
%       IncomingDirection, OutgoingDirection, TurnRadius_deg,
%       MaxVelocity_deg_s, MaxAcceleration_deg_s2, and MaxJerk_deg_s3.
%**************************************************************************
% OUTPUTS
%   - observation (10-by-1 finite numeric column)
%       Bounded dimensionless policy input. The policy output is only a
%       radius proposal and cannot bypass deterministic certification.
%**************************************************************************
% UNITS
%   - Input field suffixes give radians, degrees, and seconds. Output is
%     dimensionless.
%**************************************************************************

%% Section 1: Validate & Normalize The Corner Problem

requiredFields = ["DeflectionAngle_rad" "IncomingLength_deg" ...
    "OutgoingLength_deg" "IncomingDirection" "OutgoingDirection" ...
    "TurnRadius_deg" "MaxVelocity_deg_s" ...
    "MaxAcceleration_deg_s2" "MaxJerk_deg_s3"];
if ~isstruct(problem) || ~isscalar(problem) || ...
        ~all(isfield(problem, requiredFields))
    error("rpRetimerObservation:InvalidProblem", ...
        "problem must contain every documented scalar-structure field.");
end
validateattributes(problem.DeflectionAngle_rad, {'numeric'}, ...
    {'real', 'finite', 'scalar', '>=', 0, '<=', pi});
validateattributes(problem.IncomingLength_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(problem.OutgoingLength_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
validateattributes(problem.TurnRadius_deg, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'positive'});
incoming = axisVector(problem.IncomingDirection, ...
    "IncomingDirection", false);
outgoing = axisVector(problem.OutgoingDirection, ...
    "OutgoingDirection", false);
velocityLimit_deg_s = axisVector( ...
    problem.MaxVelocity_deg_s, "MaxVelocity_deg_s", true);
accelerationLimit_deg_s2 = axisVector( ...
    problem.MaxAcceleration_deg_s2, "MaxAcceleration_deg_s2", true);
jerkLimit_deg_s3 = axisVector( ...
    problem.MaxJerk_deg_s3, "MaxJerk_deg_s3", true);
if abs(norm(incoming) - 1) > 1e-8 || abs(norm(outgoing) - 1) > 1e-8
    error("rpRetimerObservation:NonunitDirection", ...
        "IncomingDirection and OutgoingDirection must be unit rows.");
end

%% Section 2: Form Dimensionless Physical Difficulty Features

minimumLength_deg = min( ...
    problem.IncomingLength_deg, problem.OutgoingLength_deg);
maximumLength_deg = max( ...
    problem.IncomingLength_deg, problem.OutgoingLength_deg);
finiteVelocity_deg_s = velocityLimit_deg_s(isfinite(velocityLimit_deg_s));
finiteAcceleration_deg_s2 = ...
    accelerationLimit_deg_s2(isfinite(accelerationLimit_deg_s2));
finiteJerk_deg_s3 = jerkLimit_deg_s3(isfinite(jerkLimit_deg_s3));
if isempty(finiteVelocity_deg_s)
    referenceVelocity_deg_s = minimumLength_deg;
else
    referenceVelocity_deg_s = min(finiteVelocity_deg_s);
end
if isempty(finiteAcceleration_deg_s2)
    referenceAcceleration_deg_s2 = ...
        referenceVelocity_deg_s^2 / minimumLength_deg;
else
    referenceAcceleration_deg_s2 = min(finiteAcceleration_deg_s2);
end
velocityTime_s = minimumLength_deg / referenceVelocity_deg_s;
accelerationTime_s = sqrt( ...
    minimumLength_deg / referenceAcceleration_deg_s2);
if isempty(finiteJerk_deg_s3)
    jerkTime_s = 0;
else
    jerkTime_s = nthroot( ...
        minimumLength_deg / min(finiteJerk_deg_s3), 3);
end
referenceTime_s = max([velocityTime_s accelerationTime_s jerkTime_s eps]);

%% Section 3: Assemble The Bounded Observation

observation = [ ...
    double(problem.DeflectionAngle_rad) / pi; ...
    minimumLength_deg / maximumLength_deg; ...
    min(5, log1p(minimumLength_deg)); ...
    min(2, double(problem.TurnRadius_deg) / minimumLength_deg); ...
    incoming(1); ...
    incoming(2); ...
    outgoing(1); ...
    outgoing(2); ...
    min(5, accelerationTime_s / referenceTime_s); ...
    min(5, jerkTime_s / referenceTime_s)];
observation = min(10, max(-10, observation));
end

%% Section 4: Local Functions

function value = axisVector(value, fieldName, allowInf)
% Normalize one scalar or two-axis input to a row.
value = double(value);
if isscalar(value)
    value = [value value];
else
    value = reshape(value, 1, []);
end
isValid = numel(value) == 2 && isreal(value) && all(~isnan(value));
if allowInf
    isValid = isValid && all(value > 0);
else
    isValid = isValid && all(isfinite(value));
end
if ~isValid
    error("rpRetimerObservation:InvalidAxisValue", ...
        "%s has an invalid scalar or two-axis value.", fieldName);
end
end
