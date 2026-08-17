function environment = createAzElRpRetimerEnvironment(seed, trainingData)
%% Section 0: Header & Readme
% SYNTAX
%   environment = createAzElRpRetimerEnvironment()
%   environment = createAzElRpRetimerEnvironment(seed)
%   environment = createAzElRpRetimerEnvironment(seed, trainingData)
%**************************************************************************
% PURPOSE
%   - Create a reproducible contextual RP environment that learns one
%     production-selected G3 corner-radius fraction.
%   - Keep obstacles and all acceptance decisions outside the learned policy.
%**************************************************************************
% INPUTS
%   - seed (nonnegative integer scalar, optional; default 41)
%       Seed for deterministic sampling from the supplied production data.
%   - trainingData (scalar struct, optional)
%       Data from azElInternal.createRpRetimerTrainingData. When omitted,
%       64 production cases are generated with the supplied seed.
%**************************************************************************
% OUTPUTS
%   - environment (rl.env.MATLABEnvironment)
%       One-step continuous-action environment with ten observations.
%**************************************************************************
% UNITS
%   - Observations, actions, and reward are dimensionless. Source geometry
%     uses degrees and seconds before feature normalization.
%**************************************************************************

%% Section 1: Define The Contextual Policy Interface

if nargin < 1 || isempty(seed)
    seed = 41;
end
validateattributes(seed, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'nonnegative'});
if nargin < 2 || isempty(trainingData)
    trainingData = azElInternal.createRpRetimerTrainingData(64, seed);
end
validateTrainingData(trainingData);
observationInfo = rlNumericSpec([10 1], ...
    "LowerLimit", -10 * ones(10, 1), ...
    "UpperLimit", 10 * ones(10, 1));
observationInfo.Name = "unconstrained corner problem";
actionInfo = rlNumericSpec([1 1], "LowerLimit", -1, "UpperLimit", 1);
actionInfo.Name = "normalized corner-radius proposal";

%% Section 2: Build The Reproducible One-Step Environment

stream = RandStream("Threefry", "Seed", seed);
resetHandle = @() resetEpisode(stream, trainingData);
environment = rlFunctionEnv( ...
    observationInfo, actionInfo, @stepEpisode, resetHandle);
end

%% Section 3: Local Functions

function [observation, loggedSignals] = resetEpisode(stream, trainingData)
% Select one production-labeled corner without changing its observation.
caseIndex = randi(stream, trainingData.CaseCount);
observation = trainingData.Observation(:, caseIndex);
loggedSignals = struct( ...
    "BestRadiusFraction", ...
        trainingData.BestRadiusFraction(caseIndex));
end

function [nextObservation, reward, isDone, loggedSignals] = ...
        stepEpisode(action, loggedSignals)
% Reward agreement with the production-selected obstacle-free radius.
action = min(1, max(-1, double(action)));
radiusFraction = 0.5 * (action + 1);
fractionError = radiusFraction - loggedSignals.BestRadiusFraction;
reward = -1 - 4 * fractionError^2;
nextObservation = zeros(10, 1);
isDone = true;
end

function validateTrainingData(trainingData)
% Reject incompatible data before the environment captures it.
requiredFields = ["Format" "CaseCount" "Observation" ...
    "BestRadiusFraction"];
if ~isstruct(trainingData) || ~isscalar(trainingData) || ...
        ~all(isfield(trainingData, requiredFields)) || ...
        string(trainingData.Format) ~= "AzElRpRetimerTrainingData"
    error("createAzElRpRetimerEnvironment:InvalidTrainingData", ...
        "trainingData must use the AzElRpRetimerTrainingData format.");
end
validateattributes(trainingData.CaseCount, {'numeric'}, ...
    {'real', 'finite', 'scalar', 'integer', 'positive'});
validateattributes(trainingData.Observation, {'numeric'}, ...
    {'real', 'finite', 'size', [10 trainingData.CaseCount]});
validateattributes(trainingData.BestRadiusFraction, {'numeric'}, ...
    {'real', 'finite', 'size', [1 trainingData.CaseCount], ...
        '>=', 0, '<=', 1});
end
