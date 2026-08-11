function metrics = runAcceptanceBenchmarks()
%% Section 0: Header & Readme
% SYNTAX
%   metrics = runAcceptanceBenchmarks()
%**************************************************************************
% PURPOSE
%   - Run the preserved numbered missions and return comparable physical,
%     validation, work, and guarantee evidence without changing inputs.
%**************************************************************************
% INPUTS
%   - None.
%**************************************************************************
% OUTPUTS
%   - metrics (table)
%       One row per preserved mission with input identity and evidence.
%**************************************************************************
% UNITS
%   - Column names state their units where applicable.

%% Section 1: Run Preserved Inputs
repositoryRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(repositoryRoot);
addpath(fullfile(repositoryRoot, "examples"));
caseName = ["example01Unobstructed"; "example02StaticDetour"; ...
    "example03TimedWallWait"];
result = cell(numel(caseName), 1);
result{1} = example01Unobstructed(false);
result{2} = example02StaticDetour(false);
result{3} = example03TimedWallWait(false);

%% Section 2: Extract Comparable Evidence
success = false(numel(caseName), 1);
validated = false(numel(caseName), 1);
arrivalTime_s = nan(numel(caseName), 1);
executionDuration_s = nan(numel(caseName), 1);
maximumTerminalPositionError_deg = nan(numel(caseName), 1);
maximumTerminalVelocityError_deg_s = nan(numel(caseName), 1);
maximumTerminalAccelerationError_deg_s2 = nan(numel(caseName), 1);
minimumClearance_deg = nan(numel(caseName), 1);
maximumVelocityUse = nan(numel(caseName), 1);
maximumAccelerationUse = nan(numel(caseName), 1);
angularTravel_deg = nan(numel(caseName), 1);
minimumInteriorSpeed_deg_s = nan(numel(caseName), 1);
waitCount = zeros(numel(caseName), 1);
totalWaitDuration_s = zeros(numel(caseName), 1);
planningElapsed_s = nan(numel(caseName), 1);
memoryAfter_bytes = nan(numel(caseName), 1);
optimality = strings(numel(caseName), 1);

for caseIndex = 1:numel(caseName)
    caseResult = result{caseIndex};
    evidence = caseResult.validation;
    success(caseIndex) = caseResult.success;
    validated(caseIndex) = evidence.isValid;
    arrivalTime_s(caseIndex) = caseResult.arrivalTime_s;
    executionDuration_s(caseIndex) = caseResult.executionDuration_s;
    maximumTerminalPositionError_deg(caseIndex) = max(abs( ...
        evidence.terminalPositionError_deg));
    maximumTerminalVelocityError_deg_s(caseIndex) = max(abs( ...
        evidence.terminalVelocityError_deg_s));
    maximumTerminalAccelerationError_deg_s2(caseIndex) = max(abs( ...
        evidence.terminalAccelerationError_deg_s2));
    minimumClearance_deg(caseIndex) = ...
        evidence.collision.minimumClearance_deg;
    maximumVelocityUse(caseIndex) = max( ...
        evidence.motion.maximumAbsoluteVelocity_deg_s ./ ...
        caseResult.limits.maxVelocity_deg_s);
    maximumAccelerationUse(caseIndex) = max( ...
        evidence.motion.maximumAbsoluteAcceleration_deg_s2 ./ ...
        caseResult.limits.maxAcceleration_deg_s2);
    angularTravel_deg(caseIndex) = evidence.motion.angularDistance_deg;
    minimumInteriorSpeed_deg_s(caseIndex) = ...
        evidence.motion.minimumInteriorSpeed_deg_s;
    waitCount(caseIndex) = evidence.waitCount;
    totalWaitDuration_s(caseIndex) = evidence.totalWaitDuration_s;
    planningElapsed_s(caseIndex) = caseResult.planningElapsed_s;
    optimality(caseIndex) = caseResult.guarantee.optimality;
    if ispc
        memoryStatus = memory;
        memoryAfter_bytes(caseIndex) = memoryStatus.MemUsedMATLAB;
    end
end

%% Section 3: Assemble The Benchmark Table
metrics = table(caseName, success, validated, arrivalTime_s, ...
    executionDuration_s, maximumTerminalPositionError_deg, ...
    maximumTerminalVelocityError_deg_s, ...
    maximumTerminalAccelerationError_deg_s2, minimumClearance_deg, ...
    maximumVelocityUse, maximumAccelerationUse, angularTravel_deg, ...
    minimumInteriorSpeed_deg_s, waitCount, totalWaitDuration_s, ...
    planningElapsed_s, memoryAfter_bytes, optimality);
end
