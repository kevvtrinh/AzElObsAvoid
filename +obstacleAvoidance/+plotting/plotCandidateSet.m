function handles = plotCandidateSet(candidateSet, scene, optionOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   handles = obstacleAvoidance.plotting.plotCandidateSet(candidateSet, scene)
%   handles = obstacleAvoidance.plotting.plotCandidateSet( ...
%       candidateSet, scene, optionOverrides)
%**************************************************************************
% PURPOSE
%   - Plot every attempted candidate and distinguish full-check outcomes.
%**************************************************************************
% INPUTS
%   - candidateSet (scalar candidate-set struct)
%       Contains Candidates and CheckResults returned by seed solving.
%   - scene (scalar prepared-scene struct)
%       Retained scene context; plotting does not query or rebuild it.
%   - optionOverrides (scalar struct, optional; default struct())
%       Supports FigureVisible and Title.
%**************************************************************************
% OUTPUTS
%   - handles (scalar struct)
%       Figure, axes, passing, and failed candidate graphics handles.
%**************************************************************************
% UNITS
%   - Candidate positions are degrees.
%**************************************************************************

%% Section 1: Check The Returned Stages

if nargin < 3
    optionOverrides = struct();
end
requiredCandidateSetFields = ["Candidates", "CheckResults"];
if ~isstruct(candidateSet) || ~isscalar(candidateSet) || ...
        ~all(isfield(candidateSet, cellstr(requiredCandidateSetFields))) || ...
        ~isstruct(scene) || ~isscalar(scene)
    error("plotCandidateSet:InvalidStage", ...
        "candidateSet and scene must be returned stage records.");
end
[figureHandle, axesHandle, options] = ...
    obstacleAvoidance.plotting.createStageAxes( ...
    "Solved candidate motions", optionOverrides);

%% Section 2: Plot Attempted Candidates

sceneHandles = obstacleAvoidance.plotting.plotSceneSamples( ...
    axesHandle, scene);
passingHandles = gobjects(0);
failedHandles = gobjects(0);
for candidateIndex = 1:numel(candidateSet.Candidates)
    candidate = candidateSet.Candidates{candidateIndex};
    if ~isstruct(candidate) || ~isfield(candidate, "position_deg") || ...
            isempty(candidate.position_deg)
        continue;
    end
    passed = candidateIndex <= numel(candidateSet.CheckResults) && ...
        isstruct(candidateSet.CheckResults(candidateIndex)) && ...
        isfield(candidateSet.CheckResults(candidateIndex), "Passed") && ...
        candidateSet.CheckResults(candidateIndex).Passed;
    if passed
        passingHandles(end + 1, 1) = plot(axesHandle, ...
            candidate.position_deg(:, 1), candidate.position_deg(:, 2), ...
            "-", "LineWidth", 2, ...
            "DisplayName", "Passing candidate " + candidateIndex); %#ok<AGROW>
    else
        failedHandles(end + 1, 1) = plot(axesHandle, ...
            candidate.position_deg(:, 1), candidate.position_deg(:, 2), ...
            ":", "DisplayName", ...
            "Failed candidate " + candidateIndex); %#ok<AGROW>
    end
end
subtitle(axesHandle, sprintf("attempted %d | passing %d", ...
    numel(candidateSet.Candidates), numel(passingHandles)));
if ~isempty(passingHandles) || ~isempty(failedHandles)
    legend(axesHandle, "Location", "best");
end
handles = struct("Figure", figureHandle, "Axes", axesHandle, ...
    "PassingHandles", passingHandles, "FailedHandles", failedHandles, ...
    "SceneHandles", sceneHandles, "Options", options);
end
