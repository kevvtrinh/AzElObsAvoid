function results = exampleRandomAzimuthSuite(caseIndices, exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX
%   results = exampleRandomAzimuthSuite()
%   results = exampleRandomAzimuthSuite(caseIndices)
%   results = exampleRandomAzimuthSuite(caseIndices, exampleOverrides)
%**************************************************************************
% PURPOSE
%   - Run eighty reproducible wide-azimuth slews, each with and without a
%     second static obstacle, and show the returned motions in a paged gallery.
%**************************************************************************
% INPUTS
%   - caseIndices (positive integer vector, optional; default 1:80)
%       Selects which deterministic random scenarios to reproduce.
%   - exampleOverrides (scalar struct, optional; default struct())
%       Uniform display controls and public planner option overrides.
%       PlotOutputs = false disables the gallery.
%**************************************************************************
% OUTPUTS
%   - results (N-by-2 cell array)
%       Unmodified public results, moving-only in column one and with-static in
%       column two. Every successful motion is rechecked with the public
%       independent validator; invalid input throws.
%**************************************************************************
% UNITS
%   - Degrees and seconds.
%**************************************************************************

%% Section 1: Resolve Example Controls

if nargin < 1
    caseIndices = 1:80;
end
if nargin < 2
    exampleOverrides = struct();
end
validateattributes(caseIndices, {'numeric'}, ...
    {'real', 'finite', 'vector', 'integer', 'positive', 'nonempty'});
scenarioDefaults = struct('GoalTimeMode', 'fixedArrival', 'SampleTime_s', 0.1);
[options, displayOptions] = resolveExampleOptions(exampleOverrides, scenarioDefaults, [2, 2]);
results = cell(numel(caseIndices), 2);
labels  = strings(size(results));

%% Section 2: Create Inputs, Run Planner, And Validate Paired Cases

% Variant one uses the moving obstacle alone and variant two adds the static
% obstacle, so each case index yields one directly comparable pair. Each
% deterministic scenario is created immediately before its only planner call.

for caseListIndex = 1:numel(caseIndices)
    for variantIndex = 1:2
        includeStaticObstacle = variantIndex == 2;
        scenario = createRandomAzimuthScenario(caseIndices(caseListIndex), includeStaticObstacle);
        scenario.Limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;
        result = planner(scenario.Obstacles, scenario.InitialState, scenario.GoalState, ...
            scenario.Limits, options);
        if result.Success
            validation = obstacleAvoidance.validateTrajectory(result);
            assert(validation.Passed, validation.Message);
        end
        results{caseListIndex, variantIndex} = result;
        labels(caseListIndex, variantIndex) = "Case " + caseIndices(caseListIndex) + ...
            " static=" + includeStaticObstacle;
    end
end

%% Section 3: Plot Diagnostics And Motion

if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectoryGallery(results, labels, displayOptions.FigureVisible);
end

end
