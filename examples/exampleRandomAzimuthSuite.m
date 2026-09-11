function results = exampleRandomAzimuthSuite(caseIndices, exampleOverrides)
%% Section 0: Header & Readme
% SYNTAX: results = exampleRandomAzimuthSuite(caseIndices, exampleOverrides)
% PURPOSE: Run eighty reproducible wide-azimuth slews, each with and without a
%   second static obstacle, and show the returned motions in a paged gallery.
% INPUTS: Positive integer caseIndices (default 1:80); standard example display
%   and planner overrides. PlotOutputs=false disables the gallery.
% OUTPUTS: N-by-2 cell array of public results, moving-only then with-static.
%   All successful motions are checked again with the public independent validator.
% UNITS: Degrees and seconds.

%% Section 1: Resolve The Case List And Example Controls
if nargin < 1, caseIndices = 1:80; end
if nargin < 2, exampleOverrides = struct(); end
validateattributes(caseIndices,{'numeric'},{'real','finite','vector','integer','positive','nonempty'});
[options,displayOptions] = resolveExampleOptions(exampleOverrides, ...
    struct('GoalTimeMode','fixedArrival','SampleTime_s',0.1),[2,2]);
results = cell(numel(caseIndices),2);
labels = strings(size(results));

%% Section 2: Plan And Independently Validate Every Paired Case
for k = 1:numel(caseIndices)
    for variant = 1:2
        scenario = createRandomAzimuthScenario(caseIndices(k),variant==2);
        scenario.Limits.maxJerk_units_s3 = displayOptions.MaxJerk_units_s3;
        result = planner(scenario.Obstacles,scenario.InitialState,scenario.GoalState,scenario.Limits,options);
        if result.Success
            validation = obstacleAvoidance.validateTrajectory(result);
            assert(validation.Passed,validation.Message);
        end
        results{k,variant} = result;
        labels(k,variant) = "Case "+caseIndices(k)+" static="+(variant==2);
    end
end

%% Section 3: Plot Only The Returned Core Results
if displayOptions.PlotOutputs
    obstacleAvoidance.plotting.plotTrajectoryGallery(results,labels,displayOptions.FigureVisible);
end
end
