function cases = freezePlannerRequests(baselinePath, outputPath)
%% Section 0: Header & Readme
% SYNTAX: cases = freezePlannerRequests(baselinePath,outputPath)
% PURPOSE: Freeze physical example requests, including every geographic subcase.
% INPUTS: Capture from capturePlannerRefactorBaseline and destination MAT path.
% OUTPUTS: Named source histories, normalized states, per-axis limits and options.
% UNITS: Coordinate units, seconds, and physical per-axis derivative limits.

%% Section 1: Require The Declared Baseline Outcomes
data = load(baselinePath,'baseline');
baseline = data.baseline;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
cases = struct('Name',{},'Inputs',{},'Options',{},'ExpectedSuccess',{});
for k = 1:numel(baseline.ExampleNames)
    if baseline.ExampleNames(k) == "exampleUSOutlineExtremeVisibility", continue; end
    result = baseline.ExampleResults{k};
    expected = baseline.ExampleNames(k) ~= "exampleNoPath";
    assert(result.Success == expected && baseline.IndependentChecks{k}.Passed, 'Baseline example outcome is not established.');
    cases(end+1) = makeCase(baseline.ExampleNames(k),result,expected); %#ok<AGROW>
end

%% Section 2: Keep Each Geographic Request As Its Own Case
[~,~,sequence] = exampleUSOutlineExtremeVisibility(struct('PlotOutputs',false,'FigureVisible','off','Verbose',false));
for k = 1:numel(sequence.Names)
    result = sequence.Results{k};
    check = obstacleAvoidance.validateTrajectory(result);
    assert(result.Success && check.Passed,'Geographic baseline outcome is not established.');
    cases(end+1) = makeCase("Geography"+sequence.Names(k),result,true); %#ok<AGROW>
end
save(outputPath,'cases','-v7.3');
fprintf('FROZEN_REQUESTS count=%d\n',numel(cases));
end

function item = makeCase(name,result,expected)
    inputs = result.Inputs;
    if isfield(inputs.obstacles,'InternalPreparation')
        inputs.obstacles = rmfield(inputs.obstacles,'InternalPreparation');
    end
    item = struct('Name',name,'Inputs',inputs,'Options',result.Options,'ExpectedSuccess',expected);
end
