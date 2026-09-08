function profileInfo = profileGeometryWork(casesPath, outputPath)
%% Section 0: Header & Readme
% SYNTAX: profileInfo = profileGeometryWork(casesPath,outputPath)
% PURPOSE: Attribute preparation and geometry cost separately from wall timing.
% INPUTS: Frozen requests and an output MAT path; tested implementation on path.
% OUTPUTS: Complete MATLAB profile and selected call-count/timing diagnostics.
% UNITS: Seconds and operation counts.

%% Section 1: Warm The Accelerating-History Request
data = load(casesPath,'cases');
selected = find(string({data.cases.Name}) == "exampleFourAcceleratingCircles",1);
request = data.cases(selected).Inputs;
options = data.cases(selected).Options;
[~,~] = obstacleAvoidance.planTrajectory(request.obstacles,request.initialState,request.goalState,request.limits,options);

%% Section 2: Profile Once And Retain Complete Attribution
profile clear;
profile on;
[result,diagnosis] = obstacleAvoidance.planTrajectory(request.obstacles,request.initialState,request.goalState,request.limits,options);
profile off;
profileInfo = profile('info');
save(outputPath,'profileInfo','result','diagnosis');
functions = profileInfo.FunctionTable;
for k = 1:numel(functions)
    if contains(string(functions(k).FunctionName),["prepareOneObstacle","preparedShapeAtTime","prepareObstacles","planTrajectory"])
        fprintf('GEOMETRY_PROFILE calls=%d total_s=%.9g function=%s\n',functions(k).NumCalls,functions(k).TotalTime,functions(k).FunctionName);
    end
end
end
