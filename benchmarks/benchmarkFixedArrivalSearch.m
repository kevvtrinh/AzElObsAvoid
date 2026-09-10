function [runs, results] = benchmarkFixedArrivalSearch(repetitions)
%% Section 0: Header & Readme
% SYNTAX: [runs, results] = benchmarkFixedArrivalSearch(repetitions)
% PURPOSE: Compare fixed-arrival spatial and timed search on identical requests.
% INPUTS: Measured repetitions per method after warmup; default three.
% OUTPUTS: Per-run timing/quality table and complete corresponding planner results.
% UNITS: Coordinate units and seconds; Vietnam coordinates are degrees.

%% Section 1: Load Both Deterministic Source Requests
if nargin<1, repetitions=3; end
validateattributes(repetitions,{'numeric'},{'scalar','integer','positive','finite'});
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
[obstacles,initialState,goalState,limits]=createVietnamBoundaryScenario();
cases(1)=struct('Name',"Vietnam",'obstacles',obstacles,'initial',initialState, ...
    'goal',goalState,'limits',limits,'options',struct('GoalTimeMode','fixedArrival'));
request=jsondecode(fileread(fullfile(root,'tests','fixtures','savedMovingDetour.json')));
sources=cell(numel(request.obstacles),1);
for k=1:numel(sources)
    source=request.obstacles(k); frames=source.keyframes;
    sources{k}=obstacleAvoidance.obstacles.createObstacle(source.name,[frames.time_s].', ...
        arrayfun(@(f)f.vertices_units(:,1),frames,'UniformOutput',false), ...
        arrayfun(@(f)f.vertices_units(:,2),frames,'UniformOutput',false),source.safetyMargin_units);
end
request.options.GoalTimeMode='fixedArrival';
cases(2)=struct('Name',"Saved moving detour", ...
    'obstacles',obstacleAvoidance.obstacles.combineObstacles(sources), ...
    'initial',request.initialState,'goal',request.goalState, ...
    'limits',request.limits,'options',request.options);

%% Section 2: Warm Each Method And Counterbalance Measured Run Order
rows=cell(0,1); results=cell(0,1);
for caseIndex=1:numel(cases)
    input=cases(caseIndex);
    for repetition=0:repetitions
        methods=["spatial","timeExpanded"];
        if mod(repetition,2)==0, methods=fliplr(methods); end
        for method=methods
            options=input.options; options.FixedArrivalSearch=method;
            timer=tic;
            result=planner(input.obstacles,input.initial,input.goal,input.limits,options);
            elapsed_s=toc(timer);
            passed=result.Success && obstacleAvoidance.validateTrajectory(result).Passed;
            arrival_s=NaN; length_units=NaN; searchTime_s=NaN;
            if result.Success
                arrival_s=result.ArrivalTime_s; length_units=result.MotionLength_units;
            end
            if isfield(result.VisibilityGraph,'TimedSearch')
                searchTime_s=result.VisibilityGraph.TimedSearch.ElapsedTime_s;
            end
            fprintf('%s %s repetition %d: %.6f s; valid %d; arrival %.9f; length %.12f; %s\n', ...
                input.Name,method,repetition,elapsed_s,passed,arrival_s,length_units,result.TerminationReason);
            if repetition==0, continue; end
            rows{end+1,1}=struct('Case',input.Name,'Search',method,'Repetition',repetition, ...
                'Runtime_s',elapsed_s,'Success',result.Success,'IndependentValidation',passed, ...
                'Arrival_s',arrival_s,'Length_units',length_units,'SearchTime_s',searchTime_s, ...
                'TerminationReason',result.TerminationReason); %#ok<AGROW>
            results{end+1,1}=result; %#ok<AGROW>
        end
    end
end
runs=struct2table(vertcat(rows{:}));
end
