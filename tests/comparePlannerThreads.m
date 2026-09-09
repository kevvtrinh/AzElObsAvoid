function summary = comparePlannerThreads(caseNames,repetitions,threadCounts)
%% Section 0: Header & Readme
% SYNTAX: summary = comparePlannerThreads(caseNames,repetitions,threadCounts)
% PURPOSE: Compare computational thread counts on identical validated examples.
% INPUTS: Optional example names, repeat count (three), and thread counts.
% OUTPUTS: Per-condition runtime medians and ranges, validity, duration, length.
%   Raw measurements are written to ignored benchmark CSV files.
% UNITS: Seconds and example coordinate units.

%% Section 1: Preserve The Session And Resolve The Experiment
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
previousThreads=maxNumCompThreads;
restoreThreads=onCleanup(@() maxNumCompThreads(previousThreads)); %#ok<NASGU>
if nargin<1 || isempty(caseNames)
    caseNames=["exampleMovingDeformingUSOutlineVisibility", ...
        "exampleStaticUShapedObstacle","exampleUSOutlineExtremeVisibility"];
end
if nargin<2 || isempty(repetitions), repetitions=3; end
if nargin<3 || isempty(threadCounts), threadCounts=unique([previousThreads,1],'stable'); end
validateattributes(repetitions,{'numeric'},{'scalar','integer','positive','finite'});
validateattributes(threadCounts,{'numeric'},{'vector','integer','positive','finite'});
caseNames=string(caseNames); records=struct([]);

%% Section 2: Alternate Conditions And Check Every Returned Subcase
for repetition=1:repetitions
    order=1:numel(threadCounts);
    if mod(repetition,2)==0, order=fliplr(order); end
    for caseIndex=1:numel(caseNames)
        name=caseNames(caseIndex);
        for condition=order
            maxNumCompThreads(threadCounts(condition));
            clear planner;
            timer=tic;
            if nargout(name)>=3
                [result,~,subcases]=feval(name,struct('PlotOutputs',false,'Verbose',false));
            else
                result=feval(name,struct('PlotOutputs',false,'Verbose',false));
                subcases={result};
            end
            wall_s=toc(timer);
            for subcaseIndex=1:numel(subcases)
                subcase=subcases{subcaseIndex};
                validation=obstacleAvoidance.validateTrajectory(subcase);
                valid=subcase.Success && validation.Passed;
                if name=="exampleNoPath" && ~subcase.Success
                    valid=subcase.TerminationReason=="noVisibilityRoute" && isempty(subcase.time_s);
                end
                length_units=NaN;
                if subcase.Success, length_units=subcase.MotionLength_units; end
                records=[records;struct('Case',name,'Repetition',repetition, ...
                    'Threads',threadCounts(condition),'Subcase',subcaseIndex, ...
                    'WallTime_s',wall_s,'Valid',valid, ...
                    'Duration_s',subcase.TrajectoryDuration_s,'Length_units',length_units)]; %#ok<AGROW>
            end
            fprintf('%s repeat=%d threads=%d wall=%.6g s\n',name,repetition,threadCounts(condition),wall_s);
            writetable(struct2table(records),fullfile(root,'benchmarks','thread_runs.csv'));
        end
    end
end

%% Section 3: Report Per-Subcase Quality And Whole-Example Runtime
runs=struct2table(records); summary=struct([]);
for name=reshape(caseNames,1,[])
    for threads=reshape(threadCounts,1,[])
        selected=runs(runs.Case==name & runs.Threads==threads,:);
        for subcaseIndex=reshape(unique(selected.Subcase),1,[])
            rows=selected(selected.Subcase==subcaseIndex,:);
            summary=[summary;struct('Case',name,'Threads',threads,'Subcase',subcaseIndex, ...
                'Valid',all(rows.Valid),'MedianWall_s',median(rows.WallTime_s), ...
                'MinimumWall_s',min(rows.WallTime_s),'MaximumWall_s',max(rows.WallTime_s), ...
                'MedianDuration_s',median(rows.Duration_s),'MedianLength_units',median(rows.Length_units))]; %#ok<AGROW>
        end
    end
end
summary=struct2table(summary);
writetable(summary,fullfile(root,'benchmarks','thread_summary.csv'));
end
