function [summary,trials,motions] = benchmarkC3ProfileLibrary(library,repetitions,outputStem)
%% Section 0: Header & Readme
% SYNTAX: [summary,trials,motions] = benchmarkC3ProfileLibrary(library,3,outputStem)
% PURPOSE: Compare ordinary BMTP with profile-assisted planning on eight held-out
%   detours. Include lookup, construction, validation, and fallback in wall time.
% INPUTS: Library struct/MAT filename (omitted: build the example training bank),
%   positive repetition count (default 3), optional output filename stem.
% OUTPUTS: Per-case summary table, every trial including warmups, and final public
%   planner results. An optional stem writes MAT, JSON, and summary CSV artifacts.
% UNITS: Coordinate units and seconds; jerk variation is normalized by axis limits.

%% Section 1: Resolve Inputs And Create Scenarios Disjoint From Training
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
if nargin<1 || isempty(library), library=buildExampleC3ProfileLibrary(); end
if nargin<2, repetitions=3; end
if nargin<3, outputStem=""; end
validateattributes(repetitions,{'numeric'},{'scalar','finite','integer','positive'});
outputStem=string(outputStem);
if ~isscalar(outputStem) || ismissing(outputStem), error('benchmarkC3ProfileLibrary:InvalidOutput','Output stem must be scalar.'); end
library=bmtpEngine.loadC3ProfileLibrary(library);
parameters=[1.02,1.04,-0.08;0.98,0.92,0.22;0.92,1.06,0.05;1.08,1.02,-0.25;0.99,1.01,0.1;1,1,0;1,1,0;1,1,0];
caseCount=size(parameters,1);
trials=repmat(struct('Case',0,'Variant',"",'Repetition',0,'Success',false,'Reason',"", ...
    'WallTime_s',0,'ArrivalTime_s',NaN,'MotionLength_units',NaN,'JerkVariation',NaN, ...
    'ProfileMatched',false,'ProfileAccepted',false,'FallbackUsed',false,'FallbackTime_s',0, ...
    'LookupTime_s',0,'InitializationCount',0),caseCount*2*(repetitions+1),1);
motions=cell(caseCount,2); inputs=cell(caseCount,1); trialIndex=0;
for caseIndex=1:caseCount
    row=parameters(caseIndex,:);
    vertices_units=[-8,7;-5,7;-5,-4;5,-4;5,7;8,7;8,-7;-8,-7].*row(1:2);
    initial=struct('time_s',0,'position_units',[row(3),row(3)/2]);
    goal=struct('time_s',120,'position_units',[0,-10*row(2)]);
    limits=struct('maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75],'maxJerk_units_s3',[2.5,2.5]);
    margin_units=0.2;
    if caseIndex==5
        limits.maxVelocity_units_s=[1.85,2.05];
        limits.maxAcceleration_units_s2=[0.72,0.83];
        limits.maxJerk_units_s3=[2.45,2.7];
    elseif caseIndex==6
        angle_rad=pi/6;
        rotation=[cos(angle_rad),-sin(angle_rad);sin(angle_rad),cos(angle_rad)];
        shift_units=[7,-9];
        vertices_units=vertices_units*rotation.'+shift_units;
        initial.position_units=shift_units;
        goal.position_units=[0,-10]*rotation.'+shift_units;
    elseif caseIndex==7
        vertices_units=1.4*vertices_units;
        margin_units=1.4*margin_units;
        goal.position_units=[0,-14];
    elseif caseIndex==8
        vertices_units=[-8.4,7.1;-4.4,7.1;-4.4,-3.5;4.8,-3.5;4.8,7.8;8.4,7.8;8.4,-7.5;-8.4,-7.5];
        initial.position_units=[-0.15,0.2];
        goal.position_units=[1,-10.2];
    end
    obstacles=obstacleAvoidance.obstacles.createObstacle('held-out profile benchmark',[0;120],vertices_units(:,1),vertices_units(:,2),margin_units);
    inputs{caseIndex}={obstacles,initial,goal,limits};

    %% Section 2: Alternate Identical-Input Planner Runs
    for repetition=0:repetitions
        for variant=1:2
            options=struct('GoalTimeMode','earliestArrival');
            label="ordinary";
            if variant==2
                options.C3ProfileLibrary=library;
                options.C3ProfileMode='repair';
                label="library";
            end
            timer=tic;
            result=planner(obstacles,initial,goal,limits,options);
            wallTime_s=toc(timer);
            trialIndex=trialIndex+1;
            record=trials(trialIndex);
            record.Case=caseIndex; record.Variant=label; record.Repetition=repetition;
            record.Success=result.Success; record.Reason=result.TerminationReason;
            record.WallTime_s=wallTime_s; record.ArrivalTime_s=result.ArrivalTime_s;
            if result.Success
                record.MotionLength_units=result.MotionLength_units;
                record.JerkVariation=jerkVariation(result);
            end
            if isfield(result.SolverDiagnostics,'ProfileLibrary')
                profile=result.SolverDiagnostics.ProfileLibrary;
                record.ProfileMatched=profile.Matched; record.ProfileAccepted=profile.Accepted;
                record.FallbackUsed=profile.FallbackUsed; record.FallbackTime_s=profile.FallbackTime_s;
                record.LookupTime_s=profile.LookupTime_s; record.InitializationCount=numel(profile.InitialTrials);
            end
            trials(trialIndex)=record;
            motions{caseIndex,variant}=result;
            fprintf('C3 benchmark %d/%d %s rep %d: valid %d, arrival %.6f, wall %.3f s\n', ...
                caseIndex,caseCount,label,repetition,result.Success,result.ArrivalTime_s,wallTime_s);
        end
    end
end

%% Section 3: Report Separate Quality, Timing, And Recovery Measures
rows=repmat(struct('Case',0,'ReferenceSuccess',false,'LibrarySuccess',false, ...
    'ReferenceWall_s',NaN,'LibraryWall_s',NaN,'Speedup',NaN,'ArrivalDelta_s',NaN, ...
    'LengthDelta_pct',NaN,'JerkVariationDelta_pct',NaN,'AcceptedRate',0,'FallbackRate',0),caseCount,1);
for caseIndex=1:caseCount
    reference=trials([trials.Case]==caseIndex & [trials.Variant]=="ordinary" & [trials.Repetition]>0);
    reused=trials([trials.Case]==caseIndex & [trials.Variant]=="library" & [trials.Repetition]>0);
    rows(caseIndex).Case=caseIndex;
    rows(caseIndex).ReferenceSuccess=all([reference.Success]);
    rows(caseIndex).LibrarySuccess=all([reused.Success]);
    rows(caseIndex).ReferenceWall_s=median([reference.WallTime_s]);
    rows(caseIndex).LibraryWall_s=median([reused.WallTime_s]);
    rows(caseIndex).AcceptedRate=mean([reused.ProfileAccepted]);
    rows(caseIndex).FallbackRate=mean([reused.FallbackUsed]);
    if rows(caseIndex).ReferenceSuccess && rows(caseIndex).LibrarySuccess
        rows(caseIndex).Speedup=rows(caseIndex).ReferenceWall_s/rows(caseIndex).LibraryWall_s;
        rows(caseIndex).ArrivalDelta_s=median([reused.ArrivalTime_s])-median([reference.ArrivalTime_s]);
        rows(caseIndex).LengthDelta_pct=100*(median([reused.MotionLength_units])/median([reference.MotionLength_units])-1);
        rows(caseIndex).JerkVariationDelta_pct=100*(median([reused.JerkVariation])/median([reference.JerkVariation])-1);
    end
end
summary=struct2table(rows);
if strlength(outputStem)>0
    save(outputStem+'.mat','summary','trials','motions','inputs','library');
    writetable(summary,outputStem+'_summary.csv');
    file=fopen(outputStem+'.json','w');
    if file<0, error('benchmarkC3ProfileLibrary:OutputUnavailable','Cannot write the output JSON.'); end
    cleanup=onCleanup(@() fclose(file)); %#ok<NASGU>
    fprintf(file,'%s',jsonencode(struct('Summary',rows,'Trials',trials),'PrettyPrint',true));
end
end

function total=jerkVariation(result)
    % Exact quadratic-jerk extrema plus joins; sampling cannot hide oscillations.
    variation=zeros(1,2);
    for axis=1:2
        previous=[];
        for span=1:result.Polynomial.SegmentCount
            power=reshape(result.Polynomial.jerkPower_units_s3(span,axis,:),1,3);
            turns=roots([2*power(3),power(2)]);
            turns=real(turns(abs(imag(turns))<1e-12 & real(turns)>0 & real(turns)<1));
            values=polyval(fliplr(power),sort([0;turns;1]));
            if ~isempty(previous), variation(axis)=variation(axis)+abs(values(1)-previous); end
            variation(axis)=variation(axis)+sum(abs(diff(values)));
            previous=values(end);
        end
    end
    total=sum(variation./result.Limits.maxJerk_units_s3);
end
