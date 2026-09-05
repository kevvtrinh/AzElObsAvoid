function metrics=verifyFastconeExample(name,showPlots)
%% Section 0: Header & Readme
% SYNTAX: metrics=verifyFastconeExample(name); metrics=verifyFastconeExample(name,true)
% PURPOSE: Verify one maintained example and record executed conic methods.
% INPUTS: Maintained example function name; optional plots (default false).
% OUTPUTS: Metrics plus saved original result, validation, and solver evidence.
% UNITS: Degrees and seconds. Wall time includes example setup and validation.

%% Section 1: Run One Unmodified Maintained Example
if nargin<2, showPlots=false; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'trajectory'),fullfile(root,'examples'));
destination=fullfile(root,'output'); if ~isfolder(destination), mkdir(destination); end
assert(startsWith(which('fastcone.solve'),root),'The wrong fastcone package is on the path.');
overrides=struct('PlotOutputs',showPlots,'Verbose',false,'ShowAnimation',false);
if showPlots, overrides.FigureVisible='on'; end
rng(0,'twister'); lastwarn(''); timer=tic;
result=feval(name,overrides); wallTime=toc(timer);
validation=obstacleAvoidance.validateTrajectory(result);
[warningText,warningID]=lastwarn;
expectedFailure=strcmp(name,'exampleNoPath') && ~result.Success && ...
    isempty(result.time_s) && result.TerminationReason=="noValidatedSeed" && ...
    isfield(result.SearchDiagnostics,'Grid') && ...
    isfield(result.SearchDiagnostics.Grid,'ExpandedCount');
passed=(result.Success && validation.Passed) || expectedFailure;
polyline=NaN; smooth=NaN; duration=NaN;
if result.Success
    polyline=sum(vecnorm(diff(result.SelectedSeed_deg),2,2));
    smooth=sum(vecnorm(diff(result.position_deg),2,2)); duration=result.TrajectoryDuration_s;
end
evidence=findEvidence(result); hasConic=false; native=false; recovery=false;
for k=1:numel(evidence)
    hasConic=hasConic || evidence{k}.CallCount>0;
    native=native || evidence{k}.NativeAcceptedCount>0;
    recovery=recovery || evidence{k}.RecoveryCount>0;
end

%% Section 2: Retain Validation And Append The Established Benchmark Schema
kinematic=validation.PositionWithinLimits && validation.VelocityWithinLimits && ...
    validation.AccelerationWithinLimits && validation.JerkWithinLimits;
metrics=struct('Example',string(name),'Planner',result.Success,'Independent',validation.Passed, ...
    'ExpectedFailure',expectedFailure,'Passed',passed,'Polyline_deg',polyline, ...
    'Smooth_deg',smooth,'Duration_s',duration,'WallTime_s',wallTime, ...
    'Collision',validation.CollisionFree,'Kinematics',kinematic, ...
    'PlaneCertificate',validation.PlaneCertificateCertified, ...
    'ContinuousCollision',validation.CollisionResolved,'Termination',result.TerminationReason, ...
    'HasConicCalls',hasConic,'NativeExecuted',native,'RecoveryUsed',recovery, ...
    'Warning',string(warningText),'WarningID',string(warningID));
suffix='headless'; if showPlots, suffix='plots'; end
save(fullfile(destination,['fastcone-example-' char(name) '-' suffix '.mat']), ...
    'result','validation','metrics','evidence','overrides','-v7.3');
notes=sprintf('fastcone integration; %s diagnostic wall time; conic=%d native=%d recovery=%d; expectedFailure=%d', ...
    suffix,hasConic,native,recovery,expectedFailure);
row=struct('RunDate',string(datetime('now','Format','yyyy-MM-dd')), ...
    'SourceCommit',"a072037+fastcone-integration",'Branch',"bmtp-cleanup-codex", ...
    'Example',string(name),'GoalTimeMode',string(result.Options.GoalTimeMode), ...
    'JerkConstrained',all(isfinite(result.Inputs.limits.maxJerk_deg_s3)), ...
    'PlannerSuccess',result.Success,'ValidationPassed',passed, ...
    'SelectedPolylineLength_deg',polyline,'SmoothedPathLength_deg',smooth, ...
    'MotionDuration_s',duration,'CollisionFree',validation.CollisionFree, ...
    'KinematicCertificatePassed',kinematic, ...
    'ApplicableCertificatePassed',validation.CollisionResolved, ...
    'WallTime_s',wallTime,'TerminationReason',string(result.TerminationReason),'Notes',string(notes));
writetable(struct2table(row),fullfile(root,'benchmark.csv'),'WriteMode','append','WriteVariableNames',false);
if showPlots
    figures=findall(groot,'Type','figure');
    assert(~isempty(figures),'The example did not produce a diagnostic figure.');
    for k=1:numel(figures)
        exportgraphics(figures(k),fullfile(destination,sprintf('fastcone-%s-%d.png',name,k)));
    end
    close(figures);
end
fprintf('FASTCONE_EXAMPLE %s passed=%d planner=%d independent=%d polyline=%.12g smooth=%.12g duration=%.12g wall=%.6g conic=%d native=%d recovery=%d termination=%s\n', ...
    name,passed,result.Success,validation.Passed,polyline,smooth,duration,wallTime,hasConic,native,recovery,result.TerminationReason);
assert(passed,'Independent example validation failed.');
end

function records=findEvidence(value)
% Retain phase statistics with no global state; duplicates are not summed.
records={};
if isstruct(value)
    fields=fieldnames(value);
    for element=1:numel(value)
        if isfield(value,'ConicSolver'), records{end+1}=value(element).ConicSolver; end %#ok<AGROW>
        for k=1:numel(fields)
            if strcmp(fields{k},'ConicSolver'), continue; end
            next=value(element).(fields{k});
            if isstruct(next) || iscell(next)
                records=[records findEvidence(next)]; %#ok<AGROW>
            end
        end
    end
elseif iscell(value)
    for k=1:numel(value), records=[records findEvidence(value{k})]; end %#ok<AGROW>
end
end
