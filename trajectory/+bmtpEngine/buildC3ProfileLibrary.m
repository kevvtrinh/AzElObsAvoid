function library=buildC3ProfileLibrary(results,outputFile)
%% Section 0: Header & Readme
% SYNTAX: library = bmtpEngine.buildC3ProfileLibrary(results,outputFile)
% PURPOSE: Precompute reusable C3 profiles from independently validated motions.
% INPUTS: Cell array of public planner results; optional MAT output filename.
% OUTPUTS: Versioned numeric library containing normalized geometry and motion.
% UNITS: Stored positions use unit endpoint distance and times use unit duration.

%% Section 1: Validate And Normalize Each Training Motion
if nargin<2, outputFile=""; end
if ~(ischar(outputFile) && (isrow(outputFile) || isempty(outputFile))) && ...
        ~(isstring(outputFile) && isscalar(outputFile) && ~ismissing(outputFile))
    error('bmtpEngine:InvalidProfileFile','Output filename must be a character row or string scalar.');
end
outputFile=string(outputFile);
if ~iscell(results) || isempty(results)
    error('bmtpEngine:InvalidProfileTraining','Supply a nonempty cell array of planner results.');
end
entries=cell(numel(results),1);
for index=1:numel(results)
    result=results{index};
    if ~isstruct(result) || ~isscalar(result) || ~isfield(result,'Success') || ~result.Success
        error('bmtpEngine:InvalidProfileTraining','Training result %d must be a successful planner result.',index);
    end
    validation=obstacleAvoidance.validateTrajectory(result);
    if ~validation.Passed
        error('bmtpEngine:InvalidProfileTraining','Training motion %d failed independent validation: %s',index,validation.Message);
    end
    descriptor=bmtpEngine.describeC3Profile(result.Route_units,result.Inputs.limits);
    origin=descriptor.Origin_units;
    distance=descriptor.Distance_units;
    frame=descriptor.Frame;
    power=result.Polynomial.positionPower_units;
    power(:,:,1)=power(:,:,1)-origin;
    for order=1:6, power(:,:,order)=power(:,:,order)*frame/distance; end
    times=result.Polynomial.SegmentDuration_s/result.TrajectoryDuration_s;
    phaseTime=times;
    if isfield(result.SolverDiagnostics,'QuinticPhaseTime_s')
        phaseTime=result.SolverDiagnostics.QuinticPhaseTime_s/result.TrajectoryDuration_s;
    elseif isfield(result.SolverDiagnostics,'OptimizerSpanCount') && ...
            numel(times)==2*result.SolverDiagnostics.OptimizerSpanCount && ...
            all(abs(times(1:2:end)-times(2:2:end))<1e-12)
        % Older exports split every optimizer span exactly in half.
        phaseTime=times(1:2:end)+times(2:2:end);
    end
    states=[result.Inputs.initialState.velocity_units_s,result.Inputs.initialState.acceleration_units_s2, ...
        result.Inputs.goalState.velocity_units_s,result.Inputs.goalState.acceleration_units_s2];
    entries{index}=struct('Route',descriptor.Route,'PositionPower',power,'SegmentTime',times(:), ...
        'PhaseTime',phaseTime(:),'LimitSignature',descriptor.LimitSignature,'RestEndpoints',all(states==0), ...
        'SourceDuration_s',result.TrajectoryDuration_s,'SourceLength_units',result.MotionLength_units);
end
library=struct('SchemaVersion',1,'Degree',5,'Entries',{entries});
library=bmtpEngine.loadC3ProfileLibrary(library);

%% Section 2: Save The Explicitly Requested Library Artifact
if strlength(outputFile)>0, save(outputFile,'library'); end
end
