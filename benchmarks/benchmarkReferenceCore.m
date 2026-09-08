function report = benchmarkReferenceCore(casesPath, outputPath)
%% Section 0: Header & Readme
% SYNTAX: report = benchmarkReferenceCore(casesPath, outputPath)
% PURPOSE: Run the reference core on unchanged cleanup physical requests.
% INPUTS: Frozen request MAT path and output MAT path; reference root on path.
% OUTPUTS: Three individual runs, independent checks, timings, and limitations.
% UNITS: Seconds, coordinate units, and per-axis physical derivative limits.

%% Section 1: Map Representation Names Without Changing Physical Inputs
data = load(casesPath,'cases');
cases = data.cases;
report = struct();
report.PlannerPath = which('planner');
report.MatlabVersion = version;
report.Toolboxes = ver;
report.CaseNames = string({cases.Name}).';
report.ElapsedTime_s = NaN(numel(cases),3);
report.Length_units = NaN(numel(cases),3);
report.Results = cell(numel(cases),3);
report.Validation = cell(numel(cases),3);
report.Errors = strings(numel(cases),3);
report.PhysicalRequests = cell(numel(cases),1);
optionNames = {'GoalTimeMode','SampleTime_s','ConstraintTolerance', ...
    'CollisionClearanceTolerance_units','ArrivalTimeTolerance_s','WrapX','WrapY'};
for caseIndex = 1:numel(cases)
    request = cases(caseIndex).Inputs;
    options = cases(caseIndex).Options;
    options = rmfield(options,setdiff(fieldnames(options),optionNames));
    if isfield(request.goalState,'targetTime_s') && ~isempty(request.goalState.targetTime_s)
        request.goalState.targetMotion = struct('time_s',request.goalState.targetTime_s, ...
            'position_units',request.goalState.targetPosition_units, ...
            'InterpolationMethod',request.goalState.InterpolationMethod);
    end
    request.goalState = rmfield(request.goalState,intersect(fieldnames(request.goalState), ...
        {'targetTime_s','targetPosition_units','InterpolationMethod'}));
    report.PhysicalRequests{caseIndex} = request;
    try
        planner(request.obstacles,request.initialState,request.goalState,request.limits,options);
    catch warmupError
        fprintf('REFERENCE_WARMUP name=%s error=%s\n',cases(caseIndex).Name,warmupError.identifier);
    end

    %% Section 2: Measure And Independently Validate Every Returned Motion
    for repeatIndex = 1:3
        timer = tic;
        try
            [result,~] = planner(request.obstacles,request.initialState,request.goalState,request.limits,options);
            report.ElapsedTime_s(caseIndex,repeatIndex) = toc(timer);
            % The reference validator accepts a result record. Replace its
            % prepared geometry with a fresh build of the frozen source data.
            independent = result;
            independent.PreparedObstacles = obstacleAvoidance.obstacles.prepareObstacles(request.obstacles);
            check = obstacleAvoidance.validateTrajectory(independent);
            report.Results{caseIndex,repeatIndex} = result;
            report.Validation{caseIndex,repeatIndex} = check;
            polyline_units = NaN;
            if result.Success
                polyline_units = sum(vecnorm(diff(result.Route_units),2,2));
                length_units = 0;
                for span = 1:result.Polynomial.SegmentCount
                    c = reshape(result.Polynomial.positionPower_units(span,:,:),2,[]);
                    d = c(:,2:end).*(1:size(c,2)-1);
                    speed = @(tau) hypot(polyval(fliplr(d(1,:)),tau),polyval(fliplr(d(2,:)),tau));
                    length_units = length_units + integral(speed,0,1,'AbsTol',1e-10,'RelTol',1e-10);
                end
                report.Length_units(caseIndex,repeatIndex) = length_units;
            end
            fprintf('REFERENCE name=%s repeat=%d success=%d valid=%d duration_s=%.12g polyline_units=%.12g arc_units=%.12g collision=%d kinematic=%d certificate=%d reason=%s runtime_s=%.6f\n', ...
                cases(caseIndex).Name,repeatIndex,result.Success,check.Passed,result.TrajectoryDuration_s,polyline_units, ...
                report.Length_units(caseIndex,repeatIndex),check.CollisionFree, ...
                check.VelocityWithinLimits && check.AccelerationWithinLimits && check.JerkWithinLimits, ...
                check.PlaneCertificateValid,result.TerminationReason,report.ElapsedTime_s(caseIndex,repeatIndex));
        catch runError
            report.ElapsedTime_s(caseIndex,repeatIndex) = toc(timer);
            report.Errors(caseIndex,repeatIndex) = string(runError.identifier)+": "+string(runError.message);
            fprintf('REFERENCE name=%s repeat=%d error=%s runtime_s=%.6f\n', ...
                cases(caseIndex).Name,repeatIndex,runError.identifier,report.ElapsedTime_s(caseIndex,repeatIndex));
        end
    end
end

%% Section 3: Preserve Failures Alongside Successful Measurements
report.MedianTime_s = median(report.ElapsedTime_s,2);
save(outputPath,'report','-v7.3');
end
