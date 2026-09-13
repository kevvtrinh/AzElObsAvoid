function [controlPoint_units, segmentTime_s, exitFlag, output] = solveTimedTrajectoryStep(segmentCount, degree, start_units, goal_units, limits, planes, reserve_units, maximumMotionDuration_s, goalTimeMode, options, minimumMotionDuration_s, segmentRatio)
%% Section 0: Header & Readme
% SYNTAX: [controlPoint_units, segmentTime_s, exitFlag, output] =
%   bmtpEngine.solveTimedTrajectoryStep( segmentCount, degree, start_units, goal_units, limits,
%   planes, reserve_units, maximumMotionDuration_s, goalTimeMode, options)
% PURPOSE: Solve one convex trajectory step for fixed separating lines, timing policy, and
%   derivative limits.
% INPUTS: segmentCount, degree (positive integer scalars) Composite Bezier representation size.
%   start_units, goal_units (1-by-2 numeric rows) Fixed endpoint positions.
%   limits (scalar struct) Workspace, velocity, acceleration, and jerk limits.
%   planes (S-by-R struct array) Fixed active separating-line constraints.
%   reserve_units (nonnegative scalar) Numerical separation reserve.
%   maximumMotionDuration_s (positive scalar) Upper bound or fixed motion duration.
%   goalTimeMode (scalar text) earliestArrival or fixedArrival.
%   options (coneprog options) Numerical solver controls.
%   minimumMotionDuration_s (optional nonnegative scalar) Lower arrival bound.
%   segmentRatio (optional S-by-1 positive vector) Relative physical span durations.
% OUTPUTS: controlPoint_units (S-by-(D+1)-by-2 numeric array) Solved control points, or an empty
%   array on expected solve failure.
%   segmentTime_s (scalar numeric) Common segment time, or NaN on expected solve failure.
%   exitFlag (numeric scalar), output (solver record) Original coneprog status and measured solver
%   time. Finite fixed-clock -7 iterates are proposals requiring final independent certification.
% UNITS: Position is coordinate units and time is seconds.

%% Section 1: Create Decision Bounds And Continuity Rows
if nargin<11, minimumMotionDuration_s=0; end
returnsCommonSegmentTime=nargin<12 || isempty(segmentRatio);
if returnsCommonSegmentTime,segmentRatio=ones(segmentCount,1);end
segmentRatio=double(segmentRatio(:));
validateattributes(segmentRatio,{'numeric'}, ...
    {'real','finite','positive','numel',segmentCount});
validateattributes(minimumMotionDuration_s,{'numeric'}, ...
    {'real','finite','scalar','nonnegative','<=',maximumMotionDuration_s});
controlCount           = segmentCount * (degree + 1) * 2;
powerIndex             = controlCount + (1:4);
travelBoundCount       = (goalTimeMode ~= "earliestArrival") * segmentCount * degree;
travelBoundIndex       = controlCount + 4 + (1:travelBoundCount);
variableCount          = controlCount + 4 + travelBoundCount;
activePlaneCount = nnz([planes.Active]);
planeActiveBySegment=reshape([planes.Active],size(planes));
maximumSegmentTime_s = maximumMotionDuration_s / sum(segmentRatio);
boundaryControls=zeros(segmentCount,degree+1,2);
boundaryControls(1,1:3,:)=repmat(reshape(start_units,1,1,2),1,3,1);
boundaryControls(end,end-2:end,:)=repmat(reshape(goal_units,1,1,2),1,3,1);
[A,Aeq,beq,lb,ub] = bmtpEngine.createTrajectoryConstraints( ...
    segmentCount,degree,boundaryControls,limits,variableCount,0,segmentRatio,[]);
% The clock cones need only relative powers. Scaling the three physical-time
% columns to a unit upper bound avoids conditioning the SOCP with seconds,
% seconds squared, and seconds cubed that differ by several orders.
for derivativeOrder=1:3
    A(:,powerIndex(derivativeOrder+1)) = ...
        A(:,powerIndex(derivativeOrder+1))*maximumSegmentTime_s^derivativeOrder;
end
lb(travelBoundIndex) = 0;

%% Section 2: Add Separating-Line Bounds
baseInequalityCount=4*segmentCount*(3*degree-3);
initialPlanePairs=planeActiveBySegment;
if goalTimeMode=="fixedArrival", initialPlanePairs(:)=false; end
slackColumnByPair=zeros(size(planeActiveBySegment));
[planeRows,planeBounds]=bmtpEngine.createSelectedPlaneRows(planes, ...
    initialPlanePairs,degree,variableCount,slackColumnByPair,reserve_units);
A=[A;planeRows];
b=[zeros(baseInequalityCount,1);planeBounds];

%% Section 3: Create The Objective And Solve
cones = [bmtpEngine.createTimePowerCones(variableCount, powerIndex); ...
    createTravelBoundCones(variableCount, travelBoundIndex, segmentCount, degree)];
f = zeros(variableCount, 1);
if goalTimeMode == "earliestArrival"
    f(powerIndex(4)) = 1;
else
    f(travelBoundIndex) = 1;
end
if goalTimeMode == "fixedArrival"
    lb(powerIndex) = 1;
    ub(powerIndex) = 1;
else
    ub(powerIndex) = 1;
    minimumTimeRatio=minimumMotionDuration_s/maximumMotionDuration_s;
    lb(powerIndex)=[1;minimumTimeRatio; ...
        minimumTimeRatio^2;minimumTimeRatio^3];
end
solverTimer = tic;
retainedPlanePairs=initialPlanePairs;
solveCount=0;
constraintGenerationComplete=goalTimeMode~="fixedArrival";
maximumPlaneConstraintResidual=NaN;
while true
    [x, ~, exitFlag, output] = coneprog(f,cones,A,b,Aeq,beq,lb,ub,options);
    solveCount=solveCount+1;
    if goalTimeMode~="fixedArrival" || ...
            ~bmtpEngine.hasUsableConicIterate(x,exitFlag)
        break
    end
    [violatedPairs,maximumOmittedResidual]= ...
        bmtpEngine.findViolatedPlanePairs(x,planes,planeActiveBySegment, ...
        retainedPlanePairs,degree,slackColumnByPair,reserve_units, ...
        options.ConstraintTolerance);
    if ~any(violatedPairs,'all')
        loadedResidual=-Inf;
        if size(A,1)>baseInequalityCount
            loadedResidual=max(A(baseInequalityCount+1:end,:)*x- ...
                b(baseInequalityCount+1:end));
        end
        maximumPlaneConstraintResidual=max(loadedResidual,maximumOmittedResidual);
        constraintGenerationComplete= ...
            maximumPlaneConstraintResidual<=options.ConstraintTolerance;
        break
    end
    retainedPlanePairs=retainedPlanePairs|violatedPairs;
    [newRows,newBounds]=bmtpEngine.createSelectedPlaneRows(planes, ...
        violatedPairs,degree,variableCount,slackColumnByPair,reserve_units);
    A=[A;newRows]; b=[b;newBounds]; %#ok<AGROW>
end
output.TotalTime_s=toc(solverTimer);
output.OptimizationConverged=exitFlag>0;
output.SolveCount=solveCount;
output.ConstraintGenerationApplied=goalTimeMode=="fixedArrival";
output.ConstraintGenerationRoundCount=max(0,solveCount-1);
output.ConstraintGenerationComplete=constraintGenerationComplete;
output.MaximumPlaneConstraintResidual=maximumPlaneConstraintResidual;
output.OriginalPlaneCount=activePlaneCount;
output.LoadedPlanePairCount=nnz(retainedPlanePairs);
% An optimality stall does not establish physical infeasibility. Every finite
% retained iterate remains only a proposal for independent certification.
if ~bmtpEngine.hasUsableConicIterate(x,exitFlag)
    controlPoint_units = zeros(0, degree + 1, 2);
    segmentTime_s    = NaN;
    return;
end
segmentTime_s = maximumSegmentTime_s*max(x(powerIndex(4)),0)^(1/3);
if ~returnsCommonSegmentTime,segmentTime_s=segmentTime_s*segmentRatio;end
controlPoint_units = permute(reshape(x(1:controlCount), 2, degree + 1, segmentCount), [3 2 1]);
end

%% Section 4: Local Functions
function soc = createTravelBoundCones(variableCount, travelBoundIndex, segmentCount, degree)
    % Bound travel by the sum of Bezier control-edge lengths.
    if isempty(travelBoundIndex)
        soc = repmat(secondordercone(zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0), 0, 1);
        return;
    end
    soc        = repmat(secondordercone(zeros(2, variableCount), zeros(2, 1), zeros(variableCount, 1), 0), numel(travelBoundIndex), 1);
    boundIndex = 0;
    for segmentIndex = 1:segmentCount
        for controlIndex = 0:degree - 1
            boundIndex = boundIndex + 1;
            coneA      = zeros(2, variableCount);
            for axisIndex = 1:2
                firstIndex  = controlIndexOf(segmentIndex, controlIndex, axisIndex, degree);
                secondIndex = controlIndexOf(segmentIndex, controlIndex + 1, axisIndex, degree);
                coneA(axisIndex, [firstIndex secondIndex]) = [-1 1];
            end
            coneC = zeros(variableCount, 1);
            coneC(travelBoundIndex(boundIndex)) = 1;
            soc(boundIndex) = secondordercone(coneA, zeros(2, 1), coneC, 0);
        end
    end
end

function index = controlIndexOf(segmentIndex, controlIndex, axisIndex, degree)
    % Map trajectory controls into the conic decision vector.
    index = ((segmentIndex - 1) * (degree + 1) + controlIndex) * 2 + axisIndex;
end
