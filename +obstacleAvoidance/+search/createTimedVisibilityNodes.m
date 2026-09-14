function nodes = createTimedVisibilityNodes(shape, start_units, goal_units, limits, candidateOffset_units, workBudget)
%% Section 0: Header & Readme
% SYNTAX: nodes = obstacleAvoidance.search.createTimedVisibilityNodes(
%   shape,start_units,goal_units,limits,candidateOffset_units,workBudget)
% PURPOSE: Create one deterministic staging-node set for the timed route
%   proposal. This helper does not search, retry, or certify a motion.
% INPUTS: Sampled swept proposal shape, endpoints, workspace, one physical
%   steering offset, and a finite pair-work budget.
% OUTPUTS: Raw and retained proposal nodes with explicit discard evidence.
% UNITS: Positions and offsets are coordinate units.

%% Section 1: Offset And Bound The Proposal Boundary
candidateShape = shape;
if ~isempty(shape.Vertices)
    candidateShape = polybuffer(shape,candidateOffset_units,"JointType","miter");
end
rawNodes_units = candidateShape.Vertices;
isInsideWorkspace = rawNodes_units(:,1) >= limits.xInterval_units(1) & ...
    rawNodes_units(:,1) <= limits.xInterval_units(2) & ...
    rawNodes_units(:,2) >= limits.yInterval_units(1) & ...
    rawNodes_units(:,2) <= limits.yInterval_units(2);
discardReasons = repmat("",size(rawNodes_units,1),1);
discardReasons(~isInsideWorkspace) = "outsideWorkspace";
candidateNodes_units = unique(rawNodes_units(isInsideWorkspace,:),"rows","stable");

%% Section 2: Retain A Deterministic Global Boundary Cover
% The timed graph checks every retained node pair at every physical layer.
% Bound that proposal-only quadratic work explicitly; final acceptance still
% comes exclusively from BMTP and the independent validator.
[edgeStart_units,~] = obstacleAvoidance.geometry.boundaryToEdges(shape,1e-12);
candidateLimit = max(2,floor(sqrt(2*workBudget/max(1,size(edgeStart_units,1))))-2);
if size(candidateNodes_units,1)>candidateLimit
    candidateNodes_units = selectBoundaryCover(candidateNodes_units, ...
        start_units,goal_units,candidateLimit);
end
positions_units = unique([start_units;goal_units;candidateNodes_units],"rows","stable");
nodes = struct("CandidateShape",candidateShape, ...
    "RawNodes_units",rawNodes_units, ...
    "RawNodeDiscardReasons",discardReasons, ...
    "RetainedCandidateNodes_units",candidateNodes_units, ...
    "Positions_units",positions_units, ...
    "CandidateLimit",candidateLimit, ...
    "CandidateOffset_units",candidateOffset_units);
end

function selected_units = selectBoundaryCover(candidates_units,start_units,goal_units,count)
    % Preserve endpoint access, global supports, and ring-order coverage.
    endpointCount=min(4,floor(count/6));
    directionCount=min(16,floor(count/3));
    uniformCount=count-directionCount-2*endpointCount;
    selected=unique(round(linspace(1,size(candidates_units,1),uniformCount))).';
    if directionCount>0
        angle_rad=(0:directionCount-1).'*(2*pi/directionCount);
        direction=[cos(angle_rad),sin(angle_rad)];
        [~,support]=max(candidates_units*direction.',[],1);
        selected=[selected;support(:)];
    end
    for reference_units=[start_units;goal_units].'
        [~,order]=sort(vecnorm(candidates_units-reference_units.',2,2));
        selected=[selected;order(1:endpointCount)]; %#ok<AGROW>
    end
    selected=unique(selected,"stable");
    selected_units=candidates_units(selected(1:min(count,numel(selected))),:);
end
