function scenario = createRandomAzimuthScenario(caseIndex, includeStaticObstacle, randomSeed)
%% Section 0: Header & Readme
% SYNTAX: scenario = createRandomAzimuthScenario(caseIndex, includeStaticObstacle, randomSeed)
% PURPOSE: Reproduce an angular slew crossing a translating rectangle, optionally
%   with an additional static rectangle. Every slew spans at least 80 deg in azimuth.
% INPUTS: Positive integer caseIndex (default 1), logical includeStaticObstacle
%   (default false), and integer randomSeed in [0,2^32-1] (default 831907).
% OUTPUTS: Planner arguments and source geometry metadata in a scalar struct.
%   Paired cases differ only by the presence of the static obstacle. Randomness
%   uses a local stream and does not change the caller's global random state.
% UNITS: Coordinates are azimuth/elevation in degrees; time is seconds.

%% Section 1: Validate And Draw Reproducible Physical Inputs
if nargin < 1, caseIndex = 1; end
if nargin < 2, includeStaticObstacle = false; end
if nargin < 3, randomSeed = 831907; end
validateattributes(caseIndex, {'numeric'}, {'scalar','integer','positive','finite'});
validateattributes(includeStaticObstacle, {'logical'}, {'scalar'});
validateattributes(randomSeed, {'numeric'}, {'scalar','integer','>=',0,'<=',2^32-1});
stream = RandStream('mt19937ar','Seed',randomSeed);
% Generate by column-independent case blocks so requesting a larger index does
% not change the preceding cases or the paired obstacle toggle.
draws = reshape(rand(stream,16*caseIndex,1),16,caseIndex).';
u = draws(end,:);
azimuthSpan_deg = 80 + 60*u(1);
direction = 2*(u(2)>=0.5)-1;
center_deg = [-25+50*u(3), -12+24*u(4)];
displacement_deg = [direction*azimuthSpan_deg, -65+130*u(5)];
start_deg = center_deg-displacement_deg/2;
goal_deg = center_deg+displacement_deg/2;
duration_s = 180;
along = displacement_deg/norm(displacement_deg);
across = [-along(2),along(1)];
movingFraction = 0.38+0.24*u(6);
movingCenter_deg = start_deg+movingFraction*displacement_deg;
halfSize_deg = [4+9*u(7), 7+16*u(8)];
angle_rad = pi*u(9);
rotation = [cos(angle_rad),-sin(angle_rad);sin(angle_rad),cos(angle_rad)];
movingVertices_deg = ([-1,-1;1,-1;1,1;-1,1].*halfSize_deg)*rotation.';
velocity_deg_s = (2*(u(10)>=0.5)-1)*(0.035+0.14*u(11))*across + ...
    (-0.04+0.08*u(12))*along;
% Constant translation is represented exactly by corresponding vertices.
time_s = [0;duration_s];
x_deg = cell(2,1); y_deg = cell(2,1);
for k = 1:2
    boundary_deg = movingVertices_deg+movingCenter_deg+time_s(k)*velocity_deg_s;
    x_deg{k} = boundary_deg(:,1); y_deg{k} = boundary_deg(:,2);
end
movingObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "Translating rectangle",time_s,x_deg,y_deg,0.2);
staticFraction = 0.18+0.10*u(13);
if u(14)>0.5, staticFraction = 1-staticFraction; end
staticCenter_deg = start_deg+staticFraction*displacement_deg+(-8+16*u(15))*across;
staticVertices_deg = staticCenter_deg+[-1,-1;1,-1;1,1;-1,1].*[3+4*u(16),6+4*u(13)];
staticObstacle = obstacleAvoidance.obstacles.createObstacle( ...
    "Static rectangle",time_s,staticVertices_deg(:,1),staticVertices_deg(:,2),0.2);
obstacles = movingObstacle;
if includeStaticObstacle
    obstacles = obstacleAvoidance.obstacles.combineObstacles({movingObstacle;staticObstacle});
end

%% Section 2: Return Public Planner Inputs And Reproduction Metadata
scenario = struct('Obstacles',obstacles, ...
    'InitialState',struct('time_s',0,'position_units',start_deg), ...
    'GoalState',struct('time_s',duration_s,'position_units',goal_deg), ...
    'Limits',struct('xInterval_units',[-180,180],'yInterval_units',[-90,90], ...
        'maxVelocity_units_s',[2,2],'maxAcceleration_units_s2',[0.75,0.75], ...
        'maxJerk_units_s3',[2,2]), ...
    'Options',struct('GoalTimeMode','fixedArrival','SampleTime_s',0.1), ...
    'CaseIndex',caseIndex,'IncludeStaticObstacle',includeStaticObstacle, ...
    'RandomSeed',randomSeed,'AzimuthSpan_deg',azimuthSpan_deg, ...
    'EndpointDistance_deg',norm(displacement_deg), ...
    'MovingCenterAtStart_deg',movingCenter_deg,'MovingVelocity_deg_s',velocity_deg_s, ...
    'StaticVertices_deg',staticVertices_deg);
end
