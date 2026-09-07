function buildShortcutCases(root)
addpath(fullfile(root,'on'),fullfile(root,'on','trajectory'),fullfile(root,'on','examples'));
cases    = cell(24,1);
families = ["static", "moving", "diagonal", "weave"];
% Process each family included in this benchmark measurement.
for familyIndex=1:4
    % Process each member included in this benchmark measurement.
    for member=1:5
        index        =(familyIndex-1)*5+member;
        seed         =6100+index; rng(seed,'twister');
        family       =families(familyIndex);
        initialState = struct();
        initialState.time_s       = 0;
        initialState.position_units = [-8 0];
        goalState = struct();
        goalState.time_s       = 24;
        goalState.position_units = [8 0];
        limits = struct();
        limits.maxVelocity_units_s      = [3 3];
        limits.maxAcceleration_units_s2 = [1.5 1.5];
        limits.maxJerk_units_s3         = [4 4];
        limits.xInterval_units    = [-10 10];
        limits.yInterval_units  = [-7 7];
        if family=="diagonal"
            initialState.position_units=[-6 -6]; goalState.position_units=[6 6];
        end
        count=1;
        if family=="static" || family=="moving", count=1+mod(member,3); end
        if family=="weave", count=3; end
        items=cell(count,1);
        % Process each obstacle included in this benchmark measurement.
        for obstacleIndex=1:count
            halfWidth =0.3+0.6*rand; halfHeight=0.7+1.5*rand;
            center    =[-3+6*rand, -0.6+1.2*rand];
            if family=="diagonal", center=[0 0]+0.4*(rand(1,2)-0.5); end
            if family=="weave"
                center     =[-4+4*(obstacleIndex-1),(-1)^obstacleIndex*2.8];
                halfHeight =3.8+0.3*rand;
            end
            base  =[-1 -1;1 -1;1 1;-1 1].*[halfWidth halfHeight];
            times =[0;24];
            if family=="moving", times=(0:6:24).'; end
            x     =cell(numel(times),1); y=x;
            travel =2+2*rand; rotation=pi*(rand-0.5); phase=2*pi*rand;
            % Process each k included in this benchmark measurement.
            for k=1:numel(times)
                position=center; angle=0;
                if family=="moving"
                    position =center+[0.5*sin(2*pi*times(k)/24+phase), travel*(times(k)/24-0.5)];
                    angle    =rotation*times(k)/24;
                end
                vertices=base*[cos(angle) sin(angle);-sin(angle) cos(angle)]+position;
                x{k}=vertices(:,1); y{k}=vertices(:,2);
            end
            items{obstacleIndex}=obstacleAvoidance.obstacles.createObstacle("generated "+obstacleIndex,times,x,y,0.12);
        end
        request=struct('obstacles',obstacleAvoidance.obstacles.combineObstacles(items), ...
            'initialState',initialState,'goalState',goalState,'limits',limits, ...
            'options',obstacleAvoidance.planTrajectory());
        cases{index}=struct('name',family+"_"+seed,'family',family,'seed',seed,'request',request);
    end
end
request=cases{1}.request; request.obstacles=[];
cases{21}=struct('name',"empty_control",'family',"control",'seed',0,'request',request);
wall=[-0.4 -8;0.4 -8;0.4 8;-0.4 8];
request.obstacles=obstacleAvoidance.obstacles.createObstacle('sealed wall',[0;24],wall(:,1),wall(:,2),0.12);
cases{22}=struct('name',"sealed_control",'family',"control",'seed',0,'request',request);
cases{23}=struct('name',"exampleMovingRotatingObstacleField",'family',"maintained",'seed',0, ...
    'request',capture_exampleMovingRotatingObstacleField());
cases{24}=struct('name',"exampleMovingBarrierWait",'family',"maintained",'seed',0, ...
    'request',capture_exampleMovingBarrierWait());
save(fullfile(root,'cases.mat'),'cases');
end
