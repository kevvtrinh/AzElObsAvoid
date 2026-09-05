function [x,fval,exitflag,output] = solve(f,cones,A,b,E,d,lb,ub,options)
%% Section 0: Header & Readme
% SYNTAX: [x,fval,exitflag,output] = fastcone.solve(f,cones,A,b,E,d,lb,ub,options)
% PURPOSE: Certified analytical cone blocks with reported coneprog recovery.
% INPUTS: The identical nine arguments from HS's conic call sites.
% OUTPUTS: Standard solver outputs plus Method, FallbackUsed and TotalTime_s.
% UNITS: Unchanged from HS. Used by both BMTP conic call sites.

%% Section 1: Try A Certified Analytical Plane Or The Reduced Solver
timer = tic; args = {f,cones,A,b,E,d,lb,ub,options};
nativeAvailable=exist(fullfile(fileparts(mfilename('fullpath')),['core.' mexext]),'file')==3;
planeProblem = numel(f)==7 && isequal(f(:),[zeros(6,1);1]) && ...
    isempty(E) && isempty(d) && isempty(lb) && isempty(ub) && numel(cones)==2 && ...
    size(A,2)==7 && size(A,1)==numel(b);
if planeProblem
    for k = 1:2
        expected = zeros(2,7); expected(:,2*k-1:2*k)=eye(2);
        planeProblem = planeProblem && isequal(full(cones(k).A),expected) && ...
            ~any(cones(k).b) && ~any(cones(k).d) && cones(k).gamma==-1;
    end
end
if planeProblem
    first=A(:,5)==-1 & all(A(:,[3 4 6 7])==0,2);
    second=A(:,6)==-1 & all(A(:,[1 2 5 7])==0,2);
    products=A(:,7)==-1 & all(A(:,5:6)>=0,2);
    planeProblem=any(first) && any(second) && any(products) && ...
        all(first|second|products) && isequal(A(first,1:2),A(second,3:4)) && ...
        all(b(first|second)==b(find(first,1))) && all(b(products)==0) && ...
        all(sum(A(products,5:6),2)==1);
end
accepted = false; prototype = struct();
try
    if planeProblem
        [x,fval,accepted,prototype] = fastcone.separatingLine(A,b,options.OptimalityTolerance);
        method = 'analytical single contact';
        selectionValid=prototype.Unique;
        if accepted && ~selectionValid
            [x,selectionValid,centerIterations]=fastcone.centerEndpointPlane(x,A,b,prototype.ContactRow);
            prototype.CenterIterations=centerIterations;
            prototype.CenterConverged=selectionValid;
            method='analytical contact with Newton center';
        end
        accepted = accepted && selectionValid && ...
            fastcone.residual(x,args{:})<=options.ConstraintTolerance;
    else
        [x,fval,flag,prototype] = fastcone.solveNative(args{:});
        accepted = flag==1;
        method = 'prepared analytical cone blocks';
    end
catch exception
    prototype = struct('message',exception.message);
    method = 'prototype failure';
end
prototypeTime = toc(timer);
if accepted
    exitflag = 1;
    output = struct('message','Experimental solver acceptance checks passed.', ...
        'Method',method,'FallbackUsed',false,'Prototype',prototype, ...
        'NativeAvailable',nativeAvailable,'NativeAccepted',~planeProblem, ...
        'PrototypeTime_s',prototypeTime,'TotalTime_s',toc(timer));
    return
end

%% Section 2: Account For Recovery Without Changing The Problem
[x,fval,exitflag,output] = coneprog(args{:});
output.Method = 'coneprog recovery'; output.FallbackUsed = true;
output.NativeAvailable=nativeAvailable; output.NativeAccepted=false;
output.Prototype = prototype; output.PrototypeTime_s = prototypeTime;
output.TotalTime_s = toc(timer);
end
