function [x,value,flag,output]=reference(varargin)
%% Section 0: Header & Readme
% SYNTAX: [x,value,flag,output] = fastcone.reference(f,cones,A,b,E,d,lb,ub,options)
% PURPOSE: Run explicitly requested coneprog recovery with solver diagnostics.
% INPUTS: The same nine conic arguments as fastcone.solve.
% OUTPUTS: Unchanged coneprog result and flag, with reported recovery and time.
% UNITS: Original input coordinates; elapsed time in seconds.

%% Section 1: Preserve Reference Behavior And Account For Recovery
timer=tic; [x,value,flag,output]=coneprog(varargin{:});
output.Method='coneprog recovery'; output.FallbackUsed=true;
output.NativeAvailable=false; output.NativeAccepted=false;
output.MatlabAccepted=false; output.CertifiedInfeasible=false;
output.Prototype=struct('message','Explicit reference plane selection requested.');
output.PrototypeTime_s=0; output.TotalTime_s=toc(timer);
end
