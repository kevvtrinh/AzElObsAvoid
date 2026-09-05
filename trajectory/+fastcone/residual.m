function value = residual(x,f,cones,A,b,E,d,lb,ub,varargin)
% Maximum absolute primal violation in the original, unscaled problem.
if isempty(x) || any(~isfinite(x)), value = inf; return; end
value = 0;
if ~isempty(A), value = max(value,max(A*x-b)); end
if ~isempty(E), value = max(value,norm(E*x-d,inf)); end
if ~isempty(lb), value = max(value,max(lb(:)-x)); end
if ~isempty(ub), value = max(value,max(x-ub(:))); end
for k = 1:numel(cones)
    c = cones(k);
    value = max(value,norm(c.A*x-c.b)-c.d(:)'*x+c.gamma);
end
end
