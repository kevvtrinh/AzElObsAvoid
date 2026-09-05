function [lowerBound,certificate] = dualBoxBound(cost,G,h,z,lb,ub,linearCount,coneSizes)
% Produce a weak-dual objective bound even when stationarity is not exact.
% For G*u+s=h, s in K, any z in K gives c'*u >= -h'*z +
% min_{lb<=u<=ub}(c+G'*z)'*u. Infinite endpoints are handled explicitly.

z(1:linearCount)=max(0,z(1:linearCount));
start=linearCount+1;
if all(coneSizes==3) && ~isempty(coneSizes)
    blocks=reshape(z(start:end),3,[]); radius=sqrt(sum(blocks(2:3,:).^2,1));
    negative=blocks(1,:)<=-radius;
    boundary=blocks(1,:)<radius & ~negative;
    if any(boundary)
        head=.5*(radius(boundary)+blocks(1,boundary));
        tail=blocks(2:3,boundary).*(head./radius(boundary));
        blocks(:,boundary)=[max(head,sqrt(sum(tail.^2,1)))+4*eps(max(1,head));tail];
    end
    blocks(:,negative)=0; z(start:end)=blocks(:);
else
for k=1:numel(coneSizes)
    indices=start+(0:coneSizes(k)-1); block=z(indices); radius=norm(block(2:end));
    if block(1)<radius
        if block(1)<=-radius
            block(:)=0;
        else
            head=.5*(radius+block(1)); tail=block(2:end)*(head/radius);
            block=[max(head,norm(tail))+4*eps(max(1,head));tail];
        end
        z(indices)=block;
    end
    start=start+coneSizes(k);
end
end
product=G'*z; scale=1;
upperInfinite=isinf(ub) & ub>0 & product<0;
if any(upperInfinite)
    scale=min(scale,min(cost(upperInfinite)./(-product(upperInfinite))));
end
lowerInfinite=isinf(lb) & lb<0 & product>0;
if any(lowerInfinite)
    scale=min(scale,min(-cost(lowerInfinite)./product(lowerInfinite)));
end
scale=max(0,min(1,scale));
if scale>0 && (scale<1 || any(upperInfinite | lowerInfinite))
    scale=scale*(1-8*eps);
end
residual=cost+scale*product;
endpoints=lb; endpoints(residual<0)=ub(residual<0);
nonzero=residual~=0;
if any(~isfinite(endpoints(nonzero)))
    lowerBound=-inf;
else
    lowerBound=-scale*(h'*z)+sum(residual(nonzero).*endpoints(nonzero),'all');
end
certificate=struct('DualScale',scale,'StationarityInfinityNorm',norm(residual,inf), ...
    'Finite',isfinite(lowerBound));
end
