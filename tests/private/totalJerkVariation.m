function variation=totalJerkVariation(result)
% Integrate absolute snap, including joins, and normalize by each jerk limit.
variation=zeros(1,2);
for axis=1:2
    previous=[];
    for span=1:result.Polynomial.SegmentCount
        power=reshape(result.Polynomial.jerkPower_units_s3(span,axis,:),1,3);
        turns=roots([2*power(3),power(2)]);
        turns=real(turns(abs(imag(turns))<1e-12 & real(turns)>0 & real(turns)<1));
        values=polyval(fliplr(power),sort([0;turns;1]));
        if ~isempty(previous), variation(axis)=variation(axis)+abs(values(1)-previous); end
        variation(axis)=variation(axis)+sum(abs(diff(values)));
        previous=values(end);
    end
end
variation=variation./result.Limits.maxJerk_units_s3;
end
