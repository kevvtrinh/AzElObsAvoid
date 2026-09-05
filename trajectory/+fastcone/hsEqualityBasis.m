function [base,basis,independent,recognized] = hsEqualityBasis(E,d,controlCount,segments,degree)
% Exact local C3 Bezier continuation; no numerical null-space factorization.
% Right controls [Q0 Q1 Q2 Q3]' equal R*[P(n-3) P(n-2) P(n-1) Pn]'.
% Rest endpoint triples are fixed; all untouched controls remain free.
persistent cache
base=[]; basis=[]; independent=[]; recognized=false;
if isempty(cache), cache={}; end
for k=1:numel(cache)
    item=cache{k};
    if isequal(E,item.E)
        base=item.BaseMap*d; basis=item.Basis; independent=item.Independent;
        if norm(E*base-d,inf)==0, recognized=true; return; end
    end
end
if ~ismember(degree,[7 16]) || segments<1 || segments~=fix(segments) || ...
        controlCount~=2*segments*(degree+1), return; end
nv=size(E,2); nr=12+8*(segments-1); timeRow=size(E,1)==nr+1;
if nv<controlCount || (~timeRow && size(E,1)~=nr), return; end
expected=sparse(size(E,1),nv); row=0;
for axis=1:2
    row=row+1; expected(row,index(1,0,axis))=1;
    row=row+1; expected(row,index(segments,degree,axis))=1;
    for order=1:2
        row=row+1; expected(row,index(1,[order 0],axis))=[1 -1];
        row=row+1; expected(row,index(segments,[degree-order degree],axis))=[1 -1];
    end
end
coefficients={1,[-1 1],[1 -2 1],[-1 3 -3 1]};
for segment=1:segments-1
    for order=0:3
        for axis=1:2
            row=row+1;
            expected(row,index(segment,degree-order+(0:order),axis))=coefficients{order+1};
            expected(row,index(segment+1,0:order,axis))=-coefficients{order+1};
        end
    end
end
if timeRow
    if nv<=controlCount, return; end
    expected(end,controlCount+1)=1;
end
if ~isequal(E,expected), return; end
zeroRows=true(size(d)); zeroRows([1 2 7 8])=false;
if timeRow, zeroRows(end)=false; end
if any(d(zeroRows)~=0), return; end
baseMap=sparse(nv,size(E,1)); dependent=false(nv,1);
for axis=1:2
    first=index(1,0:2,axis); last=index(segments,degree-(0:2),axis);
    baseMap(first,6*(axis-1)+1)=1; baseMap(last,6*(axis-1)+2)=1;
    dependent([first last])=true;
end
if timeRow
    dependent(controlCount+1)=true; baseMap(controlCount+1,end)=1;
end
for segment=1:segments-1
    for axis=1:2
        dependent(index(segment+1,0:3,axis))=true;
    end
end
independent=find(~dependent);
columnOf=zeros(nv,1); columnOf(independent)=1:numel(independent);
basis=sparse(independent,1:numel(independent),1,nv,numel(independent));
R=[0 0 0 1;0 0 -1 2;0 1 -4 4;-1 6 -12 8];
for segment=1:segments-1
    for axis=1:2
        left=index(segment,degree-3:degree,axis); right=index(segment+1,0:3,axis);
        basis(right,columnOf(left))=R;
    end
end
base=baseMap*d;
recognized=nnz(E*basis)==0 && norm(E*base-d,inf)==0 && ...
    size(basis,2)==nv-size(E,1);
if recognized
    item=struct('E',E,'BaseMap',baseMap,'Basis',basis,'Independent',independent);
    cache=[{item},cache]; if numel(cache)>16, cache=cache(1:16); end
end
    function columns=index(segment,control,axis)
        columns=((segment-1)*(degree+1)+control)*2+axis;
    end
end
