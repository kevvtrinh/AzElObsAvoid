function tests=testFastconeEqualityBasis
%% Section 0: Header & Readme
% SYNTAX: results=runtests('tests/testFastconeEqualityBasis.m')
% PURPOSE: Verify exact endpoint elimination and independent C3 derivatives.
% INPUTS: None; exercises degree 7/16 and one/three/eight segments.
% OUTPUTS: Deterministic MATLAB function tests without external fixtures.
% UNITS: Position coordinates and normalized segment time.
tests=functiontests(localfunctions);
end
function setupOnce(~)
% Resolve the production package from this checkout.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(root,fullfile(root,'trajectory'));
end
function testBoundaryStatesAndC3DerivativeContinuity(testCase)
% Construct boundary equations independently and check polynomial derivatives.
rng(881,'twister');
for degree=[7 16]
    for segments=[1 3 8]
        controls=2*segments*(degree+1); columns=controls+4;
        rows=13+8*(segments-1); E=sparse(rows,columns); d=zeros(rows,1);
        index=@(segment,control,axis) ((segment-1)*(degree+1)+control)*2+axis;
        row=0;
        for axis=1:2
            row=row+1; E(row,index(1,0,axis))=1; d(row)=.123+axis;
            row=row+1; E(row,index(segments,degree,axis))=1; d(row)=3.456-axis;
            for order=1:2
                row=row+1; E(row,index(1,[order 0],axis))=[1 -1];
                row=row+1; E(row,index(segments,[degree-order degree],axis))=[1 -1];
            end
        end
        for segment=1:segments-1
            for order=0:3
                coefficients=1;
                if order>0, coefficients=diff(eye(order+1),order,1); end
                for axis=1:2
                    row=row+1;
                    E(row,index(segment,degree-order:degree,axis))=coefficients;
                    E(row,index(segment+1,0:order,axis))=-coefficients;
                end
            end
        end
        E(end,controls+1)=1; d(end)=1;
        [base,Z,independent,recognized]=fastcone.hsEqualityBasis(E,d,controls,segments,degree);
        verifyTrue(testCase,recognized);
        verifyEqual(testCase,E*base,d,'AbsTol',1e-12); verifyEqual(testCase,nnz(E*Z),0);
        verifyEqual(testCase,size(Z,2),columns-rows);
        verifyEqual(testCase,full(Z(independent,:)),eye(numel(independent)));
        point=base+Z*randn(size(Z,2),1);
        for segment=1:segments-1
            for axis=1:2
                left=point(index(segment,0:degree,axis));
                right=point(index(segment+1,0:degree,axis));
                for order=0:3
                    leftDerivative=left; rightDerivative=right;
                    if order>0
                        leftDerivative=diff(left,order); rightDerivative=diff(right,order);
                    end
                    verifyEqual(testCase,leftDerivative(end),rightDerivative(1),'AbsTol',1e-12);
                end
            end
        end
        d(3)=.1;
        [~,~,~,recognized]=fastcone.hsEqualityBasis(E,d,controls,segments,degree);
        verifyFalse(testCase,recognized);
    end
end
end
