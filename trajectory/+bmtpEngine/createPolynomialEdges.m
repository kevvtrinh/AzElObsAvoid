function edges=createPolynomialEdges(polynomial,controlPoint_units)
%% Section 0: Header & Readme
% SYNTAX: edges = bmtpEngine.createPolynomialEdges(polynomial)
%         edges = bmtpEngine.createPolynomialEdges(polynomial,controls)
% PURPOSE: Expose a complete certified polynomial as lossless physical
%   edge records for future graph-to-BMTP handoff without reconstructing a
%   clock or endpoint derivatives from route positions.
% INPUTS: Canonical power polynomial and optional matching Bernstein controls.
% OUTPUTS: One edge per polynomial span, including absolute time, controls,
%   normalized powers, and authoritative physical p/v/a/j endpoint states.
% UNITS: Position is coordinate units; time is seconds; derivatives use
%   units/s, units/s^2, and units/s^3.

%% Section 1: Validate The Canonical Polynomial
requiredFields={'Degree','SegmentCount','SegmentStartTime_s', ...
    'SegmentDuration_s','positionPower_units','velocityPower_units_s', ...
    'accelerationPower_units_s2','jerkPower_units_s3'};
if ~isstruct(polynomial) || ~isscalar(polynomial) || ...
        ~all(isfield(polynomial,requiredFields))
    error('bmtpEngine:InvalidPolynomialEdges', ...
        'A complete scalar canonical polynomial is required.');
end
segmentCount=polynomial.SegmentCount;
degree=polynomial.Degree;
if nargin<2 || isempty(controlPoint_units)
    controlPoint_units=powerToBernstein(polynomial.positionPower_units);
end
expectedControlSize=[segmentCount,degree+1,2];
if ~isequal(size(controlPoint_units),expectedControlSize) || ...
        any(~isfinite(controlPoint_units),'all')
    error('bmtpEngine:InvalidPolynomialEdgeControls', ...
        'Bernstein controls must match the canonical polynomial size.');
end

%% Section 2: Materialize Lossless Physical Edge State
template=struct('Index',0,'StartTime_s',0,'EndTime_s',0, ...
    'SegmentDuration_s',0,'ControlPoint_units',zeros(0,2), ...
    'PositionPower_units',zeros(2,0),'StartJet',zeros(4,2), ...
    'EndJet',zeros(4,2),'CertificateStatus',"unresolved");
edges=repmat(template,segmentCount,1);
powerArrays={polynomial.positionPower_units, ...
    polynomial.velocityPower_units_s, ...
    polynomial.accelerationPower_units_s2, ...
    polynomial.jerkPower_units_s3};
for edgeIndex=1:segmentCount
    startJet=zeros(4,2);
    endJet=zeros(4,2);
    for derivativeOrder=0:3
        coefficients=squeeze(powerArrays{derivativeOrder+1}(edgeIndex,:,:));
        startJet(derivativeOrder+1,:)=coefficients(:,1).';
        endJet(derivativeOrder+1,:)=sum(coefficients,2).';
    end
    edges(edgeIndex).Index=edgeIndex;
    edges(edgeIndex).StartTime_s=polynomial.SegmentStartTime_s(edgeIndex);
    edges(edgeIndex).SegmentDuration_s= ...
        polynomial.SegmentDuration_s(edgeIndex);
    edges(edgeIndex).EndTime_s=edges(edgeIndex).StartTime_s+ ...
        edges(edgeIndex).SegmentDuration_s;
    edges(edgeIndex).ControlPoint_units= ...
        squeeze(controlPoint_units(edgeIndex,:,:));
    edges(edgeIndex).PositionPower_units= ...
        squeeze(polynomial.positionPower_units(edgeIndex,:,:));
    edges(edgeIndex).StartJet=startJet;
    edges(edgeIndex).EndJet=endJet;
end
end

function controlPoint_units=powerToBernstein(positionPower_units)
    degree=size(positionPower_units,3)-1;
    transform=zeros(degree+1);
    for bernsteinIndex=0:degree
        for powerIndex=0:bernsteinIndex
            transform(bernsteinIndex+1,powerIndex+1)= ...
                nchoosek(bernsteinIndex,powerIndex)/ ...
                nchoosek(degree,powerIndex);
        end
    end
    powerPages=permute(positionPower_units,[3,1,2]);
    controlPoint_units=permute(pagemtimes(transform,powerPages),[2,1,3]);
end
