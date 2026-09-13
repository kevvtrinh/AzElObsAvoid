function [rows,bounds]=createSelectedPlaneRows(planes,pairMask,degree, ...
        variableCount,slackColumnByPair,trajectoryReserve_units)
%% Section 0: Header & Readme
% SYNTAX: [rows,bounds] = bmtpEngine.createSelectedPlaneRows(...)
% PURPOSE: Materialize exact Bernstein separating rows for selected pairs.
% INPUTS: Plane array and mask, degree, decision size, optional per-pair
%   slack columns (zero means hard), and trajectory-side reserve.
% OUTPUTS: Sparse inequality rows and matching upper bounds.
% UNITS: Coordinate units.

%% Section 1: Assemble Selected Pairs In Segment-Major Order
rowCount=nnz(pairMask)*(degree+2);
bounds=zeros(rowCount,1);
maximumEntryCount=rowCount*(2*(degree+1)+1);
rowIndex=zeros(maximumEntryCount,1);
columnIndex=zeros(maximumEntryCount,1);
values=zeros(maximumEntryCount,1);
nextRow=0;
nextEntry=0;
for segmentIndex=1:size(pairMask,1)
    for regionIndex=reshape(find(pairMask(segmentIndex,:)),1,[])
        [pairRows,offset_units]=bmtpEngine.createPlaneRows( ...
            planes(segmentIndex,regionIndex),degree,variableCount,segmentIndex);
        targets=nextRow+(1:degree+2);
        [pairRowIndex,pairColumnIndex,pairValues]=find(pairRows);
        entries=nextEntry+(1:numel(pairValues));
        rowIndex(entries)=nextRow+pairRowIndex;
        columnIndex(entries)=pairColumnIndex;
        values(entries)=pairValues;
        nextEntry=nextEntry+numel(pairValues);
        slackColumn=slackColumnByPair(segmentIndex,regionIndex);
        if slackColumn>0
            entries=nextEntry+(1:degree+2);
            rowIndex(entries)=targets;
            columnIndex(entries)=slackColumn;
            values(entries)=-1;
            nextEntry=nextEntry+degree+2;
        end
        bounds(targets)=-trajectoryReserve_units-offset_units;
        nextRow=targets(end);
    end
end
rows=sparse(rowIndex(1:nextEntry),columnIndex(1:nextEntry), ...
    values(1:nextEntry),rowCount,variableCount);
end
