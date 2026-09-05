function [x,value,accepted,certificate]=exchangeContacts(A,b,tolerance)
%% Section 0: Header & Readme
% SYNTAX: [x,value,accepted,certificate] = fastcone.exchangeContacts(A,b,tolerance)
% PURPOSE: Exchange contacts until a full-program dual bound certifies a plane.
% INPUTS: Canonical plane inequalities and objective tolerance.
% OUTPUTS: Candidate, objective, acceptance, bound and working-contact counts.
% UNITS: Input coordinates. Exhausting proposals returns unresolved.

%% Section 1: Solve Restricted Contacts And Check Every Original Row
products=find(A(:,7)==-1); obstacles=find(A(:,7)==0); R=A(products,1:6);
V=-A(A(:,5)==-1 & A(:,7)==0,1:2);
[x,value,accepted,certificate]=fastcone.separatingLine(A,b,tolerance);
accepted=accepted && certificate.Unique;
if accepted, return; end
[~,worst]=max(R*x(1:6)); active=unique([certificate.ContactRow,worst]);
for iteration=1:numel(products)
    if numel(active)<2, return; end
    selected=[obstacles;products(active)]; smallA=A(selected,:); smallB=b(selected);
    if numel(active)==2
        [candidate,~,localAccepted,local]=fastcone.twoContacts(smallA,smallB,tolerance);
    else
        triples=nchoosek(1:numel(active),3)';
        [~,vertex0]=min(V*x(1:2)); [~,vertex1]=min(V*x(3:4));
        [candidate,~,localAccepted,local]=fastcone.contactPlanes(smallA,smallB,tolerance,triples,[vertex0;vertex1]);
        if ~localAccepted
            [candidate,~,localAccepted,local]=fastcone.contactPlanes(smallA,smallB,tolerance,triples);
        end
        if ~localAccepted && numel(active)>=4
            [four,~,fourAccepted,fourCertificate]=fastcone.fourContacts(smallA,smallB,tolerance);
            if fourAccepted, candidate=four; local=fourCertificate; localAccepted=true; end
        end
    end
    if isempty(candidate) || ~isfield(local,'LowerBound') || ~isfinite(local.LowerBound), return; end
    [value,worst]=max(R*candidate(1:6)); x=candidate; x(7)=value;
    certificate=local; certificate.Gap=abs(value-local.LowerBound);
    if isfield(local,'ContactRows'), localRows=local.ContactRows;
    else, localRows=local.ContactRow; end
    certificate.ContactRows=active(localRows);
    certificate.ExchangeIterations=iteration; certificate.WorkingContactCount=numel(active);
    accepted=certificate.Gap<=tolerance*(1+abs(value)) && certificate.Unique;
    if accepted || ismember(worst,active), return; end
    % The certified dual uses only these contacts, so this basis preserves
    % the restricted optimum. Every original row remains in the next check.
    if localAccepted, active=active(localRows); active=active(:)'; end
    active=sort([active,worst]);
end
end
