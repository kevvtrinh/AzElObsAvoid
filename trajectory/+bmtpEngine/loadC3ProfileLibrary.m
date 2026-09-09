function library=loadC3ProfileLibrary(source)
%% Section 0: Header & Readme
% SYNTAX: library = bmtpEngine.loadC3ProfileLibrary(source)
% PURPOSE: Load and structurally check reusable C3 profiles before planning.
% INPUTS: Library struct or filename of a MAT file containing only numeric data.
% OUTPUTS: Checked version-one profile library; malformed data raises an error.
% UNITS: Unit endpoint distance and unit full-motion duration.

%% Section 1: Read Explicit Library Data
if ischar(source) || (isstring(source) && isscalar(source))
    data=load(source,'library');
    if ~isfield(data,'library'), error('bmtpEngine:InvalidProfileLibrary','MAT file must contain library.'); end
    source=data.library;
end
valid=isstruct(source) && isscalar(source) && all(isfield(source,{'SchemaVersion','Degree','Entries'}));
if ~valid || ~isequal(source.SchemaVersion,1) || ~isequal(source.Degree,5) || ~iscell(source.Entries)
    error('bmtpEngine:InvalidProfileLibrary','Expected a version-one quintic C3 profile library.');
end
library=source;

%% Section 2: Check Coefficients, Timing, Endpoints And C3 Joins
for index=1:numel(library.Entries)
    entry=library.Entries{index};
    if ~isstruct(entry) || ~isscalar(entry) || ~all(isfield(entry, ...
            {'Route','PositionPower','SegmentTime','PhaseTime','LimitSignature','RestEndpoints'}))
        error('bmtpEngine:InvalidProfileLibrary','Profile %d is missing motion data.',index);
    end
    power=entry.PositionPower; times=entry.SegmentTime;
    valid=isnumeric(power) && isreal(power) && ndims(power)<=3 && size(power,2)==2 && size(power,3)==6 && all(isfinite(power),'all') && ...
        isnumeric(times) && isreal(times) && iscolumn(times) && numel(times)==size(power,1) && ...
        all(isfinite(times) & times>0) && abs(sum(times)-1)<1e-12 && all(diff([0;cumsum(times)])>0) && ...
        isnumeric(entry.Route) && isreal(entry.Route) && size(entry.Route,2)==2 && size(entry.Route,1)>=2 && ...
        ismatrix(entry.Route) && all(isfinite(entry.Route),'all') && all(vecnorm(diff(entry.Route),2,2)>0) && ...
        isnumeric(entry.PhaseTime) && isreal(entry.PhaseTime) && iscolumn(entry.PhaseTime) && ...
        all(isfinite(entry.PhaseTime) & entry.PhaseTime>0) && abs(sum(entry.PhaseTime)-1)<1e-12 && ...
        all(diff([0;cumsum(entry.PhaseTime)])>0) && ...
        isnumeric(entry.LimitSignature) && isreal(entry.LimitSignature) && isequal(size(entry.LimitSignature),[3,2]) && ...
        all(isfinite(entry.LimitSignature) & entry.LimitSignature>0,'all') && ...
        islogical(entry.RestEndpoints) && isscalar(entry.RestEndpoints);
    if ~valid, error('bmtpEngine:InvalidProfileLibrary','Invalid numeric data in profile %d.',index); end
    if max(abs([power(1,:,1),sum(power(end,:,:),3)-[1,0],entry.Route(1,:),entry.Route(end,:)-[1,0]]))>1e-8
        error('bmtpEngine:InvalidProfileLibrary','Profile %d has inconsistent normalized endpoints.',index);
    end
    for order=0:3
        if order>0, power=power(:,:,2:end).*reshape(1:size(power,3)-1,1,1,[])./times; end
        if any(~isfinite(power),'all')
            error('bmtpEngine:InvalidProfileLibrary','Profile %d has nonfinite derivative coefficients.',index);
        end
        residual=sum(power(1:end-1,:,:),3)-power(2:end,:,1);
        scale=max(1,max(abs(power),[],'all'));
        if any(abs(residual)>1e-8*scale,'all')
            error('bmtpEngine:InvalidProfileLibrary','Profile %d is not C3.',index);
        end
    end
    % This clock is an initialization grid, not a replacement for the stored
    % C3 polynomial. Aggregate sub-percent phases before online optimization.
    phaseBreaks=[0;cumsum(entry.PhaseTime)];
    retainedBreaks=0;
    for phase=2:numel(phaseBreaks)-1
        if phaseBreaks(phase)-retainedBreaks(end)>=0.01
            retainedBreaks(end+1,1)=phaseBreaks(phase); %#ok<AGROW>
        end
    end
    if 1-retainedBreaks(end)<0.01 && numel(retainedBreaks)>1, retainedBreaks(end)=[]; end
    library.Entries{index}.CompactPhaseTime=diff([retainedBreaks;1]);
    arc=[0;cumsum(vecnorm(diff(entry.Route),2,2))];
    library.Entries{index}.RouteSamples=interp1(arc/arc(end),entry.Route,linspace(0,1,17).');
end
end
