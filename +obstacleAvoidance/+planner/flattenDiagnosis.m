function details = flattenDiagnosis(value)
%% Section 0: Header & Readme
% SYNTAX
%   details = flattenDiagnosis(value)
% PURPOSE
%   Replace nested structures with a field/value table; retain numeric arrays.
% INPUTS
%   value: diagnostic structure or collection.
% OUTPUTS
%   details: table with Field paths and Value cells.
% UNITS
%   Values retain their original units.

%% Section 1: Collect Leaf Values
fields = strings(0,1);
values = cell(0,1);
visit(value, "");
details = table(fields, values, 'VariableNames', {'Field','Value'});

    function visit(item, prefix)
        if isstruct(item)
            names = string(fieldnames(item));
            % Process each item needed to complete t.
            for itemIndex = 1:numel(item)
                parent = prefix;
                if numel(item) > 1, parent = parent + "(" + itemIndex + ")"; end
                % Apply the required validation or transfer to each name.
                for name = reshape(names,1,[])
                    child = name;
                    if strlength(parent) > 0, child = parent + "." + name; end
                    visit(item(itemIndex).(name), child);
                end
            end
        elseif iscell(item)
            % Process each item needed to complete t.
            for itemIndex = 1:numel(item)
                visit(item{itemIndex}, prefix + "{" + itemIndex + "}");
            end
        else
            fields(end+1,1) = prefix;
            values{end+1,1} = item;
        end
    end
end
