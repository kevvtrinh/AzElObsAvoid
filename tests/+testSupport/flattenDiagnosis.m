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

leafCount = countLeaves(value);
fields    = strings(leafCount, 1);
values    = cell(leafCount, 1);
nextLeaf  = 0;
visit(value, "");
details = table(fields, values, 'VariableNames', {'Field','Value'});

    function count = countLeaves(item)
        % Count output rows once so recursive collection does not repeatedly
        % reallocate the growing string and cell arrays.
        count = 0;
        if isstruct(item)
            names = string(fieldnames(item));
            if numel(item) > 1 && hasOnlyLeafFields(item, names)
                count = numel(item) * numel(names);
                return;
            end
            for itemIndex = 1:numel(item)
                for name = reshape(names, 1, [])
                    count = count + countLeaves(item(itemIndex).(name));
                end
            end
        elseif iscell(item)
            for itemIndex = 1:numel(item)
                count = count + countLeaves(item{itemIndex});
            end
        else
            count = 1;
        end
    end

    function visit(item, prefix)
        if isstruct(item)
            names = string(fieldnames(item));
            if numel(item) > 1 && hasOnlyLeafFields(item, names)
                % Homogeneous diagnostic arrays such as separating planes
                % can emit the same paths and values without one recursive
                % function call and string concatenation per scalar field.
                itemCount = numel(item);
                nameCount = numel(names);
                parents = prefix + "(" + string((1:itemCount).') + ")";
                childGrid = parents + "." + names.';
                blockValues = cell(itemCount, nameCount);
                for nameIndex = 1:nameCount
                    blockValues(:, nameIndex) = reshape({item.(names(nameIndex))}, [], 1);
                end
                blockRows = itemCount * nameCount;
                selectedRows = nextLeaf + (1:blockRows);
                fields(selectedRows, 1) = reshape(childGrid.', [], 1);
                values(selectedRows, 1) = reshape(blockValues.', [], 1);
                nextLeaf = nextLeaf + blockRows;
                return;
            end
            % Preserve the original structure-array traversal order.
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
            % Preserve the original cell traversal order.
            for itemIndex = 1:numel(item)
                visit(item{itemIndex}, prefix + "{" + itemIndex + "}");
            end
        else
            nextLeaf = nextLeaf + 1;
            fields(nextLeaf, 1) = prefix;
            values{nextLeaf, 1} = item;
        end
    end

    function onlyLeaves = hasOnlyLeafFields(item, names)
        % Confirm every structure-array field is already a table leaf before
        % using the block collector. Mixed nested values keep normal recursion.
        onlyLeaves = true;
        for itemIndex = 1:numel(item)
            for name = reshape(names, 1, [])
                fieldValue = item(itemIndex).(name);
                if isstruct(fieldValue) || iscell(fieldValue)
                    onlyLeaves = false;
                    return;
                end
            end
        end
    end
end
