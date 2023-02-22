
clc

a = {[2,3], [4,6], [10,13,11,8], [1,7], 8, 9, 5};

% find(a == 3)
% index = find([a{:}] == 5)


for i = 1:length(a)
    if length(a{i}) == 1
        if sum(cell2mat(a)==a{i}) >= 2
            a{i} = [];
        end
    end
end

a = a(~cellfun('isempty',a))




function idx = FindGroupIdx(A, b)
    for i = 1:length(A)
        for j = 1:length(A{i})
            if ismember(A{i}(j), b)
                idx = i;
            end
        end
    end
end




