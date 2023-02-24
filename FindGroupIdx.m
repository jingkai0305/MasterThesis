function idx = FindGroupIdx(A, b)
% given a track, return the idx of the group which contains this track

    for i = 1:length(A)
        for j = 1:length(A{i})
            if ismember(A{i}(j), b)
                idx = i;
            end
        end
    end
end
