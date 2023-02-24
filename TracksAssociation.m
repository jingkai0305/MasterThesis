function group = TracksAssociation(sensor_detection, threshold)
%TRACKSASSOCIATION Summary of this function goes here
%   Detailed explanation goes here
% outputArg1 = inputArg1;
% outputArg2 = inputArg2;



%% form the association groups upon all the tracks
group = {};

n_sensor = 3;

for i = 1:n_sensor-1 % i = 1 to num_sensing-1
    for j =  i+1:n_sensor % i+1 to num_sensing
        [assignment, unassigned_1, unassigned_2] = ...
            TracksPairAssign(sensor_detection{i}, sensor_detection{j}, threshold); 
        % return assignment matrix and vectors for unassgined detections
        
        ID_matrix = FindIDPairFromAssigment(assignment, sensor_detection, i, j);
        % convert assigment matrix into the ID_matrix which contains global
        % ID for each tracks. ID matrix in shape 2*N, each column stores a
        % pair of tracks 
        
        % in each column, if upper track in ID matrix is member in the
        % group, lower track should be store in the corresponding group
        for m = 1:size(ID_matrix,2) 
            flag = ismember(ID_matrix(1,m), cell2mat(group));
                if flag
                    idx = FindGroupIdx(group, ID_matrix(1,m)); % return idx of the group 
                    group{idx}(end+1) = ID_matrix(2,m); % add new track into this group
                    group{idx} = unique(group{idx}); % make each cell in group unique
                else
                    % if upper member is not a member, create a new group at the tail
                    group{end+1} = ID_matrix(:,m)'; 
                end
        end
        
        % store all the unassigned tracks at the tail 
        for k = 1:length(unassigned_1)
            if ~ismember(sensor_detection{i}{unassigned_1(k)}.ID, cell2mat(group))
                group{end+1} = sensor_detection{i}{unassigned_1(k)}.ID;
            end
        end
        
        for k = 1:length(unassigned_2)
            if ~ismember(sensor_detection{j}{unassigned_2(k)}.ID, cell2mat(group))
                group{end+1} = sensor_detection{j}{unassigned_2(k)}.ID;
            end
        end
        
    end
end

% group cleaning up, if one-track group exists repeatly in any other group, remove
% it
for i = 1:length(group)
    if length(group{i}) == 1
        if sum(cell2mat(group)==group{i}) >= 2
            group{i} = [];
        end
    end
end

% clean up empty group, since the previous removal results in empty cell
group = group(~cellfun('isempty',group));

end

