<<<<<<< Updated upstream
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
=======
function master_criteria = TracksAssociation(detection)
%TRACKSASSOCIATION Summary of this function goes here
%   Detailed explanation goes here

n_sensor = 4;

% put detections from same sensor together
sensor_detection = cell(n_sensor,1);
% warning: this makes the lopps j times, maybe takes longer time
for j = 1:n_sensor % the sensor's idx
    sensorj_detection = {};
    for i = 1:size(detection,2) % detection idx
        if(detection{i}.source) == j % if this detection comes from this sensor
            sensorj_detection{end+1} = detection{i};
        end
    end
    sensor_detection{end+1} = sensorj_detection;
end


% To do the association, first made a matrix of unassigned pairs
% initialize unassigned_tracks

unassigned_tracks = zeros(length(detection), 2);
for i = 1: length(detection)
    unassigned_tracks(i,:) = [detection{i}.source, detection{i}.track_id_local];
end



% Start data association, calculate symetric KLD between sensor i and other sensors i+1:end
% create assign_matrix
assign_matrix  = [];

for i = 1:n_sensor % pair 1, 2, 3, main
    sensor_i_tracks = unassigned_tracks(unassigned_tracks(:,1)==i,:);
%     sensor_i_tracks
    assign_matrix_new = zeros(n_sensor, size(sensor_i_tracks,1));
    assign_matrix_new(i,:) = sensor_i_tracks(:,2)';
    
    for j = (i+1):n_sensor % pair 2, 3, 4 % is 4 already matched befor? maybe can use for j = i+1:n_sensor-1
        sensor_j_tracks = unassigned_tracks(unassigned_tracks(:,1)==j,:);
%         sensor_j_tracks
        % find cost_matrix
        cost_matrix = zeros(size(sensor_i_tracks,1), size(sensor_j_tracks,1));
        for m =1:size(sensor_i_tracks,1)
            for n =1:size(sensor_j_tracks,1)
                score1 = KLD(sensor_detection{i}{sensor_i_tracks(m,2)}.mean, sensor_detection{j}{sensor_j_tracks(n,2)}.mean, ...
                                sensor_detection{i}{sensor_i_tracks(m,2)}.cov, sensor_detection{j}{sensor_j_tracks(n,2)}.cov);
                score2 = KLD(sensor_detection{j}{sensor_j_tracks(n,2)}.mean, sensor_detection{i}{sensor_i_tracks(m,2)}.mean, ...
                                sensor_detection{j}{sensor_j_tracks(n,2)}.cov, sensor_detection{i}{sensor_i_tracks(m,2)}.cov);
                cost_matrix(m, n) = 0.5*(score1+score2);
            end
        end
%         cost_matrix
        
        % find assignment using assignDetectionsToTracks
        [assignment,~,~] = assignDetectionsToTracks(cost_matrix, threshold);
%         assignment
        
        if ~isempty(assignment)
            assign_matrix_new(j,assignment(:,1)') = sensor_j_tracks(assignment(:,2)',2); 
        end
        
    end
    % renew assign_matrix by attaching new
    assign_matrix  = [assign_matrix, assign_matrix_new];
%     assign_matrix
    
    % remove the assigned tracks from unassigned_tracks
    for x = 1:size(assign_matrix_new,1) % rows in assign_matrix
        for y = 1: size(assign_matrix_new,2) % columns in assign_matrix
            assigned_element = [x, assign_matrix_new(x,y)];
            rows_to_remove = (unassigned_tracks(:,1)==assigned_element(1) & unassigned_tracks(:,2)==assigned_element(2));
            unassigned_tracks(rows_to_remove,:)=[];
        end
    end
       
end









>>>>>>> Stashed changes

end

