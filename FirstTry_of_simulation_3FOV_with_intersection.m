% this will trying to handle the sensors with different FOV but have small
% intersection area
% Goal: draw 7 circles around 7 ground truths
clc
% clear all
close all

%% Create 3 areas with intersection
% At this small program, the ground truths are fixed
% Assume there are 3 FOVs, have tersections
f = figure(1);
% f.WindowState = 'maximized';
hold on 
axis equal
rectangle('Position',[0 0 60 60],   'EdgeColor','r');
rectangle('Position',[40 0 60 60],  'EdgeColor','g');
rectangle('Position',[20 40 60 60], 'EdgeColor','b');

text(5, 5, 'Sensor 1')
text(90, 5, 'Sensor 2')
text(25, 95, 'Sensor 3')

area1_xy = [0, 60, 60, 0;
            0, 0,  60, 60];
area2_xy = [40, 100, 100, 40;
            0,  0,   60,  60];
area3_xy = [20, 80, 80,  20;
            40, 40, 100, 100];
        
threshold = 10;
        
%% properties of detections
r1 = 0.95;
r2 = 0.8;
r3 = 0.8;
r = [r1, r2, r3];
w = [1/3, 1/3, 1/3];

%% Create some hardcoded ground truth
ground_truth{1} = struct('mean', [50;80],'covariance', 1*eye(2));
ground_truth{2} = struct('mean', [30;50],'covariance', 1*eye(2));
ground_truth{3} = struct('mean', [50;50],'covariance', 1*eye(2));
ground_truth{4} = struct('mean', [70;50],'covariance', 1*eye(2));
ground_truth{5} = struct('mean', [20;20],'covariance', 1*eye(2));
ground_truth{6} = struct('mean', [50;20],'covariance', 1*eye(2));
% ground_truth{7} = struct('mean', [80;20],'covariance', 1*eye(2));
ground_truth{7} = struct('mean', [8;2],'covariance', 1*eye(2));

for i = 1:length(ground_truth)
    plot(ground_truth{i}.mean(1), ground_truth{i}.mean(2),'r*')
end

%% Create detections depending on the FOV
detection = {}; % the existance ratio is not considered now
idx = 1;
for i = 1: length(ground_truth) % ground_truth
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area1_xy(1,:), area1_xy(2,:)) % in area 1
        detection{end+1} = struct('ID', idx, 'source', 1, 'existance', r1, ...
            'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
        idx = idx+1;
    end
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area2_xy(1,:), area2_xy(2,:)) % in area 2
        detection{end+1} = struct('ID', idx, 'source', 2, 'existance', r2, ...
            'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
        idx = idx+1;
    end
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area3_xy(1,:), area3_xy(2,:)) % in area 3
        detection{end+1} = struct('ID', idx, 'source', 3, 'existance', r3, ...
            'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
        idx = idx+1;
    end 
end

%% put detections from same sensor together
sensor_detection = {};
% warning: this makes the lopps j times, maybe takes longer time
for j = 1:3 % the sensor's idx
    sensorj_detection = {};
    for i = 1:size(detection,2) % detection idx
        if(detection{i}.source) == j % if this detection comes from this sensor
            sensorj_detection{end+1} = detection{i};
        end    
    end
    sensor_detection{end+1} = sensorj_detection;
end

%% plot the detections
for i = 1: length(sensor_detection)
    for j = 1: length(sensor_detection{i})
        plot(sensor_detection{i}{j}.mean(1), sensor_detection{i}{j}.mean(2), '.b');
        text(sensor_detection{i}{j}.mean(1)+0.1, sensor_detection{i}{j}.mean(2)-0.1, [num2str(i),',',num2str(j)])
    end
end

%% form the association groups upon all the tracks
threshold = 1;
group = {};



for i = 1:length(r)-1 % i = 1 to num_sensing-1
    for j = i+1:length(r) % i+1 to num_sensing
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
         




%% help functions
function score = KLD(m1,m2,cov1,cov2)
% calculate KLD score between two Gaussian densities

    score = 0.5*( (m2-m1)'*inv(cov2)*(m2-m1) + trace(inv(cov2)*cov1)...
        - log(det(cov1)/det(cov2)) - 2 );
end


function [assignment, unassigned_1, unassigned_2] = TracksPairAssign(detection_1, detection_2, threshold)
% Input: detection_1, detection_2, assignment threshold
% Output: assignment of two detections, unassigned detection_1, and
% unassigned detection_2


    % find KLD cost matrix
    cost_matrix = zeros(size(detection_1,2), size(detection_2,2));
    for m =1:size(detection_1,2)
        for n =1:size(detection_2,2)
            score1 = KLD(detection_1{m}.mean, detection_2{n}.mean, detection_1{m}.cov, detection_2{n}.cov);
            score2 = KLD(detection_2{n}.mean, detection_1{m}.mean, detection_2{n}.cov, detection_1{m}.cov);
            cost_matrix(m, n) = 0.5*(score1+score2);
        end
    end

    [assignment, unassigned_1, unassigned_2] = assignDetectionsToTracks(cost_matrix, threshold);
end



function ID_matrix = FindIDPairFromAssigment(assignment, sensor_detection, num_s1, num_s2)
% convert assgin matrix into ID_matrix

    % transpose the assign matrix into 2xN
    assignment = assignment';
    % preallocate ID_matrix
    ID_matrix = zeros(2,size(assignment, 2));
    
    for j = 1:size(assignment, 2)
        ID_matrix(1, j) = sensor_detection{num_s1}{assignment(1,j)}.ID;
    end
    
    for j = 1:size(assignment, 2)
        ID_matrix(2, j) = sensor_detection{num_s2}{assignment(2,j)}.ID;
    end
    
end



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





