% this will trying to handle the sensors with different FOV but have small
% intersection area
% Goal: draw 7 circles around 7 ground truths
clc
clear
close all

%% Create 3 areas with intersection
% At this small program, the ground truths are fixed
% Assume there are 3 FOVs, have tersections
f = figure(1);
f.WindowState = 'maximized';
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
ground_truth{7} = struct('mean', [80;20],'covariance', 1*eye(2));

for i = 1:length(ground_truth)
    plot(ground_truth{i}.mean(1), ground_truth{i}.mean(2),'r*')
end

%% Create detections depending on the FOV
detection = {}; % the existance ratio is not considered now
for i = 1: length(ground_truth) % ground_truth
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area1_xy(1,:), area1_xy(2,:)) % in area 1
        detection{end+1} = struct('source', 1, 'existance', r1, ...
            'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
    end
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area2_xy(1,:), area2_xy(2,:)) % in area 2
        detection{end+1} = struct('source', 2, 'existance', r2, ...
            'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
    end
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area3_xy(1,:), area3_xy(2,:)) % in area 3
        detection{end+1} = struct('source', 3, 'existance', r3, ...
            'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
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

%% plot the detections with the label
for i = 1: length(sensor_detection)
    for j = 1: length(sensor_detection{i})
        plot(sensor_detection{i}{j}.mean(1), sensor_detection{i}{j}.mean(2), 'ob');
        text(sensor_detection{i}{j}.mean(1)+0.1, sensor_detection{i}{j}.mean(2)-0.1, [num2str(i),',',num2str(j)])
    end
end

%% calculate symetric KLD score between every pair tracks
assign_matrix_line = cell(1, length(r));
unassigned_tracks =[];
i = 1;
for j = 2: length(r)
    % find KLD cost matrix
    cost_matrix = zeros(size(sensor_detection{i},2), size(sensor_detection{j},2));
    for m =1:size(sensor_detection{i},2)
        for n =1:size(sensor_detection{j},2)
            score1 = KLD(sensor_detection{i}{m}.mean, sensor_detection{j}{n}.mean, sensor_detection{i}{m}.cov, sensor_detection{j}{n}.cov);
            score2 = KLD(sensor_detection{j}{n}.mean, sensor_detection{i}{m}.mean, sensor_detection{j}{n}.cov, sensor_detection{i}{m}.cov);
            cost_matrix(m, n) = 0.5*(score1+score2);
        end
    end
    % cost_matrix
    
    [assignment,unassignedSensori,unassignedSensorj] = assignDetectionsToTracks(cost_matrix, threshold);
    assignment
    unassigned_tracks = [unassigned_tracks; repmat(j,length(unassignedSensorj),1), unassignedSensorj];
    assign_matrix_line{j} = assignment';
end
unassigned_tracks

% assign matrix related to sensor 1
assign_matrix = zeros(length(r),size(sensor_detection{1},2));
assign_matrix(1, 1:size(sensor_detection{1},2)) = 1:size(sensor_detection{1},2);
for i = 2:length(r)
    assign_matrix(i, assign_matrix_line{i}(1,:)) = assign_matrix_line{i}(2,:);
end
% assign_matrix = zeros(length(r),size(sensor_detection{1},2)+size(unassigned_tracks,1));
% assign_matrix(1, 1:size(sensor_detection{1},2)) = 1:size(sensor_detection{1},2);
% for i = 2:length(r)
%     assign_matrix(i, assign_matrix_line{i}(1,:)) = assign_matrix_line{i}(2,:);
% end
% for i = 1: size(unassigned_tracks,1)
%     assign_matrix(unassigned_tracks(i,1),size(sensor_detection{1},2)+i) = unassigned_tracks(i,2);
% end
% assign_matrix

%% deal with unassigned tracks
for i = 2 % sensor i
    for j = (i+1):length(r) % sensor j
        unassign_track_i = {};
        unassign_track_j = {};
        % use unassigned_tracks only
        for k = 1:size(unassigned_tracks) % go through all unassigned tracks
            if unassigned_tracks(k,1)==i
                unassign_track_i{end+1} = sensor_detection{i}{unassigned_tracks(k,2)};
            end
            if unassigned_tracks(k,1)==j
                unassign_track_j{end+1} = sensor_detection{j}{unassigned_tracks(k,2)};
            end
        end
        
        % find cost matrix
        cost_matrix = zeros(size(unassign_track_i,2), size(unassign_track_j,2));
        for m =1:size(unassign_track_i,2)
            for n =1:size(unassign_track_j,2)
                score1 = KLD(unassign_track_i{m}.mean, unassign_track_j{n}.mean, unassign_track_i{m}.cov, unassign_track_j{n}.cov);
                score2 = KLD(unassign_track_j{n}.mean, unassign_track_i{m}.mean, unassign_track_j{n}.cov, unassign_track_i{m}.cov);
                cost_matrix(m, n) = 0.5*(score1+score2);
            end
        end
        
        % assign2D
        [assignment,unassignedSensori,unassignedSensorj] = assignDetectionsToTracks(cost_matrix, threshold);
        assignment
        % assign matrix related to other sensors
        if ~isempty(assignment)
            for p = 1: size(assignment,1)
                assign_matrix= [assign_matrix zeros(length(r),size(assignment,1))];
                assign_i_idx = unassigned_tracks(unassigned_tracks(:,1)==i, 2);
                assign_j_idx = unassigned_tracks(unassigned_tracks(:,1)==j, 2);
                assign_matrix(i,end) = assign_i_idx(assignment(p,1));
                assign_matrix(j,end) = assign_j_idx(assignment(p,2));              
            end
            assign_matrix           
        end       
    end
end

%% find the unique detection (except sensor 1)
for i = 2:size(assign_matrix,1) % sensor idx
    all = 1:length(sensor_detection{i}); % assumned idx
    assigned_detections = assign_matrix(i,:);
    idx = ismember(all, assigned_detections);
    unassigned_detections = all(~idx);
    for j = 1:length(all(~idx)) % element of unassigned
        assign_matrix = [assign_matrix zeros(length(r),1)];
        assign_matrix(i,end) = unassigned_detections(j);
    end
end
assign_matrix  

 

%% gather the detections by objects
object_detection = cell(1,size(assign_matrix,2));
for i = 1:size(assign_matrix,2) % object
    for j = 1:length(r) % sensor
        if assign_matrix(j,i)~=0
            object_detection{i}{end+1} = sensor_detection{j}(assign_matrix(j,i));
        end
    end  
end

%% Fusion
% simple fusion
for i = 1:length(object_detection)
    object_fusion{i} = struct('mean',[],'cov',[]);
end

for i = 1:length(object_detection) % iter object
    sum_winvP = 0;
    sum_winvPm = 0;
    for j = 1:length(object_detection{i}) % iter sensor detection
        sum_winvP = sum_winvP + w(object_detection{i}{j}{1}.source)*inv(object_detection{i}{j}{1}.cov);
        sum_winvPm = sum_winvPm + w(object_detection{i}{j}{1}.source)*inv(object_detection{i}{j}{1}.cov)*object_detection{i}{j}{1}.mean;
    end

    object_fusion{i}.cov = inv(sum_winvP);
    object_fusion{i}.mean = object_fusion{i}.cov*sum_winvPm;
end

%% Draw
for i = 1:length(object_detection)
     xy = sigmaEllipse2D(object_fusion{i}.mean, object_fusion{i}.cov);
     plot(xy(1,:), xy(2,:), 'r--')
end

% find unassigned tracks, only one non-zero element in that column
num_unassign = 0;
for i = 1:size(assign_matrix,2)
    col = assign_matrix(:,i);
    if sum(col==0) == length(r)-1
        num_unassign = num_unassign+1;
    end
end
title([num2str(length(object_detection)),' objects are detected ', num2str(num_unassign),...
    ' of them are unassigned, there are ',num2str(7),' ground truth.'])


%% help functions
function score = KLD(m1,m2,cov1,cov2)
    score = 0.5*( (m2-m1)'*inv(cov2)*(m2-m1) + trace(inv(cov2)*cov1)...
        - log(det(cov1)/det(cov2)) - 2 );
end