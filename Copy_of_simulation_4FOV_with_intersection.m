clc
clear
close all

% An adjustable property in assignMunkres
threshold = 10;
%% Create 4 areas with intersection
f = figure(1);
f.WindowState = 'maximized';
hold on 
axis equal
% Draw 4 square areas to reprsent survilliance areas
rectangle('Position',[0  0  100 100],   'EdgeColor','r');
rectangle('Position',[50 0  100 100],  'EdgeColor','b');
rectangle('Position',[0  50 100 100], 'EdgeColor','g');
rectangle('Position',[50 50 100 100], 'EdgeColor','k');
text(5, 5, 'Sensor 1', 'Color',' r')
text(135, 5, 'Sensor 2','Color', 'b')
text(5, 145, 'Sensor 3', 'Color', 'g')
text(135, 145, 'Sensor 4', 'Color', 'k')

% Show areas above, but a little smaller
area1_xy = [5, 95, 95, 5;
            5, 5,  95, 95];
area2_xy = [55, 145, 145, 55;
            5, 5,  95, 95];
area3_xy = [5, 95, 95, 5;
            55, 55, 145, 145];
area4_xy = [55, 145, 145, 55;
            55, 55, 145, 145];

%% properties of detections
r1 = 0.95;
r2 = 0.8;
r3 = 0.8;
r4 = 0.8;
r = [r1, r2, r3, r4];
w = [1/4, 1/4, 1/4, 1/4];

%% Create some hardcoded ground truth
% In this small program, the ground truths are fixed
ground_truth{1} = struct('mean', [25;  25],'covariance', 1*eye(2));
ground_truth{2} = struct('mean', [75;  25],'covariance', 1*eye(2));
ground_truth{3} = struct('mean', [125; 25],'covariance', 1*eye(2));
ground_truth{4} = struct('mean', [25;  75],'covariance', 1*eye(2));
ground_truth{5} = struct('mean', [75;  75],'covariance', 1*eye(2));
ground_truth{6} = struct('mean', [125; 75],'covariance', 1*eye(2));
ground_truth{7} = struct('mean', [25; 125],'covariance', 1*eye(2));
ground_truth{8} = struct('mean', [75; 125],'covariance', 1*eye(2));
ground_truth{9} = struct('mean', [125;125],'covariance', 1*eye(2));
% ground_truth{10} = struct('mean', [90; 60],'covariance', 1*eye(2));

% write some random ground truth

% plot the ground truths
for i = 1:length(ground_truth)
    plot(ground_truth{i}.mean(1), ground_truth{i}.mean(2),'r*')
end

%% Create detections depending on the FOV
% detextion is raw detection collected
detection = {}; % the existance ratio is not considered now

% there are 6 elements in the detecttion structure.

% initialize the idx
sensor_id = 1;
sensor1_id = 1;
sensor2_id = 1;
sensor3_id = 1;
sensor4_id = 1;

for i = 1: length(ground_truth) % ground_truth
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area1_xy(1,:), area1_xy(2,:)) % in area 1
        detection{end+1} = struct('source', 1, 'existance', r1,...
        'track_id_global', sensor_id, 'track_id_local', sensor1_id, ...
        'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
    
        sensor_id = sensor_id + 1;
        sensor1_id = sensor1_id +1;      
    end
    
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area2_xy(1,:), area2_xy(2,:)) % in area 2
        detection{end+1} = struct('source', 2, 'existance', r2, ...
        'track_id_global', sensor_id, 'track_id_local', sensor2_id, ...
        'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
    
        sensor_id = sensor_id + 1;
        sensor2_id = sensor2_id +1;
    end
    
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area3_xy(1,:), area3_xy(2,:)) % in area 3
        detection{end+1} = struct('source', 3, 'existance', r3, ...
        'track_id_global', sensor_id, 'track_id_local', sensor3_id, ...
        'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
        
        sensor_id = sensor_id + 1;
        sensor3_id = sensor3_id +1; 
    end 
    
    if inpolygon(ground_truth{i}.mean(1),ground_truth{i}.mean(2),area4_xy(1,:), area4_xy(2,:)) % in area 3
        detection{end+1} = struct('source', 4, 'existance', r4, ...
        'track_id_global', sensor_id, 'track_id_local', sensor4_id, ...
        'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', 2*[2,0;0,1]);
            
        sensor_id = sensor_id + 1;
        sensor4_id = sensor4_id +1; 
    end 
end

%% put detections from same sensor together
sensor_detection = {};
% warning: this makes the lopps j times, maybe takes longer time
for j = 1:length(r) % the sensor's idx
    sensorj_detection = {};
    for i = 1:size(detection,2) % detection idx
        if(detection{i}.source) == j % if this detection comes from this sensor
            sensorj_detection{end+1} = detection{i};
        end    
    end
    sensor_detection{end+1} = sensorj_detection;
end

%% plot the detections with the label
for i = 1: length(sensor_detection) % sensor
    for j = 1: length(sensor_detection{i}) % element in sensor detection
        plot(sensor_detection{i}{j}.mean(1), sensor_detection{i}{j}.mean(2), 'ob');
        text(sensor_detection{i}{j}.mean(1)+0.1, sensor_detection{i}{j}.mean(2)-0.1, ...
            [num2str(sensor_detection{i}{j}.source),',',num2str(sensor_detection{i}{j}.track_id_local)])
    end
end




%% Start data association, calculate symetric KLD between sensor i and other sensors i+1:end
N_TRACKS = 96;
N_SENSORS = 4;

threshold = 10;

master_criteria = zeros(N_TRACKS,'single');

for i = 1:N_SENSORS-1
    for j = i+1:N_SENSORS
        cost_matrix = getKLDcostMatrix(sensor_detection{i}, sensor_detection{j});
        assignment = assignDetectionsToTracks(cost_matrix, threshold);
        
        % assign logic 1 into master criteria
        for m = 1:size(assignment,1)
            local_id_1 = assignment(m,1);
            local_id_2 = assignment(m,2);
            
            global_id_1 = sensor_detection{i}{local_id_1}.track_id_global;
            global_id_2 = sensor_detection{j}{local_id_2}.track_id_global;
            
            master_criteria(global_id_1, global_id_2) = 1;
            master_criteria(global_id_2, global_id_1) = 1;
            
            
        end           
        
    end
end


