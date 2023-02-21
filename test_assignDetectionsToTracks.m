clc
clear
close all

%% Create two areas with intersection
% At this small program, the ground truths are fixed
% Assume there are 2 FOVs, half intersection
figure(1)
hold on 
axis equal
rectangle('Position',[0 0 50 50],'EdgeColor','r');
rectangle('Position',[25 -25 50 50],'EdgeColor','g');

%% Create some ground truth
% hard coded now
ground_truth{1} = struct('mean', [10;30],'covariance', 1*eye(2)); % in zone 1
ground_truth{2} = struct('mean', [35;10],'covariance', 1*eye(2)); % in zone intersection
ground_truth{3} = struct('mean', [65;-10],'covariance', 1*eye(2)); % in zone 2

for i = 1:length(ground_truth)
    plot(ground_truth{i}.mean(1), ground_truth{i}.mean(2),'b*')
end

%%  Create detections depending on the FOV
detection{1} = struct('source', 1, 'existance', 0.8, 'mean', mvnrnd(ground_truth{1}.mean,ground_truth{1}.covariance)', 'cov', randi(10)*[2,0;0,1]);
detection{2} = struct('source', 1, 'existance', 0.8, 'mean', mvnrnd(ground_truth{2}.mean,ground_truth{2}.covariance)', 'cov', randi(10)*[2,0;0,1]);
detection{3} = struct('source', 2, 'existance', 0.8, 'mean', mvnrnd(ground_truth{2}.mean,ground_truth{2}.covariance)', 'cov', randi(10)*[2,0;0,1]);
detection{4} = struct('source', 2, 'existance', 0.8, 'mean', mvnrnd(ground_truth{3}.mean,ground_truth{1}.covariance)', 'cov', randi(10)*[2,0;0,1]);

%% put detections from same sensor together
sensor_detection = {};
% warning: this makes the lopps j times, maybe takes longer time
for j = 1:2 % the sensor's idx
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

%% try to do matching
r = [1,2];
w = [0.5, 0.5];

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
    % cost matrix get here
    cost_matrix
    [assignment,unassignedSensori,unassignedSensorj] = assignDetectionsToTracks(cost_matrix, 10)
end

%% gather the detections by objects
object_detection = cell(1,size(assignment,1)+length(unassignedSensori)+length(unassignedSensorj));
object_detection{1} = {sensor_detection{1}{1}};
object_detection{2} = {sensor_detection{1}{2},sensor_detection{2}{1}};
object_detection{3} = {sensor_detection{2}{2}};

%% Fusion
% simple fusion
for i = 1:length(object_detection)
    object_fusion{i} = struct('mean',[],'cov',[]);
end

for i = 1:length(object_detection) % iter object
    sum_winvP = 0;
    sum_winvPm = 0;
    for j = 1:length(object_detection{i}) % iter sensor detection
        sum_winvP = sum_winvP + w(object_detection{i}{j}.source)*inv(object_detection{i}{j}.cov);
        sum_winvPm = sum_winvPm + w(object_detection{i}{j}.source)*inv(object_detection{i}{j}.cov)*object_detection{i}{j}.mean;
    end

    object_fusion{i}.cov = inv(sum_winvP);
    object_fusion{i}.mean = object_fusion{i}.cov*sum_winvPm;
end

%% Draw
for i = 1:length(object_detection)
     xy = sigmaEllipse2D(object_fusion{i}.mean, object_fusion{i}.cov);
     plot(xy(1,:), xy(2,:), 'r--')
end

%% help functions
function score = KLD(m1,m2,cov1,cov2)
    score = 0.5*( (m2-m1)'*inv(cov2)*(m2-m1) + trace(inv(cov2)*cov1)...
        - log(det(cov1)/det(cov2)) - 2 );
end