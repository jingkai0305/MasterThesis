clc 
clear
close all

%% generate groud truth

N_gt = 7;
ground_truth = {};
for i = 1:N_gt
    ground_truth{i} = struct('mean', [-30 + (30-(-30))*rand(1,1); -10 + (120-(-10))*rand(1,1)],...
        'covariance', [rand(),0;0,rand()]);
end

%% properties for the sensors
r1 = 0.7;
r2 = 0.7;
r3 = 0.7;
r4 = 0.5;
r5 = 0.5;
r = [r1, r2, r3, r4, r5];
w = [1/5, 1/5, 1/5, 1/5, 1/5];

cov1 = 10*eye(2);
cov2 = 10*eye(2);
cov3 = 10*eye(2);
cov4 = 10*eye(2);

cov = [cov1,cov2,cov3,cov4];
%% generate detections using Poisson and Bernoulli
detection = {};

for i = 1:N_gt
    for j = 1:length(r)
        if rand < r(j)
            detection{end+1} = struct('source', j, 'existance', 0.8, ...
                'mean', mvnrnd(ground_truth{i}.mean,ground_truth{i}.covariance)', 'cov', randi(10)*[2,0;0,1]);
        end
    end
end

% false detection
% or unique detection?
% for i = 1:ceil(length(N_gt/10))
%     detection{end+1} = struct('source', randi([1,length(r)]), 'existance', 0.8, ...
%                 'mean', [-30 + (30-(-30))*rand(1,1); -10 + (120-(-10))*rand(1,1)], 'cov', eye(2));
% end

% test the unique detection only in sensor 1
for i = 1:ceil(length(N_gt/10))
    detection{end+1} = struct('source', 1, 'existance', 0.8, ...
                'mean', [-30 + (30-(-30))*rand(1,1); -10 + (120-(-10))*rand(1,1)], 'cov', eye(2));
end


%% plot the ground truth
figure(1)
axis equal
hold on
% plot ground truth
for i = 1:length(ground_truth)
    plot(ground_truth{i}.mean(1), ground_truth{i}.mean(2), '*r')
end
% for i = 1:length(detection) % sensor idx
%     plot(detection{i}.mean(1), detection{i}.mean(2), 'ob')
%     text(detection{i}.mean(1)+0.1, detection{i}.mean(2)-0.1, num2str(detection{i}.source))
% end
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

%% plot the detections
for i = 1: length(sensor_detection)
    for j = 1: length(sensor_detection{i})
        plot(sensor_detection{i}{j}.mean(1), sensor_detection{i}{j}.mean(2), 'ob');
        text(sensor_detection{i}{j}.mean(1)+0.1, sensor_detection{i}{j}.mean(2)-0.1, [num2str(i),',',num2str(j)])
    end
end

%% calculate symetric KLD score between every pair tracks
% N_detect = length(detection);

% cost_matrix = zeros(N_detect, N_detect);

% for i = 1:N_detect
%     for j = i+1:N_detect
%         score1 = KLD(detection{i}.mean, detection{j}.mean, detection{i}.cov, detection{j}.cov);
%         score2 = KLD(detection{j}.mean, detection{i}.mean, detection{j}.cov, detection{i}.cov);
%         cost_matrix(i, j) = 0.5*(score1+score2);
%     end
% end
tuples_collect = cell(1, length(r)-1);

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

    % use assign2D to find pairs
    [col4row, row4col, gain]=assign2D(cost_matrix);
    N=size(cost_matrix,1);

    for curRow=1:N
        tuples(1,curRow)=curRow;
        tuples(2,curRow)=col4row(curRow);
    end
    % tuples
    tuples_collect{j-1} = tuples;

end

%% gather the dettections by objects
assign_matrix = tuples_collect{1};
for j = 3: length(r)
    assign_matrix(j,:) = tuples_collect{j-1}(2,:);
end
assign_matrix

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

%% help functions
function score = KLD(m1,m2,cov1,cov2)
    score = 0.5*( (m2-m1)'*inv(cov2)*(m2-m1) + trace(inv(cov2)*cov1)...
        - log(det(cov1)/det(cov2)) - 2 );
end
