clc 
clear all


N_gt = 7; % number of groud truth;
N_m = 96; % number of maximum detection of a sensor
N_s = 3; %number of sensor


%% randomly create ground truth
ground_truth = {};
for i = 1:N_gt
    ground_truth{i} = struct('mean',[-30 + (30-(-30))*rand(); -10 + (110-(-10))*rand()],...
        'difficulty',[0 + (3-0)*rand(), 0; 0, 0 + (3-0)*rand()]);
end

%% location of the groud truth
for i = 1:N_gt
    plot(ground_truth{i}.mean(1),ground_truth{i}.mean(2),'*b');    
    hold on
end

%% generate detections under Bernoulli with Gaussian
detection = {}; % create empty detection

r1 = 0.8; 
r2 = 0.9;
r3 = 0.85;
cov1 = 1*[1,0;0,1]; % side front radar
cov2 = 2*[1,0;0,1]; % FLR
cov3 = 3*[1,0;0,1]; % FLC

r = {r1,r2,r3}; % p of each sr's bernoulli
cov = {cov1,cov2,cov3}; % accuracies of the sensors


for i=1:N_gt
    for j=1:N_s
        if rand(1) >= 1-r{j}
            detection{end+1} = struct('confidence',rand(1),...
                'mean',mvnrnd(ground_truth{i}.mean, ground_truth{i}.difficulty)',...
                'covariance', cov{j});
        end
    end
end

    
%% random fault
N_f = ceil(length(detection)/10);
for i = 1:N_f
    detection{end+1} = struct('confidence',rand(1),...
        'mean',[-30 + (30-(-30))*rand(); -10 + (110-(-10))*rand()],...
        'covariance',[0 + (3-0)*rand(), 0; 0, 0 + (3-0)*rand()]);
end

%% location of the detection
for i = 1:length(detection)
    plot(detection{i}.mean(1),detection{i}.mean(2),'.r','MarkerSize',10);
    xy = sigmaEllipse2D(detection{i}.mean, detection{i}.covariance);
    plot(xy(1,:),xy(2,:),'--r');
    hold on
end

hold off

%%

% idx1 = 1;
% idx2 = 4;

% KLD(detection{idx1}.mean, detection{idx2}.mean, detection{idx1}.covariance, detection{idx2}.covariance)
% KLD(detection{idx2}.mean, detection{idx1}.mean, detection{idx2}.covariance, detection{idx1}.covariance)



%% functions
function [dist] = KLD(m1,m2,cov1,cov2)
    dist = 0.5 * ((m2-m1)' * inv(cov2) * (m2-m1) + trace(inv(cov2)*cov1) - log(det(cov1)/det(cov2))-length(m1));
end



