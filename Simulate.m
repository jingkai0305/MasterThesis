clc
clear
close all

%% Generate ground truth
Point_truth = [30, 50, 70; 30, 50, 70];
Detection_cov = 10*eye(2);

%% draw
figure(1)
hold on
for i = 1:size(Point_truth,2)
    [ xy ] = sigmaEllipse2D(Point_truth(:,i), Detection_cov);
    plot(Point_truth(1,i),Point_truth(2,i),'r*')
    plot(xy(1,:), xy(2,:),'r--')
end

%% Generate detections from sensor 1
r1 = 0.8;



