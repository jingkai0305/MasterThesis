clc
clear

%% true point with covariance
true_track_mean = [0;0];
true_track_cov = [5,0;0,5];

figure(1)
xy = sigmaEllipse2D(true_track_mean, true_track_cov);
plot(xy(1,:), xy(2,:),'b--');
hold on
plot(true_track_mean(1), true_track_mean(2), 'b*')

%% generate Bernoulli distribution detection points
N = 3;
x = [];
r = 0.99;
for i = 1:N
    if rand<r
        xi = mvnrnd(true_track_mean, true_track_cov, 1)';
        x = [x,xi];
    else
        xi = [NaN;NaN];
        x = [x,xi];
    end   
end

% plot
plot(x(1,:), x(2,:),'g*')

%% covariance bar
weight = 1/N;
cov_bar = zeros(2,2);
for i = 1:N
    P_bar = weight*inv(true_track_cov);
    cov_bar = cov_bar + P_bar;
end
cov_bar = inv(cov_bar);

%% mean bar
wm = zeros(2,1);
for i = 1:N
    if ~isnan(x(:,i))
        wm = wm + weight*inv(true_track_cov)*x(:,i);
    end
end
mean_bar = cov_bar *wm;

xy = sigmaEllipse2D(mean_bar, cov_bar);
plot(xy(1,:), xy(2,:),'r--');
hold on
plot(mean_bar(1), mean_bar(2), 'r*')
