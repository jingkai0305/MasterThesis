function cost_matrix = getKLDcostMatrix(sensor_tracks_1, sensor_tracks_2)
%KLD_SCORE Summary of this function goes here
%   Detailed explanation goes here

cost_matrix = zeros(size(sensor_tracks_1,1), size(sensor_tracks_2,1));
for i =1:size(sensor_tracks_1,2) % get the number of the tracks in sensor1 
    for j =1:size(sensor_tracks_2,2) % get the number of the tracks in sensor2 
        score1 = KLD(sensor_tracks_1{i}.mean, sensor_tracks_2{j}.mean, ...
                        sensor_tracks_1{i}.cov, sensor_tracks_2{j}.cov);
        score2 = KLD(sensor_tracks_2{j}.mean, sensor_tracks_1{i}.mean, ...
                        sensor_tracks_2{j}.cov, sensor_tracks_1{i}.cov);
        cost_matrix(i, j) = 0.5*(score1+score2);
    end
end

end

