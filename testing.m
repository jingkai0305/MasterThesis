<<<<<<< Updated upstream

clc
clear all


a = {[2,3], [4,6], [10,13,11,8], [1,7], 8, 9, 5};
=======
% N_detection = 0;
% for i = 1:4
%     N_detection = N_detection + length(sensor_detection{i})
% end


% 
% sensor_ddddd = {};
% 
% for i = 1:length(detection)
%     sensor_ddddd{detection{i}.source}{end+1} = detection{i}; 
% end



sensor_tracks_1 = sensor_detection{3};
sensor_tracks_2 = sensor_detection{2};

cm = getKLDcostMatrix(sensor_tracks_1, sensor_tracks_2)



%% Try master thesis own association method

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
>>>>>>> Stashed changes





<<<<<<< Updated upstream




% FoV 1
R_1 = 100;
range_1 = 90 * pi/180; % in rad
triangle1_xy = [0, tan(range_1/2)*R_1, -tan(range_1/2)*R_1;
                0, R_1, R_1];
         
% FoV 2
R_2 = 80;
range_2 = 120 * pi/180; % in rad
triangle2_xy = [0, tan(range_2/2)*R_2, -tan(range_2/2)*R_2;
                0, R_2, R_2];
       
% FoV 3
R_3 = 20;
range_3 = 120 * pi/180; % in rad
triangle3_xy = [0, 0, -R_3*sin(pi-range_3)/cos(range_3/2);
                0, R_3/cos(range_3/2), -(R_3*cos(pi-range_3))/(cos(range_3/2))];
            
            
            
f = figure(1);
% f.WindowState = 'maximized';
hold on 
axis equal       
plot(triangle3_xy(1,:), triangle3_xy(2,:), '*r')
=======
%% extract the detections by source
source1_detection = lookupTable([lookupTable.source] == 1);
source2_detection = lookupTable([lookupTable.source] == 2);

 

% A = [lookupTable.source] == 1
% find(A)
% this can find the fieds in lookupTable

 

source_1 = [lookupTable.source] == 1;
source_2 = [lookupTable.source] == 2;
fields_1 = find(source_1);
fields_2 = find(source_2);

 

Long_1    = longPosVec(fields_1);
Lat_1     = latPosVec(fields_1);
LongVar_1 = longPosVarVec(fields_1);
LatVar_1  = latPosVarVec(fields_1);

 

Long_2    = longPosVec(fields_2);
Lat_2     = latPosVec(fields_2);
LongVar_2 = longPosVarVec(fields_2);
LatVar_2  = latPosVarVec(fields_2);

 

sensor_detection_1 = struct('mean', [Lat_1, Long_1])

 

 
>>>>>>> Stashed changes
