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