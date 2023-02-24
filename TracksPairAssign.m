function [assignment, unassigned_1, unassigned_2] = TracksPairAssign(detection_1, detection_2, threshold)
% Input: detection_1, detection_2, assignment threshold
% Output: assignment of two detections, unassigned detection_1, and
% unassigned detection_2


    % find KLD cost matrix
    cost_matrix = zeros(size(detection_1,2), size(detection_2,2));
    for m =1:size(detection_1,2)
        for n =1:size(detection_2,2)
            score1 = KLD(detection_1{m}.mean, detection_2{n}.mean, detection_1{m}.cov, detection_2{n}.cov);
            score2 = KLD(detection_2{n}.mean, detection_1{m}.mean, detection_2{n}.cov, detection_1{m}.cov);
            cost_matrix(m, n) = 0.5*(score1+score2);
        end
    end

    [assignment, unassigned_1, unassigned_2] = assignDetectionsToTracks(cost_matrix, threshold);
end

