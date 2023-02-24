
clc
clear all


a = {[2,3], [4,6], [10,13,11,8], [1,7], 8, 9, 5};









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