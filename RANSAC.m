clc
clf
clear all

load scan4.mat
figure
polarplot(deg2rad(theta),r,'ks','MarkerSize', 6, 'MarkerFaceColor','m')

%Cleaning data from places where neato didn't scan
index = find(r~=0);
r_clean = r(index);
theta_clean = theta(index);

[x1,y1] = pol2cart(deg2rad(theta_clean),r_clean);
figure
hold on
plot(x1,y1,'ks')
title('Converted')
hold off

% RANSAC Parameters
num_test_lines = 10;  %Number of random combinations to test
d = 0.5;
num_pts = length(r_clean);

inlier = 0;
outlier = 0;

figure  
for i = 1:num_test_lines
    p1 = randi([1 num_pts]);
    p2 = randi([1 num_pts]);
    while p1 == p2  %Make sure the two points are not randomly the same value
        p2 = radi([1 num_pts]);
    end
    
    line = inv([x1(p1) 1;x1(p2) 1])*[y1(p1);y1(p2)];
    x = linspace(-1.5,1.5);
    
    %Plotting line between two random points
    hold on
    %plot([x1(p1),y1(p1)],[x1(p2),y1(p2)])
    plot(x, line(1)*x + line(2));
    plot(x1,y1,'ks')
end
hold off
    inlier_now = 0;
    outlier_now = 0;
    slope_p = -1/line(1);
    for j = 1:length(r_clean)
        d_now = abs(slope_p*x1(p1)+(-1*y1(p
    end