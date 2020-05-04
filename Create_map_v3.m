clc
clf
clear all

load lidar_gauntlet.mat

index=find(r~=0 & r<3);
r_clean=r(index);
theta_clean=theta(index);

%location of objects with respect to LIDAR frame L
%row 1 goes with row 5, 2 with 6, 3 with 7, 4 with 8
r_L = [r_clean(:,:).*cos(theta_clean(:,:)), r_clean(:,:).*sin(theta_clean(:,:))]';

%location of objects with respect to Neato frame N
%need to subtract .084 from rows 1-4 of r_L and keep rows 5-8 the same
r_N = [r_L(:, :) - .084; r_L(:, :)];

%Translation Matrix to go from Neato Frame to Global Frame
T_GN = [1 0 0; 0 1 0; 0 0 1];
    
%Rotation Matrix to go from Neato Frame to Global Frame
R_GN = [1 0 0; 0 1 0; 0 0 1];
   
%assign the position matrix to use and do the transformation to make
%Neato Frame into Global Frame
r_N_pos = [r_N(1, :); r_N(2, :); ones(1, length(r_clean))];
r_G = T_GN * R_GN * r_N_pos;
r_G = r_G(1:2, :);

x = r_G(1, :);  %Global x and y
y = r_G(2, :);
all_endpts = [];
all_m = [];
all_b = [];
figure()
hold on
for i = 1:11
    [x, y, endpts, m, b] = ransac(x,y,1000,.001);
    all_m = [all_m m];
    all_b = [all_b b];
    %format for all_endpts: [x1 x2 y1 y2]
    all_endpts = [all_endpts; endpts(1,1) endpts(1,2) endpts(2,1)  endpts(2,2)];
    plot(all_endpts(i, 1:2), all_endpts(i, 3:4), 'k')
end
title('RANSAC Map of Gauntlet')
xlabel('[m]')
ylabel('[m]')
hold off

%save all_m, all_b and all_endpts to a mat file
%save('ransac_data.mat', 'all_m', 'all_b', 'all_endpts')

function[x,y,endpts, m, b] = ransac(x, y, n, d)
    %Defining arrays to hold indicies of BEST inliers and outliers
    bestin = zeros(1);
    bestout = zeros(1);
    points = [x;y];  %Creating matrix of global x and y values

    for i=1:n
        %Finding random indicies
        p1 = randi([1 length(x)]);
        p2 = randi([1 length(x)]);
    while p1 == p2  %Make sure the two points are not randomly the same value
        p2 = randi([1 length(x)]);
    end
    candidates = [x(p1) x(p2); y(p1) y(p2)];  %x and y values at random indicies

    %Defining arrays to hold indicies of inliers and outliers
    in = zeros(1);
    out = zeros(1);

    %Unit vector in direction between two points
    v = (candidates(:,1) - candidates(:,2))/(norm(candidates(:,1) - candidates(:,2)));

    %Vector orthogonal to vector in direction of line (both unit vectors)
    vorth = [-v(2,1);v(1,1)];

    %Distance between all of our points and one of our random candidate points
    dists = points - candidates(:,1);

    orth_dists = abs(dists'*vorth);

    % quiver(candidates(1,:),candidates(2,:),v(1,1),v(2,1))

    %Finding points that are in and out of range
    %thse are the INDICES
    in_range=find(orth_dists<d);
    out_range=find(orth_dists>d);
    % in=points(in_range)
    % out=points(out_range)

    if length(in_range(:,1)) > length(bestin(:,1))
        bestin = in_range;
        bestout = out_range;
    end
    end

    in_ranget = bestin';
    x_inrange = x(1, in_ranget(1,:));
    y_inrange = y(1, in_ranget(1,:));
    points_inrange = [x_inrange; y_inrange];
    %find the endpoints and their indices
    [least_x, least_x_index] = min(x_inrange);
    [greatest_x, greatest_x_index] = max(x_inrange);
    [least_y, least_y_index] = min(y_inrange);
    [greatest_y, greatest_y_index] = max(y_inrange);

    if greatest_y - least_y > greatest_x - least_x
        x1 = x_inrange(greatest_y_index);
        y1 = y_inrange(greatest_y_index);
        endpt1 = [x1; y1];
        x2 = x_inrange(least_y_index);
        y2 = y_inrange(least_y_index);
        endpt2 = [x2; y2];
    else
        x1 = x_inrange(greatest_x_index);
        y1 = y_inrange(greatest_x_index);
        endpt1 = [x1; y1];
        x2 = x_inrange(least_x_index);
        y2 = y_inrange(least_x_index);
        endpt2 = [x2; y2];
    end

    endpts = [endpt1 endpt2];

    coefficients = polyfit([x1, x2], [y1, y2], 1);
    m = coefficients(1);
    b = coefficients(2);
    
    %remove all inlier points from x and y
    x = x(:,bestout);
    y = y(:,bestout);
end