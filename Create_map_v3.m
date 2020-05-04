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

points = [x;y];  %Creating matrix of global x and y values

%Defining arrays to hold indicies of BEST inliers and outliers
bestin = zeros(1);
bestout = zeros(1);

%START THE FOR LOOP HERE!!!!!!!!!!!!!!!!!!!!!!!!!

%Finding random indicies
p1 = randi([1 length(r_G)]);
p2 = randi([1 length(r_G)]);
while p1 == p2  %Make sure the two points are not randomly the same value
    p2 = randi([1 length(r_G)]);
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
d=0.005;

orth_dists = dists'*vorth;

% quiver(candidates(1,:),candidates(2,:),v(1,1),v(2,1))

%Finding points that are in and out of range
in_range=find(orth_dists<d);
out_range=find(orth_dists>d);
% in=points(in_range)
% out=points(out_range)

length(in_range(:,1))
length(bestin(:,1))
if length(in_range(:,1)) > length(bestin(:,1))
    bestin = in_range;
    bestout = out_range;
end

%END OF FOR LOOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

in_ranget = in_range';
x_inrange = x(1, in_ranget(1,:));
y_inrange = y(1, in_ranget(1,:));
points_inrange = [x_inrange; y_inrange];
%find the endpoints
least_x = min(x_inrange);
greatest_x = max(x_inrange);
least_y = min(y_inrange);
greatest_y = max(y_inrange);

% if greatest_y - least_y > greatest_x - least_x
% % endpt1: want index of greatest_y in matrix y_inrange. With that index,
% % that coordinate (x,y) is endpt1
% %endpt2: same as above but with least_y
% 
% else
% %same as but with x instead of y
% end

%projectedCoordinate = dists(bestin(:),:)*v/norm(v);
%bestEndPoints = [min(projectedCoordinate); max(projectedCoordinate)]*v'/norm(v) + repmat(candidates(2, :), [2, 1])
    