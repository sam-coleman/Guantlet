load ransac_data.mat
x = [0, all_endpts(10,2)];
y = [0, all_endpts(10,4)];
test(x,y)
function [v,fx,fy] = test(x,y)
v = 0;
fx = 0;
fy = 0;
b =0;
for a = -1:0.01:1
    v = v - log(sqrt((x-a).^2 + (y-b).^2))
    fx = fx - (x-a)./((x-a).^2 + (y-b).^2)
    fy = fy - (x-a)./((x-a).^2 + (y-b).^2)
end
quiver(x,y,-fx,-fy)
end

%coming up with a path of a bunch of points that we know
%using those points to drive our neato (using code similar to flatland)

%we also wanted to see if we could use the gradient descent quiver plot to
%attain these points?


