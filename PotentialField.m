%This file takes our ransac_data.mat file from create_map_v3
%uses ransac lines to create potential field map

clf
clc
clear all

load ransac_data.mat

%m and b for source lines
m_source = all_m(1,1:8);
yint_source = all_b(1,1:8);

%m and b for sink lines
m_sink = all_m(1,9:11);
yint_sink = all_b(1,9:11);

%For loop to get 

%v = ln(sqrt((x-2)^2+y^2)

v_all = [];
v_all_source = [];

[x,y]=meshgrid(-1.5:0.05:2.5,-3.5:0.05:1);  %Overall area of gauntlet
x_array = -1.5:0.05:2.5;
y_array = -3.5:0.05:1;
v_source = 0;
gx_source=0;
gy_source=0;

n = 1;

syms x_sym y_sym

figure
hold on
while n<9
if all_endpts(n,1) > all_endpts(n,2)
    for a = all_endpts(n,2):0.001:all_endpts(n,1)
        a
        y_int = m_source(1,n)*a + yint_source(1,n);
        v_source = v_source - log(sqrt((x-a).^2 + (y-y_int).^2));
        gx_source = gx_source-((x-a)./((x-a).^2 + (y-y_int).^2));
        gy_source = gy_source-((y-y_int)./((x-a).^2 + (y-y_int).^2));
        %v_sym = -log(sqrt((x_sym-a).^2 + (y_sym-y_int).^2));
    end
else
    for a = all_endpts(n,1):0.001:all_endpts(n,2)
        a
        y_int = m_source(1,n)*a + yint_source(1,n);
        v_source = v_source - log(sqrt((x-a).^2 + (y-y_int).^2));
        gx_source = gx_source-((x-a)./((x-a).^2 + (y-y_int).^2));
        gy_source = gy_source-((y-y_int)./((x-a).^2 + (y-y_int).^2));
        %v_sym = -log(sqrt((x_sym-a).^2 + (y_sym-y_int).^2));
    end
end
v_all_source = [v_all_source v_source];
%v_all = [v_all, v_sym];
n = n+1;
end

%v_all_simple = simplify(v_all);
%v_values = double(subs(v_all_simple,[x_sym,y_sym],[x_array,y_array]));

contour(x,y,v_all_source,'k','ShowText','On')
plot(all_endpts(n,1:2),all_endpts(n,3:4),'*')
axis equal
hold off