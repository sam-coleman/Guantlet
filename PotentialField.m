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

[x,y]=meshgrid(-1.5:0.05:2.5,-3.5:0.05:1);  %Overall area of gauntlet
v_source = 0;
gx_source=0;
gy_source=0;

if all_endpts(1,1) > all_endpts(1,2)
    for a = all_endpts(1,2):0.001:all_endpts(1,1)
        a
        y_int = m_source(1,1)*a + yint_source(1,1);
        v_source = v_source - log(sqrt((x-a).^2 + (y-y_int).^2));
        gx_source = gx_source-((x-a)./((x-a).^2 + (y-y_int).^2));
        gy_source = gy_source-((y-y_int)./((x-a).^2 + (y-y_int).^2));
    end
else
    for a = all_endpts(1,1):0.001:all_endpts(1,2)
        a
        y_int = m_source(1,1)*a + yint_source(1,1);
        v_source = v_source - log(sqrt((x-a).^2 + (y-y_int).^2));
        gx_source = gx_source-((x-a)./((x-a).^2 + (y-y_int).^2));
        gy_source = gy_source-((y-y_int)./((x-a).^2 + (y-y_int).^2));
    end
end
figure
hold on
contour(x,y,v_source,'k','ShowText','On')
plot(all_endpts(1,1:2),all_endpts(1,3:4),'*')
axis equal
hold off