%This file takes our ransac_data.mat file from create_map_v3
%uses ransac lines to create potential field map

%clf
clc
clear all

load ransac_data.mat

%m and b for source lines (from ransac_data)
m_source = all_m(1,1:8);
yint_source = all_b(1,1:8);

%m and b for sink lines (from ransac_data)
m_sink = all_m(1,9:11);
yint_sink = all_b(1,9:11);

%For loop to get 

%v = ln(sqrt((x-2)^2+y^2)

%intialize variables
v_all = 0;
[x,y]=meshgrid(-1.5:0.05:2.5,-3.5:0.05:1);  %Overall area of gauntlet
x_array = -1.5:0.05:2.5;
y_array = -3.5:0.05:1;
v_source = 0;
v_sink = 0;
gx_source=0;
gy_source=0;
gx_sink = 0;
gy_sink = 0;
n = 1;

syms x_sym y_sym

figure
hold on
%iterate through our 9 source lines
while n<9
    %reset v_source for each line
    v_source = 0;
    num_points = 0;
    %if statement determines which endpoint is further left (we iterate
    %left to right based on x values)
    if all_endpts(n,1) > all_endpts(n,2)
        %for loop creates line of evenly spaced sources for each ransac
        %line
        for a = all_endpts(n,2):0.001:all_endpts(n,1)
            a; %unsurpress to see it iterating
            y_int = m_source(1,n)*a + yint_source(1,n); %find corresponding y value for each x
            v_source = v_source - log(sqrt((x-a).^2 + (y-y_int).^2)); %add sources
            gx_source = gx_source-((x-a)./((x-a).^2 + (y-y_int).^2)); %gradient vector in ihat dir
            gy_source = gy_source-((y-y_int)./((x-a).^2 + (y-y_int).^2)); %gradient vector in jhat dir
            %v_sym = -log(sqrt((x_sym-a).^2 + (y_sym-y_int).^2));
            num_points = num_points + 1;
        end
    else
        for a = all_endpts(n,1):0.001:all_endpts(n,2)
            a;
            y_int = m_source(1,n)*a + yint_source(1,n);
            v_source = v_source - log(sqrt((x-a).^2 + (y-y_int).^2));
            gx_source = gx_source-((x-a)./((x-a).^2 + (y-y_int).^2));
            gy_source = gy_source-((y-y_int)./((x-a).^2 + (y-y_int).^2));
            %v_sym = -log(sqrt((x_sym-a).^2 + (y_sym-y_int).^2));
            num_points = num_points + 1;
        end
    end
    v_source_scaled = v_source ./ num_points;
    v_all = v_all + v_source_scaled; %update the combination of all v_source_all
    %v_all_source = [v_all_source v_source];

    %v_all = [v_all, v_sym];
    n = n+1;
end
n = 9;
% while n < 12
%  %reset v_source for each line
%     v_sink = 0;
%     num_points = 0;
%     %if statement determines which endpoint is further left (we iterate
%     %left to right based on x values)
%     if all_endpts(n,1) > all_endpts(n,2)
%         %for loop creates line of evenly spaced sources for each ransac
%         %line
%         for a = all_endpts(n,2):0.001:all_endpts(n,1)
%             a; %unsurpress to see it iterating
%             y_int = all_m(1,n)*a + all_m(1,n); %find corresponding y value for each x
%             v_sink = v_sink + log(sqrt((x-a).^2 + (y-y_int).^2)); %add sources
%             gx_sink = gx_sink+((x-a)./((x-a).^2 + (y-y_int).^2)); %gradient vector in ihat dir
%             gy_sink = gy_sink+((y-y_int)./((x-a).^2 + (y-y_int).^2)); %gradient vector in jhat dir
%             %v_sym = -log(sqrt((x_sym-a).^2 + (y_sym-y_int).^2));
%             num_points = num_points + 1;
%         end
%     else
%         for a = all_endpts(n,1):0.001:all_endpts(n,2)
%             a;
%             y_int = all_m(1,n)*a + all_m(1,n);
%             v_sink = v_sink + log(sqrt((x-a).^2 + (y-y_int).^2));
%             gx_sink = gx_sink+((x-a)./((x-a).^2 + (y-y_int).^2));
%             gy_sink = gy_sink+((y-y_int)./((x-a).^2 + (y-y_int).^2));
%             %v_sym = -log(sqrt((x_sym-a).^2 + (y_sym-y_int).^2));
%             num_points = num_points + 1;
%         end
%     end
%     v_sink_scaled = v_sink ./ num_points;
%     %v_sink_scaled = v_sink_scaled .* 10;
%     v_all = v_all + v_sink_scaled; %update the combination of all v_source_all
%     %v_all_source = [v_all_source v_source];
% 
%     %v_all = [v_all, v_sym];
%     n = n+1;    
% end

%middle line of ransac is index 10
%take endpoint of index 10 and use that for the sink equation
%plot sink equation
sink_x = all_endpts(10,2);
sink_y = all_endpts(10,4);
sink_eq = log(sqrt((x-sink_x).^2 + (y-sink_y).^2));
v_all = v_all + sink_eq;
%v_all_simple = simplify(v_all);
%v_values = double(subs(v_all_simple,[x_sym,y_sym],[x_array,y_array]));

contour(x,y,v_all,'k','ShowText','On')
axis equal
hold off