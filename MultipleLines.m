close all

%Script to identify multiple lines in a scan
load playpensample

%eliminate zeros and sample farther than 3m (range of LIDAR)
index=find(r~=0 & r<3);
r_clean=r(index);
theta_clean=theta(index);

%plot the polar data as verification
figure(1)
polarplot(deg2rad(theta_clean),r_clean,'ks','MarkerSize',6,'MarkerFaceColor','m')
title('Visualization of Polar Data')

%convert to Cartesian and plot again for verification
[x,y]=pol2cart(deg2rad(theta_clean),r_clean);
points=[x,y];
figure(2)
plot(x,y,'ks')
title('Scan Data- Clean')
xlabel('[m]')
ylabel('[m]')

bestOutlierSet=points;
nn=1;
visualize=0;
d=0.005;
n=1000;
while size(bestOutlierSet,1) > 5
    [fitline_coefs(nn,:),bestInlierSet,bestOutlierSet,bestEndPoints(:,:,nn)]= robustLineFit(r_clean,theta_clean,d,n,visualize);
    
    if isnan(fitline_coefs(nn,1))
        disp('All Lines Identified')
        break;
    end
    
    [theta_clean,r_clean]=cart2pol(bestOutlierSet(:,1),bestOutlierSet(:,2));
    theta_clean=rad2deg(theta_clean);
    
    nn=nn+1;
end

figure;
plot(x,y,'ks')
hold on
for kk=1:size(bestEndPoints,3)
    plot(bestEndPoints(:,1,kk), bestEndPoints(:,2,kk), 'r')
end
title(['RANSAC with d=' num2str(d) ' and n=' num2str(n)])
xlabel('[m]')
ylabel('[m]')