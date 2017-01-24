figure;

subplot(121);
imagesc(location_ground_truth);

grid on;
%set(gca,'xtick',[1:8:40]);
%set(gca,'ytick',[1:8:40]);
grid minor
axis off;
title('Randomly generated location type ground truth')
subplot(122);

scatter(x_ind, y_ind, 1);
set(gca,'Color',[0.8 0.8 0.8]);
axis off;
title('Randomly generated rock map based on the location map')