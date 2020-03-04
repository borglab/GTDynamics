traj = csvread('traj.csv', 2);

body_traj = traj(:, 1:3);
lower0_traj = traj(:, 4:6);
lower1_traj = traj(:, 7:9);
lower2_traj = traj(:, 10:12);
lower3_traj = traj(:, 13:15);

figure(1);
plot3(body_traj(:,1), body_traj(:,2), body_traj(:,3), 'Color', [1, 0, 0]);
hold on;

plot3(lower0_traj(:,1), lower0_traj(:,2), lower0_traj(:,3), 'Color', [0, 1, 0]);
plot3(lower1_traj(:,1), lower1_traj(:,2), lower1_traj(:,3), 'Color', [0, 0, 1]);
plot3(lower2_traj(:,1), lower2_traj(:,2), lower2_traj(:,3), 'Color', [1, 1, 0]);
plot3(lower3_traj(:,1), lower3_traj(:,2), lower3_traj(:,3), 'Color', [0, 1, 1]);
axis([-1, 6, -3, 3, -.3, .3])
hold off;

